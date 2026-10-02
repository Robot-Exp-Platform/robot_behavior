#![cfg(feature = "roplat")]
mod support;
use robot_behavior::{
    RobotException,
    roplat::{ControlDomainExit, ControlRhythm, ControlSessionError},
};
use roplat::{
    Completion, ExecutionContext, Lifecycle, Node, RoplatError, RoplatResult, rhythm::Rhythm,
};
use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};
use support::{TestControl, TestRobot};

type TestRhythm = ControlRhythm<TestRobot, TestControl>;
fn session_error(error: &RoplatError) -> &ControlSessionError {
    let RoplatError::Io(error) = error else {
        panic!("expected device/session error: {error:?}")
    };
    error.get_ref().unwrap().downcast_ref().unwrap()
}

#[tokio::test]
async fn complete_returns_robot_and_same_nonclone_node_allocation() {
    let mut rhythm = TestRhythm::new();
    let nodes = Box::new(10);
    let address = (&*nodes) as *const i32 as usize;
    let (result, nodes) = rhythm
        .drive(
            nodes,
            |mut nodes, (obs, _), _| async move {
                *nodes += 1;
                (Ok(Completion::Completed((obs, obs == 2))), nodes)
            },
            Ok(TestRobot::new()),
            ExecutionContext::new(),
        )
        .await;
    let Completion::Completed(robot) = result.unwrap() else {
        panic!()
    };
    assert_eq!(robot.trace.lock().unwrap().commands, [0, 1, 2]);
    assert_eq!(*nodes, 13);
    assert_eq!((&*nodes) as *const i32 as usize, address);
}

#[tokio::test]
async fn domain_failure_returns_nodes_and_original_error_without_command() {
    let robot = TestRobot::new();
    let trace = robot.trace.clone();
    let context = ExecutionContext::new();
    let (result, nodes) = TestRhythm::new()
        .drive(
            7,
            |n, _, _| async move {
                (
                    Err(RoplatError::Arithmetic("controller failed".into())),
                    n + 1,
                )
            },
            Ok(robot),
            context.clone(),
        )
        .await;
    assert!(matches!(result, Err(RoplatError::Arithmetic(ref s)) if s == "controller failed"));
    assert_eq!(nodes, 8);
    assert!(context.is_stopping());
    let trace = trace.lock().unwrap();
    assert!(trace.commands.is_empty());
    assert_eq!(trace.callbacks, 1);
    assert_eq!(trace.exits, 1);
}

#[tokio::test]
async fn stopped_is_not_failure_and_returns_nodes_after_cleanup() {
    let robot = TestRobot::new();
    let trace = robot.trace.clone();
    let context = ExecutionContext::new();
    let (result, nodes) = TestRhythm::new()
        .drive(
            7,
            |n, _, _| async move { (Ok(Completion::Stopped), n + 1) },
            Ok(robot),
            context.clone(),
        )
        .await;
    assert!(matches!(result, Ok(Completion::Stopped)));
    assert_eq!(nodes, 8);
    assert!(context.is_stopping());
    let trace = trace.lock().unwrap();
    assert!(trace.commands.is_empty());
    assert_eq!(trace.exits, 1);
}

#[tokio::test]
async fn stopped_context_never_enters_device_session() {
    let robot = TestRobot::new();
    let trace = robot.trace.clone();
    let context = ExecutionContext::new();
    context.request_stop();
    let (result, nodes) = TestRhythm::new()
        .drive(
            7,
            |_, _, _| async { panic!("no callback") },
            Ok(robot),
            context,
        )
        .await;
    assert!(matches!(result, Ok(Completion::Stopped)));
    assert_eq!(nodes, 7);
    assert_eq!(trace.lock().unwrap().entries, 0);
}

#[tokio::test]
async fn failed_robot_input_becomes_execution_failure() {
    let context = ExecutionContext::new();
    let (result, nodes) = TestRhythm::new()
        .drive(
            7,
            |_, _, _| async { panic!("no callback") },
            Err(RobotException::NetworkError("connect".into())),
            context.clone(),
        )
        .await;
    let error = result.unwrap_err();
    assert!(
        matches!(session_error(&error).device, RobotException::NetworkError(ref s) if s == "connect")
    );
    assert_eq!(nodes, 7);
    assert!(context.is_stopping());
}

#[tokio::test]
async fn device_entry_failure_becomes_execution_failure() {
    let mut robot = TestRobot::new();
    robot.fail_entry = true;
    let context = ExecutionContext::new();
    let (result, nodes) = TestRhythm::new()
        .drive(
            7,
            |_, _, _| async { panic!("no callback") },
            Ok(robot),
            context.clone(),
        )
        .await;
    assert!(session_error(&result.unwrap_err()).domain.is_none());
    assert_eq!(nodes, 7);
    assert!(context.is_stopping());
}

#[tokio::test]
async fn domain_and_device_cleanup_failures_are_both_retained() {
    let mut robot = TestRobot::new();
    robot.fail_exit = true;
    let (result, nodes) = TestRhythm::new()
        .drive(
            7,
            |n, _, _| async move { (Err(RoplatError::NodeProcessing("original".into())), n + 1) },
            Ok(robot),
            ExecutionContext::new(),
        )
        .await;
    let error = result.unwrap_err();
    let session = session_error(&error);
    assert!(
        matches!(session.domain, Some(ControlDomainExit::Failed(RoplatError::NodeProcessing(ref s))) if s == "original")
    );
    assert!(
        matches!(session.device, RobotException::CommandException(ref s) if s == "cleanup failed")
    );
    assert_eq!(nodes, 8);
}

#[tokio::test]
async fn cleanup_failure_takes_priority_over_cooperative_stop() {
    let mut robot = TestRobot::new();
    robot.fail_exit = true;
    let (result, nodes) = TestRhythm::new()
        .drive(
            7,
            |n, _, _| async move { (Ok(Completion::Stopped), n + 1) },
            Ok(robot),
            ExecutionContext::new(),
        )
        .await;
    assert!(matches!(
        session_error(&result.unwrap_err()).domain,
        Some(ControlDomainExit::Stopped)
    ));
    assert_eq!(nodes, 8);
}

#[tokio::test]
async fn completed_feed_is_sent_and_external_stop_is_observed_next_cycle() {
    let robot = TestRobot::new();
    let trace = robot.trace.clone();
    let (result, nodes) = TestRhythm::new()
        .drive(
            7,
            |n, _, context| async move {
                context.request_stop();
                (Ok(Completion::Completed((42, false))), n + 1)
            },
            Ok(robot),
            ExecutionContext::new(),
        )
        .await;
    assert!(matches!(result, Ok(Completion::Stopped)));
    assert_eq!(nodes, 8);
    assert_eq!(trace.lock().unwrap().commands, [42]);
}

struct Controller {
    calls: usize,
    inits: Arc<AtomicUsize>,
    shutdowns: Arc<AtomicUsize>,
}
impl Lifecycle for Controller {
    type Error = RoplatError;
    async fn on_init(&mut self) -> RoplatResult<()> {
        self.inits.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
    async fn on_shutdown(&mut self) -> RoplatResult<()> {
        self.shutdowns.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}
impl Node for Controller {
    type Input = (u32, std::time::Duration);
    type Output = (u32, bool);
    async fn process(&mut self, (obs, _): Self::Input) -> Self::Output {
        self.calls += 1;
        (obs, obs == 1)
    }
}
struct Source(Option<TestRobot>);
impl Lifecycle for Source {
    type Error = RoplatError;
}
impl Node for Source {
    type Input = ();
    type Output = robot_behavior::RobotResult<TestRobot>;
    async fn process(&mut self, (): ()) -> Self::Output {
        Ok(self.0.take().unwrap())
    }
}

#[roplat::system]
async fn run_system(
    mut rhythm: TestRhythm,
    mut controller: Controller,
    robot: TestRobot,
) -> RoplatResult<(TestRobot, TestRhythm, Controller)> {
    let mut source = Source(Some(robot));
    source >> rhythm >> |state| state >> controller;
    Ok((rhythm.output, rhythm, controller))
}

#[tokio::test]
async fn real_system_reuses_external_nodes_without_reactivation() {
    let inits = Arc::new(AtomicUsize::new(0));
    let shutdowns = Arc::new(AtomicUsize::new(0));
    let mut controller =
        Controller { calls: 0, inits: inits.clone(), shutdowns: shutdowns.clone() };
    controller.on_init().await.unwrap();
    let (robot, rhythm, controller) = run_system(TestRhythm::new(), controller, TestRobot::new())
        .await
        .unwrap();
    let (robot, _, mut controller) = run_system(rhythm, controller, robot).await.unwrap();
    assert_eq!(controller.calls, 4);
    assert_eq!(robot.trace.lock().unwrap().entries, 2);
    assert_eq!(inits.load(Ordering::Relaxed), 1);
    assert_eq!(shutdowns.load(Ordering::Relaxed), 0);
    controller.on_shutdown().await.unwrap();
    assert_eq!(shutdowns.load(Ordering::Relaxed), 1);
}

struct CommandDomain {
    fail: bool,
}
impl Lifecycle for CommandDomain {
    type Error = RoplatError;
}
impl Rhythm for CommandDomain {
    type Input = (u32, std::time::Duration);
    type Yield = Self::Input;
    type Feed = (u32, bool);
    type Output = Self::Feed;
    async fn drive<N, F, Fut>(
        &mut self,
        nodes: N,
        mut domain: F,
        input: Self::Input,
        context: ExecutionContext,
    ) -> (roplat::Execution<Self::Output>, N)
    where
        N: Send,
        F: FnMut(N, Self::Yield, ExecutionContext) -> Fut + Send,
        Fut: std::future::Future<Output = (roplat::Execution<Self::Feed>, N)> + Send,
    {
        let (result, nodes) = domain(nodes, input, context.clone()).await;
        if self.fail {
            context.request_stop();
            (
                Err(RoplatError::NodeProcessing("nested domain".into())),
                nodes,
            )
        } else {
            (result, nodes)
        }
    }
}

#[roplat::system]
async fn nested_system(
    mut rhythm: TestRhythm,
    mut inner: CommandDomain,
    mut controller: Controller,
    robot: TestRobot,
) -> RoplatResult<(
    roplat::Execution<TestRobot>,
    TestRhythm,
    CommandDomain,
    Controller,
)> {
    let mut source = Source(Some(robot));
    source >> rhythm >> |state| state >> inner >> |state| state >> controller;
    Ok((rhythm.outcome, rhythm, inner, controller))
}

#[tokio::test]
async fn nested_system_failure_returns_external_nodes_and_isolates_outcome() {
    let robot = TestRobot::new();
    let trace = robot.trace.clone();
    let controller = Controller { calls: 0, inits: Arc::default(), shutdowns: Arc::default() };
    let (outcome, _, inner, controller) = nested_system(
        TestRhythm::new(),
        CommandDomain { fail: true },
        controller,
        robot,
    )
    .await
    .unwrap();
    assert!(
        matches!(outcome,Err(RoplatError::NodeProcessing(ref message)) if message == "nested domain")
    );
    assert!(inner.fail);
    assert_eq!(controller.calls, 1);
    let trace = trace.lock().unwrap();
    assert!(trace.commands.is_empty());
    assert_eq!(trace.exits, 1);
}

#[tokio::test]
async fn nested_system_returns_complete_child_feed_each_cycle() {
    let controller = Controller { calls: 0, inits: Arc::default(), shutdowns: Arc::default() };
    let (outcome, _, _, controller) = nested_system(
        TestRhythm::new(),
        CommandDomain { fail: false },
        controller,
        TestRobot::new(),
    )
    .await
    .unwrap();
    let Completion::Completed(robot) = outcome.unwrap() else {
        panic!()
    };
    assert_eq!(robot.trace.lock().unwrap().commands, [0, 1]);
    assert_eq!(controller.calls, 2);
}

#[tokio::test]
async fn final_valid_feed_completes_even_if_stop_is_requested_in_that_cycle() {
    let context = ExecutionContext::new();
    let (result, nodes) = TestRhythm::new()
        .drive(
            7,
            |n, _, context| async move {
                context.request_stop();
                (Ok(Completion::Completed((42, true))), n + 1)
            },
            Ok(TestRobot::new()),
            context.clone(),
        )
        .await;
    let Completion::Completed(robot) = result.unwrap() else {
        panic!()
    };
    assert_eq!(robot.trace.lock().unwrap().commands, [42]);
    assert_eq!(nodes, 8);
    assert!(context.is_stopping());
}
