//! Finite, device-free native-session tests. A single current-thread runtime
//! polls the driver, its callback and sibling futures; no helper thread exists.
use robot_behavior::{
    AsyncControl, AsyncControlCallback, AsyncControlWith, ControlSpace, ControlStep, Robot,
    RobotException, RobotResult,
};
use std::{
    ops::ControlFlow,
    sync::{Arc, Mutex},
    time::Duration,
};

#[derive(Default, Debug)]
struct Trace {
    commands: Vec<u32>,
    events: Vec<&'static str>,
}
struct Channel;
struct AsyncRobot {
    trace: Arc<Mutex<Trace>>,
    fail_entry: bool,
    fail_exit: bool,
}
impl AsyncRobot {
    fn new() -> Self {
        Self { trace: Arc::default(), fail_entry: false, fail_exit: false }
    }
}
impl Robot for AsyncRobot {
    type State = ();
    const CONTROL_PERIOD: f64 = 0.001;
    fn version() -> String {
        "native-async-mock".into()
    }
    fn read_state(&mut self) -> RobotResult<()> {
        Ok(())
    }
}
impl ControlSpace<AsyncRobot> for Channel {
    type Obs = u32;
    type Command = u32;
}
impl AsyncControlWith<Channel> for AsyncRobot {
    async fn control_native_async<C>(&mut self, callback: &mut C) -> RobotResult<()>
    where
        C: AsyncControlCallback<u32, u32>,
    {
        self.trace.lock().unwrap().events.push("entry");
        tokio::task::yield_now().await;
        if self.fail_entry {
            return Err(RobotException::NetworkError("entry failed".into()));
        }
        for tick in 0..100 {
            // Native I/O can suspend before a state is available.
            tokio::task::yield_now().await;
            self.trace.lock().unwrap().events.push("callback");
            match callback.call(tick, Duration::from_millis(1)).await {
                ControlFlow::Continue((command, done)) => {
                    self.trace.lock().unwrap().commands.push(command);
                    if !done {
                        continue;
                    }
                }
                ControlFlow::Break(()) => {}
            }
            self.trace.lock().unwrap().events.push("exit-start");
            tokio::task::yield_now().await;
            self.trace.lock().unwrap().events.push("exit-done");
            return if self.fail_exit {
                Err(RobotException::CommandException("exit failed".into()))
            } else {
                Ok(())
            };
        }
        panic!("mock exceeded finite cycle budget")
    }
}
fn assert_send<T: Send>(value: T) -> T {
    value
}

struct BorrowingController {
    values: Vec<u32>,
}
impl AsyncControlCallback<u32, u32> for BorrowingController {
    async fn call(&mut self, observation: u32, _: Duration) -> ControlStep<u32> {
        self.values.push(observation);
        tokio::task::yield_now().await;
        ControlFlow::Continue((*self.values.last().unwrap() + 10, self.values.len() == 3))
    }
}

#[tokio::test(flavor = "current_thread")]
async fn native_session_and_lending_callback_are_send_and_recover_callback_state() {
    let mut robot = AsyncRobot::new();
    let mut callback = BorrowingController { values: Vec::new() };
    assert_send(AsyncControl::control_native_async::<Channel, _>(
        &mut robot,
        &mut callback,
    ))
    .await
    .unwrap();
    assert_eq!(callback.values, [0, 1, 2]);
    assert_eq!(robot.trace.lock().unwrap().commands, [10, 11, 12]);
    assert_eq!(
        robot.trace.lock().unwrap().events.last(),
        Some(&"exit-done")
    );
}

#[tokio::test(flavor = "current_thread")]
async fn ordinary_future_closure_can_borrow_state_before_building_its_future() {
    let mut robot = AsyncRobot::new();
    let mut calls = 0;
    let mut callback = |obs, _| {
        calls += 1;
        let done = calls == 2;
        async move {
            tokio::task::yield_now().await;
            ControlFlow::Continue((obs + 1, done))
        }
    };
    AsyncControl::control_native_async::<Channel, _>(&mut robot, &mut callback)
        .await
        .unwrap();
    assert_eq!(calls, 2);
    assert_eq!(robot.trace.lock().unwrap().commands, [1, 2]);
}

#[tokio::test(flavor = "current_thread")]
async fn break_sends_no_command_and_awaits_device_termination() {
    let mut robot = AsyncRobot::new();
    AsyncControl::control_native_async::<Channel, _>(&mut robot, &mut |_, _| async {
        ControlFlow::Break(())
    })
    .await
    .unwrap();
    let trace = robot.trace.lock().unwrap();
    assert!(trace.commands.is_empty());
    assert_eq!(
        trace.events,
        ["entry", "callback", "exit-start", "exit-done"]
    );
}

#[cfg(feature = "roplat")]
mod rhythm {
    use super::*;
    use robot_behavior::roplat::{AsyncControlRhythm, ControlDomainExit, ControlSessionError};
    use roplat::{
        Completion, Execution, ExecutionContext, Lifecycle, Node, RoplatError, RoplatResult,
        rhythm::Rhythm,
    };
    use std::{
        future::Future,
        sync::atomic::{AtomicUsize, Ordering},
    };
    use tokio::sync::oneshot;

    type TestRhythm = AsyncControlRhythm<AsyncRobot, Channel>;
    struct NonCloneState(i32);
    fn session_error(error: &RoplatError) -> &ControlSessionError {
        let RoplatError::Io(error) = error else {
            panic!("{error:?}")
        };
        error.get_ref().unwrap().downcast_ref().unwrap()
    }

    #[tokio::test(flavor = "current_thread")]
    async fn same_task_sibling_unblocks_a_pending_domain_without_losing_nodes() {
        let robot = AsyncRobot::new();
        let trace = robot.trace.clone();
        let (started_tx, started_rx) = oneshot::channel();
        let (release_tx, release_rx) = oneshot::channel();
        let nodes = (Box::new(NonCloneState(7)), Some((started_tx, release_rx)));
        let address = (&*nodes.0) as *const NonCloneState as usize;
        let mut rhythm = TestRhythm::new();
        let drive = assert_send(rhythm.drive(
            nodes,
            |(mut n, signals), _, _| async move {
                let (started, release) = signals.unwrap();
                started.send(()).unwrap();
                // No task or thread can satisfy this except the join sibling.
                release.await.unwrap();
                n.0 += 1;
                (Ok(Completion::Completed((42, true))), (n, None))
            },
            Ok(robot),
            ExecutionContext::new(),
        ));
        let sibling = async {
            started_rx.await.unwrap();
            assert!(trace.lock().unwrap().commands.is_empty());
            trace.lock().unwrap().events.push("sibling");
            release_tx.send(()).unwrap();
        };
        let ((result, (nodes, _)), ()) = tokio::time::timeout(Duration::from_secs(2), async {
            tokio::join!(drive, sibling)
        })
        .await
        .expect("native session starved its sibling");
        assert!(matches!(result, Ok(Completion::Completed(_))));
        assert_eq!(nodes.0, 8);
        assert_eq!((&*nodes) as *const NonCloneState as usize, address);
        let trace = trace.lock().unwrap();
        assert_eq!(trace.commands, [42]);
        assert_eq!(
            trace.events,
            ["entry", "callback", "sibling", "exit-start", "exit-done"]
        );
    }

    #[tokio::test(flavor = "current_thread")]
    async fn external_stop_waits_for_pending_domain_and_commits_its_successful_feed() {
        for final_command in [false, true] {
            let robot = AsyncRobot::new();
            let trace = robot.trace.clone();
            let context = ExecutionContext::new();
            let (entered_tx, entered_rx) = oneshot::channel();
            let (release_tx, release_rx) = oneshot::channel();
            let mut rhythm = TestRhythm::new();
            let drive = rhythm.drive(
                (Box::new(0), Some((entered_tx, release_rx))),
                move |(mut n, channels), _, _| async move {
                    let (entered, release) = channels.expect("no second domain after stop");
                    entered.send(()).unwrap();
                    release.await.unwrap();
                    *n += 1;
                    (Ok(Completion::Completed((42, final_command))), (n, None))
                },
                Ok(robot),
                context.clone(),
            );
            let stop = async {
                entered_rx.await.unwrap();
                context.request_stop();
                tokio::task::yield_now().await;
                assert!(trace.lock().unwrap().commands.is_empty());
                assert!(!trace.lock().unwrap().events.contains(&"exit-start"));
                release_tx.send(()).unwrap();
            };
            let ((result, (n, _)), ()) =
                tokio::time::timeout(Duration::from_secs(2), async { tokio::join!(drive, stop) })
                    .await
                    .unwrap();
            assert_eq!(*n, 1);
            assert_eq!(trace.lock().unwrap().commands, [42]);
            assert_eq!(trace.lock().unwrap().events.last(), Some(&"exit-done"));
            assert!(if final_command {
                matches!(result, Ok(Completion::Completed(_)))
            } else {
                matches!(result, Ok(Completion::Stopped))
            });
        }
    }

    #[tokio::test(flavor = "current_thread")]
    async fn failed_domain_and_failed_device_cleanup_preserve_both_errors_and_nodes() {
        for fail_exit in [false, true] {
            let mut robot = AsyncRobot::new();
            robot.fail_exit = fail_exit;
            let trace = robot.trace.clone();
            let context = ExecutionContext::new();
            let (result, nodes) = TestRhythm::new()
                .drive(
                    Box::new(7),
                    |mut n, _, _| async move {
                        tokio::task::yield_now().await;
                        *n += 1;
                        (Err(RoplatError::NodeProcessing("domain failed".into())), n)
                    },
                    Ok(robot),
                    context.clone(),
                )
                .await;
            let error = result.err().unwrap();
            if fail_exit {
                let failure = session_error(&error);
                assert!(
                    matches!(failure.device, RobotException::CommandException(ref message) if message == "exit failed")
                );
                assert!(
                    matches!(&failure.domain, Some(ControlDomainExit::Failed(RoplatError::NodeProcessing(message))) if message == "domain failed")
                );
            } else {
                assert!(
                    matches!(error, RoplatError::NodeProcessing(ref message) if message == "domain failed")
                );
            }
            assert_eq!(*nodes, 8);
            assert!(context.is_stopping());
            assert!(trace.lock().unwrap().commands.is_empty());
            assert_eq!(trace.lock().unwrap().events.last(), Some(&"exit-done"));
        }
    }

    #[tokio::test(flavor = "current_thread")]
    async fn stopped_domain_has_no_command_and_cleanup_failure_still_wins() {
        for fail_exit in [false, true] {
            let mut robot = AsyncRobot::new();
            robot.fail_exit = fail_exit;
            let trace = robot.trace.clone();
            let (result, n) = TestRhythm::new()
                .drive(
                    Box::new(9),
                    |n, _, _| async move {
                        tokio::task::yield_now().await;
                        (Ok(Completion::Stopped), n)
                    },
                    Ok(robot),
                    ExecutionContext::new(),
                )
                .await;
            if fail_exit {
                assert!(matches!(
                    session_error(&result.err().unwrap()).domain,
                    Some(ControlDomainExit::Stopped)
                ));
            } else {
                assert!(matches!(result, Ok(Completion::Stopped)));
            }
            assert_eq!(*n, 9);
            assert!(trace.lock().unwrap().commands.is_empty());
        }
    }

    #[tokio::test(flavor = "current_thread")]
    async fn stopped_context_never_enters_device_and_keeps_node_allocation() {
        let robot = AsyncRobot::new();
        let trace = robot.trace.clone();
        let context = ExecutionContext::new();
        context.request_stop();
        let nodes = Box::new(3);
        let address = (&*nodes) as *const i32 as usize;
        let (result, nodes) = TestRhythm::new()
            .drive(
                nodes,
                |_, _, _| async { panic!("no callback") },
                Ok(robot),
                context,
            )
            .await;
        assert!(matches!(result, Ok(Completion::Stopped)));
        assert_eq!((&*nodes) as *const i32 as usize, address);
        assert!(trace.lock().unwrap().events.is_empty());
    }

    #[tokio::test(flavor = "current_thread")]
    async fn failed_input_and_failed_entry_return_nodes_without_domain_call() {
        let mut robot = AsyncRobot::new();
        robot.fail_entry = true;
        for input in [
            Err(RobotException::NetworkError("input failed".into())),
            Ok(robot),
        ] {
            let context = ExecutionContext::new();
            let (result, nodes) = TestRhythm::new()
                .drive(
                    Box::new(3),
                    |_, _, _| async { panic!("no callback") },
                    input,
                    context.clone(),
                )
                .await;
            assert!(session_error(&result.err().unwrap()).domain.is_none());
            assert_eq!(*nodes, 3);
            assert!(context.is_stopping());
        }
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
        type Input = (u32, Duration);
        type Output = (u32, bool);
        async fn process(&mut self, (observation, _): Self::Input) -> Self::Output {
            self.calls += 1;
            tokio::task::yield_now().await;
            (observation, observation == 1)
        }
    }
    struct Source(Option<AsyncRobot>);
    impl Lifecycle for Source {
        type Error = RoplatError;
    }
    impl Node for Source {
        type Input = ();
        type Output = RobotResult<AsyncRobot>;
        async fn process(&mut self, (): ()) -> Self::Output {
            Ok(self.0.take().unwrap())
        }
    }
    struct ChildDomain {
        fail: bool,
    }
    impl Lifecycle for ChildDomain {
        type Error = RoplatError;
    }
    impl Rhythm for ChildDomain {
        type Input = (u32, Duration);
        type Yield = Self::Input;
        type Feed = (u32, bool);
        type Output = Self::Feed;
        async fn drive<N, F, Fut>(
            &mut self,
            nodes: N,
            mut domain: F,
            input: Self::Input,
            context: ExecutionContext,
        ) -> (Execution<Self::Output>, N)
        where
            N: Send,
            F: FnMut(N, Self::Yield, ExecutionContext) -> Fut + Send,
            Fut: Future<Output = (Execution<Self::Feed>, N)> + Send,
        {
            let (execution, nodes) = domain(nodes, input, context.clone()).await;
            if self.fail {
                context.request_stop();
                (
                    Err(RoplatError::NodeProcessing("nested domain failed".into())),
                    nodes,
                )
            } else {
                (execution, nodes)
            }
        }
    }

    #[roplat::system]
    async fn nested_system(
        mut rhythm: TestRhythm,
        mut child: ChildDomain,
        mut controller: Controller,
        robot: AsyncRobot,
    ) -> RoplatResult<(Execution<AsyncRobot>, TestRhythm, ChildDomain, Controller)> {
        let mut source = Source(Some(robot));
        source >> rhythm >> |state| state >> child >> |state| state >> controller;
        Ok((rhythm.outcome, rhythm, child, controller))
    }

    #[tokio::test(flavor = "current_thread")]
    async fn nested_system_preserves_creator_lifecycle_and_state_across_drives() {
        let inits = Arc::new(AtomicUsize::new(0));
        let shutdowns = Arc::new(AtomicUsize::new(0));
        let mut controller =
            Controller { calls: 0, inits: inits.clone(), shutdowns: shutdowns.clone() };
        controller.on_init().await.unwrap();
        let (outcome, rhythm, child, controller) = tokio::spawn(assert_send(nested_system(
            TestRhythm::new(),
            ChildDomain { fail: false },
            controller,
            AsyncRobot::new(),
        )))
        .await
        .unwrap()
        .unwrap();
        let Completion::Completed(robot) = outcome.unwrap() else {
            panic!()
        };
        let (outcome, _, _, mut controller) = nested_system(rhythm, child, controller, robot)
            .await
            .unwrap();
        let Completion::Completed(robot) = outcome.unwrap() else {
            panic!()
        };
        assert_eq!(controller.calls, 4);
        assert_eq!(robot.trace.lock().unwrap().commands, [0, 1, 0, 1]);
        assert_eq!(inits.load(Ordering::Relaxed), 1);
        assert_eq!(shutdowns.load(Ordering::Relaxed), 0);
        controller.on_shutdown().await.unwrap();
        assert_eq!(shutdowns.load(Ordering::Relaxed), 1);
    }

    #[tokio::test(flavor = "current_thread")]
    async fn nested_failure_returns_external_nodes_without_a_fabricated_command() {
        let robot = AsyncRobot::new();
        let trace = robot.trace.clone();
        let controller = Controller { calls: 0, inits: Arc::default(), shutdowns: Arc::default() };
        let (outcome, _, child, controller) = nested_system(
            TestRhythm::new(),
            ChildDomain { fail: true },
            controller,
            robot,
        )
        .await
        .unwrap();
        assert!(
            matches!(outcome, Err(RoplatError::NodeProcessing(ref message)) if message == "nested domain failed")
        );
        assert!(child.fail);
        assert_eq!(controller.calls, 1);
        assert_eq!(controller.inits.load(Ordering::Relaxed), 0);
        assert_eq!(controller.shutdowns.load(Ordering::Relaxed), 0);
        assert!(trace.lock().unwrap().commands.is_empty());
        assert_eq!(trace.lock().unwrap().events.last(), Some(&"exit-done"));
    }
}
