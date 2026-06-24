//! roplat rhythms for robot control.
//!
//! The generic [`ControlRhythm`] adapts
//! [`ControlWith::control_with_async`](crate::ControlWith::control_with_async)
//! into roplat's async [`Rhythm`](roplat::rhythm::Rhythm) interface. It treats
//! the robot as a linear resource:
//!
//! - `Input = RobotResult<R>` receives ownership of the robot resource.
//! - The rhythm runs `robot.control_with::<S, _>(...)` until the controller
//!   reports `done` or the driver errors.
//! - `Output = RobotResult<R>` returns the robot resource to the graph.
//!
//! `ControlRhythm` calls the blocking
//! [`ControlWith::control_with_async`](crate::ControlWith::control_with_async).
//! The method accepts an async per-cycle closure but returns only after the
//! robot control loop finishes.

use std::{future::Future, marker::PhantomData, time::Duration};

use roplat::{RoplatError, rhythm::Rhythm};

use crate::{ControlSpace, ControlWith, RobotResult};

/// Generic control rhythm backed by [`ControlWith::control_with_async`].
pub struct ControlRhythm<R, S> {
    _types: PhantomData<fn(R) -> S>,
}

impl<R, S> ControlRhythm<R, S> {
    /// Create a control rhythm for robot type `R` and control space `S`.
    pub fn new() -> Self {
        Self { _types: PhantomData }
    }
}

impl<R, S> Default for ControlRhythm<R, S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<R, S> Rhythm for ControlRhythm<R, S>
where
    R: ControlWith<S> + Send,
    S: ControlSpace<R> + Send,
    S::Obs: Send,
    S::Command: Send,
{
    type Yield = (S::Obs, Duration);
    type Feed = (S::Command, bool);
    type Input = RobotResult<R>;
    type Output = RobotResult<R>;
    type Error = RoplatError;

    async fn drive<N, F, Fut>(
        &mut self,
        nodes: N,
        mut op_domain: F,
        input: Self::Input,
    ) -> (Self::Output, N)
    where
        N: Send,
        F: FnMut(N, Self::Yield) -> Fut + Send,
        Fut: Future<Output = (Self::Feed, N)> + Send,
    {
        let mut robot = match input {
            Ok(robot) => robot,
            Err(error) => return (Err(error), nodes),
        };

        let mut nodes = Some(nodes);
        let result =
            <R as ControlWith<S>>::control_with_async(&mut robot, async |obs, duration| {
                let current_nodes = nodes
                    .take()
                    .expect("control rhythm operation domain lost its node state");
                let (feed, returned_nodes) = op_domain(current_nodes, (obs, duration)).await;
                nodes = Some(returned_nodes);
                feed
            });

        let nodes = nodes.expect("control rhythm operation domain did not return node state");

        match result {
            Ok(()) => (Ok(robot), nodes),
            Err(error) => (Err(error), nodes),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::ControlRhythm;
    use crate::{ControlSpace, ControlWith, Robot, RobotResult};
    use roplat::rhythm::Rhythm;
    use std::time::Duration;

    struct TestControl;

    #[derive(Debug, PartialEq)]
    struct TestRobot {
        cycles: u32,
    }

    impl Robot for TestRobot {
        type State = ();

        const CONTROL_PERIOD: f64 = 0.001;

        fn version() -> String {
            "test".to_string()
        }

        fn read_state(&mut self) -> RobotResult<Self::State> {
            Ok(())
        }
    }

    impl ControlSpace<TestRobot> for TestControl {
        type Obs = u32;
        type Command = u32;
    }

    impl ControlWith<TestControl> for TestRobot {
        fn hold_command(obs: &u32) -> u32 {
            *obs
        }

        fn control_with<F>(&mut self, mut closure: F) -> RobotResult<()>
        where
            F: FnMut(u32, Duration) -> (u32, bool),
        {
            loop {
                let (_command, done) = closure(self.cycles, Duration::from_millis(1));
                self.cycles += 1;
                if done {
                    return Ok(());
                }
            }
        }

        fn control_with_async<F>(&mut self, mut closure: F) -> RobotResult<()>
        where
            F: async FnMut(u32, Duration) -> (u32, bool),
        {
            loop {
                let (_command, done) =
                    futures::executor::block_on(closure(self.cycles, Duration::from_millis(1)));
                self.cycles += 1;
                if done {
                    return Ok(());
                }
            }
        }
    }

    #[tokio::test(flavor = "current_thread")]
    async fn control_rhythm_returns_robot_resource_after_done() {
        let mut rhythm = ControlRhythm::<TestRobot, TestControl>::new();
        let future = rhythm.drive(
            10_u32,
            |nodes, (obs, _duration)| async move {
                let done = obs >= 2;
                ((obs + nodes, done), nodes + 1)
            },
            Ok(TestRobot { cycles: 0 }),
        );

        let (robot, nodes) = future.await;

        assert_eq!(robot.unwrap(), TestRobot { cycles: 3 });
        assert_eq!(nodes, 13);
    }
}
