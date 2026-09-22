//! Device-paced roplat control, enabled with the `roplat` feature.
//!
//! The robot enters as `RobotResult<R>` and returns as `R` on completion.
//! Framework failures and cooperative stops live in `Execution`, not in Feed:
//! a successful Feed remains `(Command, done)`. The creating layer owns object
//! lifecycle; entering a drive never activates, resets or shuts down its nodes.
//!
//! The device session remains blocking, including when its per-cycle callback
//! is async. This preserves the 0.6 contract; it does not promise fair polling
//! with sibling futures on the outer executor. An in-flight domain is always
//! awaited to recover `N`. Dropped futures and panics are outside that guarantee.
//!
//! A successfully completed Feed is committed for this cycle. A concurrent stop
//! is observed before the next callback; if this Feed already has `done = true`,
//! its final command is sent and normal `Completed(R)` wins. A domain that cannot
//! produce a command must return `Stopped` or `Err` instead of a successful Feed.

use std::{future::Future, marker::PhantomData, ops::ControlFlow, time::Duration};

use roplat::{Completion, Execution, ExecutionContext, Lifecycle, RoplatError, rhythm::Rhythm};

use crate::{ControlSpace, ControlWith, RobotException, RobotResult};

/// Why a domain asked the device session to end without another command.
#[derive(Debug)]
pub enum ControlDomainExit {
    Stopped,
    Failed(RoplatError),
}

/// Device failure, retaining the preceding domain exit if there was one.
///
/// Transported through `RoplatError::Io(std::io::Error)` so the core needs no
/// robot-specific variant. `io_error.get_ref().and_then(|e| e.downcast_ref())`
/// recovers this concrete type, including the original domain error. The I/O
/// category identifies the device/session boundary, not necessarily a socket.
/// No wrapper is allocated when only the domain fails or when control succeeds.
#[derive(Debug, thiserror::Error)]
#[error("control device/session failed: {device}; preceding domain exit: {domain:?}")]
pub struct ControlSessionError {
    pub domain: Option<ControlDomainExit>,
    #[source]
    pub device: RobotException,
}

fn device_error(device: RobotException, domain: Option<ControlDomainExit>) -> RoplatError {
    std::io::Error::other(ControlSessionError { domain, device }).into()
}

/// A blocking device session driving one complete domain per control cycle.
pub struct ControlRhythm<R, S> {
    _types: PhantomData<fn(R) -> S>,
}

impl<R, S> ControlRhythm<R, S> {
    pub fn new() -> Self {
        Self { _types: PhantomData }
    }
}

impl<R, S> Default for ControlRhythm<R, S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<R, S> Lifecycle for ControlRhythm<R, S> {
    type Error = RoplatError;
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
    type Output = R;

    async fn drive<N, F, Fut>(
        &mut self,
        nodes: N,
        mut op_domain: F,
        input: Self::Input,
        context: ExecutionContext,
    ) -> (Execution<Self::Output>, N)
    where
        N: Send,
        F: FnMut(N, Self::Yield, ExecutionContext) -> Fut + Send,
        Fut: Future<Output = (Execution<Self::Feed>, N)> + Send,
    {
        if context.is_stopping() {
            return (Ok(Completion::Stopped), nodes);
        }
        let mut robot = match input {
            Ok(robot) => robot,
            Err(error) => {
                context.request_stop();
                return (Err(device_error(error, None)), nodes);
            }
        };

        let mut nodes = Some(nodes);
        let mut domain_exit = None;
        let result =
            <R as ControlWith<S>>::control_with_flow_async(&mut robot, async |obs, duration| {
                if context.is_stopping() {
                    domain_exit = Some(ControlDomainExit::Stopped);
                    return ControlFlow::Break(());
                }
                let current_nodes = nodes
                    .take()
                    .expect("control driver invoked an overlapping domain callback");
                let (execution, returned_nodes) =
                    op_domain(current_nodes, (obs, duration), context.clone()).await;
                // Restore ownership before inspecting the exit or asking the driver to stop.
                nodes = Some(returned_nodes);
                match execution {
                    Ok(Completion::Completed(feed)) => ControlFlow::Continue(feed),
                    Ok(Completion::Stopped) => {
                        context.request_stop();
                        domain_exit = Some(ControlDomainExit::Stopped);
                        ControlFlow::Break(())
                    }
                    Err(error) => {
                        context.request_stop();
                        domain_exit = Some(ControlDomainExit::Failed(error));
                        ControlFlow::Break(())
                    }
                }
            });
        let nodes = nodes.expect("control domain did not return its node state");
        let execution = match result {
            Err(error) => {
                context.request_stop();
                Err(device_error(error, domain_exit))
            }
            Ok(()) => match domain_exit {
                Some(ControlDomainExit::Failed(error)) => Err(error),
                Some(ControlDomainExit::Stopped) => Ok(Completion::Stopped),
                None => Ok(Completion::Completed(robot)),
            },
        };
        (execution, nodes)
    }
}
