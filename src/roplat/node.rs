use std::marker::PhantomData;

use crate::{MotionSpace, MoveTo, RobotResult, SpaceMap};

/// Node adapter for a one-shot [`MoveTo<S>`] command.
///
/// The robot flows through the node as a resource value. A previous motion
/// failure short-circuits the node, while a successful input robot is used for
/// one blocking motion and then returned.
pub struct MotionNode<R, S> {
    _types: PhantomData<fn(R) -> S>,
}

impl<R, S> MotionNode<R, S> {
    pub fn new() -> Self {
        Self { _types: PhantomData }
    }
}

impl<R, S> Default for MotionNode<R, S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<R, S> ::roplat::Node for MotionNode<R, S>
where
    R: MoveTo<S> + Send,
    S: MotionSpace<R> + Send,
    S::Target: Send,
{
    type Input = (RobotResult<R>, S::Target);
    type Output = RobotResult<R>;
    type Error = ::roplat::RoplatError;

    async fn process(&mut self, input: Self::Input) -> Self::Output {
        let (robot, target) = input;
        let mut robot = robot?;
        robot.move_to(target)?;
        Ok(robot)
    }
}

/// Node adapter for a typed [`SpaceMap<From, To>`].
pub struct SpaceMapNode<M, From, To> {
    model: M,
    _space: PhantomData<(From, To)>,
}

impl<M, From, To> SpaceMapNode<M, From, To> {
    pub fn new(model: M) -> Self {
        Self { model, _space: PhantomData }
    }

    pub fn into_inner(self) -> M {
        self.model
    }
}

impl<M, From, To> ::roplat::Node for SpaceMapNode<M, From, To>
where
    M: SpaceMap<From, To> + Send + Sync,
    From: Send + Sync,
    To: Send + Sync,
    M::Input: Send,
    M::Output: Send,
{
    type Input = M::Input;
    type Output = RobotResult<M::Output>;
    type Error = ::roplat::RoplatError;

    async fn process(&mut self, input: Self::Input) -> Self::Output {
        self.model.map(input)
    }
}

/// Generic command-frame transform node.
///
/// This is intentionally minimal: specialized limit nodes can be layered later
/// once concrete downstream safety policies repeat.
pub struct SafetyNode<F, Command> {
    filter: F,
    _command: PhantomData<fn(Command) -> Command>,
}

impl<F, Command> SafetyNode<F, Command> {
    pub fn new(filter: F) -> Self {
        Self { filter, _command: PhantomData }
    }

    pub fn into_inner(self) -> F {
        self.filter
    }
}

impl<F, Command> ::roplat::Node for SafetyNode<F, Command>
where
    F: FnMut((Command, bool)) -> (Command, bool) + Send + Sync,
    Command: Send,
{
    type Input = (Command, bool);
    type Output = (Command, bool);
    type Error = ::roplat::RoplatError;

    async fn process(&mut self, input: Self::Input) -> Self::Output {
        (self.filter)(input)
    }
}
