use std::{any::type_name, io};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum RobotException {
    #[error("none")]
    NoException,

    /// NetworkException is thrown if a connection to the robot cannot be established, or when a timeout occurs.
    #[error("Network exception: {0}")]
    NetworkError(String),

    /// Unprocessable instruction error
    #[error("Unprocessable instruction error: {0}")]
    UnprocessableInstructionError(String),

    /// Conflicting instruction error
    #[error("Conflicting instruction error: {0}")]
    ConflictingInstruction(String),

    /// Invalid instruction error
    #[error("Invalid instruction error: {0}")]
    InvalidInstruction(String),

    /// Deserialize error
    #[error("Deserialize error: {0}")]
    DeserializeError(String),

    /// unwarp error
    #[error("UnWarp error: {0}")]
    UnWarpError(String),
}

pub type RobotResult<T> = Result<T, RobotException>;

impl From<io::Error> for RobotException {
    fn from(e: io::Error) -> Self {
        RobotException::NetworkError(e.to_string())
    }
}

pub fn deserialize_error<T, E>(data: &str) -> impl FnOnce(E) -> RobotException {
    move |_| {
        RobotException::DeserializeError(format!("exception {}, find {}", type_name::<T>(), data))
    }
}
