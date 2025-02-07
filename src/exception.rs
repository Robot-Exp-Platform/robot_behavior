use std::io;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum RobotException {
    /// NetworkException is thrown if a connection to the robot cannot be established, or when a timeout occurs.
    #[error("Network exception: {0}")]
    NetworkError(String),

    /// Unprocessable instruction error
    #[error("Unprocessable instruction error: {0}")]
    UnprocessableInstructionError(String),

    /// Conflicting instruction error
    #[error("Conflicting instruction error: {0}")]
    ConflictingInstruction(String),
}

pub type RobotResult<T> = Result<T, RobotException>;

impl From<io::Error> for RobotException {
    fn from(e: io::Error) -> Self {
        RobotException::NetworkError(e.to_string())
    }
}
