use std::{any::type_name, io};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum RobotException {
    #[error("none")]
    NoException,

    /// ModelException is thrown if an error occurs when loading the model library.
    #[error("Model exception: {0}")]
    ModelException(String),

    /// NetworkException is thrown if a connection to the robot cannot be established, or when a timeout occurs.
    #[error("Network exception: {0}")]
    NetworkError(String),

    /// IncompatibleVersionException is thrown if the robot does not support this version.
    #[error(
        "Incompatible version: server version {server_version}, client version {client_version}"
    )]
    IncompatibleVersionException {
        server_version: u64,
        client_version: u64,
    },

    /// RealtimeException is thrown if realtime priority cannot be set.
    #[error("Realtime exception: {0}")]
    RealtimeException(String),

    /// Unprocessable instruction error
    #[error("Unprocessable instruction error: {0}")]
    UnprocessableInstructionError(String),

    /// Conflicting instruction error
    #[error("Conflicting instruction error: {0}")]
    ConflictingInstruction(String),

    /// CommandException is thrown if an error occurs during command execution.
    #[error("Command exception: {0}")]
    CommandException(String),

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

#[cfg(feature = "to_py")]
use pyo3::exceptions::PyException;
#[cfg(feature = "to_py")]
impl From<RobotException> for pyo3::PyErr {
    fn from(e: RobotException) -> Self {
        PyException::new_err(e.to_string())
    }
}
