//! A stop shared by the control domains of one process.

use std::sync::{Arc, Mutex, PoisonError};

use crate::SafetyViolation;

/// A latched stop: once tripped it stays tripped, across sessions, until
/// cleared. Clones share one latch.
///
/// It propagates a software stop to every domain that checks it, at that
/// domain's next cycle. It is not an emergency stop: it neither reaches the
/// device between cycles nor replaces the device's own stop circuit.
#[derive(Clone, Debug, Default)]
pub struct SafetyLatch {
    reason: Arc<Mutex<Option<String>>>,
}

impl SafetyLatch {
    pub fn new() -> Self {
        Self::default()
    }

    /// Trips the latch. The first reason is kept; returns whether this call
    /// tripped it.
    pub fn trip(&self, reason: impl Into<String>) -> bool {
        let mut current = self.reason.lock().unwrap_or_else(PoisonError::into_inner);
        if current.is_some() {
            return false;
        }
        *current = Some(reason.into());
        true
    }

    /// The reason it was tripped, if it is.
    pub fn tripped(&self) -> Option<String> {
        self.reason
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    pub fn check(&self) -> Result<(), SafetyViolation> {
        match self.tripped() {
            Some(reason) => Err(SafetyViolation::Tripped { reason }),
            None => Ok(()),
        }
    }

    /// Releases the latch; only an operator decision should call this.
    /// Returns the reason it had been tripped for.
    pub fn clear(&self) -> Option<String> {
        self.reason
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take()
    }
}
