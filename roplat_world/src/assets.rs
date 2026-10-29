use std::{env, path::PathBuf};

/// The shared asset directory, `<data>/roplat/assets`, where `<data>` is
/// `%LOCALAPPDATA%` on Windows, `~/Library/Application Support` on macOS and
/// `~/.local/share` elsewhere. Relative model paths a robot declares through
/// `roplat_behavior::RobotDescription` are relative to it. `None` when the
/// environment names no home or data directory.
pub fn assets_dir() -> Option<PathBuf> {
    {
        #[cfg(target_os = "windows")]
        {
            env::var_os("LOCALAPPDATA")
                .map(PathBuf::from)
                .or_else(|| env::var_os("USERPROFILE").map(PathBuf::from))
        }

        #[cfg(target_os = "macos")]
        {
            env::var_os("HOME")
                .map(PathBuf::from)
                .map(|home| home.join("Library").join("Application Support"))
        }

        #[cfg(all(not(target_os = "windows"), not(target_os = "macos")))]
        {
            env::var_os("HOME")
                .map(PathBuf::from)
                .map(|home| home.join(".local").join("share"))
        }
    }
    .map(|path| path.join("roplat").join("assets"))
}
