//! Serial port implementation for POSIX-compliant operating systems.

pub use self::tty::*;

mod error;
mod poll;
mod tty;
