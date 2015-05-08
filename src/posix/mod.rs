//! Serial port implementation for POSIX-compliant operating systems.

pub use self::tty::*;

mod poll;
mod tty;
