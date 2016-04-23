//! Serial port implementation for Unix operating systems.

extern crate serial_core as core;
extern crate libc;
extern crate termios;
extern crate ioctl_rs as ioctl;

pub use tty::*;

mod error;
mod poll;
mod tty;
