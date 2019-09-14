//! Serial port implementation for Unix operating systems.

extern crate ioctl_rs as ioctl;
extern crate libc;
extern crate serial_core as core;
extern crate termios;

pub use tty::*;

mod error;
mod poll;
mod tty;
