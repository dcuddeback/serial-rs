//! Serial port implementation for Windows.

extern crate serial_core as core;
extern crate libc;

pub use self::com::*;

mod com;
mod error;
mod ffi;
