//! Serial port implementation for Windows.

extern crate libc;
extern crate serial_core as core;

pub use self::com::*;

mod com;
mod error;
mod ffi;
