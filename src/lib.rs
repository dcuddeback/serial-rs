pub extern crate serial_core as core;

use std::ffi::OsStr;

#[doc(no_inline)] pub use core::prelude;

#[doc(no_inline)] pub use core::{Result, Error, ErrorKind};
#[doc(no_inline)] pub use core::{PortSettings, BaudRate, CharSize, Parity, StopBits, FlowControl};
#[doc(no_inline)] pub use core::{SerialPort, SerialPortSettings};

pub use core::BaudRate::*;
pub use core::CharSize::*;
pub use core::Parity::*;
pub use core::StopBits::*;
pub use core::FlowControl::*;

#[cfg(unix)]
pub mod posix;

#[cfg(windows)]
pub mod windows;

/// A convenience type alias for the system's native serial port type.
#[cfg(unix)]
pub type SystemPort = posix::TTYPort;

/// A convenience type alias for the system's native serial port type.
#[cfg(windows)]
pub type SystemPort = windows::COMPort;

/// A convenience function for opening a native serial port.
///
/// The argument must be one that's understood by the target operating system to identify a serial
/// port. On Unix systems, it should be a path to a TTY device file. On Windows, it should be the
/// name of a COM port.
///
/// ## Errors
///
/// This function returns an error if the device could not be opened and initialized:
///
/// * `NoDevice` if the device could not be opened. This could indicate that the device is
///   already in use.
/// * `InvalidInput` if `port` is not a valid device name.
/// * `Io` for any other error while opening or initializing the device.
///
/// ## Examples
///
/// Provide a system-specific string that identifies a serial port:
///
/// ```no_run
/// let port = serial::open("/dev/ttyUSB0").unwrap();
/// ```
///
/// Hard-coding the device name dimishes the cross-platform utility of `serial::open()`. To
/// preserve cross-platform functionality, device names should come from external sources:
///
/// ```no_run
/// use std::env;
///
/// for arg in env::args_os().skip(1) {
///     let port = serial::open(&arg).unwrap();
/// }
/// ```
#[cfg(unix)]
pub fn open<T: AsRef<OsStr> + ?Sized>(port: &T) -> ::core::Result<SystemPort> {
    use std::path::Path;
    posix::TTYPort::open(Path::new(port))
}

/// A convenience function for opening a native serial port.
///
/// The argument must be one that's understood by the target operating system to identify a serial
/// port. On Unix systems, it should be a path to a TTY device file. On Windows, it should be the
/// name of a COM port.
///
/// ## Errors
///
/// This function returns an error if the device could not be opened and initialized:
///
/// * `NoDevice` if the device could not be opened. This could indicate that the device is
///   already in use.
/// * `InvalidInput` if `port` is not a valid device name.
/// * `Io` for any other error while opening or initializing the device.
///
/// ## Examples
///
/// Provide a system-specific string that identifies a serial port:
///
/// ```no_run
/// let port = serial::open("COM1").unwrap();
/// ```
///
/// Hard-coding the device name dimishes the cross-platform utility of `serial::open()`. To
/// preserve cross-platform functionality, device names should come from external sources:
///
/// ```no_run
/// use std::env;
///
/// for arg in env::args_os().skip(1) {
///     let port = serial::open(&arg).unwrap();
/// }
/// ```
#[cfg(windows)]
pub fn open<T: AsRef<OsStr> + ?Sized>(port: &T) -> ::core::Result<SystemPort> {
    windows::COMPort::open(port)
}
