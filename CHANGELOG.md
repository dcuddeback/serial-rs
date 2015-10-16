# Change Log

## 0.3.0 (2015-10-15)
### Added
* Added support for FreeBSD.

### Changed
* Replaced dependence on `time` crate with `std::time`.
* Minimum supported version of Rust is now 1.3.0.

## 0.2.1 (2015-10-03)
### Added
* ([#7](https://github.com/dcuddeback/serial-rs/issues/7))
  Added `SystemPort` type alias for native serial port type.

## 0.2.0 (2015-06-21)
### Added
* Added `SerialDevice` trait as implementation trait for new serial port types. All types that
  implement `SerialDevice` will automatically implement `SerialPort`.
* Added new `ErrorKind` variant: `NoDevice`.

### Changed
* Changed `reconfigure()` method to be object-safe and moved from `SerialPortExt` to `SerialPort`.
* Changed return type of `serial::open()` from `io::Result` to `serial::Result`.
* Improved usefulness of errors returned by many methods by explicitly handling system error codes
  and using system error descriptions.

### Removed
* Removed methods from `SerialPort` trait that required knowledge of the port's `Settings` type.
  Removed methods are `read_settings()` and `write_settings()`. They are still available via
  `SerialDevice`.
* Removed `SerialPortExt` trait.

### Fixed
* ([#4](https://github.com/dcuddeback/serial-rs/issues/4))
  Made `SerialPort` usable as a trait object. `SerialPort` objects can now be boxed as
  `Box<SerialPort>`.

## 0.1.3 (2015-06-06)
### Added
* Added support for control signals to `SerialPort` trait. RTS and DTR control signals are settable.
  CTS, DSR, RI, and CD signals are readable.
* Implemented control signals for Unix TTY devices.
* Implemented control signals for Windows COM ports.

## 0.1.2 (2015-06-02)
### Fixed
* ([#5](https://github.com/dcuddeback/serial-rs/issues/5))
  Fixed bug that kept PTY devices created with `socat` locked after closing device.

## 0.1.1 (2015-05-26)
### Added
* When opening Unix TTY ports, they are now locked for exclusive access with `TIOCEXL` ioctl.

## 0.1.0 (2015-05-25)
### Added
* Added support for stable release channel.
* Added `serial::Error` type to represent errors.
* Added `SerialPort::configure()` method to configure port directly from a `PortSettings` object.
* Implemented `AsRawFd` for Unix TTY ports.
* Implemented `AsRawHandle` for Windows COM ports.

### Changed
* Renamed `SerialPort::settings()` to `SerialPort::read_settings()`.
* Renamed `SerialPort::apply_settings()` to `SerialPort::write_settings()`.
* Changed `io::Result` return types to `serial::Result`.
* Returning `serial::Result` from `SerialPort::set_timeout()` to allow function to perform I/O.
* Changed return type of `SerialPortSettings` getter methods to `Option<T>`, so undetermined
  settings can be represented as `None`.
* Changed return type of `SerialPortSettings::set_baud_rate()` to `serial::Result` to allow the
  method to return an error for unsupported baud rates.

### Removed
* Removed uncommon baud rates: 50, 75, 134, 150, 200, 1800, and 230400 (still available with
  `BaudOther`).

## 0.0.5 (2015-04-17)
### Added
* Added support for Windows COM ports.
* Added cross-platform function `serial::open()` for opening system serial ports.

### Changed
* Return `io::Result` from `SerialPort::settings()`.

## 0.0.4 (2015-04-12)
### Changed
* Replaced use of built-in `libc` crate with `libc` package from crates.io.
* ([#1](https://github.com/dcuddeback/serial-rs/issues/1))
  Replaced use of unstable `std::time` module with `time` package from crates.io.

### Fixed
* Updated to support Rust 1.0.0-beta compiler.

## 0.0.3 (2015-04-03)
### Fixed
* Updated to support latest Rust nightly compiler.

## 0.0.2 (2015-03-29)
### Changed
* Migrated to new `std::io` module form `std::old_io`.

### Fixed
* Updated to support latest Rust nightly compiler.

## 0.0.1 (2015-03-28)
### Added
* Basic support for Unix TTY devices.
