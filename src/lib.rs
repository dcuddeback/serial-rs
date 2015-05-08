extern crate time;

use std::default::Default;
use std::ffi::OsStr;
use std::io;

use time::Duration;

pub use BaudRate::*;
pub use CharSize::*;
pub use Parity::*;
pub use StopBits::*;
pub use FlowControl::*;

/// A module that exports traits that are useful to have in scope.
///
/// It is intended to be glob imported:
///
/// ```no_run
/// use serial::prelude::*;
/// ```
pub mod prelude {
    pub use ::{SerialPort,SerialPortExt,SerialPortSettings};
}

#[cfg(unix)]
pub mod posix;

#[cfg(windows)]
pub mod windows;


/// A convenience function for opening a native serial port.
///
/// The argument must be one that's understood by the target operating system to identify a serial
/// port. On Unix systems, it should be a path to a TTY device file. On Windows, it should be the
/// name of a COM port.
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
pub fn open<T: AsRef<OsStr> + ?Sized>(port: &T) -> io::Result<posix::TTYPort> {
    use std::path::Path;
    posix::TTYPort::open(Path::new(port))
}

/// A convenience function for opening a native serial port.
///
/// The argument must be one that's understood by the target operating system to identify a serial
/// port. On Unix systems, it should be a path to a TTY device file. On Windows, it should be the
/// name of a COM port.
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
pub fn open<T: AsRef<OsStr> + ?Sized>(port: &T) -> io::Result<windows::COMPort> {
    windows::COMPort::open(port)
}


/// Serial port baud rates.
#[derive(Debug,Copy,Clone,PartialEq,Eq)]
pub enum BaudRate {
    /** 50 baud. */      Baud50,
    /** 75 baud. */      Baud75,
    /** 110 baud. */     Baud110,
    /** 134 baud. */     Baud134,
    /** 150 baud. */     Baud150,
    /** 200 baud. */     Baud200,
    /** 300 baud. */     Baud300,
    /** 600 baud. */     Baud600,
    /** 1200 baud. */    Baud1200,
    /** 1800 baud. */    Baud1800,
    /** 2400 baud. */    Baud2400,
    /** 4800 baud. */    Baud4800,
    /** 9600 baud. */    Baud9600,
    /** 19,200 baud. */  Baud19200,
    /** 38,400 baud. */  Baud38400,
    /** 57,600 baud. */  Baud57600,
    /** 115,200 baud. */ Baud115200,
    /** 230,400 baud. */ Baud230400,

    /// Non-standard baud rates.
    ///
    /// `BaudOther` can be used to set arbitrary baud rates by setting its member to be the desired
    /// baud rate.
    ///
    /// ```no_run
    /// serial::BaudOther(4_000_000); // 4,000,000 baud
    /// ```
    ///
    /// Non-standard baud rates may not be supported by all hardware.
    BaudOther(usize)
}

/// Number of bits per character.
#[derive(Debug,Copy,Clone,PartialEq,Eq)]
pub enum CharSize {
    /** 5 bits per character. */ Bits5,
    /** 6 bits per character. */ Bits6,
    /** 7 bits per character. */ Bits7,
    /** 8 bits per character. */ Bits8
}

/// Parity checking modes.
///
/// When parity checking is enabled (`ParityOdd` or `ParityEven`) an extra bit is transmitted with
/// each character. The value of the parity bit is arranged so that the number of 1 bits in the
/// character (including the parity bit) is an even number (`ParityEven`) or an odd number
/// (`ParityOdd`).
///
/// Parity checking is disabled by setting `ParityNone`, in which case parity bits are not
/// transmitted.
#[derive(Debug,Copy,Clone,PartialEq,Eq)]
pub enum Parity {
    /// No parity bit.
    ParityNone,

    /// Parity bit sets odd number of 1 bits.
    ParityOdd,

    /// Parity bit sets even number of 1 bits.
    ParityEven
}

/// Number of stop bits.
///
/// Stop bits are transmitted after every character.
#[derive(Debug,Copy,Clone,PartialEq,Eq)]
pub enum StopBits {
    /// One stop bit.
    Stop1,

    /// Two stop bits.
    Stop2
}

/// Flow control modes.
#[derive(Debug,Copy,Clone,PartialEq,Eq)]
pub enum FlowControl {
    /// No flow control.
    FlowNone,

    /// Flow control using XON/XOFF bytes.
    FlowSoftware,

    /// Flow control using RTS/CTS signals.
    FlowHardware
}

/// A trait for serial port devices.
///
/// A device's serial port settings (baud rate, parity, etc) can be configured through its
/// `Settings` type, which hides implementation details of the serial port's native configuration.
///
/// Serial port input and output is implemented through the `std::io::Read` and `std::io::Write`
/// traits. A timeout can be set with the `set_timeout()` method and applies to all subsequent I/O
/// operations.
pub trait SerialPort: io::Read+io::Write {
    /// A type that implements the settings for the serial port device.
    ///
    /// The `Settings` type is used to retrieve and modify the serial port's settings.
    type Settings: SerialPortSettings;

    /// Returns the device's current settings.
    ///
    /// This function attempts to read the current settings from the hardware. The hardware's
    /// current settings may not match the settings that were most recently written to the hardware
    /// with `apply_settings()`.
    ///
    /// ## Errors
    ///
    /// This function returns an error if the settings could not be read from the underlying
    /// hardware. An error could indicate that the device has been disconnected.
    fn settings(&self) -> io::Result<Self::Settings>;

    /// Applies new settings to the serial device.
    ///
    /// This function attempts to apply all settings to the serial device. Some settings may not be
    /// supported by the underlying hardware, in which case the result is dependent on the
    /// implementation. A successful return value does not guarantee that all settings were
    /// appliied successfully. To check which settings were applied by a successful write,
    /// applications should use the `settings()` method to obtain the latest configuration state
    /// from the device.
    ///
    /// ## Errors
    ///
    /// This function returns an error if the settings could not be applied to the underlying
    /// hardware. An error could indicate that the device has been disconnected or that the device
    /// is not compatible with the given configuration settings.
    fn apply_settings(&mut self, settings: &Self::Settings) -> io::Result<()>;

    /// Returns the current timeout.
    fn timeout(&self) -> Duration;

    /// Sets the timeout for future I/O operations.
    fn set_timeout(&mut self, timeout: Duration);
}

/// An extension trait that provides convenience methods for serial ports.
pub trait SerialPortExt: SerialPort {
    /// Alter the serial port's configuration.
    ///
    /// This method expects a function, which takes a mutable reference to the serial port's
    /// configuration settings. The serial port's current settings, read from the device, are
    /// yielded to the provided function. After the function returns, any changes made to the
    /// settings object will be written back to the device.
    ///
    /// ## Errors
    ///
    /// If this function encounters any kind of I/O error while reading or writing the device's
    /// configuration settings, a `std::io::Error` will be returned.
    ///
    /// ## Example
    ///
    /// The following is a function that toggles a serial port's settings between one and two stop
    /// bits:
    ///
    /// ```no_run
    /// use std::io;
    /// use serial::prelude::*;
    ///
    /// fn toggle_stop_bits<T: SerialPort>(port: &mut T) -> io::Result<()> {
    ///     port.configure(|settings| {
    ///         let stop_bits = match settings.stop_bits() {
    ///             serial::Stop1 => serial::Stop2,
    ///             serial::Stop2 => serial::Stop1
    ///         };
    ///
    ///         settings.set_stop_bits(stop_bits);
    ///     })
    /// }
    /// ```
    fn configure<F: FnOnce(&mut <Self as SerialPort>::Settings) -> ()>(&mut self, setup: F) -> io::Result<()> {
        let mut settings = try!(self.settings());
        setup(&mut settings);
        self.apply_settings(&settings)
    }
}

impl<T> SerialPortExt for T where T: SerialPort { }

/// A trait for objects that implement serial port configurations.
pub trait SerialPortSettings {
    /// Returns the current baud rate.
    fn baud_rate(&self) -> BaudRate;

    /// Returns the character size.
    fn char_size(&self) -> CharSize;

    /// Returns the parity-checking mode.
    fn parity(&self) -> Parity;

    /// Returns the number of stop bits.
    fn stop_bits(&self) -> StopBits;

    /// Returns the flow control mode.
    fn flow_control(&self) -> FlowControl;

    /// Sets the baud rate.
    fn set_baud_rate(&mut self, baud_rate: BaudRate);

    /// Sets the character size.
    fn set_char_size(&mut self, char_size: CharSize);

    /// Sets the parity-checking mode.
    fn set_parity(&mut self, parity: Parity);

    /// Sets the number of stop bits.
    fn set_stop_bits(&mut self, stop_bits: StopBits);

    /// Sets the flow control mode.
    fn set_flow_control(&mut self, flow_control: FlowControl);
}

/// A device-indepenent implementation of serial port settings.
#[derive(Debug,Copy,Clone,PartialEq,Eq)]
pub struct PortSettings {
    /// Baud rate.
    pub baud_rate: BaudRate,

    /// Character size.
    pub char_size: CharSize,

    /// Parity checking mode.
    pub parity: Parity,

    /// Number of stop bits.
    pub stop_bits: StopBits,

    /// Flow control mode.
    pub flow_control: FlowControl
}

impl Default for PortSettings {
    fn default() -> Self {
        PortSettings {
            baud_rate: BaudRate::Baud9600,
            char_size: CharSize::Bits8,
            parity: Parity::ParityNone,
            stop_bits: StopBits::Stop1,
            flow_control: FlowControl::FlowNone
        }
    }
}

impl SerialPortSettings for PortSettings {
    fn baud_rate(&self) -> BaudRate {
        self.baud_rate
    }

    fn char_size(&self) -> CharSize {
        self.char_size
    }

    fn parity(&self) -> Parity {
        self.parity
    }

    fn stop_bits(&self) -> StopBits {
        self.stop_bits
    }

    fn flow_control(&self) -> FlowControl {
        self.flow_control
    }

    fn set_baud_rate(&mut self, baud_rate: BaudRate) {
        self.baud_rate = baud_rate;
    }

    fn set_char_size(&mut self, char_size: CharSize) {
        self.char_size = char_size;
    }

    fn set_parity(&mut self, parity: Parity) {
        self.parity = parity
    }

    fn set_stop_bits(&mut self, stop_bits: StopBits) {
        self.stop_bits = stop_bits;
    }

    fn set_flow_control(&mut self, flow_control: FlowControl) {
        self.flow_control = flow_control;
    }
}

#[cfg(test)]
mod tests {
    use std::default::Default;
    use super::*;

    #[test]
    fn port_settings_manipulates_baud_rate() {
        let mut settings: PortSettings = Default::default();
        settings.set_baud_rate(Baud115200);
        assert_eq!(settings.baud_rate(), Baud115200);
    }

    #[test]
    fn port_settings_manipulates_char_size() {
        let mut settings: PortSettings = Default::default();
        settings.set_char_size(Bits7);
        assert_eq!(settings.char_size(), Bits7);
    }

    #[test]
    fn port_settings_manipulates_parity() {
        let mut settings: PortSettings = Default::default();
        settings.set_parity(ParityEven);
        assert_eq!(settings.parity(), ParityEven);
    }

    #[test]
    fn port_settings_manipulates_stop_bits() {
        let mut settings: PortSettings = Default::default();
        settings.set_stop_bits(Stop2);
        assert_eq!(settings.stop_bits(), Stop2);
    }

    #[test]
    fn port_settings_manipulates_flow_control() {
        let mut settings: PortSettings = Default::default();
        settings.set_flow_control(FlowSoftware);
        assert_eq!(settings.flow_control(), FlowSoftware);
    }
}
