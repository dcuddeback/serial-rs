extern crate libc;
extern crate termios;
extern crate time;

use std::io;

use std::ffi::CString;
use std::path::Path;
use time::Duration;

use std::os::unix::io::{AsRawFd,RawFd};
use std::os::unix::prelude::OsStrExt;

use self::libc::{c_int,c_void,size_t};

use ::prelude::*;


#[cfg(target_os = "linux")]
const O_NOCTTY: c_int = 0x00000100;

#[cfg(target_os = "macos")]
const O_NOCTTY: c_int = 0x00020000;

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
const O_NOCTTY: c_int = 0;


/// A TTY-based serial port implementation.
pub struct TTYPort {
  fd: RawFd,
  timeout: Duration
}

impl TTYPort {
    /// Opens a TTY device as a serial port.
    ///
    /// `path` should be the path to a TTY device, e.g., `/dev/ttyS0`.
    ///
    /// ```no_run
    /// use std::path::Path;
    ///
    /// serial::posix::TTYPort::open(Path::new("/dev/ttyS0")).unwrap();
    /// ```
    pub fn open(path: &Path) -> io::Result<Self> {
        use self::libc::{O_RDWR,O_NONBLOCK};

        let cstr = try!(CString::new(path.as_os_str().as_bytes()));

        let fd = unsafe { libc::open(cstr.as_ptr(), O_RDWR | O_NOCTTY | O_NONBLOCK, 0) };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }

        let mut port = TTYPort {
            fd: fd,
            timeout: Duration::milliseconds(100)
        };

        // apply initial settings
        let settings = try!(port.read_settings());
        try!(port.write_settings(&settings));

        Ok(port)
    }
}

impl Drop for TTYPort {
    fn drop(&mut self) {
        unsafe {
            libc::close(self.fd);
        }
    }
}

impl AsRawFd for TTYPort {
    fn as_raw_fd(&self) -> RawFd {
        self.fd
    }
}

impl io::Read for TTYPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        try!(super::poll::wait_read_fd(self.fd, self.timeout));

        let len = unsafe { libc::read(self.fd, buf.as_ptr() as *mut c_void, buf.len() as size_t) };

        if len >= 0 {
            Ok(len as usize)
        }
        else {
            Err(io::Error::last_os_error())
        }
    }
}

impl io::Write for TTYPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        try!(super::poll::wait_write_fd(self.fd, self.timeout));

        let len = unsafe { libc::write(self.fd, buf.as_ptr() as *mut c_void, buf.len() as size_t) };

        if len >= 0 {
            Ok(len as usize)
        }
        else {
            Err(io::Error::last_os_error())
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        termios::tcdrain(self.fd)
    }
}

impl SerialPort for TTYPort {
    type Settings = TTYSettings;

    fn read_settings(&self) -> ::Result<TTYSettings> {
        use self::termios::{CREAD,CLOCAL}; // cflags
        use self::termios::{ICANON,ECHO,ECHOE,ECHOK,ECHONL,ISIG,IEXTEN}; // lflags
        use self::termios::{OPOST}; // oflags
        use self::termios::{INLCR,IGNCR,ICRNL,IGNBRK}; // iflags
        use self::termios::{VMIN,VTIME}; // c_cc indexes

        let mut termios = try!(termios::Termios::from_fd(self.fd));

        // setup TTY for binary serial port access
        termios.c_cflag |= CREAD | CLOCAL;
        termios.c_lflag &= !(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
        termios.c_oflag &= !OPOST;
        termios.c_iflag &= !(INLCR | IGNCR | ICRNL | IGNBRK);

        termios.c_cc[VMIN] = 0;
        termios.c_cc[VTIME] = 0;

        Ok(TTYSettings::new(termios))
    }

    fn write_settings(&mut self, settings: &TTYSettings) -> ::Result<()> {
        use self::termios::{tcsetattr,tcflush};
        use self::termios::{TCSANOW,TCIOFLUSH};

        // write settings to TTY
        try!(tcsetattr(self.fd, TCSANOW, &settings.termios));
        try!(tcflush(self.fd, TCIOFLUSH));
        Ok(())
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn set_timeout(&mut self, timeout: Duration) -> ::Result<()> {
        self.timeout = timeout;
        Ok(())
    }
}

/// Serial port settings for TTY devices.
#[derive(Copy,Clone)]
pub struct TTYSettings {
    termios: termios::Termios
}

impl TTYSettings {
    fn new(termios: termios::Termios) -> Self {
        TTYSettings {
            termios: termios
        }
    }
}

impl SerialPortSettings for TTYSettings {
    fn baud_rate(&self) -> Option<::BaudRate> {
        use self::termios::{cfgetospeed,cfgetispeed};
        use self::termios::{B50,B75,B110,B134,B150,B200,B300,B600,B1200,B1800,B2400,B4800,B9600,B19200,B38400};
        use self::termios::os::target::{B57600,B115200,B230400};

        let ospeed = cfgetospeed(&self.termios);
        let ispeed = cfgetispeed(&self.termios);

        if ospeed != ispeed {
            return None
        }

        match ospeed {
            B50     => Some(::BaudOther(50)),
            B75     => Some(::BaudOther(75)),
            B110    => Some(::Baud110),
            B134    => Some(::BaudOther(134)),
            B150    => Some(::BaudOther(150)),
            B200    => Some(::BaudOther(200)),
            B300    => Some(::Baud300),
            B600    => Some(::Baud600),
            B1200   => Some(::Baud1200),
            B1800   => Some(::BaudOther(1800)),
            B2400   => Some(::Baud2400),
            B4800   => Some(::Baud4800),
            B9600   => Some(::Baud9600),
            B19200  => Some(::Baud19200),
            B38400  => Some(::Baud38400),
            B57600  => Some(::Baud57600),
            B115200 => Some(::Baud115200),
            B230400 => Some(::BaudOther(230400)),

            _ => None
        }
    }

    fn char_size(&self) -> Option<::CharSize> {
        use self::termios::{CSIZE,CS5,CS6,CS7,CS8};

        match self.termios.c_cflag & CSIZE {
            CS8 => Some(::Bits8),
            CS7 => Some(::Bits7),
            CS6 => Some(::Bits6),
            CS5 => Some(::Bits5),

            _ => None
        }
    }

    fn parity(&self) -> Option<::Parity> {
        use self::termios::{PARENB,PARODD};

        if self.termios.c_cflag & PARENB != 0 {
            if self.termios.c_cflag & PARODD != 0 {
                Some(::ParityOdd)
            }
            else {
                Some(::ParityEven)
            }
        }
        else {
            Some(::ParityNone)
        }
    }

    fn stop_bits(&self) -> Option<::StopBits> {
        use self::termios::{CSTOPB};

        if self.termios.c_cflag & CSTOPB != 0 {
            Some(::Stop2)
        }
        else {
            Some(::Stop1)
        }
    }

    fn flow_control(&self) -> Option<::FlowControl> {
        use self::termios::{IXON,IXOFF};
        use self::termios::os::target::{CRTSCTS};

        if self.termios.c_cflag & CRTSCTS != 0 {
            Some(::FlowHardware)
        }
        else if self.termios.c_iflag & (IXON | IXOFF) != 0 {
            Some(::FlowSoftware)
        }
        else {
            Some(::FlowNone)
        }
    }

    fn set_baud_rate(&mut self, baud_rate: ::BaudRate) -> ::Result<()> {
        use self::termios::cfsetspeed;
        use self::termios::{B50,B75,B110,B134,B150,B200,B300,B600,B1200,B1800,B2400,B4800,B9600,B19200,B38400};
        use self::termios::os::target::{B57600,B115200,B230400};

        let baud = match baud_rate {
            ::BaudOther(50)     => B50,
            ::BaudOther(75)     => B75,
            ::Baud110           => B110,
            ::BaudOther(134)    => B134,
            ::BaudOther(150)    => B150,
            ::BaudOther(200)    => B200,
            ::Baud300           => B300,
            ::Baud600           => B600,
            ::Baud1200          => B1200,
            ::BaudOther(1800)   => B1800,
            ::Baud2400          => B2400,
            ::Baud4800          => B4800,
            ::Baud9600          => B9600,
            ::Baud19200         => B19200,
            ::Baud38400         => B38400,
            ::Baud57600         => B57600,
            ::Baud115200        => B115200,
            ::BaudOther(230400) => B230400,
            ::BaudOther(_)      => return Err(::Error::new(::ErrorKind::InvalidInput, "baud rate is not supported"))
        };

        try!(cfsetspeed(&mut self.termios, baud));
        Ok(())
    }

    fn set_char_size(&mut self, char_size: ::CharSize) {
        use self::termios::{CSIZE,CS5,CS6,CS7,CS8};

        let size = match char_size {
            ::Bits5 => CS5,
            ::Bits6 => CS6,
            ::Bits7 => CS7,
            ::Bits8 => CS8
        };

        self.termios.c_cflag &= !CSIZE;
        self.termios.c_cflag |= size;
    }

    fn set_parity(&mut self, parity: ::Parity) {
        use self::termios::{PARENB,PARODD,INPCK,IGNPAR};

        match parity {
            ::ParityNone => {
                self.termios.c_cflag &= !(PARENB | PARODD);
                self.termios.c_iflag &= !INPCK;
                self.termios.c_iflag |= IGNPAR;
            },
            ::ParityOdd => {
                self.termios.c_cflag |= PARENB | PARODD;
                self.termios.c_iflag |= INPCK;
                self.termios.c_iflag &= !IGNPAR;
            },
            ::ParityEven => {
                self.termios.c_cflag &= !PARODD;
                self.termios.c_cflag |= PARENB;
                self.termios.c_iflag |= INPCK;
                self.termios.c_iflag &= !IGNPAR;
            }
        };
    }

    fn set_stop_bits(&mut self, stop_bits: ::StopBits) {
        use self::termios::{CSTOPB};

        match stop_bits {
            ::Stop1 => self.termios.c_cflag &= !CSTOPB,
            ::Stop2 => self.termios.c_cflag |= CSTOPB
        };
    }

    fn set_flow_control(&mut self, flow_control: ::FlowControl) {
        use self::termios::{IXON,IXOFF};
        use self::termios::os::target::{CRTSCTS};

        match flow_control {
            ::FlowNone => {
                self.termios.c_iflag &= !(IXON | IXOFF);
                self.termios.c_cflag &= !CRTSCTS;
            },
            ::FlowSoftware => {
                self.termios.c_iflag |= IXON | IXOFF;
                self.termios.c_cflag &= !CRTSCTS;
            },
            ::FlowHardware => {
                self.termios.c_iflag &= !(IXON | IXOFF);
                self.termios.c_cflag |= CRTSCTS;
            }
        };
    }
}


#[cfg(test)]
mod tests {
    use std::mem;

    use super::TTYSettings;
    use ::prelude::*;

    fn default_settings() -> TTYSettings {
        TTYSettings {
            termios: unsafe { mem::uninitialized() }
        }
    }

    #[test]
    fn tty_settings_sets_baud_rate() {
        let mut settings = default_settings();

        settings.set_baud_rate(::Baud600).unwrap();
        assert_eq!(settings.baud_rate(), Some(::Baud600));
    }

    #[test]
    fn tty_settings_overwrites_baud_rate() {
        let mut settings = default_settings();

        settings.set_baud_rate(::Baud600).unwrap();
        settings.set_baud_rate(::Baud1200).unwrap();
        assert_eq!(settings.baud_rate(), Some(::Baud1200));
    }

    #[test]
    fn tty_settings_sets_char_size() {
        let mut settings = default_settings();

        settings.set_char_size(::Bits8);
        assert_eq!(settings.char_size(), Some(::Bits8));
    }

    #[test]
    fn tty_settings_overwrites_char_size() {
        let mut settings = default_settings();

        settings.set_char_size(::Bits8);
        settings.set_char_size(::Bits7);
        assert_eq!(settings.char_size(), Some(::Bits7));
    }

    #[test]
    fn tty_settings_sets_parity_even() {
        let mut settings = default_settings();

        settings.set_parity(::ParityEven);
        assert_eq!(settings.parity(), Some(::ParityEven));
    }

    #[test]
    fn tty_settings_sets_parity_odd() {
        let mut settings = default_settings();

        settings.set_parity(::ParityOdd);
        assert_eq!(settings.parity(), Some(::ParityOdd));
    }

    #[test]
    fn tty_settings_sets_parity_none() {
        let mut settings = default_settings();

        settings.set_parity(::ParityEven);
        settings.set_parity(::ParityNone);
        assert_eq!(settings.parity(), Some(::ParityNone));
    }

    #[test]
    fn tty_settings_sets_stop_bits_1() {
        let mut settings = default_settings();

        settings.set_stop_bits(::Stop2);
        settings.set_stop_bits(::Stop1);
        assert_eq!(settings.stop_bits(), Some(::Stop1));
    }

    #[test]
    fn tty_settings_sets_stop_bits_2() {
        let mut settings = default_settings();

        settings.set_stop_bits(::Stop1);
        settings.set_stop_bits(::Stop2);
        assert_eq!(settings.stop_bits(), Some(::Stop2));
    }

    #[test]
    fn tty_settings_sets_flow_control_software() {
        let mut settings = default_settings();

        settings.set_flow_control(::FlowSoftware);
        assert_eq!(settings.flow_control(), Some(::FlowSoftware));
    }

    #[test]
    fn tty_settings_sets_flow_control_hardware() {
        let mut settings = default_settings();

        settings.set_flow_control(::FlowHardware);
        assert_eq!(settings.flow_control(), Some(::FlowHardware));
    }

    #[test]
    fn tty_settings_sets_flow_control_none() {
        let mut settings = default_settings();

        settings.set_flow_control(::FlowHardware);
        settings.set_flow_control(::FlowNone);
        assert_eq!(settings.flow_control(), Some(::FlowNone));
    }
}
