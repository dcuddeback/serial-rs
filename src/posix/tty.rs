extern crate libc;
extern crate termios;
extern crate time;

use std::io;

use std::default::Default;
use std::ffi::CString;
use std::path::Path;
use time::Duration;

use std::os::unix::prelude::OsStrExt;

use self::libc::{c_int,c_void,size_t};

use ::prelude::*;


#[cfg(target_os = "linux")]
const O_NOCTTY: c_int = 0x00000100;

#[cfg(target_os = "macos")]
const O_NOCTTY: c_int = 0x00020000;

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
const O_NOCTTY: c_int = 0;


pub struct TTYPort {
  fd: c_int,
  timeout: Duration,
  settings: TTYSettings
}

impl TTYPort {
  pub fn open(path: &Path) -> io::Result<Self> {
    use self::libc::{O_RDWR,O_NONBLOCK};

    let cstr = try!(CString::new(path.as_os_str().as_bytes()));

    let fd = unsafe { libc::open(cstr.as_ptr(), O_RDWR | O_NOCTTY | O_NONBLOCK, 0) };
    if fd < 0 {
      return Err(io::Error::last_os_error());
    }

    let mut port = TTYPort {
      fd: fd,
      timeout: Duration::milliseconds(100),
      settings: TTYSettings {
        serial: Default::default()
      }
    };

    // apply initial settings
    let settings = port.settings();
    try!(port.apply_settings(&settings));

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

  fn settings(&self) -> TTYSettings {
    self.settings
  }

  fn apply_settings(&mut self, settings: &TTYSettings) -> io::Result<()> {
    use self::termios::Termios;
    use self::termios::{cfsetspeed,tcsetattr,tcflush};
    use self::termios::{TCSANOW,TCIOFLUSH};

    use self::termios::{CREAD,CLOCAL}; // cflags
    use self::termios::{ICANON,ECHO,ECHOE,ECHOK,ECHONL,ISIG,IEXTEN}; // lflags
    use self::termios::{OPOST}; // oflags
    use self::termios::{INLCR,IGNCR,ICRNL,IGNBRK}; // iflags
    use self::termios::{VMIN,VTIME}; // c_cc indexes
    use self::termios::{CS5,CS6,CS7,CS8}; // character size
    use self::termios::{PARENB,PARODD,INPCK,IGNPAR}; // parity
    use self::termios::{CSTOPB}; // stop bits
    use self::termios::{IXON,IXOFF}; // flow control
    use self::termios::ffi::{CRTSCTS}; // flow control

    let mut termios = try!(Termios::from_fd(self.fd));

    // setup TTY for binary serial port access
    termios.c_cflag |= CREAD | CLOCAL;
    termios.c_lflag &= !(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
    termios.c_oflag &= !OPOST;
    termios.c_iflag &= !(INLCR | IGNCR | ICRNL | IGNBRK);

    termios.c_cc[VMIN] = 0;
    termios.c_cc[VTIME] = 0;

    // set baud rate
    let baud = match settings.baud_rate() {
      ::Baud50       => termios::B50,
      ::Baud75       => termios::B75,
      ::Baud110      => termios::B110,
      ::Baud134      => termios::B134,
      ::Baud150      => termios::B150,
      ::Baud200      => termios::B200,
      ::Baud300      => termios::B300,
      ::Baud600      => termios::B600,
      ::Baud1200     => termios::B1200,
      ::Baud1800     => termios::B1800,
      ::Baud2400     => termios::B2400,
      ::Baud4800     => termios::B4800,
      ::Baud9600     => termios::B9600,
      ::Baud19200    => termios::B19200,
      ::Baud38400    => termios::B38400,
      ::Baud57600    => termios::ffi::B57600,
      ::Baud115200   => termios::ffi::B115200,
      ::Baud230400   => termios::ffi::B230400,
      ::BaudOther(_) => return Err(io::Error::new(io::ErrorKind::InvalidInput, "baud rate is not supported"))
    };
    try!(cfsetspeed(&mut termios, baud));

    // set character size
    let size = match settings.char_size() {
      ::Bits5 => CS5,
      ::Bits6 => CS6,
      ::Bits7 => CS7,
      ::Bits8 => CS8
    };
    termios.c_cflag |= size;

    // set parity
    match settings.parity() {
      ::ParityNone => {
        termios.c_cflag &= !(PARENB | PARODD);
        termios.c_iflag &= !INPCK;
        termios.c_iflag |= IGNPAR;
      },
      ::ParityOdd => {
        termios.c_cflag |= PARENB | PARODD;
        termios.c_iflag |= INPCK;
        termios.c_iflag &= !IGNPAR;
      },
      ::ParityEven => {
        termios.c_cflag &= !PARODD;
        termios.c_cflag |= PARENB;
        termios.c_iflag |= INPCK;
        termios.c_iflag &= !IGNPAR;
      }
    };

    // set stop bits
    match settings.stop_bits() {
      ::Stop1 => termios.c_cflag &= !CSTOPB,
      ::Stop2 => termios.c_cflag |= CSTOPB
    };

    // set flow control
    match settings.flow_control() {
      ::FlowNone => {
        termios.c_iflag &= !(IXON | IXOFF);
        termios.c_cflag &= !CRTSCTS;
      },
      ::FlowSoftware => {
        termios.c_iflag |= IXON | IXOFF;
        termios.c_cflag &= !CRTSCTS;
      },
      ::FlowHardware => {
        termios.c_iflag &= !(IXON | IXOFF);
        termios.c_cflag |= CRTSCTS;
      }
    };

    // write settings to TTY
    try!(tcsetattr(self.fd, TCSANOW, &termios));
    tcflush(self.fd, TCIOFLUSH)
  }

  fn timeout(&self) -> Duration {
    self.timeout
  }

  fn set_timeout(&mut self, timeout: Duration) {
    self.timeout = timeout;
  }
}

#[derive(Copy,Clone,Default)]
pub struct TTYSettings {
  serial: ::PortSettings
}

impl SerialPortSettings for TTYSettings {
  fn baud_rate(&self) -> ::BaudRate {
    self.serial.baud_rate()
  }

  fn char_size(&self) -> ::CharSize {
    self.serial.char_size()
  }

  fn parity(&self) -> ::Parity {
    self.serial.parity()
  }

  fn stop_bits(&self) -> ::StopBits {
    self.serial.stop_bits()
  }

  fn flow_control(&self) -> ::FlowControl {
    self.serial.flow_control()
  }

  fn set_baud_rate(&mut self, baud_rate: ::BaudRate) {
    self.serial.set_baud_rate(baud_rate);
  }

  fn set_char_size(&mut self, char_size: ::CharSize) {
    self.serial.set_char_size(char_size);
  }

  fn set_parity(&mut self, parity: ::Parity) {
    self.serial.set_parity(parity);
  }

  fn set_stop_bits(&mut self, stop_bits: ::StopBits) {
    self.serial.set_stop_bits(stop_bits);
  }

  fn set_flow_control(&mut self, flow_control: ::FlowControl) {
    self.serial.set_flow_control(flow_control);
  }
}
