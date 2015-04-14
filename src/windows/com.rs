extern crate libc;
extern crate time;

use std::io;
use std::ptr;

use std::ffi::OsStr;
use std::os::windows::prelude::OsStrExt;

use self::libc::c_void;
use time::Duration;

use super::ffi::*;
use ::prelude::*;


pub struct COMPort {
  handle: Handle,
  timeout: Duration,
  timeout_changed: bool
}

impl COMPort {
    pub fn open<T: AsRef<OsStr> + ?Sized>(port: &T) -> io::Result<Self> {
        let mut name: Vec<u16> = port.as_ref().encode_wide().collect();
        name.push(0);

        let handle = unsafe {
            CreateFileW(name.as_ptr(), GENERIC_READ | GENERIC_WRITE, 0, ptr::null_mut(), OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0 as Handle)
        };

        if handle != INVALID_HANDLE_VALUE {
            Ok(COMPort {
                handle: handle,
                timeout: Duration::milliseconds(100),
                timeout_changed: true
            })
        }
        else {
            Err(io::Error::last_os_error())
        }
    }

    fn apply_timeout(&mut self) -> io::Result<()> {
        let timeout_ms = self.timeout.num_milliseconds() as DWORD;

        let timeouts = COMMTIMEOUTS {
            ReadIntervalTimeout: 0,
            ReadTotalTimeoutMultiplier: 0,
            ReadTotalTimeoutConstant: timeout_ms,
            WriteTotalTimeoutMultiplier: 0,
            WriteTotalTimeoutConstant: 0
        };

        match unsafe { SetCommTimeouts(self.handle, &timeouts) } {
            0 => Err(io::Error::last_os_error()),
            _ => Ok(())
        }
    }
}

impl Drop for COMPort {
    fn drop(&mut self) {
        unsafe {
            CloseHandle(self.handle);
        }
    }
}

impl io::Read for COMPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.timeout_changed {
            try!(self.apply_timeout());
        }

        let mut len: DWORD = 0;

        match unsafe { ReadFile(self.handle, buf.as_mut_ptr() as *mut c_void, buf.len() as DWORD, &mut len, ptr::null_mut()) } {
            0 => Err(io::Error::last_os_error()),
            _ => {
                if len != 0 {
                    Ok(len as usize)
                }
                else {
                    Err(io::Error::new(io::ErrorKind::TimedOut, "operation timed out"))
                }
            }
        }
    }
}

impl io::Write for COMPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut len: DWORD = 0;

        match unsafe { WriteFile(self.handle, buf.as_ptr() as *mut c_void, buf.len() as DWORD, &mut len, ptr::null_mut()) } {
            0 => Err(io::Error::last_os_error()),
            _ => Ok(len as usize)
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        match unsafe { FlushFileBuffers(self.handle) } {
            0 => Err(io::Error::last_os_error()),
            _ => Ok(())
        }
    }
}

impl SerialPort for COMPort {
    type Settings = COMSettings;

    fn settings(&self) -> io::Result<COMSettings> {
        let mut dcb = DCB::new();

        match unsafe { GetCommState(self.handle, &mut dcb) } {
            0 => Err(io::Error::last_os_error()),
            _ => Ok(COMSettings { inner: dcb })

        }
    }

    fn apply_settings(&mut self, settings: &COMSettings) -> io::Result<()> {
        match unsafe { SetCommState(self.handle, &settings.inner) } {
            0 => Err(io::Error::last_os_error()),
            _ => Ok(())
        }
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn set_timeout(&mut self, timeout: Duration) {
        self.timeout = timeout;
    }
}


#[derive(Copy,Clone,Debug)]
pub struct COMSettings {
    inner: DCB
}

impl SerialPortSettings for COMSettings {
    fn baud_rate(&self) -> ::BaudRate {
        match self.inner.BaudRate {
            CBR_110    => ::Baud110,
            CBR_300    => ::Baud300,
            CBR_600    => ::Baud600,
            CBR_1200   => ::Baud1200,
            CBR_2400   => ::Baud2400,
            CBR_4800   => ::Baud4800,
            CBR_9600   => ::Baud9600,
            CBR_14400  => ::BaudOther(14400),
            CBR_19200  => ::Baud19200,
            CBR_38400  => ::Baud38400,
            CBR_56000  => ::BaudOther(56000),
            CBR_57600  => ::Baud57600,
            CBR_115200 => ::Baud115200,
            CBR_128000 => ::BaudOther(128000),
            CBR_256000 => ::BaudOther(256000),
            n          => ::BaudOther(n as usize)
        }
    }

    fn char_size(&self) -> ::CharSize {
        match self.inner.ByteSize {
            5     => ::Bits5,
            6     => ::Bits6,
            7     => ::Bits7,
            8 | _ => ::Bits8
        }
    }

    fn parity(&self) -> ::Parity {
        match self.inner.Parity {
            ODDPARITY    => ::ParityOdd,
            EVENPARITY   => ::ParityEven,
            NOPARITY | _ => ::ParityNone
        }
    }

    fn stop_bits(&self) -> ::StopBits {
        match self.inner.StopBits {
            TWOSTOPBITS | ONE5STOPBITS => ::Stop2,
            ONESTOPBIT  | _            => ::Stop1
        }
    }

    fn flow_control(&self) -> ::FlowControl {
        if self.inner.fBits & (fOutxCtsFlow | fRtsControl) != 0 {
            ::FlowHardware
        }
        else if self.inner.fBits & (fOutX | fInX) != 0 {
            ::FlowSoftware
        }
        else {
            ::FlowNone
        }
    }

    fn set_baud_rate(&mut self, baud_rate: ::BaudRate) {
        self.inner.BaudRate = match baud_rate {
            ::Baud50       => 50,
            ::Baud75       => 75,
            ::Baud110      => CBR_110,
            ::Baud134      => 134,
            ::Baud150      => 150,
            ::Baud200      => 200,
            ::Baud300      => CBR_300,
            ::Baud600      => CBR_600,
            ::Baud1200     => CBR_1200,
            ::Baud1800     => 1800,
            ::Baud2400     => CBR_2400,
            ::Baud4800     => CBR_4800,
            ::Baud9600     => CBR_9600,
            ::Baud19200    => CBR_19200,
            ::Baud38400    => CBR_38400,
            ::Baud57600    => CBR_57600,
            ::Baud115200   => CBR_115200,
            ::Baud230400   => 230400,
            ::BaudOther(n) => n as DWORD
        }
    }

    fn set_char_size(&mut self, char_size: ::CharSize) {
        self.inner.ByteSize = match char_size {
            ::Bits5 => 5,
            ::Bits6 => 6,
            ::Bits7 => 7,
            ::Bits8 => 8
        }
    }

    fn set_parity(&mut self, parity: ::Parity) {
        self.inner.Parity = match parity {
            ::ParityNone => NOPARITY,
            ::ParityOdd  => ODDPARITY,
            ::ParityEven => EVENPARITY
        }
    }

    fn set_stop_bits(&mut self, stop_bits: ::StopBits) {
        self.inner.StopBits = match stop_bits {
            ::Stop1 => ONESTOPBIT,
            ::Stop2 => TWOSTOPBITS
        }
    }

    fn set_flow_control(&mut self, flow_control: ::FlowControl) {
        match flow_control {
            ::FlowNone => {
                self.inner.fBits &= !(fOutxCtsFlow | fRtsControl);
                self.inner.fBits &= !(fOutX | fInX);
            },
            ::FlowSoftware => {
                self.inner.fBits &= !(fOutxCtsFlow | fRtsControl);
                self.inner.fBits |= fOutX | fInX;
            },
            ::FlowHardware => {
                self.inner.fBits |= fOutxCtsFlow | fRtsControl;
                self.inner.fBits &= !(fOutX | fInX);
            }
        }
    }
}
