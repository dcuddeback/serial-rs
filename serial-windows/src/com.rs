use core;
use error;

use std::ffi::OsStr;
use std::io;
use std::mem;
use std::ptr;
use std::time::Duration;

use std::os::windows::prelude::*;

use core::{SerialDevice, SerialPortSettings};

use libc::c_void;
use ffi::*;

/// A serial port implementation for Windows COM ports.
///
/// The port will be closed when the value is dropped.
pub struct COMPort {
    handle: HANDLE,
    timeout: Duration,
}

unsafe impl Send for COMPort {}

impl COMPort {
    /// Opens a COM port as a serial device.
    ///
    /// `port` should be the name of a COM port, e.g., `COM1`.
    ///
    /// ```no_run
    /// serial_windows::COMPort::open("COM1").unwrap();
    /// ```
    ///
    /// ## Errors
    ///
    /// * `NoDevice` if the device could not be opened. This could indicate that the device is
    ///   already in use.
    /// * `InvalidInput` if `port` is not a valid device name.
    /// * `Io` for any other I/O error while opening or initializing the device.
    pub fn open<T: AsRef<OsStr> + ?Sized>(port: &T) -> core::Result<Self> {
        let mut name = Vec::<u16>::new();

        name.extend(OsStr::new("\\\\.\\").encode_wide());
        name.extend(port.as_ref().encode_wide());
        name.push(0);

        let handle = unsafe {
            CreateFileW(name.as_ptr(), GENERIC_READ | GENERIC_WRITE, 0, ptr::null_mut(), OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0 as HANDLE)
        };

        let timeout = Duration::from_millis(100);

        if handle != INVALID_HANDLE_VALUE {
            let mut port = COMPort {
                handle: handle,
                timeout: timeout,
            };

            try!(port.set_timeout(timeout));
            Ok(port)
        }
        else {
            Err(error::last_os_error())
        }
    }

    fn escape_comm_function(&mut self, function: DWORD) -> core::Result<()> {
        match unsafe { EscapeCommFunction(self.handle, function) } {
            0 => Err(error::last_os_error()),
            _ => Ok(()),
        }
    }

    fn read_pin(&mut self, pin: DWORD) -> core::Result<bool> {
        let mut status: DWORD = unsafe { mem::uninitialized() };

        match unsafe { GetCommModemStatus(self.handle, &mut status) } {
            0 => Err(error::last_os_error()),
            _ => Ok(status & pin != 0),
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

impl AsRawHandle for COMPort {
    fn as_raw_handle(&self) -> RawHandle {
        unsafe {
            mem::transmute(self.handle)
        }
    }
}

impl io::Read for COMPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut len: DWORD = 0;

        match unsafe { ReadFile(self.handle, buf.as_mut_ptr() as *mut c_void, buf.len() as DWORD, &mut len, ptr::null_mut()) } {
            0 => Err(io::Error::last_os_error()),
            _ => {
                if len != 0 {
                    Ok(len as usize)
                }
                else {
                    Err(io::Error::new(io::ErrorKind::TimedOut, "Operation timed out"))
                }
            }
        }
    }

    fn read_to_end(&mut self, buf: &mut Vec<u8>) -> io::Result<usize> {
        let mut len: DWORD = 0;

        // Saefty: Vec allocate on heap u8*vec.capacity()!
        match unsafe { ReadFile(self.handle, buf.as_mut_ptr() as *mut c_void, buf.capacity() as DWORD, &mut len, ptr::null_mut()) } {
            0 => Err(io::Error::last_os_error()),
            _ => {
                if len != 0 {
                    // Safety: We asume ReadFile will never return more bytes as given by nNumberOfBytesToRead!
                    unsafe {
                        buf.set_len(len as usize);
                    }
                    Ok(len as usize)
                }
                else {
                    Err(io::Error::new(io::ErrorKind::TimedOut, "Operation timed out"))
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
            _ => Ok(len as usize),
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        match unsafe { FlushFileBuffers(self.handle) } {
            0 => Err(io::Error::last_os_error()),
            _ => Ok(()),
        }
    }
}

impl SerialDevice for COMPort {
    type Settings = COMSettings;

    fn read_settings(&self) -> core::Result<COMSettings> {
        let mut dcb = DCB::new();

        match unsafe { GetCommState(self.handle, &mut dcb) } {
            0 => Err(error::last_os_error()),
            _ => {
                dcb.fBits |= fBinary;
                dcb.fBits &= fDtrControl;

                Ok(COMSettings { inner: dcb })
            }

        }
    }

    fn write_settings(&mut self, settings: &COMSettings) -> core::Result<()> {
        match unsafe { SetCommState(self.handle, &settings.inner) } {
            0 => Err(error::last_os_error()),
            _ => Ok(()),
        }
    }

    fn timeout(&self) -> Duration {
        self.timeout
    }

    fn set_timeout(&mut self, timeout: Duration) -> core::Result<()> {
        let milliseconds = timeout.as_secs() * 1000 + timeout.subsec_nanos() as u64 / 1_000_000;

        let timeouts = COMMTIMEOUTS {
            ReadIntervalTimeout: 0,
            ReadTotalTimeoutMultiplier: 0,
            ReadTotalTimeoutConstant: milliseconds as DWORD,
            WriteTotalTimeoutMultiplier: 0,
            WriteTotalTimeoutConstant: 0,
        };

        if unsafe { SetCommTimeouts(self.handle, &timeouts) } == 0 {
            return Err(error::last_os_error());
        }

        self.timeout = timeout;
        Ok(())
    }

    fn set_rts(&mut self, level: bool) -> core::Result<()> {
        if level {
            self.escape_comm_function(SETRTS)
        }
        else {
            self.escape_comm_function(CLRRTS)
        }
    }

    fn set_dtr(&mut self, level: bool) -> core::Result<()> {
        if level {
            self.escape_comm_function(SETDTR)
        }
        else {
            self.escape_comm_function(CLRDTR)
        }
    }

    fn read_cts(&mut self) -> core::Result<bool> {
        self.read_pin(MS_CTS_ON)
    }

    fn read_dsr(&mut self) -> core::Result<bool> {
        self.read_pin(MS_DSR_ON)
    }

    fn read_ri(&mut self) -> core::Result<bool> {
        self.read_pin(MS_RING_ON)
    }

    fn read_cd(&mut self) -> core::Result<bool> {
        self.read_pin(MS_RLSD_ON)
    }
}


/// Serial port settings for COM ports.
#[derive(Copy,Clone,Debug)]
pub struct COMSettings {
    inner: DCB,
}

impl SerialPortSettings for COMSettings {
    fn baud_rate(&self) -> Option<core::BaudRate> {
        match self.inner.BaudRate {
            CBR_110    => Some(core::Baud110),
            CBR_300    => Some(core::Baud300),
            CBR_600    => Some(core::Baud600),
            CBR_1200   => Some(core::Baud1200),
            CBR_2400   => Some(core::Baud2400),
            CBR_4800   => Some(core::Baud4800),
            CBR_9600   => Some(core::Baud9600),
            CBR_14400  => Some(core::BaudOther(14400)),
            CBR_19200  => Some(core::Baud19200),
            CBR_38400  => Some(core::Baud38400),
            CBR_56000  => Some(core::BaudOther(56000)),
            CBR_57600  => Some(core::Baud57600),
            CBR_115200 => Some(core::Baud115200),
            CBR_128000 => Some(core::BaudOther(128000)),
            CBR_256000 => Some(core::BaudOther(256000)),
            n          => Some(core::BaudOther(n as usize)),
        }
    }

    fn char_size(&self) -> Option<core::CharSize> {
        match self.inner.ByteSize {
            5 => Some(core::Bits5),
            6 => Some(core::Bits6),
            7 => Some(core::Bits7),
            8 => Some(core::Bits8),
            _ => None,
        }
    }

    fn parity(&self) -> Option<core::Parity> {
        match self.inner.Parity {
            ODDPARITY  => Some(core::ParityOdd),
            EVENPARITY => Some(core::ParityEven),
            NOPARITY   => Some(core::ParityNone),
            _          => None,
        }
    }

    fn stop_bits(&self) -> Option<core::StopBits> {
        match self.inner.StopBits {
            TWOSTOPBITS => Some(core::Stop2),
            ONESTOPBIT  => Some(core::Stop1),
            _           => None,
        }
    }

    fn flow_control(&self) -> Option<core::FlowControl> {
        if self.inner.fBits & (fOutxCtsFlow | fRtsControl) != 0 {
            Some(core::FlowHardware)
        }
        else if self.inner.fBits & (fOutX | fInX) != 0 {
            Some(core::FlowSoftware)
        }
        else {
            Some(core::FlowNone)
        }
    }

    fn set_baud_rate(&mut self, baud_rate: core::BaudRate) -> core::Result<()> {
        self.inner.BaudRate = match baud_rate {
            core::Baud110      => CBR_110,
            core::Baud300      => CBR_300,
            core::Baud600      => CBR_600,
            core::Baud1200     => CBR_1200,
            core::Baud2400     => CBR_2400,
            core::Baud4800     => CBR_4800,
            core::Baud9600     => CBR_9600,
            core::Baud19200    => CBR_19200,
            core::Baud38400    => CBR_38400,
            core::Baud57600    => CBR_57600,
            core::Baud115200   => CBR_115200,
            core::BaudOther(n) => n as DWORD,
        };

        Ok(())
    }

    fn set_char_size(&mut self, char_size: core::CharSize) {
        self.inner.ByteSize = match char_size {
            core::Bits5 => 5,
            core::Bits6 => 6,
            core::Bits7 => 7,
            core::Bits8 => 8,
        };
    }

    fn set_parity(&mut self, parity: core::Parity) {
        self.inner.Parity = match parity {
            core::ParityNone => NOPARITY,
            core::ParityOdd  => ODDPARITY,
            core::ParityEven => EVENPARITY,
        };

        if parity == core::ParityNone {
            self.inner.fBits &= !fParity;
        }
        else {
            self.inner.fBits |= fParity;
        }
    }

    fn set_stop_bits(&mut self, stop_bits: core::StopBits) {
        self.inner.StopBits = match stop_bits {
            core::Stop1 => ONESTOPBIT,
            core::Stop2 => TWOSTOPBITS,
        };
    }

    fn set_flow_control(&mut self, flow_control: core::FlowControl) {
        match flow_control {
            core::FlowNone => {
                self.inner.fBits &= !(fOutxCtsFlow | fRtsControl);
                self.inner.fBits &= !(fOutX | fInX);
            }
            core::FlowSoftware => {
                self.inner.fBits &= !(fOutxCtsFlow | fRtsControl);
                self.inner.fBits |= fOutX | fInX;
            }
            core::FlowHardware => {
                self.inner.fBits |= fOutxCtsFlow | fRtsControl;
                self.inner.fBits &= !(fOutX | fInX);
            }
        }
    }
}
