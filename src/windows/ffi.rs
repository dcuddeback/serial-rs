#![allow(non_upper_case_globals,dead_code)]

extern crate libc;

use std::mem;

pub use self::libc::{c_char,BYTE,WORD,DWORD,BOOL};
pub use self::libc::{GENERIC_READ,GENERIC_WRITE,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,INVALID_HANDLE_VALUE};
pub use self::libc::{CreateFileW,CloseHandle,ReadFile,WriteFile,FlushFileBuffers};


pub type Handle = self::libc::HANDLE;

#[allow(non_snake_case)]
#[derive(Copy,Clone,Debug)]
#[repr(C)]
pub struct DCB {
    pub DCBlength:  DWORD,
    pub BaudRate:   DWORD,
    pub fBits:      DWORD,
    pub wReserved:  WORD,
    pub XonLim:     WORD,
    pub XoffLim:    WORD,
    pub ByteSize:   BYTE,
    pub Parity:     BYTE,
    pub StopBits:   BYTE,
    pub XonChar:    c_char,
    pub XoffChar:   c_char,
    pub ErrorChar:  c_char,
    pub EofChar:    c_char,
    pub EvtChar:    c_char,
    pub wReserved1: WORD
}

// BaudRate values
pub const CBR_110:    DWORD = 110;
pub const CBR_300:    DWORD = 300;
pub const CBR_600:    DWORD = 600;
pub const CBR_1200:   DWORD = 1200;
pub const CBR_2400:   DWORD = 2400;
pub const CBR_4800:   DWORD = 4800;
pub const CBR_9600:   DWORD = 9600;
pub const CBR_14400:  DWORD = 14400;
pub const CBR_19200:  DWORD = 19200;
pub const CBR_38400:  DWORD = 38400;
pub const CBR_56000:  DWORD = 56000;
pub const CBR_57600:  DWORD = 57600;
pub const CBR_115200: DWORD = 115200;
pub const CBR_128000: DWORD = 128000;
pub const CBR_256000: DWORD = 256000;

// fBits masks
pub const fBinary:           DWORD = 0x00000001;
pub const fParity:           DWORD = 0x00000002;
pub const fOutxCtsFlow:      DWORD = 0x00000004;
pub const fOutxDsrFlow:      DWORD = 0x00000008;
pub const fDtrControl:       DWORD = 0x00000030;
pub const fDsrSensitivity:   DWORD = 0x00000040;
pub const fTXContinueOnXoff: DWORD = 0x00000080;
pub const fOutX:             DWORD = 0x00000100;
pub const fInX:              DWORD = 0x00000200;
pub const fErrorChar:        DWORD = 0x00000400;
pub const fNull:             DWORD = 0x00000800;
pub const fRtsControl:       DWORD = 0x00003000;
pub const fAbortOnError:     DWORD = 0x00004000;
pub const fDummy2:           DWORD = 0xFFFF8000;

// Parity values
pub const NOPARITY:    BYTE = 0;
pub const ODDPARITY:   BYTE = 1;
pub const EVENPARITY:  BYTE = 2;
pub const MARKPARITY:  BYTE = 3;
pub const SPACEPARITY: BYTE = 4;

// StopBits values
pub const ONESTOPBIT:   BYTE = 0;
pub const ONE5STOPBITS: BYTE = 1;
pub const TWOSTOPBITS:  BYTE = 2;

impl DCB {
    pub fn new() -> Self {
        let mut dcb: DCB = unsafe { mem::zeroed() };
        dcb.DCBlength = mem::size_of_val(&dcb) as DWORD;
        dcb
    }
}


#[allow(non_snake_case)]
#[derive(Copy,Clone,Debug)]
#[repr(C)]
pub struct COMMTIMEOUTS {
    pub ReadIntervalTimeout: DWORD,
    pub ReadTotalTimeoutMultiplier: DWORD,
    pub ReadTotalTimeoutConstant: DWORD,
    pub WriteTotalTimeoutMultiplier: DWORD,
    pub WriteTotalTimeoutConstant: DWORD
}

extern "system" {
    pub fn GetCommState(hFile: Handle, lpDCB: *mut DCB) -> BOOL;
    pub fn SetCommState(hFile: Handle, lpDCB: *const DCB) -> BOOL;
    pub fn GetCommTimeouts(hFile: Handle, lpCommTimeouts: *mut COMMTIMEOUTS) -> BOOL;
    pub fn SetCommTimeouts(hFile: Handle, lpCommTimeouts: *const COMMTIMEOUTS) -> BOOL;
}
