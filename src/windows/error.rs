extern crate libc;

use std::io;
use std::ptr;

use self::libc::{c_void,c_int};

// missing from libc
const ERROR_PATH_NOT_FOUND: c_int = 3;

pub fn last_os_error() -> ::Error {
    use self::libc::{ERROR_FILE_NOT_FOUND,ERROR_ACCESS_DENIED};

    let errno = errno();

    let kind = match errno {
        ERROR_FILE_NOT_FOUND | ERROR_PATH_NOT_FOUND | ERROR_ACCESS_DENIED => ::ErrorKind::NoDevice,
        _ => ::ErrorKind::Io(io::ErrorKind::Other)
    };

    ::Error::new(kind, error_string(errno).trim())
}

// the rest of this module is borrowed from libstd

fn errno() -> i32 {
    unsafe { libc::GetLastError() as i32 }
}

fn error_string(errnum: i32) -> String {
    #![allow(non_snake_case)]

    use self::libc::types::os::arch::extra::DWORD;
    use self::libc::types::os::arch::extra::LPWSTR;
    use self::libc::types::os::arch::extra::LPVOID;
    use self::libc::types::os::arch::extra::WCHAR;

    #[link_name = "kernel32"]
    extern "system" {
        fn FormatMessageW(flags: DWORD,
                          lpSrc: LPVOID,
                          msgId: DWORD,
                          langId: DWORD,
                          buf: LPWSTR,
                          nsize: DWORD,
                          args: *const c_void)
                          -> DWORD;
    }

    const FORMAT_MESSAGE_FROM_SYSTEM: DWORD = 0x00001000;
    const FORMAT_MESSAGE_IGNORE_INSERTS: DWORD = 0x00000200;

    // This value is calculated from the macro
    // MAKELANGID(LANG_SYSTEM_DEFAULT, SUBLANG_SYS_DEFAULT)
    let langId = 0x0800 as DWORD;

    let mut buf = [0 as WCHAR; 2048];

    unsafe {
        let res = FormatMessageW(FORMAT_MESSAGE_FROM_SYSTEM |
                                 FORMAT_MESSAGE_IGNORE_INSERTS,
                                 ptr::null_mut(),
                                 errnum as DWORD,
                                 langId,
                                 buf.as_mut_ptr(),
                                 buf.len() as DWORD,
                                 ptr::null());
        if res == 0 {
            // Sometimes FormatMessageW can fail e.g. system doesn't like langId,
            let fm_err = errno();
            return format!("OS Error {} (FormatMessageW() returned error {})",
                           errnum, fm_err);
        }

        let b = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
        let msg = String::from_utf16(&buf[..b]);
        match msg {
            Ok(msg) => msg,
            Err(..) => format!("OS Error {} (FormatMessageW() returned \
                                invalid UTF-16)", errnum),
        }
    }
}
