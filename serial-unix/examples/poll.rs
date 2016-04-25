extern crate serial_unix;
extern crate libc;

use std::io;
use std::path::Path;

use std::io::prelude::*;
use std::os::unix::prelude::*;

fn main() {
    let mut port = serial_unix::TTYPort::open(Path::new("/dev/ttyUSB0")).unwrap();

    let mut fds = vec![libc::pollfd {
        fd: port.as_raw_fd(),
        events: libc::POLLIN,
        revents: 0,
    }];

    loop {
        let retval = unsafe { libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, 100) };

        if retval < 0 {
            panic!("{:?}", io::Error::last_os_error());
        }

        if retval > 0 && fds[0].revents & libc::POLLIN != 0 {
            let mut buffer = Vec::<u8>::new();
            port.read_to_end(&mut buffer).unwrap();

            println!("{:?}", buffer);
        }
    }
}
