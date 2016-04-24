extern crate serial;

#[cfg(unix)]
fn main() {
    use std::path::Path;

    serial::open("/dev/ttyUSB0").unwrap();
    serial::open(Path::new("/dev/ttyUSB0")).unwrap();
    serial::unix::TTYPort::open(Path::new("/dev/ttyUSB0")).unwrap();
}

#[cfg(windows)]
fn main() {
    serial::open("COM1").unwrap();
    serial::windows::COMPort::open("COM1").unwrap();
}
