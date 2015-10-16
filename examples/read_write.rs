extern crate serial;

use std::env;
use std::time::Duration;

use std::io::prelude::*;
use serial::prelude::*;

const SETTINGS: serial::PortSettings = serial::PortSettings {
    baud_rate:    serial::Baud9600,
    char_size:    serial::Bits8,
    parity:       serial::ParityNone,
    stop_bits:    serial::Stop1,
    flow_control: serial::FlowNone
};

fn main() {
    for arg in env::args_os().skip(1) {
        println!("opening port: {:?}", arg);
        let mut port = serial::open(&arg).unwrap();

        interact(&mut port).unwrap();
    }
}

fn interact<T: SerialPort>(port: &mut T) -> serial::Result<()> {
    try!(port.configure(&SETTINGS));
    try!(port.set_timeout(Duration::from_secs(1)));

    let mut buf: Vec<u8> = (0..255).collect();

    println!("writing bytes");
    try!(port.write(&buf[..]));

    println!("reading bytes");
    try!(port.read(&mut buf[..]));

    Ok(())
}
