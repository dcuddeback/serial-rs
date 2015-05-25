extern crate serial;
extern crate time;

use std::env;

use time::Duration;

use std::io::prelude::*;
use serial::prelude::*;

fn main() {
    for arg in env::args_os().skip(1) {
        println!("opening port: {:?}", arg);
        let mut port = serial::open(&arg).unwrap();

        interact(&mut port).unwrap();
    }
}

fn interact<T: SerialPort>(port: &mut T) -> serial::Result<()> {
    try!(port.reconfigure(|settings| {
        try!(settings.set_baud_rate(serial::Baud9600));
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);

        Ok(())
    }));

    try!(port.set_timeout(Duration::seconds(1)));

    let mut buf: Vec<u8> = (0..255).collect();

    println!("writing bytes");
    try!(port.write(&buf[..]));

    println!("reading bytes");
    try!(port.read(&mut buf[..]));

    Ok(())
}
