extern crate serial;

use std::env;
use std::thread;
use std::time::Duration;

use serial::prelude::*;

const SETTINGS: serial::PortSettings = serial::PortSettings {
    baud_rate:    serial::Baud9600,
    char_size:    serial::Bits8,
    parity:       serial::ParityNone,
    stop_bits:    serial::Stop1,
    flow_control: serial::FlowNone,
};

fn main() {
    for arg in env::args_os().skip(1) {
        let mut port = serial::open(&arg).unwrap();
        println!("opened device {:?}", arg);
        probe_pins(&mut port).unwrap();
    }
}

fn probe_pins<T: SerialPort>(port: &mut T) -> serial::Result<()> {
    try!(port.configure(&SETTINGS));
    try!(port.set_timeout(Some(Duration::from_millis(100))));

    try!(port.set_rts(false));
    try!(port.set_dtr(false));

    let mut rts = false;
    let mut dtr = false;
    let mut toggle = true;

    loop {
        thread::sleep(Duration::from_secs(1));

        if toggle {
            rts = !rts;
            try!(port.set_rts(rts));
        }
        else {
            dtr = !dtr;
            try!(port.set_dtr(dtr));
        }

        println!("RTS={:5?} DTR={:5?} CTS={:5?} DSR={:5?} RI={:5?} CD={:?}",
                 rts,
                 dtr,
                 try!(port.read_cts()),
                 try!(port.read_dsr()),
                 try!(port.read_ri()),
                 try!(port.read_cd()));

        toggle = !toggle;
    }
}
