# Serial
This crate provides Rust programs with access to serial ports. Serials ports are defined as traits
to support extension through custom serial port implementations. The goals of the `serial` crate are
to support all platforms supported by Rust. Currently, only Unix TTY devices are supported.

## Usage
Add `serial` as a dependency in `Cargo.toml`:

```toml
[dependencies]
serial = "0.0.1"
```

Import the `serial` crate and everything from the `serial::prelude` module. The `serial::prelude`
module contains traits that are useful to have in scope. All the traits in `prelude` begin with
`Serial` to avoid name conflicts with other crates. Having the traits in scope avoids compiler
errors when using serial ports.

For now, you must open a serial port using a system-specific method. Once you've opened a serial
port, you can interact with it using the `SerialPort` and `SerialPortExt` traits. By depending on
the traits, your code will support future implementations of serial ports, including custom
implementations such as those for embedded systems.

```rust
extern crate serial;

// import useful traits
use serial::prelude::*;

fn main() {
  // opening port is system-specific
  let mut port = match serial::posix::TTYPort::open(&Path::new("/dev/ttyUSB0")).unwrap();
  do_something(port).unwrap();
}

// use SerialPort trait to program generically
fn do_something<T: SerialPort>(port: T) -> IoError<()> {
  try!(port.configure(|settings| {
    settings.set_baud_rate(serial::Baud115200);
    settings.set_char_size(serial::Bits8);
    settings.set_parity(serial::ParityNone);
    settings.set_stop_bits(serial::Stop1);
    settings.set_flow_control(serial::FlowNone);
  }));

  // read and write to port using Reader and Writer traits
  try!(port.read(...));
  try!(port.write_all(...));

  Ok(())
}
```

## License
Copyright © 2015 David Cuddeback

Distributed under the [MIT License](LICENSE).
