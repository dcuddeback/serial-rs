# Serial
The `serial` crate provides Rust programs with access to serial ports. Serial ports are defined as
traits to support extension through custom implementations. Unix TTY devices and Windows COM ports
are supported out of the box.

## Usage
Add `serial` as a dependency in `Cargo.toml`:

```toml
[dependencies]
serial = "0.0.4"
```

Import the `serial` crate and everything from the `serial::prelude` module. The traits in the
`serial::prelude` module are are useful to have in scope when working with serial ports, and they
are unlikely to conflict with other crates.

To open a serial port, call `serial::open()` with any type that's convertable to `OsStr`.  With an
open serial port, you can interact with it using the `SerialPort` and `SerialPortExt` traits. By
depending on the traits, your code will support future implementations of serial ports, including
custom implementations.

```rust
extern crate serial;
extern crate time;

use std::env;
use std::io;

use time::Duration;

use std::io::prelude::*;
use serial::prelude::*;

fn main() {
    for arg in env::args_os().skip(1) {
        let mut port = serial::open(&arg).unwrap();
        interact(&mut port).unwrap();
    }
}

fn interact<T: SerialPort>(port: &mut T) -> io::Result<()> {
    try!(port.configure(|settings| {
        settings.set_baud_rate(serial::Baud9600);
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);
    }));

    port.set_timeout(Duration::milliseconds(1000));

    let mut buf: Vec<u8> = (0..255).collect();

    try!(port.write(&buf[..]));
    try!(port.read(&mut buf[..]));

    Ok(())
}
```

## Contributors
* [dcuddeback](https://github.com/dcuddeback)
* [willem66745](https://github.com/willem66745)

## License
Copyright © 2015 David Cuddeback

Distributed under the [MIT License](LICENSE).
