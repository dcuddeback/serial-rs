# Serial

The `serial` crate provides Rust programs with access to serial ports. Serial ports are defined as
traits to support extension through custom implementations. Unix TTY devices and Windows COM ports
are supported out of the box.

* [Documentation](http://dcuddeback.github.io/serial-rs/serial/)
* [Change Log](CHANGELOG.md)

## Compatibility
The `serial` crate is compatible with Windows and any Unix operating system that implements the
termios API. The following platforms are confirmed to be compatible:

* Linux (x86_64, armv6l)
* OS X (x86_64)
* FreeBSD (amd64)
* Windows (x86_64)

Compiling the `serial` crate requires Rust 1.3 or later.

## Usage
Add `serial` as a dependency in `Cargo.toml`:

```toml
[dependencies]
serial = "0.2.1"
```

Import the `serial` crate and everything from the `serial::prelude` module. The traits in the
`serial::prelude` module are are useful to have in scope when working with serial ports, and they
are unlikely to conflict with other crates.

To open a serial port, call `serial::open()` with any type that's convertable to `OsStr`.  With an
open serial port, you can interact with it using the `SerialPort` trait. By depending on the traits,
your code will support future implementations of serial ports, including custom implementations.

```rust
extern crate serial;

use std::env;
use std::io;
use std::time::Duration;

use std::io::prelude::*;
use serial::prelude::*;

fn main() {
    for arg in env::args_os().skip(1) {
        let mut port = serial::open(&arg).unwrap();
        interact(&mut port).unwrap();
    }
}

fn interact<T: SerialPort>(port: &mut T) -> io::Result<()> {
    try!(port.reconfigure(&|settings| {
        try!(settings.set_baud_rate(serial::Baud9600));
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);
        Ok(())
    }));

    try!(port.set_timeout(Duration::from_millis(1000)));

    let mut buf: Vec<u8> = (0..255).collect();

    try!(port.write(&buf[..]));
    try!(port.read(&mut buf[..]));

    Ok(())
}
```

### Cross-Compiling
Cross-compiling the `serial` crate requires only that the `--target` option is provided to `cargo
build`. The following is an example of cross-compiling for `arm-unknown-linux-gnueabihf` (Raspberry
Pi):

```
cargo build --target=arm-unknown-linux-gnueabihf
```

## Contributors
* [dcuddeback](https://github.com/dcuddeback)
* [willem66745](https://github.com/willem66745)

## License
Copyright © 2015 David Cuddeback

Distributed under the [MIT License](LICENSE).
