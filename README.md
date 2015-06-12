# Serial

[Documentation](http://dcuddeback.github.io/serial-rs/serial/)

The `serial` crate provides Rust programs with access to serial ports. Serial ports are defined as
traits to support extension through custom implementations. Unix TTY devices and Windows COM ports
are supported out of the box.

## Usage
Add `serial` as a dependency in `Cargo.toml`:

```toml
[dependencies]
serial = "0.1.3"
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
    try!(port.reconfigure(&|settings| {
        try!(settings.set_baud_rate(serial::Baud9600));
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);
        Ok(())
    }));

    try!(port.set_timeout(Duration::milliseconds(1000)));

    let mut buf: Vec<u8> = (0..255).collect();

    try!(port.write(&buf[..]));
    try!(port.read(&mut buf[..]));

    Ok(())
}
```

## Cross-Compiling
Some of the `serial` crate's dependencies use the `gcc` crate in their build scripts, which requires
setting environment variables to ensure they compile properly. Compiling without the environment
variables set causes the following error when linking with the dependencies: `could not read
symbols: File format not recognized`.

To cross-compile correctly, it's necessary to set `CC` or `CC_<target>` to an appropriate
cross-compiler in addition to providing the `--target` option to `cargo build`. The following is an
example of cross-compiling for `arm-unknown-linux-gnueabihf` (Raspberry Pi):

```
CC_arm_unknown_linux_gnueabihf=arm-linux-gnueabihf-gcc cargo build --target=arm-unknown-linux-gnueabihf
```

See the [`gcc` crate's README](https://github.com/alexcrichton/gcc-rs) for details.

## Contributors
* [dcuddeback](https://github.com/dcuddeback)
* [willem66745](https://github.com/willem66745)

## License
Copyright © 2015 David Cuddeback

Distributed under the [MIT License](LICENSE).
