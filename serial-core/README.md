# Serial Core

The `serial-core` crate provides abstract types used to interface with and implement serial ports.
They can be used to write code that functions generically over all serial port types and to
implement new serial port types that work with generic code.

* [Documentation](http://dcuddeback.github.io/serial-rs/serial_core/)


## Usage

Add `serial-core` as a dependency in `Cargo.toml`:

```toml
[dependencies]
serial-core = "0.4"
```

### Interfacing with an Open Serial Port

Import the `serial-core` crate and everything from the `serial_core::prelude` module. The traits in
the `serial_core::prelude` module are useful to have in scope to resolve trait methods when
working with serial ports, and they are unlikely to conflict with other crates.

```rust
extern crate serial_core as serial;

use std::io;
use std::time::Duration;

use std::io::prelude::*;
use serial::prelude::*;
```

#### Configuration and I/O

Interfacing with a serial port is done through the `SerialPort`, `std::io::Read`, and
`std::io::Write` traits, which provide methods for configuration, I/O, and control signals.

```rust
fn probe<P: SerialPort>(port: &mut P) -> io::Result<()> {
    let mut buf: Vec<u8> = (0..255).collect();

    // configuration
    try!(port.reconfigure(&|settings| {
        try!(settings.set_baud_rate(serial::Baud9600));
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);
        Ok(())
    }));

    // I/O
    try!(port.set_timeout(Some(Duration::from_millis(100))));
    try!(port.write(&buf[..]));
    try!(port.read(&mut buf[..]));

    // control signals
    try!(port.set_dtr(true));
    try!(port.read_dsr());

    Ok(())
}
```

For details on using an open serial port, refer to the documentation for the [`SerialPort`
trait](http://dcuddeback.github.io/serial-rs/serial-core/trait.SerialPort.html).

#### Taking Ownership of a Serial Port

Often times, you'll want to implement a higher-level protocol over a serial port, in which case
you'll probably want a handle object that owns the serial port and provides an interface specific to
the higher-level protocol.

To implement a handle for generic serial ports without requiring a proliferation of generics, one
can define the higher-level protocol in a trait:

```rust
struct Handle<P: SerialPort> {
    port: P,
}

impl<P: SerialPort> Handle<P> {
    fn new(port: P) -> Self {
        Handle { port: port }
    }
}

impl<P: SerialPort> Greet for Handle<P> {
    fn get_name(&mut self) -> serial::Result<String> {
        let mut name = String::new();

        try!(self.port.write("What is your name? "));
        try!(self.port.read_to_string(&mut name));

        Ok(name)
    }

    fn say_hello(&mut self, name: &String) -> serial::Result<()> {
        try!(writeln!(&mut self.port, "Hello, {}!", name));
        Ok(())
    }
}

fn greet(greeter: &mut Greet) -> serial::Result<()> {
    let name = try!(greeter.get_name());

    greeter.say_hello(name)
}
```

Note that in the above example, the `greet()` function can interact with the `Handle` struct via the
`Greet` trait without caring what type of `SerialPort` it's talking to.

Alternatively, since `SerialPort` is object-safe, it can be boxed inside of the handle. However,
this approach may introduce an extra pointer indirection:

```rust
struct Handle {
    port: Box<SerialPort>,
}

impl Handle {
    fn new<P: SerialPort>(port: P) -> Self {
        Handle { port: Box::new(port) }
    }

    fn get_name(&mut self) -> serial::Result<String> {
        // ...
    }

    fn say_hello(&mut self, name: &String) -> serial::Result<()> {
        // ...
    }
}
```

### Implementing a Custom Serial Port

The serial port crates are designed to be extensible, allowing third parties to define custom serial
port types. Reasons for implementing a custom serial port type may include bridging a serial port to
a TCP socket or other I/O device, providing a fake implementation for testing, or integrating with
custom hardware.

To define a custom serial port type, start by importing the `serial_core` crate:

```rust
extern crate serial_core as serial;

use std::io;
use std::time::Duration;
```

Next, define a type that will implement the new serial port and optionally a type for its settings:

```rust
struct CustomSerialPort {
    // ...
}

struct CustomSerialPortSettings {
    // ...
}
```

Implement
[`SerialDevice`](http://dcuddeback.github.io/serial-rs/serial-core/trait.SerialDevice.html),
`std::io::Read`, and `std::io::Write` for the new serial port type:

```rust
impl serial::SerialDevice for CustomSerialPort {
    type Settings = CustomSerialPortSettings;

    fn read_settings(&self) -> serial::Result<Self::Settings> { ... }
    fn write_settings(&mut self, settings: &Self::Settings) -> serial::Result<()> { ... }
    fn timeout(&self) -> Option<Duration> { ... }
    fn set_timeout(&mut self, timeout: Option<Duration>) -> serial::Result<()> { ... }

    fn set_rts(&mut self, level: bool) -> serial::Result<()> { ... }
    fn set_dtr(&mut self, level: bool) -> serial::Result<()> { ... }
    fn read_cts(&mut self) -> serial::Result<bool> { ... }
    fn read_dsr(&mut self) -> serial::Result<bool> { ... }
    fn read_ri(&mut self) -> serial::Result<bool> { ... }
    fn read_cd(&mut self) -> serial::Result<bool> { ... }
}

impl io::Read for CustomSerialPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> { ... }
}

impl io::Write for CustomSerialPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> { ... }
    fn flush(&mut self) -> io::Result<()> { ... }
}
```

If a custom settings type is not needed, then the [`PortSettings`
struct](http://dcuddeback.github.io/serial-rs/serial-core/struct.PortSettings.html) can be used for
the `SerialDevice::Settings` associated type. Otherwise, the `Settings` type must implement
[`SerialPortSettings`](http://dcuddeback.github.io/serial-rs/serial-core/trait.SerialPortSettings.html):

```rust
impl serial::SerialPortSettings for CustomSerialPortSettings {
    fn baud_rate(&self) -> Option<BaudRate> { ... }
    fn char_size(&self) -> Option<CharSize> { ... }
    fn parity(&self) -> Option<Parity> { ... }
    fn stop_bits(&self) -> Option<StopBits> { ... }
    fn flow_control(&self) -> Option<FlowControl> { ... }

    fn set_baud_rate(&mut self, baud_rate: BaudRate) -> serial::Result<()> { ... }
    fn set_char_size(&mut self, char_size: CharSize) { ... }
    fn set_parity(&mut self, parity: Parity) { ... }
    fn set_stop_bits(&mut self, stop_bits: StopBits) { ... }
    fn set_flow_control(&mut self, flow_control: FlowControl) { ... }
}
```

For details on implementing a new serial port type, refer to the documentation for the
[`SerialDevice` trait](http://dcuddeback.github.io/serial-rs/serial-core/trait.SerialDevice.html).

## License

Copyright © 2015 David Cuddeback

Distributed under the [MIT License](LICENSE).
