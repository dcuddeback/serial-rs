# Unix Serial Ports

The `serial-unix` crate provides a serial port implementation for Unix operating systems.

* [Documentation](http://dcuddeback.github.io/serial-rs/serial_unix/)

## Compatibility

The `serial-unix` crate is compatible with any Unix operating system that implements the termios
API. The following platforms are confirmed to be compatible:

* Linux (x86_64, armv6l)
* OS X (x86_64)
* FreeBSD (amd64)
* OpenBSD (amd64)

## Usage

In general, one shouldn't need to use the `serial-unix` library directly. The implementation
provided by `serial-unix` is also exposed through a cross-platform API in the [`serial`
crate](../serial/).

The serial port type defined in `serial-unix` works with any Unix TTY device. In addition to
implementing the standard serial port traits, it also implements `std::os::unix::io::AsRawFd`, which
can be used for integrating with other I/O libraries. See [`examples/poll.rs`](examples/poll.rs) for
an example of using `AsRawFd` for event-driven I/O.

## License

Copyright © 2015 David Cuddeback

Distributed under the [MIT License](LICENSE).
