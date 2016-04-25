# Windows Serial Ports

The `serial-windows` crate provides a serial port implementation for the Windows operating system.

* [Documentation](http://dcuddeback.github.io/serial-rs/serial_windows/)

## Compatibility

The `serial-windows` crate is compatible with recent versions of the Windows operating system. The
following versions are confirmed to be compatible:

* Windows 7 (x86_64)
* Windows 8 (x86_64)

## Usage

In general, one shouldn't need to use the `serial-windows` library directly. The implementation
provided by `serial-windows` is also exposed through a cross-platform API in the [`serial`
crate](../serial/).

The serial port type defined in `serial-windows` works with Windows COM ports. In addition to
implementing the standard serial port traits, it also implements
`std::os::windows::io::AsRawHandle`, which can be used for integrating with other I/O libraries.

## License

Copyright © 2015 David Cuddeback

Distributed under the [MIT License](LICENSE).
