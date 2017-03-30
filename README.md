# Serial

The serial port crates provide Rust interfaces for working with serial ports. Serial ports are
defined as traits to support extension through custom implementations.

**NOTE**: With the release of `serial` v0.4, the implementation is now split into several crates.
While the new organization is still experimental, the `serial` crate reexports many of the types so
that it's mostly backward-compatible with v0.3.

* [Change Log](CHANGELOG.md)

## Usage
### In Libraries
Libraries should link to [`serial-core`](serial-core/).

### In Executables
Executables should choose a serial port implementation.
A cross platform implementation is provided in the [`serial`](serial/) crate.

## Contributors
* [dcuddeback](https://github.com/dcuddeback)
* [willem66745](https://github.com/willem66745)
* [apoloval](https://github.com/apoloval)

## License
Copyright © 2015 David Cuddeback

Distributed under the [MIT License](LICENSE).
