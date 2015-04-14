extern crate time;

use std::io;

use std::default::Default;
use time::Duration;

pub use BaudRate::*;
pub use CharSize::*;
pub use Parity::*;
pub use StopBits::*;
pub use FlowControl::*;

pub mod prelude {
  pub use ::{SerialPort,SerialPortExt,SerialPortSettings};
}

#[cfg(unix)]
pub mod posix;

#[cfg(windows)]
pub mod windows;


#[derive(Copy,Clone)]
pub enum BaudRate {
  Baud50,
  Baud75,
  Baud110,
  Baud134,
  Baud150,
  Baud200,
  Baud300,
  Baud600,
  Baud1200,
  Baud1800,
  Baud2400,
  Baud4800,
  Baud9600,
  Baud19200,
  Baud38400,
  Baud57600,
  Baud115200,
  Baud230400,
  BaudOther(usize)
}

#[derive(Copy,Clone)]
pub enum CharSize {
  Bits5,
  Bits6,
  Bits7,
  Bits8
}

#[derive(Copy,Clone)]
pub enum Parity {
  ParityNone,
  ParityOdd,
  ParityEven
}

#[derive(Copy,Clone)]
pub enum StopBits {
  Stop1,
  Stop2
}

#[derive(Copy,Clone)]
pub enum FlowControl {
  FlowNone,
  FlowSoftware,
  FlowHardware
}

pub trait SerialPort: io::Read+io::Write {
  type Settings: SerialPortSettings;

  fn settings(&self) -> io::Result<Self::Settings>;
  fn apply_settings(&mut self, settings: &Self::Settings) -> io::Result<()>;

  fn timeout(&self) -> Duration;
  fn set_timeout(&mut self, timeout: Duration);
}

pub trait SerialPortExt: SerialPort {
  fn configure<F: FnOnce(&mut <Self as SerialPort>::Settings) -> ()>(&mut self, setup: F) -> io::Result<()> {
    let mut settings = try!(self.settings());
    setup(&mut settings);
    self.apply_settings(&settings)
  }
}

impl<T> SerialPortExt for T where T: SerialPort { }


pub trait SerialPortSettings {
  fn baud_rate(&self) -> BaudRate;
  fn char_size(&self) -> CharSize;
  fn parity(&self) -> Parity;
  fn stop_bits(&self) -> StopBits;
  fn flow_control(&self) -> FlowControl;

  fn set_baud_rate(&mut self, baud_rate: BaudRate);
  fn set_char_size(&mut self, char_size: CharSize);
  fn set_parity(&mut self, parity: Parity);
  fn set_stop_bits(&mut self, stop_bits: StopBits);
  fn set_flow_control(&mut self, flow_control: FlowControl);
}

#[derive(Copy,Clone)]
pub struct PortSettings {
  pub baud_rate: BaudRate,
  pub char_size: CharSize,
  pub parity: Parity,
  pub stop_bits: StopBits,
  pub flow_control: FlowControl
}

impl Default for PortSettings {
  fn default() -> Self {
    PortSettings {
      baud_rate: BaudRate::Baud9600,
      char_size: CharSize::Bits8,
      parity: Parity::ParityNone,
      stop_bits: StopBits::Stop1,
      flow_control: FlowControl::FlowNone
    }
  }
}

impl SerialPortSettings for PortSettings {
  fn baud_rate(&self) -> BaudRate {
    self.baud_rate
  }

  fn char_size(&self) -> CharSize {
    self.char_size
  }

  fn parity(&self) -> Parity {
    self.parity
  }

  fn stop_bits(&self) -> StopBits {
    self.stop_bits
  }

  fn flow_control(&self) -> FlowControl {
    self.flow_control
  }

  fn set_baud_rate(&mut self, baud_rate: BaudRate) {
    self.baud_rate = baud_rate;
  }

  fn set_char_size(&mut self, char_size: CharSize) {
    self.char_size = char_size;
  }

  fn set_parity(&mut self, parity: Parity) {
    self.parity = parity
  }

  fn set_stop_bits(&mut self, stop_bits: StopBits) {
    self.stop_bits = stop_bits;
  }

  fn set_flow_control(&mut self, flow_control: FlowControl) {
    self.flow_control = flow_control;
  }
}
