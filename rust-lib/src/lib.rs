#![no_std]
use core::{convert::Infallible, panic::PanicInfo};

use arrayvec::ArrayString;
use ufmt::uWrite;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

extern "C" {
    pub fn ffi_usb_serial_write(buffer: &u8, size: u32);
}
fn usb_serial_write(buffer: &[u8]) {
    unsafe {
        let size = buffer.len() as u32;
        let buffer = &buffer[0];
        ffi_usb_serial_write(buffer, size);
    }
}

const LINE_SIZE: usize = 128;
struct WritableLine {
    s: ArrayString<LINE_SIZE>,
}

impl WritableLine {
    pub fn new() -> Self {
        Self {
            s: ArrayString::new(),
        }
    }
}

impl uWrite for WritableLine {
    type Error = Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.s.push_str(s);
        Ok(())
    }
}

macro_rules! uprintln {
    // IMPORTANT use `tt` fragments instead of `expr` fragments (i.e. `$($exprs:expr),*`)
    ($($tt:tt)*) => {{
        let mut line = WritableLine::new();
        ufmt::uwriteln!(&mut line, $($tt)*).ok();
        usb_serial_write(line.s.as_str().as_bytes());
        usb_serial_write(b"\r");
    }}
}

#[no_mangle]
pub extern "C" fn print_tick(tick: i32) {
    uprintln!("tick: {}", tick);
}
