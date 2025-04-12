use alloc::fmt;
use alloc::fmt::Write;

pub fn print(args: fmt::Arguments) {
    struct PutcWriter;

    impl fmt::Write for PutcWriter {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            for c in s.chars() {
                unsafe {
                    crate::hal::newlib::putchar(c as core::ffi::c_int);
                }
            }
            Ok(())
        }
    }

    PutcWriter.write_fmt(args).unwrap();
}

#[macro_export]
macro_rules! mprint {
    ($($arg:tt)*) => {{
        $crate::hal::fmt::print(::core::format_args!($($arg)*));
    }};
}

#[macro_export]
macro_rules! mprintln {
    () => {{
        $crate::mprint!("\n");
    }};
    ($($arg:tt)*) => {{
        $crate::mprint!($($arg)*);
        $crate::mprint!("\n");
    }};
}
