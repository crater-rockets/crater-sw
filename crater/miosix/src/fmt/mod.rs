use alloc::fmt;
use alloc::fmt::Write;

#[cfg(target_arch = "arm")]
pub fn print(args: fmt::Arguments) {
    struct PutcWriter;

    impl fmt::Write for PutcWriter {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            for c in s.chars() {
                unsafe {
                    crate::newlib::putchar(c as core::ffi::c_int);
                }
            }
            Ok(())
        }
    }

    PutcWriter.write_fmt(args).unwrap();
}

#[cfg(target_arch = "arm")]
#[macro_export]
macro_rules! mprint {
    ($($arg:tt)*) => {{
        $crate::fmt::print(::core::format_args!($($arg)*));
    }};
}

#[cfg(target_arch = "arm")]
#[macro_export]
macro_rules! mprintln {
    () => {{
        $crate::fmt::mprint!("\n");
    }};
    ($($arg:tt)*) => {{
        $crate::fmt::mprint!($($arg)*);
        $crate::fmt::mprint!("\n");
    }};
}
