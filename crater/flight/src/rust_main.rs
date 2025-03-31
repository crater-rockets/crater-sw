#[cfg(target_arch = "arm")]
#[unsafe(no_mangle)]
pub extern "C" fn rust_main() -> core::ffi::c_int {
    use alloc::{
        boxed::Box,
        string::{String, ToString},
        sync::Arc,
    };
    use miosix::{mprintln, mprint, sync::mutex::Mutex, LinkTest};
    
    let lt: f32 = LinkTest(1.3f32).0;

    mprint!("Hello world {}\n", lt);

    let v_t1 = Arc::new(Mutex::new(1i32));
    let v_t2 = v_t1.clone();

    let t1 = miosix::sync::thread::spawn(move || {
        for _ in 0..1000 {
            let mut val = v_t1.lock().unwrap();
            *val += 1;
            mprint!("Thread 1 {}\n", *val);
        }

        123
    });

    let t2 = miosix::sync::thread::spawn(move || {
        for _ in 0..1000 {
            let mut val = v_t2.lock().unwrap();
            *val += 1;
            mprint!("Thread 2 {}\n", *val);
        }

        "T2 Hello!".to_string()
    });

    mprint!("Joining!\n");
    let v1 = t1.join();
    let v2 = t2.join();

    mprint!("Joined! t1={}, t2={}\n", v1.unwrap(), v2.unwrap());

    0
}