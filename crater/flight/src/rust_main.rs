#[unsafe(no_mangle)]
pub extern "C" fn rust_main() -> core::ffi::c_int {
    use alloc::{string::ToString, sync::Arc};
    use crater_hal::{
        hal::{sync::Mutex, thread},
        mprintln,
    };

    let lt: f32 = 1.23f32;

    mprintln!("Hello world {}", lt);

    let v_t1 = Arc::new(Mutex::new(1i32));
    let v_t2 = v_t1.clone();

    let t1 = thread::spawn(move || {
        for _ in 0..1000 {
            let mut val = v_t1.lock().unwrap();
            *val += 1;
            mprintln!("Thread 1 {}", *val);
        }

        123
    });

    let t2 = thread::spawn(move || {
        for _ in 0..1000 {
            let mut val = v_t2.lock().unwrap();
            *val += 1;
            mprintln!("Thread 2 {}", *val);
        }

        "T2 Hello!".to_string()
    });

    let v1 = t1.join();
    let v2 = t2.join();

    mprintln!("Joined! t1={}, t2={}", v1.unwrap(), v2.unwrap());
    0
}
