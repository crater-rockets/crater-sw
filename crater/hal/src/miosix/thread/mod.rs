use core::{any::Any, cell::UnsafeCell, ffi::c_void, marker::PhantomData, mem::ManuallyDrop};

use alloc::{boxed::Box, sync::Arc};

use crate::hal::newlib;

pub type Result<T> = core::result::Result<T, Box<dyn Any + Send + Sync + 'static>>;

#[derive(Debug, PartialEq, Eq)]
pub struct ThreadId(newlib::pthread_t);

struct JoinInner<T> {
    result: UnsafeCell<Option<Result<T>>>,
}

pub struct JoinHandle<T> {
    inner: Arc<JoinInner<T>>,
    thread: ThreadId,

    _phantom: PhantomData<T>,
}

impl<T: Send> JoinHandle<T> {
    pub fn join(self) -> Result<T> {
        let md = ManuallyDrop::new(self);

        unsafe {
            let res = newlib::pthread_join(md.thread.0, core::ptr::null_mut());
            assert!(res == 0, "Failed to join thread: {}", res);

            // Thread has joined, we can safely access the result
            md.inner.result.get().read().unwrap()
        }
    }
}

impl<T> Drop for JoinHandle<T> {
    fn drop(&mut self) {
        let res = unsafe { newlib::pthread_detach(self.thread.0) };

        debug_assert_eq!(res, 0);
    }
}

pub fn spawn<F, T>(f: F) -> JoinHandle<T>
where
    F: FnOnce() -> T + Send + 'static,
    T: Send + 'static,
{
    let mut tid: newlib::pthread_t = 0;

    let my_inner = Arc::new(JoinInner::<T> {
        result: UnsafeCell::new(None),
    });

    let their_inner = my_inner.clone();

    let data = Box::new(ThreadData {
        f,
        inner: their_inner,
    });

    let data: *mut ThreadData<F, T> = Box::into_raw(data);

    unsafe {
        newlib::pthread_create(
            &mut tid,
            core::ptr::null(),
            Some(thread_fn::<F, T>),
            data as *mut c_void,
        );
    }

    JoinHandle {
        inner: my_inner,
        thread: ThreadId(tid),
        _phantom: PhantomData,
    }
}

struct ThreadData<F, T> {
    f: F,
    inner: Arc<JoinInner<T>>,
}

extern "C" fn thread_fn<F, T>(data: *mut c_void) -> *mut c_void
where
    F: FnOnce() -> T + Send + 'static,
    T: Send + 'static,
{
    let data = unsafe { Box::from_raw(data as *mut ThreadData<F, T>) };

    let t = (data.f)();

    unsafe {
        *(data.inner.result.get()) = Some(Ok(t));
    }

    core::ptr::null_mut::<c_void>()
}
