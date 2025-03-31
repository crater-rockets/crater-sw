use core::{
    cell::UnsafeCell,
    fmt::Debug,
    marker::PhantomData,
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
};

use crate::newlib::{self, EBUSY};

#[derive(Debug)]
pub struct PoisonError<T> {
    _phantom: PhantomData<T>,
}

pub enum TryLockError<T> {
    Poisoned(PoisonError<T>),
    WouldBlock,
}

pub type LockResult<T> = Result<T, PoisonError<T>>;
pub type TryLockResult<T> = Result<T, TryLockError<T>>;

pub struct Mutex<T> {
    value: UnsafeCell<T>,
    mutex: UnsafeCell<newlib::pthread_mutex_t>,
}

unsafe impl<T: Send> Send for Mutex<T> {}
unsafe impl<T: Send> Sync for Mutex<T> {}

impl<T> Mutex<T> {
    pub fn new(value: T) -> Self {
        let mutex = unsafe {
            let mut mutex = MaybeUninit::zeroed().assume_init();
            let res = newlib::pthread_mutex_init(&mut mutex, core::ptr::null_mut());

            assert!(res == 0, "Error initializing mutex: {}", res);

            mutex
        };

        Self {
            value: UnsafeCell::new(value),
            mutex: UnsafeCell::new(mutex),
        }
    }

    pub fn lock(&self) -> LockResult<MutexGuard<'_, T>> {
        let res = unsafe { newlib::pthread_mutex_lock(self.mutex.get()) };
        assert!(res == 0, "Error locking mutex: {}", res);

        Ok(MutexGuard {
            lock: &self,
            _not_send: PhantomData,
        })
    }

    pub fn try_lock(&self) -> TryLockResult<MutexGuard<'_, T>> {
        let res = unsafe { newlib::pthread_mutex_trylock(self.mutex.get()) };

        if res == 0 {
            Ok(MutexGuard {
                lock: &self,
                _not_send: PhantomData,
            })
        } else if res as u32 == EBUSY {
            Err(TryLockError::WouldBlock)
        } else {
            panic!("Error locking mutex: {}", res);
        }
    }
}

impl<T> Drop for Mutex<T> {
    fn drop(&mut self) {
        let res = unsafe { newlib::pthread_mutex_destroy(self.mutex.get()) };

        debug_assert!(res == 0, "Error destroying mutex: {}", res);
    }
}

impl<T> From<T> for Mutex<T> {
    fn from(value: T) -> Self {
        Mutex::new(value)
    }
}

impl<T: Default> Default for Mutex<T> {
    fn default() -> Self {
        Mutex::new(T::default())
    }
}

impl<T: Debug> Debug for Mutex<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut d = f.debug_struct("Mutex");
        match self.try_lock() {
            Ok(guard) => {
                d.field("data", &&*guard);
            }
            Err(TryLockError::Poisoned(_)) => {
                d.field("data", &"not_impl");
            }
            Err(TryLockError::WouldBlock) => {
                d.field("data", &format_args!("<locked>"));
            }
        }
        d.finish_non_exhaustive()
    }
}

pub struct MutexGuard<'a, T> {
    lock: &'a Mutex<T>,
    _not_send: PhantomData<*const ()>,
}

unsafe impl<'a, T: Sync> Sync for MutexGuard<'a, T> {}

impl<'a, T> Deref for MutexGuard<'a, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { &*self.lock.value.get() }
    }
}

impl<'a, T> DerefMut for MutexGuard<'a, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { &mut *self.lock.value.get() }
    }
}

impl<'a, T> Drop for MutexGuard<'a, T> {
    fn drop(&mut self) {
        unsafe {
            //TODO: Return value check
            newlib::pthread_mutex_unlock(self.lock.mutex.get());
        }
    }
}

impl<T: Debug> Debug for MutexGuard<'_, T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        Debug::fmt(&**self, f)
    }
}
