#![no_std]
#![no_main]

use core::{fmt::Write, marker::PhantomData};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{self, AnyPin, Output},
    interrupt::typelevel::{Handler, Interrupt},
    pac::EXTI,
    peripherals, spi,
    time::Hertz,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    signal::Signal,
};
use crater_fsw::{device::{
    bsp::{CraterBsp, INT_RESPONSE_PIN, SPI_1},
    spi::{SpiDevice, SpiTransaction},
}, sensors::bmp390::{self, Bmp390}};
use embassy_time::{Instant, Timer};
use heapless::String;
use static_cell::StaticCell;
use uom::si::{pressure::pascal, thermodynamic_temperature::degree_celsius};
use {defmt_rtt as _, panic_probe as _};
// use {defmt_serial as _, panic_probe as _};

struct TimestampInterruptHandler<I> {
    _phantom: PhantomData<I>,
}

impl<I: Interrupt> Handler<I> for TimestampInterruptHandler<I> {
    unsafe fn on_interrupt() {
        if let Ok(mut pin) = INT_RESPONSE_PIN.try_lock() {
            if let Some(pin) = pin.as_mut() {
                pin.set_high();
            }
        }
        SOME_SIGNAL.signal(Instant::now().as_micros());
        EXTI.pr(0).write(|w| w.set_line(2, true));
    }
}

static SOME_SIGNAL: Signal<CriticalSectionRawMutex, u64> = Signal::new();

bind_interrupts!(
    struct Irqs {
        EXTI2 => TimestampInterruptHandler<embassy_stm32::interrupt::typelevel::EXTI2>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let bsp = CraterBsp::init().await;

    let dev_bmp390 = SpiDevice::new(&SPI_1, bsp.bmp390.cs);

    let mut bmp390 = Bmp390::init(
        dev_bmp390,
        bmp390::Config {
            odr: bmp390::Reg::DataRateValue::Odr50,
            osr_p: bmp390::Reg::OversamplingValue::Times4,
            osr_t: bmp390::Reg::OversamplingValue::None,
        },
    )
    .await
    .expect("Could not init bmp390!");

    _spawner.spawn(timestamp_printer()).unwrap();

    Timer::after_millis(20).await;

    loop {
        let sample = bmp390.sample().await;
        info!(
            "{}    \t{}",
            sample.press_pa.get::<pascal>(),
            sample.temp_degc.get::<degree_celsius>()
        );

        Timer::after_millis(200).await;
    }
}

#[embassy_executor::task]
async fn timestamp_printer() {
    loop {
        let ts = SOME_SIGNAL.wait().await;
        SOME_SIGNAL.reset();
        if let Some(pin) = INT_RESPONSE_PIN.lock().await.as_mut() {
            pin.set_low();
        }
        info!("Timestamp: {}", ts);
    }
}
