use core::marker::PhantomData;

use embassy_stm32::{
    Config, bind_interrupts,
    gpio::{self, AnyPin, Input, Output, Pin},
    interrupt::typelevel::{Handler, Interrupt},
    mode::Blocking,
    pac::{EXTI, SYSCFG},
    peripherals,
    spi::{self, Spi},
    time::Hertz,
    usart::{self, BufferedUart, Uart, UartTx},
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    mutex::Mutex,
};
use embassy_time::Instant;
use static_cell::StaticCell;

use crate::HEAP;

pub struct BspSensBmp390 {
    pub cs: Output<'static>,
}

pub struct BspSensIcm42688 {
    pub cs: Output<'static>,
}
pub struct CraterBsp {
    pub sens_bmp390: BspSensBmp390,
    pub sens_icm42688: BspSensIcm42688,
}

pub mod bus {
    use embassy_stm32::{
        mode::{Async, Blocking},
        spi::Spi,
        usart::{BufferedUartRx, BufferedUartTx, UartTx},
    };
    use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

    pub type SpiType<SpiMode> = Mutex<ThreadModeRawMutex, Option<Spi<'static, SpiMode>>>;
    pub static SPI_1: SpiType<Blocking> = Mutex::new(None);

    // pub static DEBUG_SERIAL_TX: Mutex<ThreadModeRawMutex, Option<BufferedUartTx<'static>>> =
    //     Mutex::new(None);
    pub static DEBUG_SERIAL_TX: Mutex<ThreadModeRawMutex, Option<UartTx<'static, Async>>> =
        Mutex::new(None);
    pub static DEBUG_SERIAL_RX: Mutex<ThreadModeRawMutex, Option<BufferedUartRx<'static>>> =
        Mutex::new(None);
}

pub mod interrupts {
    use embassy_stm32::gpio::{AnyPin, Input};
    use embassy_sync::{
        blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal,
    };
    use embassy_time::Instant;

    pub static EXTI_PIN_ICM_42688_DRDY: Mutex<CriticalSectionRawMutex, Option<Input<'static>>> =
        Mutex::new(None);
    pub static SIGNAL_ICM_42688_DRDY: Signal<CriticalSectionRawMutex, (Instant, u8)> =
        Signal::new();
}

pub mod channels {
    use crater_gnc::{
        common::Ts,
        components::ada::AdaResult,
        datatypes::{
            pin::DigitalInputState,
            sensors::{ImuSensorSample, PressureSensorSample},
        },
    };
    use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, pubsub::PubSubChannel};

    use crate::sensors::icm42688::Icm42688Sample;

    pub static EVENTS: PubSubChannel<ThreadModeRawMutex, crater_gnc::events::EventItem, 50, 1, 1> =
        PubSubChannel::new();

    pub static SENS_BMP_390_SAMPLE: PubSubChannel<
        ThreadModeRawMutex,
        Ts<PressureSensorSample>,
        5,
        1,
        1,
    > = PubSubChannel::new();

    pub static SENS_ICM_42688_SAMPLE: PubSubChannel<
        ThreadModeRawMutex,
        Ts<Icm42688Sample>,
        20,
        1,
        1,
    > = PubSubChannel::new();

    pub static SENS_PIN_LIFOTFF: PubSubChannel<ThreadModeRawMutex, Ts<DigitalInputState>, 1, 1, 1> =
        PubSubChannel::new();

    pub static COMP_ADA_RESULT: PubSubChannel<ThreadModeRawMutex, Ts<AdaResult>, 1, 1, 1> =
        PubSubChannel::new();
}

struct Icm42688InterruptHandler<I> {
    _phantom: PhantomData<I>,
}

impl<I: Interrupt> Handler<I> for Icm42688InterruptHandler<I> {
    unsafe fn on_interrupt() {
        let ts = Instant::now();

        let overrun_count = if let Some((_, count)) = interrupts::SIGNAL_ICM_42688_DRDY.try_take() {
            count + 1
        } else {
            0
        };

        interrupts::SIGNAL_ICM_42688_DRDY.signal((ts, overrun_count));

        // Clear pending flag
        EXTI.pr(0).write(|w| w.set_line(2, true));
    }
}

bind_interrupts!(struct Irqs {
    // USART3 => usart::BufferedInterruptHandler<peripherals::USART3>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    EXTI2 =>  Icm42688InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI2>;
});

static USART_TX_BUF: StaticCell<[u8; 5600]> = StaticCell::new();
static USART_RX_BUF: StaticCell<[u8; 5600]> = StaticCell::new();

impl CraterBsp {
    pub async fn init() -> CraterBsp {
        // Initalize the heap
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 1024 * 50;
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
        }

        let p = embassy_stm32::init(Default::default());

        let pin_icm_42688_drdy = p.PB2.degrade();
        enable_exti_interrupt(&pin_icm_42688_drdy);
        let input = Input::new(pin_icm_42688_drdy, gpio::Pull::Up);

        *interrupts::EXTI_PIN_ICM_42688_DRDY.lock().await = Some(input);

        let usart3_tx_buf = USART_TX_BUF.init([0; 5600]);
        let usart3_rx_buf = USART_RX_BUF.init([0; 5600]);

        let mut usart3_cfg = usart::Config::default();
        usart3_cfg.baudrate = 921600;

        // let usart3 = BufferedUart::new(
        //     p.USART3,
        //     Irqs,
        //     p.PD9,
        //     p.PD8,
        //     usart3_tx_buf,
        //     usart3_rx_buf,
        //     usart3_cfg,
        // )
        // .unwrap();

        // let (tx, rx) = usart3.split();
        // *bus::DEBUG_SERIAL_TX.lock().await = Some(tx);
        // *bus::DEBUG_SERIAL_RX.lock().await = Some(rx);

        let usart3_tx = UartTx::new(p.USART3, p.PD8, p.DMA1_CH4, usart3_cfg).unwrap();
        *bus::DEBUG_SERIAL_TX.lock().await = Some(usart3_tx);

        let mut config = spi::Config::default();
        config.rise_fall_speed = gpio::Speed::Medium;
        config.frequency = Hertz(10_000_000);

        *bus::SPI_1.try_lock().unwrap() =
            Some(spi::Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, config));

        let sens_bmp390 = BspSensBmp390 {
            cs: Output::new(
                AnyPin::from(p.PF12),
                gpio::Level::High,
                gpio::Speed::VeryHigh,
            ),
        };

        let sens_icm42688 = BspSensIcm42688 {
            cs: Output::new(
                #[cfg(feature = "nucleo_stm32f756")]
                AnyPin::from(p.PG9),
                #[cfg(feature = "crater_stm32f767")]
                AnyPin::from(p.PF11),
                gpio::Level::High,
                gpio::Speed::VeryHigh,
            ),
        };

        CraterBsp {
            sens_bmp390,
            sens_icm42688,
        }
    }
}

fn enable_exti_interrupt(exti_pin: &AnyPin) {
    let pin = exti_pin.pin();
    let port = exti_pin.port();

    critical_section::with(|_| {
        let pin = pin as usize;
        SYSCFG.exticr(pin / 4).modify(|w| w.set_exti(pin % 4, port));

        EXTI.rtsr(0).modify(|w| w.set_line(pin, false));
        EXTI.ftsr(0).modify(|w| w.set_line(pin, true));

        EXTI.pr(0).write(|w| w.set_line(pin, true));

        EXTI.imr(0).modify(|w| w.set_line(pin, true));
    });

    match pin {
        0 => {
            embassy_stm32::interrupt::typelevel::EXTI0::unpend();
            unsafe {
                embassy_stm32::interrupt::typelevel::EXTI0::enable();
            }
        }
        1 => {
            embassy_stm32::interrupt::typelevel::EXTI1::unpend();
            unsafe {
                embassy_stm32::interrupt::typelevel::EXTI1::enable();
            }
        }
        2 => {
            embassy_stm32::interrupt::typelevel::EXTI2::unpend();
            unsafe {
                embassy_stm32::interrupt::typelevel::EXTI2::enable();
            }
        }
        3 => {
            embassy_stm32::interrupt::typelevel::EXTI3::unpend();
            unsafe {
                embassy_stm32::interrupt::typelevel::EXTI3::enable();
            }
        }
        4 => {
            embassy_stm32::interrupt::typelevel::EXTI4::unpend();
            unsafe {
                embassy_stm32::interrupt::typelevel::EXTI4::enable();
            }
        }
        5..10 => {
            embassy_stm32::interrupt::typelevel::EXTI9_5::unpend();
            unsafe {
                embassy_stm32::interrupt::typelevel::EXTI9_5::enable();
            }
        }
        10..16 => {
            embassy_stm32::interrupt::typelevel::EXTI15_10::unpend();
            unsafe {
                embassy_stm32::interrupt::typelevel::EXTI15_10::enable();
            }
        }
        _ => {
            panic!(
                "Unsupported interrupt pin {} por {}",
                exti_pin.pin(),
                exti_pin.port()
            )
        }
    }
}
