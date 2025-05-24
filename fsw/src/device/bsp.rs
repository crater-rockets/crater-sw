use embassy_stm32::{
    Config, bind_interrupts,
    gpio::{self, AnyPin, Output, Pin},
    interrupt::typelevel::Interrupt,
    mode::Blocking,
    pac::{EXTI, SYSCFG},
    peripherals,
    spi::{self, Spi},
    time::Hertz,
    usart::{self, BufferedUart, Uart},
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    mutex::Mutex,
};
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
        mode::Blocking,
        spi::Spi,
        usart::{BufferedUartRx, BufferedUartTx},
    };
    use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

    pub type SpiType<SpiMode> = Mutex<ThreadModeRawMutex, Option<Spi<'static, SpiMode>>>;
    pub static SPI_1: SpiType<Blocking> = Mutex::new(None);

    pub static DEBUG_SERIAL_TX: Mutex<ThreadModeRawMutex, Option<BufferedUartTx<'static>>> =
        Mutex::new(None);
    pub static DEBUG_SERIAL_RX: Mutex<ThreadModeRawMutex, Option<BufferedUartRx<'static>>> =
        Mutex::new(None);
}

pub mod pins {
    use embassy_stm32::gpio::Output;
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

    pub static INT_RESPONSE_PIN: Mutex<CriticalSectionRawMutex, Option<Output<'static>>> =
        Mutex::new(None);
}

pub mod channels {
    use crater_gnc::{
        common::Ts,
        components::ada::AdaResult,
        datatypes::{pin::DigitalInputState, sensors::{ImuSensorSample, PressureSensorSample}},
    };
    use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, pubsub::PubSubChannel};

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
    Ts<ImuSensorSample>,
    20,
    1,
    1,
> = PubSubChannel::new();

    pub static SENS_PIN_LIFOTFF: PubSubChannel<ThreadModeRawMutex, Ts<DigitalInputState>, 1, 1, 1> =
        PubSubChannel::new();

    pub static COMP_ADA_RESULT: PubSubChannel<ThreadModeRawMutex, Ts<AdaResult>, 1, 1, 1> =
        PubSubChannel::new();
}

bind_interrupts!(struct Irqs {
    USART3 => usart::BufferedInterruptHandler<peripherals::USART3>;
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

        let exti_pin = p.PG2;
        let pin = exti_pin.pin();
        let port = exti_pin.port();

        critical_section::with(|_| {
            let pin = pin as usize;
            SYSCFG.exticr(pin / 4).modify(|w| w.set_exti(pin % 4, port));

            EXTI.rtsr(0).modify(|w| w.set_line(pin, true));
            EXTI.ftsr(0).modify(|w| w.set_line(pin, false));

            EXTI.pr(0).write(|w| w.set_line(pin, true));

            EXTI.imr(0).modify(|w| w.set_line(pin, true));
        });

        embassy_stm32::interrupt::typelevel::EXTI2::unpend();
        unsafe {
            embassy_stm32::interrupt::typelevel::EXTI2::enable();
        }

        let int_reponse = Output::new(p.PG3, gpio::Level::Low, gpio::Speed::VeryHigh);

        *pins::INT_RESPONSE_PIN.lock().await = Some(int_reponse);

        let usart3_tx_buf = USART_TX_BUF.init([0; 5600]);
        let usart3_rx_buf = USART_RX_BUF.init([0; 5600]);

        let mut usart3_cfg = usart::Config::default();
        usart3_cfg.baudrate = 921600;

        let usart3 = BufferedUart::new(
            p.USART3,
            Irqs,
            p.PD9,
            p.PD8,
            usart3_tx_buf,
            usart3_rx_buf,
            usart3_cfg,
        )
        .unwrap();

        let (tx, rx) = usart3.split();
        *bus::DEBUG_SERIAL_TX.lock().await = Some(tx);
        *bus::DEBUG_SERIAL_RX.lock().await = Some(rx);

        let mut config = spi::Config::default();
        config.rise_fall_speed = gpio::Speed::VeryHigh;
        config.frequency = Hertz(1_000_000);

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

        CraterBsp { sens_bmp390, sens_icm42688}
    }
}
