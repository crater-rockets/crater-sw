use embassy_stm32::{
    Config,
    gpio::{self, AnyPin, Output, Pin},
    interrupt::typelevel::Interrupt,
    mode::Blocking,
    pac::{EXTI, SYSCFG},
    spi::{self, Spi},
    time::Hertz,
    usart::{self, Uart},
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    mutex::Mutex,
};
use static_cell::StaticCell;

use crate::HEAP;

pub struct Bmp390 {
    pub cs: Output<'static>,
}

pub struct CraterBsp {
    pub bmp390: Bmp390,
}

pub type SpiType<SpiMode> = Mutex<ThreadModeRawMutex, Option<Spi<'static, SpiMode>>>;
pub static SPI_1: SpiType<Blocking> = Mutex::new(None);

// pub type UartType<UartMode> = Mutex<ThreadModeRawMutex, Option<Uart<'static, UartMode>>>;
// pub static USART_3: UartType<Async> = Mutex::new(None);
static SERIAL: StaticCell<Uart<'static, Blocking>> = StaticCell::new();
pub static INT_RESPONSE_PIN: Mutex<CriticalSectionRawMutex, Option<Output<'static>>> =
    Mutex::new(None);

// bind_interrupts!(struct Irqs {
//     USART3 => usart::InterruptHandler<peripherals::USART3>;
// });

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

        *INT_RESPONSE_PIN.lock().await = Some(int_reponse);

        let usart3_cfg = usart::Config::default();
        let mut usart3 = Uart::new_blocking(p.USART3, p.PD9, p.PD8, usart3_cfg).unwrap();
        // .expect("Error creating USART3 interface");

        let ser = SERIAL.init(usart3);
        // defmt_serial::defmt_serial(ser);

        let mut config = spi::Config::default();
        config.rise_fall_speed = gpio::Speed::VeryHigh;
        config.frequency = Hertz(1_000_000);

        *SPI_1.try_lock().unwrap() =
            Some(spi::Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, config));

        // *USART_3.try_lock().unwrap() = Some(usart3);

        let bmp390 = Bmp390 {
            cs: Output::new(
                AnyPin::from(p.PF12),
                gpio::Level::High,
                gpio::Speed::VeryHigh,
            ),
        };

        CraterBsp { bmp390 }
    }
}
