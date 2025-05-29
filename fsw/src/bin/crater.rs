#![no_std]
#![no_main]

use core::{array, f32};

use alloc::boxed::Box;
use crater_fsw::{
    device::{
        bsp::{self, CraterBsp},
        spi::{SpiDevice, SpiDeviceConfig},
    },
    io::channel::EmbassyReceiver,
    sensors::{
        self,
        bmp390::{self, Bmp390, Bmp390Sample},
        icm42688::{Icm42688, Icm42688Sample},
    },
};
use crater_gnc::{
    MavHeader,
    common::Ts,
    datatypes::sensors::{ImuSensorSample, PressureSensorSample},
    hal::channel::Receiver,
    mav_crater::{
        self, ImuSensorId, MavMessage, PressureSensorId, SensImuSample_DATA,
        SensPressureSample_DATA,
    },
    write_v2_msg_async,
};
use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::pubsub::DynPublisher;
use embassy_time::Timer;
use uom::si::{
    angular_absement::degree_second, pressure::pascal, thermodynamic_temperature::degree_celsius,
};
use {defmt_rtt as _, panic_probe as _};
extern crate alloc;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let bsp = CraterBsp::init().await;
    Timer::after_millis(100).await;

    // let dev_bmp390 = SpiDevice::new(
    //     &bsp::bus::SPI_1,
    //     bsp.sens_bmp390.cs,
    //     SpiDeviceConfig {
    //         read_padding_byte: true,
    //     },
    // );

    // let bmp390 = Bmp390::init(
    //     dev_bmp390,
    //     bmp390::Config {
    //         odr: bmp390::regs::DataRateValue::Odr50,
    //         osr_p: bmp390::regs::OversamplingValue::Times4,
    //         osr_t: bmp390::regs::OversamplingValue::None,
    //     },
    // )
    // .await
    // .expect("Could not init bmp390!");

    // info!("BMP390 Initialized!");

    let dev_icm_42688 = SpiDevice::new(
        &bsp::bus::SPI_1,
        bsp.sens_icm42688.cs,
        SpiDeviceConfig::default(),
    );

    let config_icm42688 = sensors::icm42688::Config {
        accel_fs: sensors::icm42688::regs::AccelFullScale::Fs2g,
        accel_odr: sensors::icm42688::regs::AccelDataRate::Odr200hz,
        gyro_fs: sensors::icm42688::regs::GyroFullScale::Fs15_625dps,
        gyro_odr: sensors::icm42688::regs::GyroDataRate::Odr200hz,
    };

    let icm42688 = Icm42688::init(
        dev_icm_42688,
        config_icm42688,
        &bsp::interrupts::SIGNAL_ICM_42688_DRDY,
    )
    .await
    .expect("Could not init Icm42688!");

    let tx_bmp390 = bsp::channels::SENS_BMP_390_SAMPLE.dyn_publisher().unwrap();
    let mut rx_bmp390 = bsp::channels::SENS_BMP_390_SAMPLE.dyn_subscriber().unwrap();

    let tx_icm42688 = bsp::channels::SENS_ICM_42688_SAMPLE
        .dyn_publisher()
        .unwrap();
    let mut rx_icm42688 = bsp::channels::SENS_ICM_42688_SAMPLE
        .dyn_subscriber()
        .unwrap();

    // spawner.spawn(sens_press(bmp390, tx_bmp390)).unwrap();
    spawner.spawn(sens_imu(icm42688, tx_icm42688)).unwrap();
    // spawner.spawn(interru()).unwrap();

    let mut seq_cnt: u8 = 0;
    let mut header = MavHeader {
        ..Default::default()
    };

    loop {
        let mut uart_tx = bsp::bus::DEBUG_SERIAL_TX.lock().await;

        while let Some(sample) = rx_bmp390.try_next_message_pure() {
            let mav = sample.v.to_mavlink(PressureSensorId::Bmp390, sample.t);

            header.sequence = seq_cnt;
            seq_cnt += 1;

            write_v2_msg_async(uart_tx.as_mut().unwrap(), header, &mav)
                .await
                .unwrap();
        }

        while let Some(sample) = rx_icm42688.try_next_message_pure() {
            let mav = sample.v.data.to_mavlink(ImuSensorId::Icm42688, sample.t);

            header.sequence = seq_cnt;
            seq_cnt += 1;

            write_v2_msg_async(uart_tx.as_mut().unwrap(), header, &mav)
                .await
                .unwrap();
        }

        Timer::after_millis(2).await;
    }
}

#[embassy_executor::task]
async fn sens_imu(mut icm: Icm42688, tx: DynPublisher<'static, Ts<Icm42688Sample>>) {
    info!("Running IMU");
    loop {
        let sample = icm.sample().await;
        tx.publish_immediate(Ts::new(sample.t, sample.v));
        
    }
}

#[embassy_executor::task]
async fn sens_press(mut bmp390: Bmp390, tx: DynPublisher<'static, Ts<PressureSensorSample>>) {
    info!("Running press");
    loop {
        let sample = bmp390.sample().await;
        tx.publish_immediate(Ts::new(sample.t, sample.v.value));
        Timer::after_millis(20).await;
    }
}
