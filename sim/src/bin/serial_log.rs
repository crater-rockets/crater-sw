use std::time::Duration;

use anyhow::{Result, anyhow};
use clap::Parser;
use crater::{
    core::time::Timestamp,
    crater::logging::rerun::{
        RerunWrite,
        crater_log_impl::{ImuSensorSampleLog, PressureSensorSampleLog},
    },
};
use crater_gnc::{
    mav_crater::{ImuSensorId, MavMessage, PressureSensorId},
    peek_reader::PeekReader,
    read_v2_msg,
};
use rerun::RecordingStream;
use serialport::SerialPort;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    port: String,

    #[arg(short, long, default_value_t = 921600)]
    baud_rate: u32,

    #[arg(short, long, default_value_t = 1000)]
    timeout_ms: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let serial = serialport::new(args.port.clone(), args.baud_rate)
        .timeout(Duration::from_millis(args.timeout_ms))
        .open()
        .expect(format!("Could not open serial port {}", args.port).as_str());

    println!("baud: {}", serial.baud_rate()?);
    println!("bits: {}", serial.data_bits()?);
    println!("fc: {}", serial.flow_control()?);
    println!("par: {}", serial.parity()?);
    println!("sb: {}", serial.stop_bits()?);

    let mut rec = rerun::RecordingStreamBuilder::new("serial_log").connect_grpc_opts(
        "rerun+http://127.0.0.1:9876/proxy",
        Some(Duration::from_secs(10)),
    )?;

    let mut reader: PeekReader<Box<dyn SerialPort + 'static>, 280> = PeekReader::new(serial);

    loop {
        match read_v2_msg::<MavMessage, _>(&mut reader) {
            Ok((_, msg)) => {
                handle_message(&mut rec, &msg)?;
            }
            // Err(MessageReadError::Io(io)) => {}
            Err(err) => {
                println!("Error reading Mavlink message from serial: {:#?}", err);
            }
        }
    }
}

fn handle_message(rec: &mut RecordingStream, msg: &MavMessage) -> Result<()> {
    match msg {
        MavMessage::SensPressureSample(data) => {
            let sensors_name = match data.sensor_id {
                PressureSensorId::Bmp390 => "bmp390",
            };

            PressureSensorSampleLog.write(
                rec,
                "time",
                format!("sensors/{sensors_name}").as_str(),
                Timestamp::from_micros(data.timestamp_us),
                data.into(),
            )
        }
        MavMessage::SensImuSample(data) => {
            let sensors_name = match data.sensor_id {
                ImuSensorId::Icm42688 => "icm42688",
            };

            ImuSensorSampleLog.write(
                rec,
                "time",
                format!("sensors/{sensors_name}").as_str(),
                Timestamp::from_micros(data.timestamp_us),
                data.into(),
            )
        }

        msg => Err(anyhow!("Unrecognized mavlink message: {:#?}", msg)),
    }
}
