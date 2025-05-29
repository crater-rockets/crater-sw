use std::{
    path::PathBuf,
    sync::mpsc::channel,
    time::{Duration, Instant},
};

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

    #[arg(short, long)]
    output: Option<PathBuf>,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let (ctrlc_tx, ctrlc_rx) = channel::<()>();

    ctrlc::set_handler(move || {
        let _ = ctrlc_tx.send(());
    })?;

    let serial = serialport::new(args.port.clone(), args.baud_rate)
        .timeout(Duration::from_millis(args.timeout_ms))
        .open()
        .expect(format!("Could not open serial port {}", args.port).as_str());

    println!("baud: {}", serial.baud_rate()?);
    println!("bits: {}", serial.data_bits()?);
    println!("fc: {}", serial.flow_control()?);
    println!("par: {}", serial.parity()?);
    println!("sb: {}", serial.stop_bits()?);

    let mut rec = if let Some(file_path) = args.output {
        println!(
            "Saving output to file: {}",
            file_path.clone().into_os_string().into_string().unwrap()
        );
        rerun::RecordingStreamBuilder::new("serial_log").save(file_path)
    } else {
        rerun::RecordingStreamBuilder::new("serial_log").connect_grpc_opts(
            "rerun+http://127.0.0.1:9876/proxy",
            Some(Duration::from_secs(10)),
        )
    }?;

    serial.clear(serialport::ClearBuffer::All)?;
    let mut reader: PeekReader<Box<dyn SerialPort + 'static>, 280> = PeekReader::new(serial);
    // reader.consume(1000000000);
    let mut initial_system_time = None;
    let log_start_time = Instant::now();

    while let Err(_) = ctrlc_rx.try_recv() {
        match read_v2_msg::<MavMessage, _>(&mut reader) {
            Ok((_, msg)) => {
                // Discard first few instants of data or so as it might be stale (clearing the port buffer does not seem to work)
                if Instant::now() - log_start_time < Duration::from_millis(500) {
                    continue;
                }
                handle_message(&mut rec, &msg, &mut initial_system_time)?;
            }
            // Err(MessageReadError::Io(io)) => {}
            Err(err) => {
                println!("Error reading Mavlink message from serial: {:#?}", err);
            }
        }
    }

    println!("End");
    Ok(())
}

fn handle_message(
    rec: &mut RecordingStream,
    msg: &MavMessage,
    initial_time: &mut Option<i64>,
) -> Result<()> {
    fn rec_log_time(rec: &mut RecordingStream, ts: i64, initial_time: &mut Option<i64>) {
        match initial_time {
            Some(initial_time) => {
                rec.set_duration_secs(
                    "rel_system_time",
                    Timestamp::from_micros(ts - *initial_time)
                        .monotonic
                        .elapsed_seconds_f64(),
                );
            }
            None => {
                *initial_time = Some(ts);
                rec.set_duration_secs("rel_system_time", 0.0);
            }
        }
    }
    match msg {
        MavMessage::SensPressureSample(data) => {
            let sensors_name = match data.sensor_id {
                PressureSensorId::Bmp390 => "bmp390",
            };

            rec_log_time(rec, data.timestamp_us, initial_time);

            PressureSensorSampleLog.write(
                rec,
                "system_time",
                format!("sensors/{sensors_name}").as_str(),
                Timestamp::from_micros(data.timestamp_us),
                data.into(),
            )
        }
        MavMessage::SensImuSample(data) => {
            let sensors_name = match data.sensor_id {
                ImuSensorId::Icm42688 => "icm42688",
            };
            rec_log_time(rec, data.timestamp_us, initial_time);

            ImuSensorSampleLog.write(
                rec,
                "system_time",
                format!("sensors/{sensors_name}").as_str(),
                Timestamp::from_micros(data.timestamp_us),
                data.into(),
            )
        }

        msg => Err(anyhow!("Unrecognized mavlink message: {:#?}", msg)),
    }
}
