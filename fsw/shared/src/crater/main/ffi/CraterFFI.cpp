#include "crater/main/ffi/CraterFFI.hpp"

#include <cstdint>
#include <vector>

#include "crater/core/io/SpanReader.hpp"
#include "crater/core/io/VecWriter.hpp"
#include "crater/main/io/ChannelReader.hpp"
#include "crater/main/io/ChannelWriter.hpp"

namespace crt::ffi {

struct CraterCpp::Data {
    // crt::main::MainExec exec;

    crt::io::ChannelReader reader;
    crt::io::ChannelWriter writer;

    Channel<::mavlink::crater::msg::Sensor6DOFImu> ch_imu;
    Channel<::mavlink::crater::msg::ServoTarget> ch_servo;

    Receiver<::mavlink::crater::msg::Sensor6DOFImu> rx_imu;
    Sender<::mavlink::crater::msg::ServoTarget> tx_servo;

    std::vector<uint8_t> output_buf;
};

CraterCpp::CraterCpp() : data_(nullptr) {
    Channel<::mavlink::crater::msg::Sensor6DOFImu> ch_imu {};
    auto tx_imu = ch_imu.sender();
    auto rx_imu = ch_imu.receiver(10);

    Channel<::mavlink::crater::msg::ServoTarget> ch_servo {};
    auto tx_servo = ch_servo.sender();
    auto rx_servo = ch_servo.receiver(10);

    data_ = new Data(
        Data {
            .reader = io::ChannelReader(mavlink::MavlinkChannel::RustFFI),
            .writer = io::ChannelWriter(),
            .ch_imu = std::move(ch_imu),
            .ch_servo = std::move(ch_servo),
            .rx_imu = std::move(rx_imu),
            .tx_servo = std::move(tx_servo),
            .output_buf = {}
        }
    );

    data_->reader.add_channel(
        std::move(tx_imu),
        0,
        static_cast<uint8_t>(::mavlink::crater::ComponentID::Rocket)
    );

    data_->writer.add_channel(
        std::move(rx_servo),
        0,
        static_cast<uint8_t>(::mavlink::crater::ComponentID::Rocket)
    );
}

CraterCpp::~CraterCpp() {
    delete data_;
}

void CraterCpp::step(Buffer input, Buffer* output) {
    crt::io::ChannelReader& reader = data_->reader;

    io::SpanReader sr {nonstd::span(input.data, input.length)};

    reader.process(sr);

    if (auto imu = data_->rx_imu.try_receive()) {
        ::mavlink::crater::msg::ServoTarget servo {};
        servo.timestamp_us = imu->timestamp_us;
        servo.s1_target_deg = imu->acc_x_body_m_s2;
        servo.s2_target_deg = imu->acc_y_body_m_s2;
        servo.s3_target_deg = imu->gyro_x_body_rad_s;
        servo.s4_target_deg = imu->gyro_y_body_rad_s;

        data_->tx_servo.send(servo);
    }

    data_->output_buf.clear();
    io::VecWriter w {data_->output_buf};
    data_->writer.process(w);

    output->data = data_->output_buf.data();
    output->length = data_->output_buf.size();
}

} // namespace crt::ffi