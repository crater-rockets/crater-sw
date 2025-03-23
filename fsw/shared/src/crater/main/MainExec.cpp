#include "crater/main/MainExec.hpp"

using namespace mavlink::crater::msg;

namespace crt::main {

MainExec::MainExec(InputHarness&& inputs, OutputHarness&& outputs) :
    inputs_(std::move(inputs)),
    outputs_(std::move(outputs)) {}

void MainExec::step() {
    std::optional<SensorStaticPressure> press = inputs_.sens_static_press.try_receive();
    if (press) {
        SensorStaticPressure out;
        out.pressure_pa = press->pressure_pa + 1;
        out.press_sensor_id = press->press_sensor_id;
        out.timestamp_us = press->timestamp_us;

        outputs_.modified_static_press.send(out);
    }
}

} // namespace crt::main