#pragma once

#include <mavlink/crater/crater.hpp>
#include "crater/core/Channel.hpp"
#include "crater/core/exec/Executor.hpp"

namespace crt::main {

class MainExec {
public:
    struct InputHarness {
        Receiver<mavlink::crater::msg::SensorStaticPressure> sens_static_press;
    };

    struct OutputHarness {
        Sender<mavlink::crater::msg::SensorStaticPressure> modified_static_press;
    };

    MainExec(InputHarness&& inputs, OutputHarness&& outputs);
    
    void step();

private:
    InputHarness inputs_;
    OutputHarness outputs_;
};

} // namespace crt::main