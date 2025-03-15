# Crater Software

## Flight Dynamics Simulator (`sim/`)

### Setup
1. Install the [rerun.io](https://rerun.io/) Viewer following the guide on the website (https://rerun.io/docs/getting-started/installing-viewer)
    - Suggested method: `cargo install rerun-cli`
2. Build with `cargo build`, as usual

### Run
1. Start the Rerun Viewer with `rerun`
2. On a separate terminal, run the simulator with `cargo run`


## Flight Software (`fsw/`)
Divided in three folders:
- `shared`: Cross platform logic code
- `embedded`: HAL and wrapper code to run on microcontrollers with Miosix
- `proto`: Mavlink message definitions and codegeneration scripts

### Setup
(optional)
> python3 -m venv .venv && source .venv/bin/activate

> cd fsw/proto
> pip install -r requirements.txt

### Building (x86)
From the root directory
> cmake -Bbuild
> cmake --build build

### Building (miosix)
From the `fsw/embedded` directory
> cmake -Bbuild -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f756.toolchain.cmake
> cmake --build build