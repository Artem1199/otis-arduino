#!/bin/bash
cargo build --release --target=thumbv6m-none-eabi --verbose
cbindgen --config cbindgen.toml --crate pid_control --output pid_control.h
cp pid_control.h ~/Arduino/libraries/pid_control/src
cp -r ./target/thumbv6m-none-eabi/release/libpid_control.a ~/Arduino/libraries/pid_control/src/cortex-m0plus/
