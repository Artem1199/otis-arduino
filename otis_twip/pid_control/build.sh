#!/bin/bash
cargo build --release --target=thumbv6m-none-eabi --verbose
cbindgen --config cbindgen.toml --crate pid_control --output pid_control.h

