# README

## Compilation
1. Compile Rust PID library /Rust-Cortex-M-PID/pid_control/build.sh, build.sh move the library and header to ~/Arduino/libraries/pid_control/src
2. The library must have a library.properties file with precompile=true
2. Compile .ino file for MKR 1010 and upload

Requires: `Rust`,`Rust nightly`, `cbindgen`, `SAMD Arduino Libraries`, `WifiNINA`

Rust: 

`curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`

Rust nightly and m0+ target:

`Rust default nightly && rustup target add thumbv6m-none-eabi`

cbindgen:

`cargo install --force cbindgen`

SAMD Library and WifiNINA library

Inclue -> Manage Libraries -> search for `SAMD` 

`Secrets.h` holds Wifi ssid/pass in format:

`const char* ssid = "XXXXXXXXXX";`
`const char* pass = "XXXXXXXXXX";`

Place `pid_control folder in arduino auto install libraries directory (usually ~/Arduino/libraries).  Run ./build.sh to recompile and replace PID library.

