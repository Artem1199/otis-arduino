# Setting up Arduino Code with Rust libraries

Requires: `Rust`,`Rust nightly`, `cbindgen`, `SAMD Arduino Libraries`, `WifiNINA`

otis_twip is for the older robot, otis_twipv2 is for the newer robot.  

### Arduino Setup
Rust generates .a type library for use with Arduino IDE.  Arduino IDE wants .a type libraries in the library folder with a setting that enables the specific library.
This can be done by placing our library in the same location where arduino downloads it's libraries to.  For linux that
is `~/Arduino/libraries/pid_control` with the following folder structure:
<pre>
pid_control
├── library.properties
└── src
    ├── cortex-m0plus
    │   └── libpid_control.a
    └── pid_control.h
</pre>

The library must have a library.properties file with `precompile=true`. 
The `build.sh` file makes this much easier by building the program and 
directly placing the .a/.h files in the libraries folder.

Arduino IDE settings: `File -> Preferences -> Enable "Show verbose output for compilation" and "upload"`

General controller modification process:
1. Compile Rust PID library /Rust-Cortex-M-PID/pid_control/build.sh, build.sh move the library and header to ~/Arduino/libraries/pid_control/src
2. Compile .ino file for MKR 1010 and upload


### Rust: 

`curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`

Rust nightly and m0+ target:

`Rust default nightly && rustup target add thumbv6m-none-eabi`

### cbindgen:

`cargo install --force cbindgen`

### SAMD Library and *WifiNINA library
<pre>
Inclue -> Manage Libraries -> search for SAMD and install
Inclue -> Manage Libraries -> search for WifiNINA and install 
</pre>

`Secrets.h` holds Wifi ssid/pass in format:
<pre>
const char* ssid = "XXXXXXXXXX";
const char* pass = "XXXXXXXXXX";
</pre>

*Note on WIFI.  I could not find a way to use WPA2-Personal wifi connection with the MKR1010.
I got around this by setting my router settings to WPA-Personal and connected to that.  WPA2-Enterprise is another option, but I never we around to testing it.

Alternatively comment out `#define WIFI` to disable the wifi.  This is present in v2 version.  

## Calling Rust from C++.
When writing Rust structures, `cbindgen` can be used to generate the header C++ OR C header file.
This can be modified in the `cbindgen.toml` file.  Accessing basic Rust functions is as easy as calling any function from a C++ library.
While structures and methods require a bit more work.  The only way I was able to find ways to access a rust structure was by passing pointers to memory for example:
<pre>
/* in C++ */
PID *pid_ptr = new PID; //PID is a structure in Rust, and a class in the librarie's .h file

</pre>
