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
FUZ *fuz_ptr = new FUZ; //FUZ is a structure in Rust, and a class in the librarie's .h file
</pre>

Now that we have a Rust object, we need a way to call the Rust method, and that has to be done from Rust.  This can be done by creating a `shim` function.  This is a basic Rust function that can be called by C++ i.e.:
<pre>
/*  in C++ */
InitLustreFUZ(fuz_ptr,input,millis(),setpoint,Kpf,Kif,Kdf,sampleRate);  //
</pre>
This shim function requires passing the pointer to the structure in memory to Rust.  Along with the controller variables, and inputs.  In Rust we can use this pointer to call the structure's method in that function:
<pre>
/* in Rust */
#[no_mangle]
pub extern "C" fn InitLustreFUZ(raw_ptr: *mut fuz::FUZ, input: f64, now: f64, setpoint: f64, Kp: f64, Ki: f64, Kd: f64, sampleTime:f64) {
    let PIDC_ref = unsafe{
        assert!(!raw_ptr.is_null());
        &mut *raw_ptr  // * derefs the pointer and returns a mutable reference to our data location
    };
    let inputArray= (input, now, Kp, Ki, Kd, setpoint, sampleTime);
    *PIDC_ref = fuz::FUZ::read_init(inputArray)  // Place struct in deferenced location PIDC_ref
}

</pre>
`#[no_mangle]` is necessary for compiling the library properly.  (add it before every function you want to access in C++).

<pre>
    let PIDC_ref = unsafe{
        assert!(!raw_ptr.is_null());
        &mut *raw_ptr  // * derefs the pointer and returns a mutable reference to our data location
    };
</pre>
Here we have to use a bit of unsafe code to convert our pointer to the structure to a mutable reference.  This shouldn't be too unsafe since this pointer is a global in C++.

<pre>
*PIDC_ref = fuz::FUZ::read_init(inputArray)  // Place struct in deferenced location PIDC_ref
</pre>
This function is the initialization of the structure, so we derefence the mutable reference to the memory location, and store out structure there.

This same process is used for other methods like the PID or Fuzzy logic calculations where we return a floating point result value.

**If you find a better way to access Rust structs from C++ then pls let me know.  Thanks!


## Dealing with the MKR1010
1. One of the most useful tricks I've found when uploading code is quickly pressing the reset button **twice**, I have gotten into a habit of doing this everytime I upload, because it often hangs up if I don't.
2. The MKR1010 board has a habit of handing up when communicating with the MPU6050.  This can be scary because it will leave the motors on, and just run away!  I suspect this occurs because the MPU6050 has too many buffer overflows, and fails.  I tried adding some pullup resistors to the I2C lines try help reduce this issue.  It seems like it's more common when using `Serial.print()` functions.
