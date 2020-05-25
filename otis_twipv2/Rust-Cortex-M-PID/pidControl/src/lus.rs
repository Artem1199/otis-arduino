//! Implementation for lustre node `Compute` (see [Compute](struct.Compute.html)).
//!
//! Code generated by the [Kind 2 model checker][kind 2].
//!
//! [kind 2]: http://kind2-mc.github.io/kind2/ (The Kind 2 model checker)

// Deactiving lint warnings the transformation does not respect.
#![allow(
  non_upper_case_globals, non_snake_case, non_camel_case_types,
  unused_variables, unused_parens
)]

#![no_std]
use crate::util::wraptopi_r;

use helpers::* ;

/// Entry point.
fn main() {
  //clap_and_run()
}


/// Stores the state for **top node** `Compute`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `Input` | Real |
/// | `Now` | Real |
/// | `Setpoint` | Real |
/// | `Kp` | Real |
/// | `Ki` | Real |
/// | `Kd` | Real |
/// | `SampleTime` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `Output` | Real |
///
/// # Sub systems
///
/// | Lustre identifier | Struct | Inputs | Outputs | Position |
/// |:---:|:---:|:---:|:---:|:---:|
/// | `Compute_calc` | [Compute_calc](struct.Compute_calc.html) | `Input`, `Now`, `Setpoint`, `Kp`, `Ki`, `Kd`, `SampleTime` | `abs_0` | [pid.lus line 12](../src/lus/pid.lus.html#12) |
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
#[repr(C)]
pub struct Compute {
  /// Input: `Compute.usr.Input`
  pub svar_Input: Real,
  /// Input: `Compute.usr.Now`
  pub svar_Now: Real,
  /// Input: `Compute.usr.Setpoint`
  pub svar_Setpoint: Real,
  /// Input: `Compute.usr.Kp`
  pub svar_Kp: Real,
  /// Input: `Compute.usr.Ki`
  pub svar_Ki: Real,
  /// Input: `Compute.usr.Kd`
  pub svar_Kd: Real,
  /// Input: `Compute.usr.SampleTime`
  pub svar_SampleTime: Real,

  /// Output: `Compute.usr.Output`
  pub svar_Output: Real,

  /// Local, call: `Compute.res.abs_0`
  pub svar_abs_0: Real,

  /// Call to `Compute_calc` ([pid.lus line 12](../src/lus/pid.lus.html#12)).
  pub Compute_calc_0: Compute_calc,
}

impl Sys for Compute {
  type Input = (
    Real, // svar_Input (Compute.usr.Input)
    Real, // svar_Now (Compute.usr.Now)
    Real, // svar_Setpoint (Compute.usr.Setpoint)
    Real, // svar_Kp (Compute.usr.Kp)
    Real, // svar_Ki (Compute.usr.Ki)
    Real, // svar_Kd (Compute.usr.Kd)
    Real, // svar_SampleTime (Compute.usr.SampleTime)
  ) ;

  type Array = [Real; 7] ;
  
  type Output = (
    Real, // svar_Output (Compute.usr.Output)
  ) ;
  fn arity() -> usize { 7 }
  fn input_of(array: Self::Array) -> Self::Input {
    (
      array[0], 
      array[1], 
      array[2], 
      array[3], 
      array[4], 
      array[5], 
      array[6],
    ) 
  }

  fn init(input: Self::Input) -> Self {
    // |===| Retrieving inputs.
    let svar_Input = input.0 ;
    let svar_Now = input.1 ;
    let svar_Setpoint = input.2 ;
    let svar_Kp = input.3 ;
    let svar_Ki = input.4 ;
    let svar_Kd = input.5 ;
    let svar_SampleTime = input.6 ;
    
    // |===| Computing initial state.
    let Compute_calc_0 =  Compute_calc::init( (
      svar_Input,
      svar_Now,
      svar_Setpoint,
      svar_Kp,
      svar_Ki,
      svar_Kd,
      svar_SampleTime,
    ) )  ;
    let (
      svar_abs_0,
    ) = Compute_calc_0.output() ;
    
    let svar_Output = ( if ((svar_Now - 0f64) >= svar_SampleTime) { svar_abs_0 } else {0f64 } ) ;
    
    // |===| Checking assertions.
    
    
    
    
    // |===| Returning initial state.
    Compute {
      // |===| Inputs.
      svar_Input: svar_Input,
      svar_Now: svar_Now,
      svar_Setpoint: svar_Setpoint,
      svar_Kp: svar_Kp,
      svar_Ki: svar_Ki,
      svar_Kd: svar_Kd,
      svar_SampleTime: svar_SampleTime,
      
      // |===| Outputs.
      svar_Output: svar_Output,
      
      // |===| Locals.
      svar_abs_0: svar_abs_0,
      
      // |===| Calls.
      Compute_calc_0: Compute_calc_0,
    }
  }

  fn next(&mut self, input: Self::Input) {
    // |===| Retrieving inputs.
    let svar_Input = input.0 ;
    let svar_Now = input.1 ;
    let svar_Setpoint = input.2 ;
    let svar_Kp = input.3 ;
    let svar_Ki = input.4 ;
    let svar_Kd = input.5 ;
    let svar_SampleTime = input.6 ;
    
    // |===| Computing next state.
    /*let Compute_calc_0 = */self.Compute_calc_0.next( (
      svar_Input,
      svar_Now,
      svar_Setpoint,
      svar_Kp,
      svar_Ki,
      svar_Kd,
      svar_SampleTime,
    ) ) ;
    let (
      svar_abs_0,
    ) = self.Compute_calc_0.output() ;
    let svar_Output = ( if ((svar_Now - self.svar_Now) >= svar_SampleTime) { svar_abs_0 } else {self.svar_Output } ) ;
    
    // |===| Checking assertions.
    
    
    // |===| Checking assumptions.
    
    
    // |===| Updating next state.
    // |===| Inputs.
    self.svar_Input = svar_Input ;
    self.svar_Now = svar_Now ;
    self.svar_Setpoint = svar_Setpoint ;
    self.svar_Kp = svar_Kp ;
    self.svar_Ki = svar_Ki ;
    self.svar_Kd = svar_Kd ;
    self.svar_SampleTime = svar_SampleTime ;
    
    // |===| Outputs.
    self.svar_Output = svar_Output ;
    
    // |===| Locals.
    self.svar_abs_0 = svar_abs_0 ;
    
    // |===| Calls.
    //self.Compute_calc_0 = Compute_calc_0 ;
    
    // |===| Return new state.
    //*self
  }

  fn output(& self) -> Self::Output {(
    self.svar_Output,
  )}
  // fn output_str(& self) -> String {
  //   format!(
  //     "{}",
  //     self.svar_Output
  //   )
  // }

  fn output_flt(& self) -> Real {
    self.svar_Output
  }
}

/// Stores the state for sub-node `Compute_calc`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `Input` | Real |
/// | `Now` | Real |
/// | `Setpoint` | Real |
/// | `Kp` | Real |
/// | `Ki` | Real |
/// | `Kd` | Real |
/// | `SampleTime` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `Output` | Real |
///
/// # Sub systems
///
/// | Lustre identifier | Struct | Inputs | Outputs | Position |
/// |:---:|:---:|:---:|:---:|:---:|
/// | `limit` | [Limit](struct.Limit.html) | `abs_2` | `abs_3` | [pid.lus line 35](../src/lus/pid.lus.html#35) |
/// | `limit` | [Limit](struct.Limit.html) | `abs_0` | `abs_1` | [pid.lus line 33](../src/lus/pid.lus.html#33) |
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
#[repr(C)]
pub struct Compute_calc {
  /// Input: `Compute_calc.usr.Input`
  pub svar_Input: Real,
  /// Input: `Compute_calc.usr.Now`
  pub svar_Now: Real,
  /// Input: `Compute_calc.usr.Setpoint`
  pub svar_Setpoint: Real,
  /// Input: `Compute_calc.usr.Kp`
  pub svar_Kp: Real,
  /// Input: `Compute_calc.usr.Ki`
  pub svar_Ki: Real,
  /// Input: `Compute_calc.usr.Kd`
  pub svar_Kd: Real,
  /// Input: `Compute_calc.usr.SampleTime`
  pub svar_SampleTime: Real,

  /// Output: `Compute_calc.usr.Output`
  pub svar_Output: Real,

  /// Local, alias(Compute_calc.usr.Output): `Compute_calc.res.abs_3`
  pub svar_abs_3: Real,
  /// Local, invisible local: `Compute_calc.res.abs_2`
  pub svar_abs_2: Real,
  /// Local, alias(Compute_calc.impl.usr.outputSum): `Compute_calc.res.abs_1`
  pub svar_abs_1: Real,
  /// Local, invisible local: `Compute_calc.res.abs_0`
  pub svar_abs_0: Real,
  /// Local, local: `Compute_calc.impl.usr.outputSum`
  pub svar_outputSum: Real,
  /// Local, local: `Compute_calc.impl.usr.error`
  pub svar_error: Real,
  /// Local, local: `Compute_calc.impl.usr.dInput`
  pub svar_dInput: Real,

  /// Call to `limit` ([pid.lus line 33](../src/lus/pid.lus.html#33)).
  pub limit_1: Limit,
  /// Call to `limit` ([pid.lus line 35](../src/lus/pid.lus.html#35)).
  pub limit_0: Limit,
}

impl Sys for Compute_calc {
  type Input = (
    Real, // svar_Input (Compute_calc.usr.Input)
    Real, // svar_Now (Compute_calc.usr.Now)
    Real, // svar_Setpoint (Compute_calc.usr.Setpoint)
    Real, // svar_Kp (Compute_calc.usr.Kp)
    Real, // svar_Ki (Compute_calc.usr.Ki)
    Real, // svar_Kd (Compute_calc.usr.Kd)
    Real, // svar_SampleTime (Compute_calc.usr.SampleTime)
  ) ;

  type Array = [Real; 7] ;

  type Output = (
    Real, // svar_Output (Compute_calc.usr.Output)
  ) ;
  fn arity() -> usize { 7 }
  fn input_of(array: Self::Array) -> Self::Input {
    (
      array[0], 
      array[1], 
      array[2], 
      array[3], 
      array[4], 
      array[5], 
      array[6],
    )

  }

  fn init(input: Self::Input) -> Self {
    // |===| Retrieving inputs.
    let svar_Input = input.0 ;
    let svar_Now = input.1 ;
    let svar_Setpoint = input.2 ;
    let svar_Kp = input.3 ;
    let svar_Ki = input.4 ;
    let svar_Kd = input.5 ;
    let svar_SampleTime = input.6 ;
    
    // |===| Computing initial state.
    let svar_dInput = (svar_Input - 0f64) ;
    let svar_error = wraptopi_r(svar_Setpoint - svar_Input) ;
    let svar_abs_0 = 0f64 ;
    let svar_abs_2 = (((svar_Kp * svar_error) + (((svar_Ki * svar_error) * svar_SampleTime) / 1000f64)) - (((svar_Kd * svar_dInput) * 1000f64) / svar_SampleTime)) ;
    let limit_1 = Limit::init( (
      svar_abs_0,
    ) ) ;
    let (
      svar_abs_1,
    ) = limit_1.output() ;
    
    let limit_0 = Limit::init( (
      svar_abs_2,
    ) ) ;
    let (
      svar_abs_3,
    ) = limit_0.output() ;
    
    let svar_Output = svar_abs_3 ;
    let svar_outputSum = svar_abs_1 ;
    
    // |===| Checking assertions.
    
    
    
    
    // |===| Returning initial state.
    Compute_calc {
      // |===| Inputs.
      svar_Input: svar_Input,
      svar_Now: svar_Now,
      svar_Setpoint: svar_Setpoint,
      svar_Kp: svar_Kp,
      svar_Ki: svar_Ki,
      svar_Kd: svar_Kd,
      svar_SampleTime: svar_SampleTime,
      
      // |===| Outputs.
      svar_Output: svar_Output,
      
      // |===| Locals.
      svar_abs_3: svar_abs_3,
      svar_abs_2: svar_abs_2,
      svar_abs_1: svar_abs_1,
      svar_abs_0: svar_abs_0,
      svar_outputSum: svar_outputSum,
      svar_error: svar_error,
      svar_dInput: svar_dInput,
      
      // |===| Calls.
      limit_1: limit_1,
      limit_0: limit_0,
    }
  }

  fn next(&mut self, input: Self::Input) {
    // |===| Retrieving inputs.
    let svar_Input = input.0 ;
    let svar_Now = input.1 ;
    let svar_Setpoint = input.2 ;
    let svar_Kp = input.3 ;
    let svar_Ki = input.4 ;
    let svar_Kd = input.5 ;
    let svar_SampleTime = input.6 ;
    
    // |===| Computing next state.
    let svar_dInput = (svar_Input - self.svar_Input) ;
    let svar_error = wraptopi_r(svar_Setpoint - svar_Input) ;
    // let svar_abs_0 = (self.svar_outputSum + (((svar_Ki * svar_error) * svar_SampleTime) / 1000f64)) ;  Original before correction to PID
    let svar_abs_0 = (self.svar_outputSum + (((svar_Ki * svar_error) * svar_SampleTime) / 1000f64)  - (svar_Kp * svar_dInput)) ;

    /*let limit_1 = */self.limit_1.next( (
      svar_abs_0,
    ) )  ;

    let (
      svar_abs_1,
    ) = self.limit_1.output() ;


    // let svar_abs_2 = (((svar_Kp * svar_error) + (((svar_Ki * svar_error) * svar_SampleTime) / 1000f64)) - (((svar_Kd * svar_dInput) * 1000f64) / svar_SampleTime)) ;
    let svar_abs_2 = (svar_abs_1 - (((svar_Kd * svar_dInput) * 1000f64) / svar_SampleTime)) ;
    
    /*let limit_0 = */self.limit_0.next( (
      svar_abs_2,
    ) )  ;
    let (
      svar_abs_3,
    ) = self.limit_0.output() ;
    
    let svar_Output = svar_abs_3 ;
    let svar_outputSum = svar_abs_1 ;
    
    // |===| Checking assertions.
    
    
    // |===| Checking assumptions.
    
    
    // |===| Updating next state.
    // |===| Inputs.
    self.svar_Input = svar_Input ;
    self.svar_Now = svar_Now ;
    self.svar_Setpoint = svar_Setpoint ;
    self.svar_Kp = svar_Kp ;
    self.svar_Ki = svar_Ki ;
    self.svar_Kd = svar_Kd ;
    self.svar_SampleTime = svar_SampleTime ;
    
    // |===| Outputs.
    self.svar_Output = svar_Output ;
    
    // |===| Locals.
    self.svar_abs_3 = svar_abs_3 ;
    self.svar_abs_2 = svar_abs_2 ;
    self.svar_abs_1 = svar_abs_1 ;
    self.svar_abs_0 = svar_abs_0 ;
    self.svar_outputSum = svar_outputSum ;
    self.svar_error = svar_error ;
    self.svar_dInput = svar_dInput ;
    
    // |===| Calls.
    //self.limit_1 = limit_1 ;
    //self.limit_0 = limit_0 ;
    
    // |===| Return new state.
     //*self
  }

  fn output(& self) -> Self::Output {(
    self.svar_Output,
  )}
  // fn output_str(& self) -> String {
  //   format!(
  //     "{}",
  //     self.svar_Output
  //   )
  // }

  fn output_flt(& self) -> Real {
    self.svar_Output
  }
}

/// Stores the state for sub-node `limit`.
///
/// # Inputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `x` | Real |
///
/// # Outputs
///
/// | Lustre identifier | Type |
/// |:---:|:---|
/// | `y` | Real |
///
/// # Sub systems
///
/// No subsystems for this system.
///
/// # Assertions
///
/// /// No assertions for this system.
///
/// # Assumptions
///
/// No assumptions for this system.
///
#[repr(C)]
pub struct Limit {
  /// Input: `limit.usr.x`
  pub svar_x: Real,

  /// Output: `limit.usr.y`
  pub svar_y: Real,


}

impl Sys for Limit {
  type Input = (
    Real, // svar_x (limit.usr.x)
  ) ;
  type Output = (
    Real, // svar_y (limit.usr.y)
  ) ;

  type Array = [Real; 1] ;

  fn arity() -> usize { 1 }
  fn input_of(array: Self::Array) -> Self::Input {
    (
      array[0],
    ) 
  }

  fn init(input: Self::Input) -> Self {
    // |===| Retrieving inputs.
    let svar_x = input.0 ;
    
    // |===| Computing initial state.
    let svar_y = ( if (svar_x > 255f64) { 255f64 } else {( if (svar_x < - 255f64) { - 255f64 } else {svar_x } ) } ) ;
    
    // |===| Checking assertions.
    
    
    
    
    // |===| Returning initial state.
     Limit {
      // |===| Inputs.
      svar_x: svar_x,
      
      // |===| Outputs.
      svar_y: svar_y,
      
      // |===| Locals.
      
      
      // |===| Calls.
      
    }
  }

  fn next(&mut self, input: Self::Input) {
    // |===| Retrieving inputs.
    let svar_x = input.0 ;
    
    // |===| Computing next state.
    let svar_y = ( if (svar_x > 255f64) { 255f64 } else {( if (svar_x < - 255f64) { - 255f64 } else {svar_x } ) } ) ;
    
    // |===| Checking assertions.
    
    
    // |===| Checking assumptions.
    
    
    // |===| Updating next state.
    // |===| Inputs.
    self.svar_x = svar_x ;
    
    // |===| Outputs.
    self.svar_y = svar_y ;
    
    // |===| Locals.
    
    
    // |===| Calls.
    
    
    // |===| Return new state.
    // *self
  }

  fn output(& self) -> Self::Output {(
    self.svar_y,
  )}
  // fn output_str(& self) -> String {
  //   format!(
  //     "{}",
  //     self.svar_y
  //   )
  // }

  fn output_flt(& self) -> Real {
    self.svar_y
  }
}



/// Types and structures for systems.
pub mod helpers {
  // use std::io::{ Stdin, stdin } ;
  // use std::process::exit ;

  /// Prints usage.
//   pub fn help() {
//     println!("") ;
//     println!("\
// Options:
//   -h, --help
//     prints this message
//   --limit
//     inputs:  Real (x)
//     outputs: Real (y)
//   --compute_calc
//     inputs:  Real (Input)
//              Real (Now)
//              Real (Setpoint)
//              Real (Kp)
//              Real (Ki)
//              Real (Kd)
//              Real (SampleTime)
//     outputs: Real (Output)
//   --compute
//     inputs:  Real (Input)
//              Real (Now)
//              Real (Setpoint)
//              Real (Kp)
//              Real (Ki)
//              Real (Kd)
//              Real (SampleTime)
//     outputs: Real (Output)
// Usage:
//   Inputs (outputs) are read (printed) as comma-separated values on a single
//   line.
//   The read-eval-print loop runs forever, write \"exit\" or \"quit\"
//   to exit it cleanly.
// Default system: \"compute\".\
//     ") ;
//     println!("")
//   }

//   /// Prints usage, an error, and exits with status `2`.
//   pub fn error<T: ::std::fmt::Display>(e: T) {
//     help() ;
//     println!("Error: {}", e) ;
//     println!("") ;
//     exit(2)
//   }

  /// Handles CLA.
  // pub fn clap_and_run() {
  //   use std::env::args ;
  //   let mut args = args() ;
  //   // Skipping first argument (name of binary).
  //   match args.next() {
  //     Some(_) => (),
  //     None => unreachable!(),
  //   } ;
  //   if let Some(arg) = args.next() {
  //     match & arg as & str {
  //       "-h" | "--help" => {
  //         help() ;
  //         exit(0)
  //       },
  //       "--limit" => super::Limit::run(),
  //       "--compute_calc" => super::Compute_calc::run(),
  //       "--compute" => super::Compute::run(),
  //       arg => error(
  //         format!("unexpected argument \"{}\".", arg)
  //       ),
  //     }
  //   } ;
  //   // If no argument given, run top system.
  //   // super::Compute::run()
  // }

  /// Alias for `i64`.
  pub type Int = i64 ;
  /// Alias for `f64`.
  pub type Real = f64 ;
  /// Alias for `bool`.
  pub type Bool = bool ;

  /// Stores an `Stdin` and a buffer to read lines.
  // pub struct InputReader {
  //   /// Standard input.
  //   stdin: Stdin,
  //   /// String buffer.
  //   buff: String,
  // }
  // // impl InputReader {
  //   /// Creates an input reader.
  //   pub fn mk() -> Self {
  //     InputReader {
  //       stdin: stdin(),
  //       buff: String::with_capacity(100),
  //     }
  //   }
  //   Reads comma separated inputs from standard input.
  //   pub fn read_inputs(& mut self) -> Result<Vec<String>, String> {
  //     self.buff.clear() ;
  //     match self.stdin.read_line(& mut self.buff) {
  //       Ok(_) => (),
  //       Err(e) => return Err(
  //         format!("could not read line from stdin: {}", e)
  //       ),
  //     } ;
  //     let chars = self.buff.trim_left().chars() ;
  //     let mut buff = String::new() ;
  //     let mut vec = vec![] ;
  //     for c in chars {
  //       match c {
  //         ' ' | '\t' => (),
  //         ',' | '\n' => {
  //           vec.push(buff.clone()) ;
  //           buff.clear()
  //         },
  //         _ => buff.push(c),
  //       }
  //     } ;
  //     if vec.len() > 1 {
  //       match vec[0].trim() {
  //         "exit" | "quit" => exit(0),
  //         _ => ()
  //       }
  //     } ;
  //     Ok(vec)
  //   }
  // }

  /// Trait all systems must implement.
  pub trait Sys: Sized {
    /// Type of inputs.
    type Input ;
    /// Type of outputs.
    type Output ;
    /// Type of array input.
    type Array ;
    /// Number of inputs expected.
    fn arity() -> usize ;
    /// Parses a vector of inputs.
    fn input_of(array: Self::Array) -> Self::Input;
    /// Initial state of the system.
    fn init(input: Self::Input) -> Self ;
    /// Computes the next step.
    fn next(&mut self, input: Self::Input) ;

    fn read_init(input: Self::Input) -> Self{
     // let inputs = Self::input_of(array);
      Self::init(input)
    }

    fn read_next(&mut self, input: Self::Input){
     // let inputs = Self::input_of(array);
      self.next(input);
    }
    fn output_flt(& self) -> Real;
    // Reads inputs from standard input, computes initial state, prints output.
    // fn read_init(reader: & mut InputReader) -> Self {
    //   match Self::input_of( try!(reader.read_inputs()) ) {
    //     Ok(inputs) => {
    //       let init = try!( Self::init(inputs) ) ;
    //       println!("{}", init.output_str()) ;
    //       Ok(init)
    //     },
    //     Err(s) => Err(s),
    //   }
    // }
    // Reads inputs from standard input, computes next step, prints output.
    // fn read_next(self, reader: & mut InputReader) -> Result<Self, String> {
    //   match Self::input_of( try!(reader.read_inputs()) ) {
    //     Ok(inputs) => {
    //       let next = try!( self.next(inputs) ) ;
    //       println!("{}", next.output_str()) ;
    //       Ok(next)
    //     },
    //     Err(s) => Err(s),
    //   }
    // }
    /// Output of the system.
    fn output(& self) -> Self::Output ;
    // String representation of the output.
   // fn output_str(& self) -> String ;
    // Runs a never-ending, read-eval-print loop on the system.
    // fn run() -> ! {
    //   let mut reader = InputReader::mk() ;
    //   let mut state = match Self::read_init(& mut reader) {
    //     Ok(init) => init,
    //     Err(e) => {
    //       println!("(Error: {})", e) ;
    //       exit(2)
    //     }
    //   } ;
    //   loop {
    //     match state.read_next(& mut reader) {
    //       Ok(next) => state = next,
    //       Err(e) => {
    //         println!("(Error: {})", e) ;
    //         exit(2)
    //       }
    //     }
    //   }
    // }
  }
}
