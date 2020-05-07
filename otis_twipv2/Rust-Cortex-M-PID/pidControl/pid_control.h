#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <new>

struct PIDC {
  double kp;
  double ki;
  double kd;
  double SampleTime;
  double Output;
  double Setpoint;
  double errSum;
  double lastInput;
  double lastTime;
  double outputSum;
};

/// Alias for `f64`.
using Real = double;

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
struct Limit {
  /// Input: `limit.usr.x`
  Real svar_x;
  /// Output: `limit.usr.y`
  Real svar_y;
};

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
struct Compute_calc {
  /// Input: `Compute_calc.usr.Input`
  Real svar_Input;
  /// Input: `Compute_calc.usr.Now`
  Real svar_Now;
  /// Input: `Compute_calc.usr.Setpoint`
  Real svar_Setpoint;
  /// Input: `Compute_calc.usr.Kp`
  Real svar_Kp;
  /// Input: `Compute_calc.usr.Ki`
  Real svar_Ki;
  /// Input: `Compute_calc.usr.Kd`
  Real svar_Kd;
  /// Input: `Compute_calc.usr.SampleTime`
  Real svar_SampleTime;
  /// Output: `Compute_calc.usr.Output`
  Real svar_Output;
  /// Local, alias(Compute_calc.usr.Output): `Compute_calc.res.abs_3`
  Real svar_abs_3;
  /// Local, invisible local: `Compute_calc.res.abs_2`
  Real svar_abs_2;
  /// Local, alias(Compute_calc.impl.usr.outputSum): `Compute_calc.res.abs_1`
  Real svar_abs_1;
  /// Local, invisible local: `Compute_calc.res.abs_0`
  Real svar_abs_0;
  /// Local, local: `Compute_calc.impl.usr.outputSum`
  Real svar_outputSum;
  /// Local, local: `Compute_calc.impl.usr.error`
  Real svar_error;
  /// Local, local: `Compute_calc.impl.usr.dInput`
  Real svar_dInput;
  /// Call to `limit` ([pid.lus line 33](../src/lus/pid.lus.html#33)).
  Limit limit_1;
  /// Call to `limit` ([pid.lus line 35](../src/lus/pid.lus.html#35)).
  Limit limit_0;
};

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
struct Compute {
  /// Input: `Compute.usr.Input`
  Real svar_Input;
  /// Input: `Compute.usr.Now`
  Real svar_Now;
  /// Input: `Compute.usr.Setpoint`
  Real svar_Setpoint;
  /// Input: `Compute.usr.Kp`
  Real svar_Kp;
  /// Input: `Compute.usr.Ki`
  Real svar_Ki;
  /// Input: `Compute.usr.Kd`
  Real svar_Kd;
  /// Input: `Compute.usr.SampleTime`
  Real svar_SampleTime;
  /// Output: `Compute.usr.Output`
  Real svar_Output;
  /// Local, call: `Compute.res.abs_0`
  Real svar_abs_0;
  /// Call to `Compute_calc` ([pid.lus line 12](../src/lus/pid.lus.html#12)).
  Compute_calc Compute_calc_0;
};

extern "C" {

double compute_PIDC(PIDC *raw_ptr, double Input, double now);

void compute_pid(double Input,
                 double *Output,
                 double Setpoint,
                 double Kp,
                 double Ki,
                 double Kd,
                 double now,
                 double *lastTime,
                 double SampleTime,
                 double *lastInput,
                 double *outputSum);

void create_PIDC(PIDC *raw_ptr, double kp, double ki, double kd, double SampleTime);

void drop_PIDC(PIDC *x);

void init_PIDC_lus(Compute *raw_ptr,
                   double input,
                   double now,
                   double setpoint,
                   double Kp,
                   double Ki,
                   double Kd,
                   double sampleTime);

double next_PIDC_lus(Compute *raw_ptr,
                     double input,
                     double now,
                     double setpoint,
                     double Kp,
                     double Ki,
                     double Kd,
                     double sampleTime);

double wraptopi_r(double x);

} // extern "C"
