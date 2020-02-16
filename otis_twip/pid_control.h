#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <new>

/// PID Controller.
///
/// A PID controller, supporting the `Controller` interface. Any public values
/// are safe to modify while in operation.
///
/// `p_gain`, `i_gain` and `d_gain` are the respective gain values. The
/// controlller internally stores an already adjusted integral, making it safe
/// to alter the `i_gain` - it will *not* result in an immediate large jump in
/// controller output.
///
/// `i_min` and `i_max` are the limits for the internal integral storage.
/// Similarly, `out_min` and `out_max` clip the output value to an acceptable
/// range of values. By default, all limits are set to +/- infinity.
///
/// `d_mode` The `DerivativeMode`, the default is `OnMeasurement`.
struct PIDController;

extern "C" {

/// Creates a new PID Controller.
PIDController new(double p_gain, double i_gain, double d_gain);

/// Convenience function to set `i_min`/`i_max` and `out_min`/`out_max`
/// to the same values simultaneously.
void set_limits(PIDController *self, double min, double max);

} // extern "C"
