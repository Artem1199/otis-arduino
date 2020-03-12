#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <new>

extern "C" {

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

int32_t foo();

double wraptopi_r(double x);

} // extern "C"
