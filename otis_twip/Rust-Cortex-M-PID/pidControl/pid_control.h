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

double wraptopi_r(double x);

} // extern "C"
