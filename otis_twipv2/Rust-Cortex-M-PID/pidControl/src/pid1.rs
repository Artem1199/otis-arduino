#![no_std]
use num_traits::float::FloatCore;


#[repr(C)]
pub struct PIDC {
    kp: f64,
    ki: f64,
    kd: f64,
    SampleTime: f64,
    Output: f64,
    Setpoint: f64,
    errSum: f64,
    lastInput: f64,
    lastTime: f64,
    outputSum: f64,


}

impl PIDC {
   pub fn new(kp:f64 , ki:f64, kd:f64, SampleTime:f64) -> PIDC{
        PIDC{
            kp:kp,
            ki:ki,
            kd:kd,
            SampleTime: SampleTime,
            Output: 0.0,
            Setpoint: 0.0,
            errSum: 0.0,
            lastInput: 0.0,
            lastTime: 0.0,
            outputSum: 0.0,
        }
    }

    pub fn Compute(&mut self, Input:f64, now: f64)-> f64{

        let timeChange = now - self.lastTime;

        if timeChange >= self.SampleTime{

            let error = wraptopi_r(self.Setpoint - Input);
            let dInput = Input - self.lastInput;

            self.outputSum += self.ki * error * self.SampleTime/1000.0;

                if self.outputSum > 1000.0 
                    {self.outputSum = 1000.0;}
                else if self.outputSum < -1000.0
                    {self.outputSum = -1000.0;}

            self.Output = self.kp * error + self.ki * error * self.SampleTime/1000.0 - self.kd * dInput * 1000.0/self.SampleTime;


            if self.Output > 1000.0 {self.Output = 1000.0;}
            else if self.Output < -1000.0 {self.Output = -1000.0;}


            self.lastTime = now;
            self.lastInput = Input;
            self.Output
        } 
        
        else{
            self.Output
        }
        
    }


}

impl Drop for PIDC {
	fn drop(&mut self) {
	}
}


// #[no_mangle]
// pub extern "C" fn create_PIDC(kp: f64, ki: f64, kd: f64) -> *mut PIDC {
//     let new_PIDC = PIDC::new(kp,ki, kd);
//    Box::into_raw(Box::new(new_PIDC))
// }

//**************************************************************************************************************************************************** *//

#[no_mangle]
pub extern "C" fn compute_pid(Input: f64, Output: &mut f64, Setpoint: f64, Kp: f64, Ki: f64, Kd: f64,
                                now: f64, lastTime: &mut f64, SampleTime: f64, lastInput: &mut f64, outputSum: &mut f64)
{
    let timeChange = now - *lastTime;
    if timeChange >= SampleTime
    {
        let error = wraptopi_r(Setpoint - Input);
        let dInput = Input - *lastInput;

        *outputSum += Ki * error * SampleTime/1000.0;

      //P_ON_M
        *outputSum -= Kp * dInput;

        if *outputSum > 255.0 
            {*outputSum = 255.0;}
        else if *outputSum < -255.0
            {*outputSum = -255.0;}

     //P_ON_E
       // *Output = Kp * error;

        //P_ON_M
        *Output = 0.0;
        *Output += *outputSum - Kd * dInput * 1000.0/SampleTime;

       // *Output += Kp * error + Ki * error * SampleTime/1000.0 - Kd * dInput * 1000.0/SampleTime;

        if *Output > 255.0 {*Output = 255.0;}
            else if *Output < -255.0 {*Output = -255.0;}

        *lastInput = Input;
        *lastTime = now;
    }
        
}


pub extern "C" fn wraptopi_r(x: f64)-> f64{
    let pi = 3.141592;
    let mut ans = x;
    ans -= (ans/(2.0*pi)).floor() * 2.0 * pi;
    if ans > pi{
        ans -= 2.0 * pi;
    }
    ans
  }

