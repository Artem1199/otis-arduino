#![feature(alloc_error_handler)]
#![no_std]

extern crate panic_halt;
extern crate alloc;
extern crate alloc_cortex_m;
extern crate cortex_m_rt as rt; // v0.5.x


use alloc::boxed::Box;
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use cortex_m::asm;
use num_traits::float::FloatCore;



#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();





#[no_mangle]
pub extern "C" fn foo() -> i32 {
	66
}

// #[repr(C)]
// pub struct PIDC {
//     kp: f64,
//     ki: f64,
//     kd: f64,
//     Output: f64,
//     Setpoint: f64,
//     errSum: f64,
//     lastErr: f64,
//     lastTime: f64,


// }

// impl PIDC {
//     fn new(kp:f64 , ki:f64, kd:f64) -> PIDC{
//         PIDC{
//             kp:kp,
//             ki:ki,
//             kd:kd,
//             Output: 0.0,
//             Setpoint: 0.0,
//             errSum: 0.0,
//             lastErr: 0.0,
//             lastTime: 0.0,
//         }
//     }

//     fn Compute(&mut self, Input:f64, now: f64)-> f64{
//         let timeChange = now - self.lastTime;

//         let error = self.Setpoint - Input;

//         self.errSum += error*timeChange;

//         let dErr = (error - self.lastErr) / timeChange;

//         self.Output = self.kp * error + self.ki * self.errSum + self.kd * dErr;

//         self.lastErr = error;

//         self.lastTime = now;

//         self.Output
        
//     }
// }

// impl Drop for PIDC {
// 	fn drop(&mut self) {
// 	}
// }


// #[no_mangle]
// pub extern "C" fn drop_PIDC(x: *mut PIDC){
//     unsafe{
        
//         Box::from_raw(x);
//     }
// }

// #[no_mangle]
// pub extern "C" fn create_PIDC(kp: f64, ki: f64, kd: f64) -> *mut PIDC {
//     let new_PIDC = PIDC::new(kp,ki, kd);
//     Box::into_raw(Box::new(new_PIDC))
// }

// #[no_mangle]
// pub extern "C" fn compute_PIDC(raw_ptr: *mut PIDC, Input: f64, now: f64) -> f64 {

//     let PIDC_ref = unsafe{
//         assert!(!raw_ptr.is_null());
//         &mut *raw_ptr
//     };
//     PIDC::Compute(PIDC_ref, Input, now)

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

        *outputSum += Ki * error * 0.1;

        if *outputSum > 255.0 
            {*outputSum = 255.0;}
        else if *outputSum < -255.0
            {*outputSum = -255.0;}



        *Output = Kp * error + *outputSum - Kd * dInput * 10.0;

        if *Output > 255.0 {*Output = 255.0;}
            else if*Output < -255.0 {*Output = 255.0;}

        *lastInput = Input;
        *lastTime = now;
    }
        
} 


#[no_mangle]
pub extern "C" fn wraptopi_r(x: f64)-> f64{
    let pi = 3.141592;
    let mut ans = x;
    ans -= (ans/(2.0*pi)).floor() * 2.0 * pi;
    if ans > pi{
        ans -= 2.0 * pi;
    }
    ans
}










//allocation error handling
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt();

    loop {}
}