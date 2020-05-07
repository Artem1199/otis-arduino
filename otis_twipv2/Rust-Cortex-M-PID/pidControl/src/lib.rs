#![feature(alloc_error_handler)]
#![no_std]

extern crate panic_halt;
extern crate alloc;
extern crate alloc_cortex_m;
extern crate cortex_m_rt as rt; // v0.5.x
extern crate nb;

use alloc::boxed::Box;
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use core::marker::PhantomData;
use cortex_m::asm;
use num_traits::float::FloatCore;
use crate::lus::helpers::Sys;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

mod lus;
mod pid1;


#[no_mangle]
pub extern "C" fn drop_PIDC(x: *mut pid1::PIDC){
    unsafe{
        
        Box::from_raw(x);
    }
}


#[no_mangle]
pub extern "C" fn create_PIDC(raw_ptr: *mut pid1::PIDC, kp: f64, ki: f64, kd: f64, SampleTime: f64){

    //let PIDC_rust = unsafe{Box::from_raw(raw_ptr)};

    let PIDC_ref = unsafe{
        assert!(!raw_ptr.is_null());
        &mut *raw_ptr
    };

    *PIDC_ref =  pid1::PIDC::new(kp, ki, kd, SampleTime);

   // Box::into_raw(Box::new(new_PIDC))
}

#[no_mangle]
pub extern "C" fn compute_PIDC(raw_ptr: *mut pid1::PIDC, Input: f64, now: f64) -> f64 {

    let PIDC_ref = unsafe{
        assert!(!raw_ptr.is_null());
        &mut *raw_ptr
    };
    pid1::PIDC::Compute(PIDC_ref, Input, now)

}




// # Kind2 Generated Lustre Shim Functions #
#[no_mangle]
pub extern "C" fn init_PIDC_lus(raw_ptr: *mut lus::Compute, inputArray: [f64; 7]) {

    let PIDC_ref = unsafe{
        assert!(!raw_ptr.is_null());
        &mut *raw_ptr  // * derefs the pointer and returns a mutable reference to our data location
    };

    *PIDC_ref = lus::Compute::read_init(inputArray)  // Place struct in deferenced location PIDC_ref

}

#[no_mangle]
pub extern "C" fn next_PIDC_lus(raw_ptr: *mut lus::Compute, inputArray: [f64; 7]) -> f64 {

    let mut PIDC_ref = unsafe{
        assert!(!raw_ptr.is_null());
        &mut *raw_ptr
    };

    lus::Compute::read_next(PIDC_ref, inputArray)
   // lus::Compute::read_next(PIDC_ref, inputArray)

   // *PIDC_ref = lus::Compute::read_next(PIDC_ref, inputArray)

}




//allocation error handling
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt();

    loop {}
}