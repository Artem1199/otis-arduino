#![no_std]
use num_traits::float::FloatCore;

#[no_mangle]
pub mod util{

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
