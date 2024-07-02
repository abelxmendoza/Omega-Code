use std::ffi::CString;
use std::os::raw::c_char;

#[no_mangle]
pub extern "C" fn process_ultrasonic_data(input: *const c_char) -> *mut c_char {
    let c_str = unsafe { CString::from_raw(input as *mut c_char) };
    let r_str = c_str.to_str().unwrap();
    let result = format!("Processed: {}", r_str);
    CString::new(result).unwrap().into_raw()
}

#[no_mangle]
pub extern "C" fn process_line_tracking_data(input: *const c_char) -> *mut c_char {
    let c_str = unsafe { CString::from_raw(input as *mut c_char) };
    let r_str = c_str.to_str().unwrap();
    let result = format!("Processed: {}", r_str);
    CString::new(result).unwrap().into_raw()
}