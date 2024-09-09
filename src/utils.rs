use u5_lib::clock::{delay_ms, delay_us};

/// the input is `val` mA
pub fn cur_coding(val: f32) -> u8 {
    // 800 uA  each bit
    // let ret = abs(val) / 1.6 * 127.0;
    let tmpv = if val > 0.0 { val } else { -val };
    let ret = tmpv / 1.6 * 127.0;
    let mut ret: u8 = (ret + 0.5) as u8;
    if val > 0.0 {
        ret |= 1 << 7;
    }
    return ret;
}
