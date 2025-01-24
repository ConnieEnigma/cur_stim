use u5_lib::clock::{delay_ms, delay_us};
use libm::{log10, pow};


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


pub fn capacitor_calculate(arr1: &[u16], arr2: &[f64], val1: f64, val2: f64, val3:f64) -> (f64, f64) {
    let target_imp:f64 = val1/2.0 + val2;
   // defmt::info!("target imp: {}", target_imp);
    let f1 = arr1[0] as f64;
    let f2 = arr1[1] as f64;
    let mut closest_freq1 :f64 = 1000000.0 / f1; // Initialize with the first frequency
    let mut closest_freq2 :f64 = 1000000.0 / f2; // Initialize with the second frequency
    let mut closest_imp1 = arr2[0]; // Initialize with the first impedance
    let mut closest_imp2 = arr2[1]; // Initialize with the second impedance
    let mut smallest_diff1 = (arr2[0] - target_imp); 
    let mut smallest_diff2 = (arr2[1] - target_imp); 
    if arr2[0] - target_imp < 0.0{
        let smallest_diff1 = target_imp - arr2[0];
    } else {
        let smallest_diff1 = arr2[0] - target_imp;
    }// Initialize with the difference for the first impedance
    if arr2[1] - target_imp < 0.0{
        let smallest_diff2 = target_imp - arr2[1];
    } else{
        let smallest_diff2 = arr2[1] - target_imp;
    }
     // Initialize with the difference for the second impedance
    let mut closest_i1 = 0;
    let mut closest_i2 = 1;
    if smallest_diff2 < smallest_diff1{
        let temp = smallest_diff1;
        let smallest_diff1 = smallest_diff2;
        let smallest_diff2 = temp;
    }
    
    let mut diff = 0.0;
    let mut freq1_f64 = 0.0;
    let mut freq2_f64 = 0.0;
    let len = arr2.len();
    //defmt::info!("smallest diff 1:{}", smallest_diff1);
    //defmt::info!("smallest diff 2:{}", smallest_diff2);
    for i in 2..arr2.len() {
        if (arr2[i] - target_imp) < 0.0{
            diff = target_imp - arr2[i];
        } else{
            diff = arr2[i] - target_imp;
        }
        //defmt::info!("New diff is:{}, i is:{}", diff, i);
        if diff < smallest_diff1 {
            smallest_diff2 = smallest_diff1;
            smallest_diff1 = diff;
            closest_freq2 = closest_freq1;
            closest_imp2 = closest_imp1;
            closest_imp1 = arr2[i];
            freq1_f64 = arr1[i] as f64;
            closest_freq1 = 1000000.0 / freq1_f64;
        } else if diff < smallest_diff2 {
            // defmt::info!("new smallest diff 2:{}", diff);
            smallest_diff2 = diff;
            closest_imp2 = arr2[i];
            freq2_f64 = arr1[i] as f64;
            closest_freq2 = 1000000.0 /freq2_f64;
        }
    }
    defmt::info!("closet imp1:{}, closet imp2: {}", closest_imp1, closest_imp2);
    defmt::info!("closet freq1:{}, closet freq2: {}", closest_freq1, closest_freq2);
    let base:f64 = 10.0;
    let slope:f64 = (closest_imp2 - closest_imp1) / (closest_freq2 - closest_freq1));
    let eq_para:f64 = closest_imp1 - slope * closest_freq1; 
    let fc:f64 = (target_imp - eq_para)/slope)*0.8;
    let cap = (1.0 / (2.0 * 3.1415926 * fc * val1))* 1000000.0;//The first capacitor value must be too small. Need fix
    (fc, cap)
}
