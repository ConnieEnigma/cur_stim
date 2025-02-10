use u5_lib::clock::{delay_ms, delay_us};
use libm::{log10, pow};
struct Point {
    x: f64,
    y: f64,
}

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
    let f3 = arr1[2] as f64;
    let mut closest_freq1 :f64 = 100000.0 / f1; // Initialize with the first frequency
    let mut closest_freq2 :f64 = 100000.0 / f2; // Initialize with the second frequency
    let mut closest_freq3 :f64 = 100000.0 / f3; // Initialize with the third frequency
    let mut closest_imp1 = arr2[0]; // Initialize with the first impedance
    let mut closest_imp2 = arr2[1]; // Initialize with the second impedance
    let mut closest_imp3 = arr2[2]; // Initialize with the second impedance
    let mut smallest_diff1 = (arr2[0] - target_imp); 
    let mut smallest_diff2 = (arr2[1] - target_imp); 
    let mut smallest_diff3 = (arr2[2] - target_imp);
    let mut temp = 0.0;
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
    if arr2[2] - target_imp < 0.0{
        let smallest_diff3 = target_imp - arr2[2];
    } else{ 
        let smallest_diff3 = arr2[2] - target_imp;
    }
     // Initialize with the difference for the second impedance
    if smallest_diff3 < smallest_diff1{
        temp = smallest_diff1;
        smallest_diff1 = smallest_diff3;
        smallest_diff3 = temp;
    } else if smallest_diff3 < smallest_diff2{
        temp = smallest_diff2;
        smallest_diff2 = smallest_diff3;
        smallest_diff3 = temp;
    } else if smallest_diff2 < smallest_diff1{
        temp = smallest_diff1;
        smallest_diff1 = smallest_diff2;
        smallest_diff2 = temp;
    }
    
    let mut diff = 0.0;
    let mut freq1_f64 = 0.0;
    let mut freq2_f64 = 0.0;
    let mut freq3_f64 = 0.0;
    let len = arr2.len();
    //defmt::info!("smallest diff 1:{}", smallest_diff1);
    //defmt::info!("smallest diff 2:{}", smallest_diff2);
    for i in 3..arr2.len() {
        if (arr2[i] - target_imp) < 0.0{
            diff = target_imp - arr2[i];
        } else{
            diff = arr2[i] - target_imp;
        }
        //defmt::info!("New diff is:{}, i is:{}", diff, i);
        if diff < smallest_diff1 {
            smallest_diff3 = smallest_diff2;
            smallest_diff2 = smallest_diff1;
            smallest_diff1 = diff;
            closest_freq3 = closest_freq2;
            closest_freq2 = closest_freq1;
            freq1_f64 = arr1[i] as f64;
            closest_freq1 = 100000.0 / freq1_f64;
            closest_imp3 = closest_imp2;
            closest_imp2 = closest_imp1;
            closest_imp1 = arr2[i];
        } else if diff < smallest_diff2 {
            // defmt::info!("new smallest diff 2:{}", diff);
            smallest_diff3 = smallest_diff2;
            smallest_diff2 = diff;
            closest_imp3 = closest_imp2;
            closest_imp2 = arr2[i];
            closest_freq3 = closest_freq2;
            freq2_f64 = arr1[i] as f64;
            closest_freq2 = 100000.0 /freq2_f64;
        } else if diff < smallest_diff3 {
            // defmt::info!("new smallest diff 3:{}", diff);
            smallest_diff3 = diff;
            closest_imp3 = arr2[i];
            freq3_f64 = arr1[i] as f64;
            closest_freq3 = 100000.0 / freq3_f64;
        }
    }
    defmt::info!("closest imp1:{}, closest imp2: {}, closest imp3:{}", 
    closest_imp1, closest_imp2, closest_imp3);
    defmt::info!("closest freq1:{}, closest freq2: {}, closest freq3:{}", 
    closest_freq1, closest_freq2, closest_freq3);

    let points = [
        Point { x: closest_freq1, y: closest_imp1 },
        Point { x: closest_freq2, y: closest_imp2 },
        Point { x: closest_freq3, y: closest_imp3 },
    ];

    let sum_x: f64 = points.iter().map(|p| p.x).sum();
    let sum_y: f64 = points.iter().map(|p| p.y).sum();
    let sum_xx: f64 = points.iter().map(|p| p.x * p.x).sum();
    let sum_xy: f64 = points.iter().map(|p| p.x * p.y).sum();

    let slope: f64 = (3.0 * sum_xy - sum_x * sum_y) / (3.0 * sum_xx - sum_x * sum_x);
    let intercept = (sum_y - slope * sum_x) / 3.0;

    let fc:f64 = ((target_imp - intercept)/slope)* 0.9;
    let cap = (1.0 / (2.0 * 3.1415926 * fc * val1))* 1000000.0;//The first capacitor value must be too small. Need fix
    
    (fc, cap)
}
