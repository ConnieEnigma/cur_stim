#![feature(noop_waker)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use cortex_m::delay;
use defmt_rtt as _;
use embassy_executor::Spawner;
use libm;
mod utils;
use core::array;
use libm::exp;
use libm::fabs;
//use numeric_sort::sort;
//use std::fs::File;
//use std::io::{self, Write};

use u5_lib::{
    clock::{self, delay_ms, delay_s, delay_us},
    com_interface::ComInterface,
    exti,
    gpio::{self, GpioPort,  TIM1_CH2_PA9, TIM1_CH3_PA10, TIM3_CH1_PA6},
    i2c::{self, I2c},
    low_power::{no_deep_sleep_request, Executor},
    task,
    tim::{Config, TIM1, TIM3},
    *,
};

//use tim::{Config, TIM1};
fn bubble_sort<T: Ord>(arr: &mut [T]) {
    let len = arr.len();
    for i in 0..len {
        for j in 0..len - i - 1 {
            if arr[j] < arr[j + 1] {
                arr.swap(j, j + 1);
            }
        }
    }
}

fn i2c_init() -> (I2c, I2c) {
    let i2c_config_plus = i2c::I2cConfig::new(1, 100_000, gpio::I2C1_SCL_PB6, gpio::I2C1_SDA_PB3);
    let i2c_plus = I2c::new(i2c_config_plus).unwrap();
    let i2c_config_minus = i2c::I2cConfig::new(2, 100_000, gpio::I2C2_SCL_PB13, gpio::I2C2_SDA_PB14);
    let i2c_minus = I2c::new(i2c_config_minus).unwrap();
    (i2c_plus, i2c_minus)
}
fn switch_led_setup() -> ( gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort,
    gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort){
    let red: gpio::GpioPort = gpio::PB7;
    let green: gpio::GpioPort = gpio::PB8;
    let s0: gpio::GpioPort = gpio::PB15;
    let s1: gpio::GpioPort = gpio::PA9;
    let s2: gpio::GpioPort = gpio::PA10;
    let s3: gpio::GpioPort = gpio::PB4;
    let s4: gpio::GpioPort = gpio::PB5;
    let s5: gpio::GpioPort = gpio::PA5;
    let s6: gpio::GpioPort = gpio::PA6;
    let s7: gpio::GpioPort = gpio::PA4;
    let s8: gpio::GpioPort = gpio::PA3;
    let chopper_input2: gpio::GpioPort = gpio::PA7;
    
    
    green.setup();
    red.setup();
    s0.setup();
    s1.setup();
    s2.setup();
    s3.setup();
    s4.setup();
    s5.setup();
    s6.setup();
    s7.setup();
    s8.setup();
    chopper_input2.setup();
    (green, red, s0, s1, s2, s3, s4, s5, s6, s7, s8, chopper_input2)

}

fn i2c_send( i2c:&mut I2c, addr: u16, mut data: [u8; 2]) {
    let i2c_message = i2c::I2cMessage {
        addr,
        data:&mut data,
    };
    i2c.send(&i2c_message).unwrap();
}

struct Point {
    x: f64,
    y: f64,
}
const POS_DAC_1_ADDR: u16  = 0x20;
const POS_DAC_2_ADDR: u16  = 0x60;
const NEG_DAC_1_ADDR: u16  = 0xA0;
const NEG_DAC_2_ADDR: u16  = 0xE0;
const DAC_REG_BASE: u8 = 0xF8;

#[task]
async fn async_main(spawner: Spawner) {
    // be careful, if the dbg is not enabled, but using deep sleep. This framework will not able to connect to chip.
    // stm32cube programmer, stmcubeide can be used to program the chip, then this framework can be used to debug.
    clock::init_clock(true, true,  16_000_000, true, clock::ClockFreqs::KernelFreq1Mhz);
    unsafe {
        no_deep_sleep_request();
    }
    // TIM1_CH2_PA9.setup(); //s1 
    // TIM1_CH3_PA10.setup(); // s2 
    // TIM3_CH1_PA6.setup(); // s6. chopper frequency. 
    let mut tim1_config = Config::default();
    tim1_config.prescaler = 10 - 1;
    let _ = TIM1.init(tim1_config);
    
    // let _ = TIM1.init(Config::default());
    // let _ = TIM3.init(Config::default());
    //TIM1.set_pwm(2, 500, 250);   
    //TIM1.set_pwm(3, 16000, 4000);
    // TIM3.set_pwm(1, 10, 5);
    //TIM1.enable_output(2);
    //TIM1.enable_output(3);
    // TIM3.enable_output(1);
    clock::set_mco(
        gpio::GPIO_MCO_PA8,
        clock::Mcosel::HSE,
        clock::Mcopre::DIV16,
    ); //filter cut off clock. which use PA8 as clock output

    defmt::info!("setup led finished!");

    let (green, red, s0,s1, s2, s3, s4, s5, s6, s7, s8, chopper_input2) = switch_led_setup();
    s0.set_low(); // capacitor connection
    s1.set_low(); //pos DAC connection
    s2.set_low(); //neg DAC connection
    s3.set_high(); // pos DAC & capacitor -> Vstm 
    s4.set_high(); //neg DAC power
    s5.set_high(); //pos DAC power 
    s6.set_low(); //chopper switch
    s7.set_low(); // chopper and filter power
    s8.set_low(); // 1.5v DC connection
    chopper_input2.set_low();

    let Ipos = 1.0;
    let Ipos_f64 = Ipos as f64;
    let Ineg = -0.7;
    let Ipos_hex = utils::cur_coding(Ipos);
    let Ineg_hex = utils::cur_coding(Ineg);

    let (mut i2c_plus, mut i2c_minus) = i2c_init();
    for i in 0..4{
        i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, Ipos_hex]);
    }
    for i in 0..4{         
        i2c_send(&mut i2c_plus, POS_DAC_2_ADDR, [DAC_REG_BASE + i, Ipos_hex]);
    }
    for i in 0..4 {
        i2c_send(&mut i2c_minus, NEG_DAC_1_ADDR, [DAC_REG_BASE + i, Ineg_hex]);
    }
    for i in 0..4 {
        i2c_send(&mut i2c_minus, NEG_DAC_2_ADDR, [DAC_REG_BASE + i, Ineg_hex]);
    }
    defmt::info!("i2c finished!");


    let tmp_adc = adc::ADC1;
    tmp_adc.init();
    let adc_pin = gpio::ADC1_IN5_PA0;
    adc_pin.setup();
    let mut counter = 0;
    defmt::info!("ADC init!");
    delay_ms(100);
    let vref_data = tmp_adc.start_conversion_sw(0);
    delay_ms(100);
    let vref_data = tmp_adc.start_conversion_sw(0);
    //defmt::info!("Vref_data init! {}", vref_data);
    let vref_raw =  tmp_adc.get_vref_int_raw();
    //defmt::info!("Vref_raw init! {}", vref_raw);
    let vref = 3.0 * vref_raw as f64 / vref_data as f64;
    //defmt::info!("Vref init! {}", vref);
    let vref = vref / 16384.0;
    defmt::info!("Vref init! {}", vref);
    delay_s(1);

    let mut adc_sum_min: f64 = 5.0;
    let mut adc_sum_max: f64 = 0.0;

    let mut adc_min: f64 = 5.0;
    let mut adc_max: f64 = 0.0;
    let mut abnormal_counter = 0; 


    
    // measure Rp before stimulation begin
    // TIM1_CH2_PA9.setup();
    // TIM1.set_pwm(2, 64000, 32000); 
    // TIM1.enable_output(2);
    TIM1_CH3_PA10.setup();
    TIM1.set_pwm(3, 64000, 32000);
    TIM1.enable_output(3);
    delay_s(1);
    for i in 0..1000{
        let res = tmp_adc.start_conversion_sw(5);
        let vpos = res as f64 * vref;
        if adc_max < vpos {
            adc_max = vpos;
        }
        if adc_min > vpos {
            adc_min = vpos;
        
        }
    }
    //     defmt::info!("Vmax is {}, Vmin is {}, Difference is {}", adc_max, adc_min, adc_max- adc_min);
    //     delay_s(1);
    
    let Ipos_f64:f64 = Ipos as f64;
    let Ineg_f64:f64 = -Ineg as f64;
    // //let R_total = (adc_max - adc_min)* 1000.0 / Ipos_f64 ; 
    let R_total = (adc_max - adc_min)* 1000.0 / Ineg_f64 ; 
    //let R_total = 1143.496047;
    defmt::info!("Rtotal is: {}", R_total);
    TIM1.disable_output(3);

    let mut j = 0;
    let mut i = 1;
    let mut array_impedance:[f64; 32] = [R_total, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let mut array_impedance_fix:[f64; 32] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let frequency_divider:[u16; 32] = [64000, 50000, 40000, 25000, 20000, 15873, 12500, 10000, //1.5, 2, 2.5, 4, 5, 6.3, 8, 10
    6400, 5000, 4000, 2500, 2000, 1587, 1250, 1000, //15, 20, 25, 40, 50, 63, 80, 100
    640, 500, 400, 250, 200, 159, 125, 100, //150, 200, 250, 400, 500, 630, 800, 1000
    64, 50, 40, 25, 20, 16, 13, 10]; //1500, 2000, 2500, 4000, 5000, 6300, 8000, 10000
    let dutycycle_divider:[u16; 32] = [32000, 25000, 20000, 12500, 10000, 7936, 6250, 5000, 
    3200, 2500, 2000, 1250, 1000, 793, 625, 500, 
    320, 250, 200, 125, 100, 79, 63, 50,
    32, 25, 20, 13, 10, 8, 6, 5];
    
    let mut t1 = 100; //unit is us
    let mut t2 = 500; //unit is us 
    let mut tauRC = 0.0;

    let mut Rs = 200.0; //unit is ohm
    let mut Rp = 500.0; //unit is ohm
    let mut Cp = 1.0; //unit is uF
    let mut V0 = 0.0;
    let mut I0 = 0.0;
    loop {
        // s2.set_high();
        // delay_ms(100);
        // s2.set_low();
        // delay_us(100);
        // s1.set_high();
        // delay_us(200);
        // s1.set_low();
        // delay_us(600);
        
        // s1.set_high();
        // s2.set_high();
        // delay_us(t1);
        // s2.set_low();
        // s1.set_low();
        // delay_us(100);
        // s3.set_high();
        // delay_us(t2);
        // s3.set_low();
        // delay_us(300);

        // s4.set_low();
        // s5.set_low();
        // delay_ms(8);
        // s4.set_high();
        // s5.set_high();
        // delay_us(500);

        // for i in 0..4{
        //     i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, Ipos_hex]);
        // }
        // for i in 0..4{         
        //     i2c_send(&mut i2c_plus, POS_DAC_2_ADDR, [DAC_REG_BASE + i, Ipos_hex]);
        // }
        // for i in 0..4 {
        //     i2c_send(&mut i2c_minus, NEG_DAC_1_ADDR, [DAC_REG_BASE + i, Ineg_hex]);
        // }
        // for i in 0..4 {
        //     i2c_send(&mut i2c_minus, NEG_DAC_2_ADDR, [DAC_REG_BASE + i, Ineg_hex]);
        // }
        
        //TIM1.enable_output(2);        
        counter += 1;


        if counter >= 10{
            if i < 32 {
                TIM1.enable_output(3);
                TIM1_CH3_PA10.setup();
                TIM1.set_pwm(3, frequency_divider[i], dutycycle_divider[i]);
                delay_ms(500);
                for q in 0..1000{
                    let res1 = tmp_adc.start_conversion_sw(5); 
                    let vpos1 = res1 as f64 * vref;

                    //defmt::info!("ADC value is {}", vpos1);
                    if vpos1 > adc_sum_max {
                        adc_sum_max = vpos1;
                    //    defmt::info!("max value change");
                    //    defmt::info!("adc max value is {}", adc_sum_max);
                    }
                    if vpos1 < adc_sum_min {
                        adc_sum_min = vpos1;
                    //    defmt::info!("min value change");
                    //    defmt::info!("adc min value is {}", adc_sum_min);
                    }
                }
                //defmt::info!("ADC difference is {} - {} = {}", adc_sum_max, adc_sum_min, adc_sum_max - adc_sum_min);
                let R_measure = (adc_sum_max - adc_sum_min) * 1000.0 / Ineg_f64;
                defmt::info!("Impedance is {}", R_measure);
                j = j + 1 ;

                j = 0;

                array_impedance[i] = (adc_sum_max - adc_sum_min) * 1000.0 / Ineg_f64; 
                if i == array_impedance.len() - 1 {
                //     if array_impedance[31] > array_impedance[30]{
                //         i = i - 1;
                //     } else{
                        array_impedance = [923.1134684951807, 934.5679467870392, 916.0645587771136, 914.889740490769, 895.211534194499, 
                        874.3585096118842, 877.2955553277457, 874.9459187550565, 877.8829644709176, 879.9388964720205, 881.1137147583651, 
                        875.5333278982288, 851.1558484565803, 834.4146878761719, 790.6527067098395, 743.65997525606, 654.9611946370513, 
                        585.3532111711405, 560.9757317294924, 522.2067282801243, 494.5984985510287, 452.0113356710412, 434.0953568042881, 
                        408.5430590762954, 404.43119507408977, 407.07453621836487, 389.74596649478343, 388.2774436368528, 399.4382173571255, 
                        399.7319219287115, 400.0256265002978, 398.85080821395326];
                        // array_impedance = [739.4932168831016, 735.4932168831016, 735.6396711278014, 733.0034947232025, 724.509148530606, 
                        // 712.7928089546107, 712.2069919758111, 677.9366987160249, 632.3894286143434, 635.1720592636422, 
                        // 556.2332213703743, 541.1484341662804, 469.2394000186095, 385.3211178055436, 343.28874957666073, 
                        // 282.94960076028514, 254.39102304379688, 260.54210132119425, 254.2445687990968, 233.00870331760538, 
                        // 227.15053352960769, 207.96502747391563];  //1 uF

                        // array_impedance = [739.4336928110467, 735.4932168831016, 736.5144914971461, 732.135689526295, 722.6482852561177, 
                        // 719.729083942217, 718.7073634823518, 703.8194367814583, 603.3989115832742, 523.8506757794802, 
                        // 435.98271623106933, 431.6039142602183, 351.0339579965589, 340.67079333221153, 331.1833890620341, 
                        // 270.6099617985947, 267.98268061608394, 253.5326341122757, 252.3649535867154, 250.32151266698472, 
                        // 243.3154295136233, 237.91490708290686];  // 2.2 uF

                        // array_impedance = [735.4932168831016, 729.27328210865, 726.352099324318, 722.846679983118, 702.982637049656, 
                        // 678.590760800478, 646.603809312036, 554.148374187907, 409.695885502656, 396.696622112376, 
                        // 334.47542880609, 302.342418178431, 286.714090282251, 273.130590335104, 270.063348411555, 
                        // 255.603493629108, 247.27812269376, 245.08723560551, 242.604230238828, 239.536988315278, 
                        // 237.200042087812, 230.33526254463]; // 4.7uF
                        

                        // array_impedance = [1143.496047, 1062.312077, 1022.14546, 932.2156719, 891.3296168, 
                        // 861.4365558, 819.31194, 789.8296346, 715.9714989, 650.4451377, 627.1140433, 540.0124619, 
                        // 514.1121927, 482.7363788, 451.7707062, 433.7244864, 368.1018691, 357.2331231, 335.9057725, 
                        // 287.3040215, 279.7164064, 279.5113357, 253.8775008, 256.5434196, 244.2391789, 243.6239668, 
                        // 238.9073412, 249.1608752, 240.342836, 253.6724301, 248.1355218, 239.9326946];
                        // let R_total = 1149.410835;

                        defmt::info!("Final value is {}", array_impedance);
                        delay_s(1);
                        defmt::info!("Final value is {}", array_impedance);
                        delay_s(1);
                        Rs = (array_impedance[29] + array_impedance[30] + array_impedance[31])/3.0;
                        defmt::info!("Rs is {}", Rs);
                        Rp = R_total - Rs;
                        defmt::info!("Rp is {}", Rp);
                        let mut target_imp = Rp/2.0 + Rs;
                        defmt::info!("Target impedance is {}", target_imp);
                        delay_s(3);

                        let (fc, Cp) = utils::capacitor_calculate(&frequency_divider, &array_impedance, Rp, Rs, 1.0);
                        defmt::info!("fc is {:?} Hz", fc);
                        defmt::info!("Cp is {:?} uF", Cp);
                        delay_s(3);
                        ///////////////////////////////////
                        // Change the value to reduce error
                        ///////////////////////////////////

                        for i in 0..array_impedance.len(){
                            let fix_para = 6.28*Rp*Cp/1000000.0;
                            let mut freq_div_64 = frequency_divider[i] as f64;
                            let freq_64 = 100000.0 / freq_div_64;  
                            let imp_image = fix_para*Rp*freq_64/(1.0 + (fix_para*freq_64)*(fix_para*freq_64));
                            let imp_real = libm::sqrt(array_impedance[i]*array_impedance[i] - imp_image*imp_image);
                            array_impedance_fix[i] = imp_real;
                        }
                        delay_ms(100);  
                        defmt::info!("Fixed impedance value is {}", array_impedance_fix);
                        delay_s(1);
                        defmt::info!("Fixed impedance value is {}", array_impedance_fix);
                        delay_s(1);
                        // Need to find the target frequency again. Probably change. 
                        Rs = (array_impedance_fix[29] + array_impedance_fix[30] + array_impedance_fix[31])/3.0;
                        defmt::info!("Fixed Rs is {}", Rs);
                        Rp = R_total - Rs;
                        defmt::info!("Fixed Rp is {}", Rp);
                        target_imp = Rp/2.0 + Rs;
                        defmt::info!("Fixed target impedance is {}", target_imp);

                        let (fc, Cp) = utils::capacitor_calculate(&frequency_divider, &array_impedance_fix, Rp, Rs, 0.0);
                        
                        defmt::info!("Fixed fc is {:?} Hz", fc);
                        defmt::info!("Fixed Cp is {:?} uF", Cp);
                        delay_s(5);

                        /////////////////////////////////////////////////////////////
                        // Change the value to reduce error twice for better result//
                        /////////////////////////////////////////////////////////////
                        for _ in 0..1 {
                            for k in 0..array_impedance.len(){
                                let fix_para = 6.28*Rp*Cp/1000000.0;
                                let mut freq_div_64 = frequency_divider[k] as f64;
                                let freq_64 = 100000.0 / freq_div_64;  
                                let imp_image = fix_para*Rp*freq_64/(1.0 + (fix_para*freq_64)*(fix_para*freq_64));
                                let imp_real = libm::sqrt(array_impedance_fix[k]*array_impedance_fix[k] - imp_image*imp_image);
                                array_impedance_fix[k] = imp_real;
                            }  
                            let n = array_impedance.len();
                            for a in 0..n {
                                for b in 0..n - 1 - a {
                                    if array_impedance_fix[b] < array_impedance_fix[b + 1] {
                                    array_impedance_fix.swap(b, b + 1);
                                    }
                                }
                            }
                            delay_ms(100);
                            defmt::info!("Fixed impedance value is {}", array_impedance_fix);
                            delay_s(1);
                            defmt::info!("Fixed impedance value is {}", array_impedance_fix);
                            delay_s(1);
                            // defmt::info!("Fixed impedance value is {}", array_impedance_fix);
                            // Need to find the target frequency again. Probably change. 
                            Rs = (array_impedance_fix[29] + array_impedance_fix[30] + array_impedance_fix[31])/3.0;
                            defmt::info!("Fixed Rs is {}", Rs);
                            Rp = R_total - Rs;
                            defmt::info!("Fixed Rp is {}", Rp);
                            target_imp = Rp/2.0 + Rs;
                            defmt::info!("Fixed target impedance is {}", target_imp);

                            let (fc, Cp) = utils::capacitor_calculate(&frequency_divider, &array_impedance_fix, Rp, Rs, 0.0);
                            
                            defmt::info!("Fixed fc is {:?} Hz", fc);
                            defmt::info!("Fixed Cp is {:?} uF", Cp);
                            delay_s(5); 
                        }
                        /////////////////////////////////////////////////////////////
                        /// calculate the pulse length according to impedance////////
                        /////////////////////////////////////////////////////////////
                        let t1_f64 = t1 as f64;
                        defmt::info!("Rs={}, Rp={}, Cp={}", Rs, Rp, Cp);
                        delay_s(1);
                        tauRC =  (Rs * Rp * Cp*1e-6)/(Rs + Rp); //verify it again
                        defmt::info!("Tau is {}", tauRC);
                        let V0 :f64 = Ipos_f64*1e-3 * t1_f64 / Cp; //V0太小了，为啥
                        defmt::info!("V0 is {}", V0);
                        let I0 = V0 / (Rs + Rp);
                        defmt::info!("I0 is {}", I0);
                        let base :f64 = 1.0 - Ineg_f64* 1e-3* t1_f64 *1e-6 / (I0 * tauRC);
                        defmt::info!("Base is {}", base);
                        let t2_f64 = -tauRC * exp(base)*1000000.0;
                        t2 = t2_f64 as u32;
                        defmt::info!("t2 is {}", t2);
                //    } //The calculation of capacitor is not correct. Need some fix.
                // } else if i == 1{
                //     i = i + 1;
                // } else if array_impedance[i] < (array_impedance[i-1]+ i as f64/2.0){
                //     i = i + 1;
                // } else {
                //     abnormal_counter = abnormal_counter + 1;
                //     if abnormal_counter > 3 {
                //         defmt::info!("Abnormal impedance measurement. Frequency is {}", i);
                //         defmt::info!("Present is {}", array_impedance[i]);
                //         defmt::info!("Former is {}", array_impedance[i-1]);
                //         i = i - 1;
                //         abnormal_counter = 0;
                //         delay_s(3);
                //     }
                // }
                } else {
                    i = i + 1;
                }
            } 
            adc_sum_max = 0.0;
            adc_sum_min = 3.0;
            counter = 0;  
        }
        TIM1.disable_output(3);
        green.toggle();
        // red.toggle();
        delay_ms(4);
    }
}


#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        spawner.spawn(async_main(spawner)).unwrap();
    });
}