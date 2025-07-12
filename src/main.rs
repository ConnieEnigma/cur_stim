// #![feature(noop_waker)]
#![no_std]
#![no_main]
// #![feature(type_alias_impl_trait)]
// #![feature(impl_trait_in_assoc_type)]
#![allow(non_snake_case)]
#![allow(unused)]
// #![feature(new_range_api)]

use cortex_m::delay;
use defmt_rtt as _;
use embassy_executor::Spawner;
use libm;
mod utils;
// use core::{array, range};
use core::array;
//use libm::exp;
use libm::log;
//use libm::fabs;
//use numeric_sort::sort;
//use std::fs::File;
//use std::io::{self, Write};

use u5_lib::{
    clock::{self, delay_ms, delay_s, delay_us}, exti, gpio::{self, GpioPort, I2C1_SCL_PB6, I2C1_SDA_PB3, TIM1_CH2_PA9, TIM1_CH3_PA10, TIM3_CH1_PA6}, hal::I2c,  low_power::{no_deep_sleep_request, Executor}, task, tim::{Config, TIM1, TIM3}, *
};

//use tim::{Config, TIM1};

// fn i2c_init() -> (dyn I2c, I2c) {
//     // let i2c_config_plus = i2c::I2cConfig::new(1, 100_000, gpio::I2C1_SCL_PB6, gpio::I2C1_SDA_PB3);
//     // let i2c_plus = I2c::new(i2c_config_plus).unwrap();

//     // let i2c_config_minus = i2c::I2cConfig::new(2, 100_000, gpio::I2C2_SCL_PB13, gpio::I2C2_SDA_PB14);
//     // let i2c_minus = I2c::new(i2c_config_minus).unwrap();
//     let i2c_plus = I2c::new(hal::I2cFrequency::Freq100khz, I2C1_SDA_PB3, I2C1_SCL_PB6).unwrap();
//     let i2c_minus = I2c::new(hal::I2cFrequency::Freq100khz, gpio::I2C2_SDA_PB14, gpio::I2C2_SCL_PB13).unwrap();
//     (i2c_plus, i2c_minus)
// }
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

// fn i2c_send( i2c:&mut I2c, addr: u16, mut data: [u8; 2]) {
//     // let i2c_message = i2c::I2cMessage {
//     //     addr,
//     //     data:&mut data,
//     // };
//     // i2c.send(&i2c_message).unwrap();
//     i2c.write(addr, data);
// }

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
    // clock::init_clock(true, true,  16_000_000, true, clock::ClockFreqs::KernelFreq1Mhz);
    clock::init_clock(true, clock::ClockFreqs::KernelFreq1Mhz);
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

    let Ipos = 0.5;
    let Ipos_f64 = Ipos as f64;
    let Ineg = -0.5;
    let Ipos_hex = utils::cur_coding(Ipos);
    let Ineg_hex = utils::cur_coding(Ineg);

    // let (mut i2c_plus, mut i2c_minus) = i2c_init();
    let i2c_plus: u5_lib::i2c::I2c = I2c::new(hal::I2cFrequency::Freq100khz, I2C1_SDA_PB3, I2C1_SCL_PB6).unwrap();
    let i2c_minus: u5_lib::i2c::I2c = I2c::new(hal::I2cFrequency::Freq100khz, gpio::I2C2_SDA_PB14, gpio::I2C2_SCL_PB13).unwrap();
    for i in 0..4{
        // i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, Ipos_hex]);
        i2c_plus.write(POS_DAC_1_ADDR, &[DAC_REG_BASE + i, Ipos_hex]);
    }
    for i in 0..4{         
        // i2c_send(&mut i2c_plus, POS_DAC_2_ADDR, [DAC_REG_BASE + i, Ipos_hex]);
        i2c_plus.write(POS_DAC_2_ADDR, &[DAC_REG_BASE + i, Ipos_hex]);

    }
    for i in 0..4 {
        i2c_minus.write(NEG_DAC_1_ADDR, &[DAC_REG_BASE + i, Ineg_hex]);
    }
    for i in 0..4 {
        i2c_minus.write(NEG_DAC_2_ADDR, &[DAC_REG_BASE + i, Ineg_hex]);
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
    // TIM1_CH3_PA10.setup();
    // TIM1.set_pwm(3, 64000, 32000);
    // TIM1.enable_output(3);
    // delay_s(1);
    // for i in 0..1000{
    //     let res = tmp_adc.start_conversion_sw(5);
    //     let vpos = res as f64 * vref;
    //     if adc_max < vpos {
    //         adc_max = vpos;
    //     }
    //     if adc_min > vpos {
    //         adc_min = vpos;
        
    //     }
    // }
    // //     defmt::info!("Vmax is {}, Vmin is {}, Difference is {}", adc_max, adc_min, adc_max- adc_min);
    // //     delay_s(1);
    
    let Ipos_f64:f64 = Ipos as f64;
    let Ineg_f64:f64 = -Ineg as f64; 
    // let R_total = (adc_max - adc_min)* 1000.0 / Ineg_f64 ; 
    let R_total = 1143.496047;
    // defmt::info!("Rtotal is: {}", R_total);
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
    
    let mut t1 = 1000; //unit is us
    let mut t2 = 1200; //unit is us 
    let mut tauRC = 0.0;

    let mut Rs = 200.0; //unit is ohm
    let mut Rp = 1000.0; //unit is ohm
    let mut Cp = 1.0; //unit is uF
    let mut V0 = 0.0;
    let mut I0 = 0.0;
    loop {
        s2.set_high();
        delay_ms(10);
        s2.set_low();
        delay_ms(10);
        s1.set_high();
        delay_ms(10);
        s1.set_low();
        delay_ms(60);
        i = 0;
        // s2.set_high();
        // s1.set_high();
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
        


        if counter >= 10000{
            if i < 32 {
                TIM1.enable_output(3);
                TIM1_CH3_PA10.setup();
                //TIM1.set_pwm(3, 5000, 2500);
                TIM1.set_pwm(3, frequency_divider[i], dutycycle_divider[i]);
                delay_ms(100);
                for q in 0..100{
                    let res1 = tmp_adc.start_conversion_sw(5); 
                    let vpos1 = res1 as f64 * vref;

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

                array_impedance[i] = (adc_sum_max - adc_sum_min) * 1000.0 / Ineg_f64; 
                if i == array_impedance.len() - 1 {
                    // if array_impedance[31] > array_impedance[30]{
                    //     i = i - 1;
                    // } else{
                    i = i + 1;
                    // array_impedance =[758.1608515, 760.2084592, 758.9798946, 759.5259233, 760.6179807, 
                    // 755.5672152, 755.430708, 753.6561148, 752.1545358, 754.7481722, 753.6561148, 
                    // 750.3799426, 747.1037703, 749.6974067, 744.510134, 734.6816174, 676.9390822, 
                    // 629.9806139, 588.4824327, 508.2162136, 462.8958314, 421.9436788, 391.0930572, 
                    // 367.7503302, 320.6553547, 313.1474601, 293.626934, 282.5698528, 274.7889439, 
                    // 272.4683219, 265.3699488, 262.5032981]; //2.2uF

                    array_impedance = [754.3719372, 755.3268383, 754.917595, 754.7811805, 754.6447661, 
                    753.5534504, 753.2806215, 753.1442071, 751.3708191, 747.6876288, 747.141971, 
                    738.138617, 727.6347039, 706.7632923, 673.7509941, 628.1885661, 544.4300906, 
                    490.9556242, 449.2128009, 377.4587975, 347.8568607, 329.8501526, 312.1162735, 
                    291.1591362, 268.3272333, 268.1908189, 265.8717731, 260.9608528, 258.9146359, 
                    258.5053926, 256.5955902, 248.137894]; // 4.7uF


                        // array_impedance = [1143.496047, 1062.312077, 1022.14546, 932.2156719, 891.3296168, 
                        // 861.4365558, 819.31194, 789.8296346, 715.9714989, 650.4451377, 627.1140433, 540.0124619, 
                        // 514.1121927, 482.7363788, 451.7707062, 433.7244864, 368.1018691, 357.2331231, 335.9057725, 
                        // 287.3040215, 279.7164064, 279.5113357, 253.8775008, 256.5434196, 244.2391789, 243.6239668, 
                        // 238.9073412, 249.1608752, 240.342836, 253.6724301, 248.1355218, 239.9326946];
                        // let R_total = 1149.410835;

                        for p in 0..array_impedance.len(){
                            array_impedance[p] = array_impedance[p] - 25.0;
                        } 

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

                        for l in 0..array_impedance.len(){
                            let fix_para = 6.28*Rp*Cp/1000000.0;
                            let mut freq_div_64 = frequency_divider[l] as f64;
                            let freq_64 = 100000.0 / freq_div_64;  
                            let imp_image = fix_para*Rp*freq_64/(1.0 + (fix_para*freq_64)*(fix_para*freq_64));
                            let imp_real = libm::sqrt(array_impedance[l]*array_impedance[l] - imp_image*imp_image);
                            array_impedance_fix[l] = imp_real;
                        }
                        delay_ms(100);  
                        //defmt::info!("Fixed impedance value is {}", array_impedance_fix);
                        //delay_s(1);
                        //defmt::info!("Fixed impedance value is {}", array_impedance_fix);
                        //delay_s(1);
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
                        ///calculate the pulse length according to impedance/////////
                        /////////////////////////////////////////////////////////////
                        let t1_f64 = t1 as f64;
                        defmt::info!("Rs={}, Rp={}, Cp={}", Rs, Rp, Cp);
                        delay_s(1);
                        tauRC =  (Rs * Rp * Cp*1e-6)/(Rs + Rp); //verify it again
                        defmt::info!("Tau is {}", tauRC);
                        let V0 :f64 = Ipos_f64 * t1_f64 * 1e-2; //V0太小了，为啥
                        defmt::info!("V0 is {}", V0);
                        let I0 = V0 / (Rs + Rp);
                        defmt::info!("I0 is {}", I0);
                        let base :f64 = 1.0 - Ineg_f64* 1e-3* t1_f64 *1e-6 / (I0 * tauRC);
                        defmt::info!("Base is {}", base);
                        let t2_f64 = -tauRC * log(base)*1000000.0;
                        t2 = t2_f64 as u32;
                        defmt::info!("t2 is {}", t2);
                //    } //The calculation of capacitor is not correct. Need some fix.
                // } else if i == 1{
                //     i = i + 1;
                // } else if array_impedance[i] < (array_impedance[i-1]+3.0){
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
        } else{
            counter += 1;
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