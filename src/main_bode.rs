#![feature(noop_waker)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use embassy_executor::Spawner;
use libm;
mod utils;
use core::array;
use libm::exp;
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
    //TIM1_CH2_PA9.setup(); //s1 
    TIM1_CH3_PA10.setup(); // s2 
    // TIM3_CH1_PA6.setup(); // s6. chopper frequency. 
    let _ = TIM1.init(Config::default());
    // let _ = TIM3.init(Config::default());
    //TIM1.set_pwm(2, 16000, 1600);   . 
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
    let Ineg = -1.0;
    let Ipos_hex = utils::cur_coding(Ipos);
    let Ineg_hex = utils::cur_coding(Ineg);

    let (mut i2c_plus, mut i2c_minus) = i2c_init();
    for i in 0..4{
        //i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, 0xA8]);
        i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, Ipos_hex]);
    }
    for i in 0..4{     
        //i2c_send(&mut i2c_plus, POS_DAC_2_ADDR, [DAC_REG_BASE + i, 0xA8]);     
        i2c_send(&mut i2c_plus, POS_DAC_2_ADDR, [DAC_REG_BASE + i, Ipos_hex]);
    }
    for i in 0..4 {
        i2c_send(&mut i2c_minus, NEG_DAC_1_ADDR, [DAC_REG_BASE + i, Ineg_hex]);
        //i2c_send(&mut i2c_minus, NEG_DAC_1_ADDR, [DAC_REG_BASE + i, 50]);
    }
    for i in 0..4 {
        i2c_send(&mut i2c_minus, NEG_DAC_2_ADDR, [DAC_REG_BASE + i, Ineg_hex]);
        //i2c_send(&mut i2c_minus, NEG_DAC_2_ADDR, [DAC_REG_BASE + i, 50]);
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
    // let mut adc_values = [0.0; 5];
    // let mut adc_indexer = 0;
    // let mut adc_sum = 0.0;
    // let mut adc_values1 = [0.0; 5];
    // let mut adc_indexer1 = 0;
    // let mut adc_sum1 = 0.0;
    // let mut vbase = 0.0;
    // let mut vbase_sum = 0.0;
    // let mut vbase_avg: f64 = 0.0;

    // let mut adc_sum_min: f64 = 3.0;
    // let mut adc_sum_max: f64 = 0.0;

    // let mut adc_min: f64 = 3.0;
    // let mut adc_max: f64 = 0.0;

    //let mut extra_count = 0;

    
    // // measure Rp before stimulation begin
    // //TIM1_CH2_PA9.setup();
    // //TIM1.set_pwm(2, 64000, 32000); // 64000, 62.5Hz; 40000, 100Hz; 20000, 200Hz; 8000, 500Hz; 4000, 1kHz;
    // //TIM1.enable_output(2);
    // TIM1_CH3_PA10.setup();
    // TIM1.set_pwm(3, 64000, 32000);
    // TIM1.enable_output(3);
    // for i in 0..9{
    //     let res = tmp_adc.start_conversion_sw(5);
    //     let vpos = res as f64 * vref;
    //     //adc_sum -= adc_values[adc_indexer]; 
    //     //adc_values[adc_indexer] = vpos; 
    //     //adc_sum += vpos;
    //     if adc_max < vpos {
    //         adc_max = vpos;
    //     }
    //     if adc_min > vpos {
    //         adc_min = vpos;
        
    //     }
    //     defmt::info!("Vmax is {}, Vmin is {}, Difference is {}", adc_max, adc_min, adc_max- adc_min);
    //     // defmt::info!("Vtotal is {}", (adc_sum - 0.3)/5.0);
    //     delay_ms(500);
    // //     //adc_indexer += 1;
    // //     //adc_indexer %= 5;
    // }
    
    // let Ipos_f64:f64 = Ipos as f64;
    // let Ineg_f64:f64 = -Ineg as f64;
    // //let R_total = (adc_max - adc_min)* 1000.0 * 2.0 / Ipos_f64 ; 
    // let R_total = (adc_max - adc_min)* 1000.0 / Ineg_f64 ; 
    // defmt::info!("Rtotal is: {}", R_total);
    // // s1.setup();
    // //let R_total = 710.0;
    // let mut j = 0;
    // let mut i = 0;
    // let mut array_impedance:[f64; 15] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    // let frequency_divider:[u16; 15] = [50000, 40000, 20000, 12500, 10000, 5000, 4000, 2000, 1250, 1000, 500, 400, 200, 125, 100]; //20, 25, 50, 80, 100
    // //let dutycycle_divider:[u16; 15] = [20000, 16000, 8000, 5000, 4000, 2000, 1600, 800, 500, 400, 200, 160, 80, 50, 40];
    // let dutycycle_divider:[u16; 15] = [25000, 20000, 10000, 6250, 5000, 2500, 2000, 1000, 625, 500, 250, 200, 100, 63, 50];
    // //let frequency_divider:[u16; 12] = [53333, 40000, 20000, 16000, 8000, 5333, 4000, 2000, 1600, 800, 533, 400];
    // //let dutycycle_divider:[u16; 12] = [26667, 20000, 10000, 8000, 4000, 2667, 2000, 1000, 800, 400, 267, 200];
    // //let file_name = "output.txt";
    // //let mut file = File::create(file_name).expect("Failed to create file");

    loop {
        s2.set_high();
        delay_us(100);
        s2.set_low();
        delay_us(100);
        s1.set_high();
        delay_us(250);
        s1.set_low();
        delay_us(550);
        
        // s1.set_high();;
        // s2.set_high();
        // delay_us(100);
        // s2.set_low();
        // s1.set_low();
        // delay_us(100);
        // s3.set_high();
        // delay_us(500);
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
        // 64000, 62.5Hz; 53333, 75Hz; 40000, 100Hz; 20000, 200Hz; 16000, 250Hz; 
        //8000, 500Hz; 5333, 750Hz; 4000, 1kHz; 2000, 2kHz; 1600, 2.5KHz; 800, 5kHz; 533, 7.5KhZ; 400, 10kHz;
        //TIM1.enable_output(2);
        //TIM1.enable_output(3);
        //counter += 1;


        // if counter >= 100{
        //     if j < 10 {
        //         //TIM1_CH2_PA9.setup();
        //         TIM1_CH3_PA10.setup();
        //         //TIM1.set_pwm(2, frequency_divider[i], dutycycle_divider[i]);
        //         //TIM1.set_pwm(3, frequency_divider[i], dutycycle_divider[i]);
        //         TIM1.set_pwm(3, 50000, 25000);
        //         //delay_ms(3);
        //         let res1 = tmp_adc.start_conversion_sw(5); //五次取平均的话，永远没法知道正确的最大值和最小值，因为被“平均”了。只能取值1次。
        //         let vpos1 = res1 as f64 * vref;
        //         //adc_sum1 -= adc_values1[adc_indexer1]; 
        //         //adc_values1[adc_indexer1] = vpos1; 
        //         //adc_sum1 += vpos1;
        //         //defmt::info!("ADC value is {}", adc_sum1/5.0 - 0.07);
        //         defmt::info!("ADC value is {}", vpos1);
        //         if vpos1 > adc_sum_max {
        //             //adc_sum_max = adc_sum1/5.0;
        //             adc_sum_max = vpos1;
        //             defmt::info!("max value change");
        //             defmt::info!("adc max value is {}", adc_sum_max);
        //         }
        //         if vpos1 < adc_sum_min {
        //             //adc_sum_min = adc_sum1/5.0;
        //             adc_sum_min = vpos1;
        //             defmt::info!("min value change");
        //             defmt::info!("adc min value is {}", adc_sum_min);
        //         }
        //         defmt::info!("ADC difference is {} - {} = {}", adc_sum_max, adc_sum_min, adc_sum_max - adc_sum_min);
        //         //let Rs = (adc_sum_max - adc_sum_min) * 1000.0 *2.0 / Ipos_f64;
        //         let Rs = (adc_sum_max - adc_sum_min) * 1000.0 / Ineg_f64;
        //         let Rp = R_total - Rs;
        //         defmt::info!("Rs is {}", Rs);
        //         // defmt::info!("Rp is {}", Rp);
        //         // let C0 = 4.7 * 1e-6;
                
        //         // let t1:f64 = 100.0 * 1e-6;
        //         // let tauRC =  (Rs * Rp * C0)/(Rs + Rp); //verify it again
        //         // let Ipos64 = Ipos as f64;
        //         // let Ineg64:f64 = Ineg as f64;
        //         // let V0 :f64 = Ipos64 * t1 / C0;
        //         // let I0 = V0 / (Rs + Rp);
        //         // let base :f64 = 1.0 - Ineg64* t1 *1e-3 / (I0 * tauRC);
        //         // let t2 = -tauRC*exp(base);
        //         // defmt::info!("t2 is {}", t2);

        //         // s8.set_high();
        //         // delay_ms(1);
        //         // s8.set_low();

        //     //     if adc_sum / 5.0 > 1.6 {
        //     //         red.set_high();
        //     //     } 
        //     //     else {
        //     //         red.set_low();
        //     //     }
        //     //     // s1.setup();
        //     //     // s2.setup();
        //         j = j + 1 ;
        //     } else {
        //         j = 0;
        //         // array_impedance[i] = (adc_sum_max - adc_sum_min) * 1000.0 *2.0 / Ipos_f64; 
        //         array_impedance[i] = (adc_sum_max - adc_sum_min) * 1000.0 / Ineg_f64; 
        //         if i == 14{
        //             i = 0;
        //             defmt::info!("Final value is {:?}", array_impedance);
        //             delay_s(5);
        //         } else {
        //             i = i + 1;
        //         }
        //         adc_sum_max = 0.0;
        //         adc_sum_min = 3.0;
        //     }
        //     //adc_indexer1 += 1;
        //     //adc_indexer1 %= 5;
        //     counter = 0;
        // }
        // green.toggle();
        red.toggle();
        //delay_ms(2);
        //defmt::info!("toggle leds");
    }
}


#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        spawner.spawn(async_main(spawner)).unwrap();
    });
}
