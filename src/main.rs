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
    clock::{self, delay_ms, delay_s, delay_us, hclk_request}, exti, gpio::{self, GpioPort, TIM1_CH2_PA9, TIM1_CH3_PA10, TIM3_CH1_PA6}, hal::I2c,  low_power::{Executor, no_deep_sleep_request}, task, tim::{Config, TIM1, TIM3}, *
};


fn switch_led_setup() -> ( gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort,
    gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort,
    gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort,
    gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, 
    gpio::GpioPort, gpio::GpioPort){

    let s0: gpio::GpioPort = gpio::PB12;
    let s0_en: gpio::GpioPort = gpio::PB13;
    let s1: gpio::GpioPort = gpio::PB14; 
    let s2: gpio::GpioPort = gpio::PB15;
    let s2_en: gpio::GpioPort = gpio::PC6;
    let s3: gpio::GpioPort = gpio::PC7;
    let s3_en: gpio::GpioPort = gpio::PC8;
    let s4: gpio::GpioPort = gpio::PC10;
    let s4_en: gpio::GpioPort = gpio::PA15;
    let s5: gpio::GpioPort = gpio::PA10;
    let s6: gpio::GpioPort = gpio::PA9;
    let s6_en: gpio::GpioPort = gpio::PA8;
    let s7: gpio::GpioPort = gpio::PC9;
    let s7_en: gpio::GpioPort = gpio::PB2;
    let DAC_shutdown1: gpio::GpioPort = gpio::PC11;
    let LDAC1: gpio::GpioPort = gpio::PC12;
    let CS1: gpio::GpioPort = gpio::PC4;
    let CLK1: gpio::GpioPort = gpio::PB6;
    let DIN1: gpio::GpioPort = gpio::PB3;
    let MOSI1: gpio::GpioPort = gpio::PA7;
    let SPI_CLK1: gpio::GpioPort = gpio::PA1;
    let SPI_CS1: gpio::GpioPort = gpio::PC4;


    s0.setup();
    s0_en.setup();
    s1.setup();
    s2.setup();
    s2_en.setup();
    s3.setup();
    s3_en.setup();
    s4.setup();
    s4_en.setup();
    s5.setup();
    s6.setup();
    s6_en.setup();
    s7.setup();
    s7_en.setup();
    DAC_shutdown1.setup();
    LDAC1.setup();
    CS1.setup();
    CLK1.setup();
    DIN1.setup();
    MOSI1.setup();
    SPI_CLK1.setup();
    SPI_CS1.setup();
    
    (s0, s0_en, s1, s2, s2_en, s3, s3_en, s4, s4_en, s5, s6, s6_en, s7, s7_en, DAC_shutdown1, LDAC1, CS1, CLK1, DIN1, MOSI1, SPI_CLK1, SPI_CS1)

}


struct Point {
    x: f64,
    y: f64,
}


#[embassy_executor::task]
async fn async_main(spawner: Spawner) {
    // be careful, if the dbg is not enabled, but using deep sleep. This framework will not able to connect to chip.
    // stm32cube programmer, stmcubeide can be used to program the chip, then this framework can be used to debug.
    // clock::init_clock(true, true,  16_000_000, true, clock::ClockFreqs::KernelFreq1Mhz);
    clock::init_clock(true, clock::ClockFreqs::KernelFreq16Mhz);
    unsafe {
        no_deep_sleep_request();

    let mut tim1_config = Config::default();
    tim1_config.prescaler = 10 - 1;
    let _ = TIM1.init(tim1_config);

    clock::set_mco(
        gpio::GPIO_MCO_PA8,
        clock::Mcosel::HSE,
        clock::Mcopre::DIV16,
    ); //filter cut off clock. which use PA8 as clock output

    defmt::info!("setup led finished!");

    let (s0, s0_en, s1, s2, s2_en, s3, s3_en, s4, s4_en, s5, s6, s6_en, s7, s7_en, DAC_shutdown1, LDAC1, CS1, CLK1, DIN1, MOSI1, SPI_CLK1, SPI_CS1) = switch_led_setup();
    s0.set_low(); 
    s0_en.set_low(); 
    s1.set_low(); 
    s2.set_low(); 
    s2_en.set_low();
    s3.set_high();
    s3_en.set_low();
    s4.set_high(); 
    s4_en.set_low();
    s5.set_high(); 
    s6.set_low(); 
    s6_en.set_low();
    s7.set_low(); 
    s7_en.set_low(); 
    DAC_shutdown1.set_high();
    LDAC1.set_high();
    CS1.set_high();
    CLK1.set_low();
    DIN1.set_low();
    MOSI1.set_low();
    SPI_CLK1.set_low();
    SPI_CS1.set_low();

    let Ipos = 2.7; // current range is -10mA to +10mA/ 
    let Ineg = -5.4;
    let Ineg_code = utils::cur_coding(Ineg);
    let Ipos_code = (Ipos/20.0  * 65536.0) as u16;       

    for i in 15..0{
            let mut bit = (Ineg_code >> i) & 1;
            if (bit == 1){
                DIN1.set_high();
                delay_us(100);
                CLK1.set_high();
                delay_us(100);
                CLK1.set_low();
                delay_us(100);
                DIN1.set_low();
            } else {
                CLK1.set_high();
                delay_us(100);
                CLK1.set_low();
                delay_us(100);
            }
        } //SPI, but in stupid way


    LDAC1.set_low();
    delay_us(100);
    LDAC1.set_high(); //negative edge of the latch
    
    let ctrl_reg_code: u32 = 0x551006;
    let DAC_reg_code = 0x010000 + Ipos_code as u32;
    
    for j in 31..0{
        let mut bit = (ctrl_reg_code >> j) & 1;
            if (bit == 1){
                MOSI1.set_high();
                delay_us(100);
                SPI_CLK1.set_high();
                delay_us(100);
                SPI_CLK1.set_low();
                delay_us(100);
                MOSI1.set_low();
            } else {
                SPI_CLK1.set_high();
                delay_us(100);
                SPI_CLK1.set_low();
                delay_us(100);
            }
    }
    
    delay_us(20);
    SPI_CS1.set_high();
    delay_us(100);
    SPI_CS1.set_low();

     for j in 31..0 {
        let mut bit = (DAC_reg_code >> j) & 1;
            if (bit == 1){
                MOSI1.set_high();
                delay_us(100);
                SPI_CLK1.set_high();
                delay_us(100);
                SPI_CLK1.set_low();
                delay_us(100);
                MOSI1.set_low();
            } else {
                SPI_CLK1.set_high();
                delay_us(100);
                SPI_CLK1.set_low();
                delay_us(100);
            }
    }

    delay_us(20);
    SPI_CS1.set_high();
    delay_us(100);
    SPI_CS1.set_low();

    loop {
        s1.set_high();
        delay_us(100);
        s1.set_low();
    }
}
}


#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        spawner.spawn(async_main(spawner)).unwrap();
    });
}