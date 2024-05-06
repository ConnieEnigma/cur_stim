#![feature(noop_waker)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use embassy_executor::Spawner;

use u5_lib::{
    *,
    low_power::{Executor, no_deep_sleep_request},
    clock, clock::delay_ms, com_interface::ComInterface, exti, gpio, task,
    i2c, i2c::I2c,
};

fn i2c_init() -> (I2c, I2c) {
    let i2c_config_plus = i2c::I2cConfig::new(1, 100_000, gpio::I2C1_SCL_PB6, gpio::I2C1_SDA_PB3);
    let i2c_plus = I2c::new(i2c_config_plus).unwrap();
    let i2c_config_minus = i2c::I2cConfig::new(2, 100_000, gpio::I2C2_SCL_PB13, gpio::I2C2_SDA_PB14);
    let i2c_minus = I2c::new(i2c_config_minus).unwrap();
    (i2c_plus, i2c_minus)
}
fn switch_led_setup() -> ( gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, 
    gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, 
    gpio::GpioPort,  gpio::GpioPort, gpio::GpioPort){
    let red: gpio::GpioPort = gpio::PB7;
    let green: gpio::GpioPort = gpio::PB8;
    let s0: gpio::GpioPort = gpio::PB15;
    let s1: gpio::GpioPort = gpio::PA9;
    let s2: gpio::GpioPort = gpio::PA10;
    let s3: gpio::GpioPort = gpio::PB4;
    let s4: gpio::GpioPort = gpio::PB5;
    let s5: gpio::GpioPort = gpio::PA5;
    let s7: gpio::GpioPort = gpio::PA4;
    let s8: gpio::GpioPort = gpio::PA3;
    
    green.setup();
    red.setup();
    s0.setup();
    s1.setup();
    s2.setup();
    s3.setup();
    s4.setup();
    s5.setup();
    s7.setup();
    s8.setup();
    (green, red, s0, s1, s2, s3, s4, s5, s7, s8)
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
    clock::init_clock(true, true, 16_000_000, true, clock::ClockFreqs::KernelFreq160Mhz);
    no_deep_sleep_request();
    defmt::info!("setup led finished!");
    clock::set_mco(
        gpio::GPIO_MCO_PA8,
        clock::Mcosel::HSE,
        clock::Mcopre::DIV4,
    ); // clock. which use PA8 as clock output
    let (green, red, s0, s1, s2, s3, s4, s5, s7, s8) = switch_led_setup();
    s0.set_low();
    s1.set_low();
    s2.set_low();
    s3.set_high();
    s4.set_high();
    s5.set_high();
    s7.set_low();
    s8.set_low();
    
    let (mut i2c_plus, mut i2c_minus) = i2c_init();
    for i in 0..4{
        i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, 0xA8]);
    }
    for i in 0..4{
        i2c_send(&mut i2c_plus, POS_DAC_2_ADDR, [DAC_REG_BASE + i, 0xA8]);
    }
    for i in 0..4 {
        i2c_send(&mut i2c_minus, NEG_DAC_1_ADDR, [DAC_REG_BASE + i, 0x50]);
    }
    for i in 0..4 {
        i2c_send(&mut i2c_minus, NEG_DAC_2_ADDR, [DAC_REG_BASE + i, 0x50]);
    }
    defmt::info!("i2c finished!");


    let tmp_adc = adc::ADC1;
    tmp_adc.init();
    let adc_pin = gpio::ADC1_IN5_PA0;
    adc_pin.setup();
    
    let res = tmp_adc.start_conversion_sw(5);
    let mut vref:f32 = (res as f32/16383f32)*3.3;
    defmt::info!("vref value: {}", vref);


    let mut counter = 0;

    loop {
        s1.set_high();
        if (counter == 1000){
            let res = tmp_adc.start_conversion_sw(5);
            let mut vpos : f32 = (res as f32 / 16383f32) * 3.3;
            defmt::info!("pos pulse value: {}", vpos);
        }
        delay_ms(2);
        s1.set_low();
        delay_ms(1);
        s2.set_high();
        if (counter == 1000){
            let res = tmp_adc.start_conversion_sw(5);
            let mut vneg : f32 = (res as f32 / 16383f32) * 3.3;
            defmt::info!("neg pulse value: {}", vneg);
            counter = 0;
        }
        else{
            counter += 1;
        }
        delay_ms(1);
        s2.set_low();
        delay_ms(6);
        
        // green.toggle();
        //red.toggle();
        // delay_ms(1000);
        // defmt::info!("toggle leds");
    }
}


#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        spawner.spawn(async_main(spawner)).unwrap();
    });
}

