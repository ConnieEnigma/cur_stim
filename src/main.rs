#![feature(noop_waker)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use embassy_executor::Spawner;

use u5_lib::{
    clock::{self, delay_ms, delay_s, delay_us}, com_interface::ComInterface, exti, gpio, i2c::{self, I2c}, low_power::{no_deep_sleep_request, Executor}, task, *
};

fn i2c_init() -> (I2c, I2c) {
    let i2c_config_plus = i2c::I2cConfig::new(1, 100_000, gpio::I2C1_SCL_PB6, gpio::I2C1_SDA_PB3);
    let i2c_plus = I2c::new(i2c_config_plus).unwrap();
    let i2c_config_minus = i2c::I2cConfig::new(2, 100_000, gpio::I2C2_SCL_PB13, gpio::I2C2_SDA_PB14);
    let i2c_minus = I2c::new(i2c_config_minus).unwrap();
    (i2c_plus, i2c_minus)
}
fn switch_led_setup() -> ( gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort,
    gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort,gpio::GpioPort,gpio::GpioPort,){
    let red: gpio::GpioPort = gpio::PB7;
    let green: gpio::GpioPort = gpio::PB8;
    let s0: gpio::GpioPort = gpio::PB15;
    let s0: gpio::GpioPort = gpio::PB15;
    let s1: gpio::GpioPort = gpio::PA9;
    let s2: gpio::GpioPort = gpio::PA10;
    let s3: gpio::GpioPort = gpio::PB4;
    let s4: gpio::GpioPort = gpio::PB5;
    let s5: gpio::GpioPort = gpio::PA5;
    let s6: gpio::GpioPort = gpio::PA6;
    let s7: gpio::GpioPort = gpio::PA4;
    let s8: gpio::GpioPort = gpio::PA3;
    
    green.setup();
    red.setup();
    s0.setup();
    s0.setup();
    s1.setup();
    s2.setup();
    s3.setup();
    s4.setup();
    s5.setup();
    s6.setup();
    s7.setup();
    s8.setup();
    (green, red, s0, s1, s2, s3, s4, s5, s6, s7, s8)
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
    clock::init_clock(false, true, 16_000_000, true, clock::ClockFreqs::KernelFreq160Mhz);
    unsafe {
        no_deep_sleep_request();
    }
    clock::set_mco(
        gpio::GPIO_MCO_PA8,
        clock::Mcosel::HSE,
        clock::Mcopre::DIV4,
    ); // clock. which use PA8 as clock output

    defmt::info!("setup led finished!");
    let (green, red, s0, s1, s2, s3, s4, s5, s6, s7, s8) = switch_led_setup();
    s0.set_low();
    s1.set_low();
    s2.set_low();
    s3.set_high();
    s4.set_low();
    s5.set_high();
    s6.set_low();
    s7.set_high();
    s8.set_low();

    let (mut i2c_plus, mut i2c_minus) = i2c_init();
    for i in 0..4{
        //i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, 0xA8]);
        i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, 0xA8]);
    }
    for i in 0..4{
        //i2c_send(&mut i2c_plus, POS_DAC_2_ADDR, [DAC_REG_BASE + i, 0xA8]);
        i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, 0xA8]);
    }
    // for i in 0..4 {
    //     i2c_send(&mut i2c_minus, NEG_DAC_1_ADDR, [DAC_REG_BASE + i, 0x50]);
    // }
    // for i in 0..4 {
    //     i2c_send(&mut i2c_minus, NEG_DAC_2_ADDR, [DAC_REG_BASE + i, 0x50]);
    // }
    defmt::info!("i2c finished!");


    let tmp_adc = adc::ADC1;
    tmp_adc.init();
    let adc_pin = gpio::ADC1_IN5_PA0;
    adc_pin.setup();
    let mut counter = 4900;
    defmt::info!("ADC init!");
    delay_ms(100);
    let vref_data = tmp_adc.start_conversion_sw(0);
    delay_ms(100);
    let vref_data = tmp_adc.start_conversion_sw(0);
    defmt::info!("Vref_data init! {}", vref_data);
    let vref_raw =  tmp_adc.get_vref_int_raw();
    defmt::info!("Vref_raw init! {}", vref_raw);
    let vref = 3.0 * vref_raw as f64 / vref_data as f64;
    defmt::info!("Vref init! {}", vref);
    let vref = vref / 16384.0;
    delay_s(2);

    s1.set_high();
    delay_ms(100);
    s5.set_low();
    delay_ms(10);
    s5.set_high();
    for i in 0..4{
        i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, 0xD0]);
    }
    for i in 0..4{
        i2c_send(&mut i2c_plus, POS_DAC_2_ADDR, [DAC_REG_BASE + i, 0xD0]);
    }
    loop {
        // s2.set_high();
        // delay_us(100);
        // s2.set_low();
        // delay_us(100);
        //s1.set_low();
        //delay_ms(10); 
        // counter += 1;
        // if counter >= 100 {
        //     let res = tmp_adc.start_conversion_sw(5);
        //     let vpos = res as f64 * vref;
        //     defmt::info!("adc value: {}", vpos);
        //     counter = 0;
        // }
        //delay_ms(1);
        //s1.set_low();
        //delay_ms(1);
        // delay_us(200);
        // s1.set_low();
        // delay_us(600);
        green.toggle();
        // red.toggle();
        delay_ms(1000);
        defmt::info!("toggle leds");
    }
}


#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        spawner.spawn(async_main(spawner)).unwrap();
    });
}

