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
    gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort){
    let red: gpio::GpioPort = gpio::PB7;
    let green: gpio::GpioPort = gpio::PB8;
    let s1: gpio::GpioPort = gpio::PA9;
    let s2: gpio::GpioPort = gpio::PA10;
    let s3: gpio::GpioPort = gpio::PB4;
    let s4: gpio::GpioPort = gpio::PB5;
    let s5: gpio::GpioPort = gpio::PA5;
    
    green.setup();
    red.setup();
    s1.setup();
    s2.setup();
    s3.setup();
    s4.setup();
    s5.setup();
    (green, red, s1, s2, s3,s4,s5)
}

fn i2c_send( i2c:&mut I2c, addr: u16, mut data: [u8; 2]) {
    let i2c_message = i2c::I2cMessage {
        addr,
        data:&mut data,
    };
    i2c.send(&i2c_message).unwrap();
}

const POS_DAC_1_ADDR: u16  = 0x20;

const DAC_REG_BASE: u8 = 0xF8;
const DAC_REG_1: u8 = 0xF8;
const DAC_REG_2: u8 = 0xF9;
const DAC_REG_3: u8 = 0xFA;
const DAC_REG_4: u8 = 0xFB;

#[task]
async fn async_main(spawner: Spawner) {
    // be careful, if the dbg is not enabled, but using deep sleep. This framework will not able to connect to chip.
    // stm32cube programmer, stmcubeide can be used to program the chip, then this framework can be used to debug.
    clock::init_clock(false, true, 16_000_000, true, clock::ClockFreqs::KernelFreq160Mhz);
    no_deep_sleep_request();
    defmt::info!("setup led finished!");
    let (green, red, s1, s2, s3, s4, s5) = switch_led_setup();
    s1.set_low();
    s2.set_high();
    s3.set_low();
    s4.set_high();
    s5.set_low();

    let (mut i2c_plus, mut i2c_minus) = i2c_init();
    for i in 0..4{
        i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, 0x50]);
    }
    for i in 0..4 {
        i2c_send(&mut i2c_minus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, 0x50]);
    }
    defmt::info!("i2c finished!");

    loop {
        green.toggle();
        // red.toggle();
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

