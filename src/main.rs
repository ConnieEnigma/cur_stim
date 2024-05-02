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

#[task]
async fn async_main(spawner: Spawner) {
    // be careful, if the dbg is not enabled, but using deep sleep. This framework will not able to connect to chip.
    // stm32cube programmer, stmcubeide can be used to program the chip, then this framework can be used to debug.
    clock::init_clock(false, true, 16_000_000, true, clock::ClockFreqs::KernelFreq4Mhz);
    no_deep_sleep_request();
    defmt::info!("setup led finished!");
    let red: gpio::GpioPort = gpio::PB7;
    let green: gpio::GpioPort = gpio::PB8;
    green.setup();
    red.setup();
    let (mut i2c_plus, mut i2c_minus) = i2c_init();
    // let i2c_message = i2c::I2cMessage {
    //     addr: 0x68,
    //     data: &mut [0x75],
    // };
    // i2c_plus.send(&i2c_message).unwrap();
    defmt::info!("i2c init finished!");
    loop {
        // exti::EXTI13_PC13.wait_for_raising().await;
        green.toggle();
        red.toggle();
        delay_ms(500);
        defmt::info!("toggle leds");
    }
}


#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        spawner.spawn(async_main(spawner)).unwrap();
    });
}

