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

fn i2c_init() -> I2c {
    let i2c_config = i2c::I2cConfig::new(1, 100_000, gpio::I2C1_SCL_PB6, gpio::I2C1_SDA_PB7);
    I2c::new(i2c_config).unwrap()
}

#[task]
async fn async_main(spawner: Spawner) {
    // becareful, if the dbg is not enabled, but using deep sleep. This framework will not able to connect to chip.
    // stm32cube programmer, stmcubeide can be used to program the chip, then this framework can be used to debug.
    clock::init_clock(false, true, 16_000_000, true, clock::ClockFreqs::KernelFreq4Mhz);
    no_deep_sleep_request();
    defmt::info!("setup led finished!");
    let green: gpio::GpioPort = gpio::PB7;
    green.setup();
    green.set_high();
    let i2c = i2c_init();
    panic!("panic");
    loop {
        exti::EXTI13_PC13.wait_for_raising().await;
        green.toggle();
        defmt::info!("toggle");
    }
}


#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        spawner.spawn(async_main(spawner)).unwrap();
    });
}

