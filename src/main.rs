#![feature(noop_waker)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::panic::PanicInfo;
use aligned::Aligned;
use defmt::println;

use defmt_rtt as _;
use embassy_executor::Spawner;

use u5_lib::{
    *,
    clock::delay_ms,
    com_interface::ComInterface,
};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    defmt::info!("panic");
    defmt::error!(
        "Location file name: {:?}, line: {:?}, col: {:?}",
        _info.location().unwrap().file(),
        _info.location().unwrap().line(),
        _info.location().unwrap().column()
    );
    loop {}
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
    loop {
        exti::EXTI13_PC13.wait_for_raising().await;
        green.toggle();
        defmt::info!("toggle");
    }
}

use low_power::Executor;
use u5_lib::low_power::no_deep_sleep_request;

#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        spawner.spawn(async_main(spawner)).unwrap();
    });
}