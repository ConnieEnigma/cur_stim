#![feature(noop_waker)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
use core::panic::PanicInfo;
use defmt_rtt as _;
use embassy_executor::Spawner;

use u5_lib::{
    usb_otg_hs::mod_new::{cdc_acm_ep2_read },
    *,
};
use u5_lib::usb_otg_hs::control_pipe::setup_process;
use u5_lib::usb_otg_hs::power::power_up_init;

const GREEN: gpio::GpioPort = gpio::PB7;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    clock::init_clock(false, true, true, clock::ClockFreqs::KernelFreq4Mhz);
    low_power::no_deep_sleep_request();
    GREEN.setup();
    // low_power::no_deep_sleep_request();
    // mcu_no_deep_sleep();
    defmt::info!("setup led finished!");
    // spawner.spawn(btn()).unwrap();
    // spawner.spawn(pwr::vddusb_monitor_up()).unwrap();
    // use some delay to wait for usb power up
    pwr::vddusb_monitor_up_tmp();
    power_up_init();
    defmt::info!("vddusb monitor finished!");

    spawner.spawn(setup_process()).unwrap();

    defmt::info!("usb init finished!");
    spawner.spawn(usb_task()).unwrap();
    loop {
        exti::EXTI13_PC13.wait_for_raising().await;
        GREEN.toggle();
    }
}

#[embassy_executor::task]
async fn usb_task() {
    // the maximum size of the command is 64 bytes
    defmt::info!("start usb handler");
    // wait for end of suspend here

    loop {
        // todo: in read function, we need to wait for usbepen to be set.
        let (ret, len) = cdc_acm_ep2_read().await;
        defmt::info!("read ret: {:?}", &ret[0..len]);
    }
}


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