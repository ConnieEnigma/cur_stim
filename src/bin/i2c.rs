// #[!no_std]
#![feature(noop_waker)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::default;
use core::panic::PanicInfo;
use core::default::Default;

use u5_lib::{*};
use u5_lib::com_interface::ComInterface;

#[derive(defmt::Format)]
pub enum UsbError {
    BufferOverflow,
    Disabled,
}

const BLUE: gpio::GpioPort = gpio::PB7;
// const USART: usart::Usart = usart::USART1;

use u5_lib::low_power::Executor;

#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        spawner.spawn(async_main(spawner)).unwrap();
    });
}

#[task]
async fn async_main(_spawner: Spawner) {
    clock::init_clock(true, false, clock::ClockFreqs::KernelFreq160Mhz);
    BLUE.setup();
    defmt::info!("setup led finished!");
    loop {
        let mut i2c = i2c::I2c::new(default::Default::default()).unwrap();
        let mut data = [0x00];
        // i2c.send(i2c::I2cMessage { addr: 0x68, data: &mut data }).unwrap();
        clock::delay_ms(1000);
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
