#![feature(noop_waker)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::default::Default;
use core::panic::PanicInfo;
use cortex_m::asm::delay;

use defmt_rtt as _;
use embassy_executor::{Spawner};
use embassy_usb::{
    Builder,
    class::cdc_acm::{CdcAcmClass, State},
    driver::EndpointError,
};
use futures::future::{join};

use u5_lib::{gpio::{SDMMC2_CMD_PD7, SDMMC2_D0_PB14}, *};
use u5_lib::clock::delay_ms;
use u5_lib::com_interface::ComInterface;

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


fn setup_camera() -> (gpio::GpioPort, i2c::I2c) {
    clock::set_mco(gpio::GPIO_MCO_PA8, clock::Mcosel::HSI, clock::Mcopre::DIV1);    // clock. which use PA8 as clock output
    let cam_down = gpio::PD13;
    cam_down.setup();
    cam_down.set_high();
    cam_down.set_low();
    // wait for short time
    delay_ms(50);
    let mut i2c = i2c::I2c::new(i2c::I2cConfig::new(2, 100_000, gpio::I2C2_SDA_PF0, gpio::I2C2_SCL_PF1)).unwrap();
    // delay_ms(1);
    camera::setup_camera(&mut i2c);
    (cam_down, i2c)
}

fn setup_led() -> gpio::GpioPort {
    let green: gpio::GpioPort = gpio::PD15;
    green.setup();
    green
}
fn setup_sd() -> sdmmc::SdInstance {
    let mut sd = sdmmc::SdInstance::new( sdmmc::SDMMC2);    
    sd.init(gpio::SDMMC2_CK_PD6, SDMMC2_CMD_PD7, SDMMC2_D0_PB14);
    sd
}

// #[embassy_executor::main]
#[task]
async fn async_main(spawner: Spawner) {
    // clock::init_clock(true, false, clock::ClockFreqs::KernelFreq4Mhz);
    clock::init_clock(false, true, clock::ClockFreqs::KernelFreq160Mhz);
    // let (cam_down, i2c) = setup_camera();
    // cam_down.set_high();
    delay_ms(200);
    defmt::info!("camera init finished!");
    let sd = setup_sd();
    defmt::info!("sd init finished!");
    let green = setup_led();
    spawner.spawn(btn()).unwrap();
    spawner.spawn(pwr::vddusb_monitor_up()).unwrap();
    spawner.spawn(usb_task()).unwrap();
    // start emmc


    defmt::info!("usb init finished!");
    loop {
        exti::EXTI2_PB2.wait_for_raising().await;
        green.toggle();
        defmt::info!("exti2 triggered");
        unsafe {defmt::info!("deep sleep flag: {}", low_power::REF_COUNT_DEEP);}
    }
}

use low_power::Executor;
#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        // unwrap!(spawner.spawn(async_main(spawner)));
        spawner.spawn(async_main(spawner)).unwrap();
    });
}



#[embassy_executor::task]
async fn btn() {
    let _last_time: (u8, u8, u8) = (0, 0, 0);
    defmt::info!("waiting for btn");
    loop {
        exti::EXTI13_PC13.wait_for_raising().await;
    }
}

#[embassy_executor::task]
pub async fn usb_task() {
    let _ep_out_buffer = [0u8; 256];
    let mut config = usb_otg::Config::default();
    config.vbus_detection = false;
    let driver = usb_otg::Driver::new(config, gpio::USB_DM_PA11, gpio::USB_DP_PA12);

    // // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xaaaa, 0xefba);
    config.manufacturer = Some("ggeta");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");

    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    let mut device_descriptor = [0; 512];
    let mut config_descriptor = [0; 512];
    let mut bos_descriptor = [0; 512];
    let mut control_buf = [0; 64];
    let mut msos_descriptor = [0; 512];

    let mut state = State::new();
    // USART1.send("starting usb task new!\n\n".as_bytes());

    let mut builder = Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    // Build the builder.
    let mut usb = builder.build();
    let usb_fut = usb.run(); // Run the USB device.
    let handler_fut = async {
        loop {
            class.wait_connection().await;
            defmt::info!("connected");
            let _ = usb_handler(&mut class).await;
            defmt::info!("disconnected");
        }
    };
    // USART1.send("start usb task success!\n".as_bytes());
    join(usb_fut, handler_fut).await; // Run everything concurrently.
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn usb_handler<'d>(class: &mut CdcAcmClass<'d, usb_otg::Driver>) -> Result<(), Disconnected> {
    let mut buf: [u8; 128] = [0; 128];
    // the maximum size of the command is 64 bytes
    defmt::info!("start usb handler");
    loop {
        // select(future1, future2)
        let ret = class.read_packet(&mut buf).await;
        match ret {
            Ok(n) => {
                defmt::info!("read {} bytes", n);
                class.write_packet(&buf[0..n]).await.unwrap();
            }
            Err(e) => {
                defmt::info!("error: {:?}", e);
                return Err(e.into());
            }
        }
        // class.write_packet(&buf[0..n]).await.unwrap();
    }
}
