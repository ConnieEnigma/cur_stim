#![feature(noop_waker)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::default::Default;
use core::panic::PanicInfo;
use cortex_m::asm::delay;

use defmt_rtt as _;
use eb_cmds::Command;
use embassy_executor::Spawner;
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State},
    driver::EndpointError,
    Builder,
};
use futures::future::join;

use u5_lib::clock::delay_ms;
use u5_lib::com_interface::ComInterface;
use u5_lib::{
    gpio::{SDMMC2_CMD_PD7, SDMMC2_D0_PB14},
    *,
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

fn setup_camera() -> (gpio::GpioPort, i2c::I2c) {
    let cam_down = gpio::PD13;
    cam_down.setup();
    let rst = gpio::PD12;
    rst.setup();
    cam_down.set_high();
    rst.set_low();
    clock::set_mco(gpio::GPIO_MCO_PA8, clock::Mcosel::HSI48, clock::Mcopre::DIV2); // clock. which use PA8 as clock output
    // cam_down.set_high();
    delay_ms(5);
    cam_down.set_low();
    delay_ms(5);
    rst.set_high();
    // wait for short time
    delay_ms(50);
    let mut i2c = i2c::I2c::new(i2c::I2cConfig::new(
        2,
        100_000,
        gpio::I2C2_SDA_PF0,
        gpio::I2C2_SCL_PF1,
    ))
        .unwrap();
    delay_ms(1);

    camera::setup_camera(&mut i2c);
    delay_ms(500);
    cam_down.set_high();
    (cam_down, i2c)
}

fn setup_led() -> gpio::GpioPort {
    let green: gpio::GpioPort = gpio::PD15;
    green.setup();
    green
}

fn setup_sd() -> sdmmc::SdInstance {
    let mut sd = sdmmc::SdInstance::new(sdmmc::SDMMC2);
    sd.init(gpio::SDMMC2_CK_PD6, SDMMC2_CMD_PD7, SDMMC2_D0_PB14, SDMMC2_D1_PB15, SDMMC2_D2_PB3, SDMMC2_D3_PB4, SDMMC2_D4_PB8, SDMMC2_D5_PB9, SDMMC2_D6_PC6, SDMMC2_D7_PC7);
    sd
}

fn set_dcmi() -> dcmi::DcmiPort {
    let dcmi = dcmi::DCMI;

    dcmi.init(
        gpio::DCMI_D0_PA9, // todo: updat the pin
        gpio::DCMI_D1_PA10,
        gpio::DCMI_D2_PE0,
        gpio::DCMI_D3_PE1,
        gpio::DCMI_D4_PE4,
        gpio::DCMI_D5_PB6,
        gpio::DCMI_D6_PE5,
        gpio::DCMI_D7_PE6,
        gpio::DCMI_HSYNC_PA4,
        gpio::DCMI_VSYNC_PB7,
        gpio::DCMI_PIXCLK_PD9,
    );
    dcmi
}

// #[embassy_executor::main]
#[task]
async fn async_main(spawner: Spawner) {
    // clock::init_clock(true, false, clock::ClockFreqs::KernelFreq4Mhz);
    clock::init_clock(false, true, true, clock::ClockFreqs::KernelFreq160Mhz);
    // cam_down.set_high();
    delay_ms(200);
    let green = setup_led();
    spawner.spawn(btn()).unwrap();
    spawner.spawn(pwr::vddusb_monitor_up()).unwrap();
    spawner.spawn(usb_task()).unwrap();
    // init dcmi

    let (cam_down, mut i2c, sd, dcmi) =
        clock::hclk_request(clock::ClockFreqs::KernelFreq160Mhz, || {
            let sd = setup_sd();
            defmt::info!("sd init finished!");
            let (cam_down, i2c) = setup_camera();
            defmt::info!("camera init finished!");
            let dcmi = set_dcmi();
            (cam_down, i2c, sd, dcmi)
        });

    defmt::info!("usb init finished!");
    let mut power_on = false;
    // let mut green = gpio::PD10;
    gpio::PD10.setup();
    gpio::PD14.setup();
    gpio::PD15.setup();
    // set high 
    gpio::PD10.set_high();
    gpio::PD14.set_high();
    gpio::PD15.set_high();
    delay_ms(500);
    gpio::PD10.set_low();
    gpio::PD14.set_low();
    gpio::PD15.set_low();


    loop {
        if !power_on {
            let val = POWER_SIGNAL.wait().await;
            if val {
                defmt::info!("power on");
                power_on = true;
            }
        } else {
            if POWER_SIGNAL.signaled() {
                let val = POWER_SIGNAL.wait().await;
                power_on = val;
            }
        }
        if !power_on {
            continue;
        }

        // exti::EXTI2_PB2.wait_for_raising().await;
        // clock::init_clock(false, true, clock::ClockFreqs::KernelFreq160Mhz);
        // delay_ms(1);
        // rtc::rtc_interrupt().await;
        green.toggle();
        let mut pic_buf = [0u8; 1_600_000];
        // deep sleep is not allowed
        clock::hclk_request_async(clock::ClockFreqs::KernelFreq160Mhz, || async {
            low_power::run_no_deep_sleep_async(|| async {
                defmt::info!("start capture, ########################");
                camera::capture(&cam_down, &mut i2c, &dcmi, &mut pic_buf).await;
                defmt::info!("finish capture, ########################");
                camera::save_picture(&mut pic_buf, &sd).await;
                defmt::info!("finish save picture, ########################");
            })
                .await;
        })
            .await;
    }
}

use low_power::Executor;

#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        spawner.spawn(async_main(spawner)).unwrap();
    });
}

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use u5_lib::gpio::{SDMMC2_D1_PB15, SDMMC2_D2_PB3, SDMMC2_D3_PB4, SDMMC2_D4_PB8, SDMMC2_D5_PB9, SDMMC2_D6_PC6, SDMMC2_D7_PC7};

// static mut POWER_STATE: bool = false;
static POWER_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

#[embassy_executor::task]
async fn btn() {
    let _last_time: (u8, u8, u8) = (0, 0, 0);
    defmt::info!("waiting for btn");
    unsafe {
        static mut POWER_STATE: bool = false;
        loop {
            exti::EXTI2_PB2.wait_for_raising().await;
            defmt::info!("btn pressed");
            let green: gpio::GpioPort = gpio::PD14;
            green.toggle();
            POWER_STATE = !POWER_STATE;
            POWER_SIGNAL.signal(POWER_STATE);
        }
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

// todo: these should be rewrite
const IMG_START_BLOCK: u32 = 10;
const IMG_SIZE: u32 = 2000;
// 2000 block = 2000 * 512 = 1M
const SIZE_BLOCK: u32 = 1; // first block store the number of image files

async fn usb_handler<'d>(class: &mut CdcAcmClass<'d, usb_otg::Driver>) -> Result<(), Disconnected> {
    let mut in_buf: [u8; 128] = [0; 128];
    // the maximum size of the command is 64 bytes

    let sd = setup_sd();
    defmt::info!("start usb handler");
    loop {
        // select(future1, future2)
        let ret = class.read_packet(&mut in_buf).await;
        let n = match ret {
            Ok(n) => n,
            Err(EndpointError::Disabled) => return Err(Disconnected {}),
            Err(EndpointError::BufferOverflow) => panic!("Buffer overflow"),
        };

        let command = eb_cmds::Command::from_array(&in_buf[..n]);
        match command {
            Command::SetTim(year, month, day, hour, minute, second, period) => {
                // rtc::set_time(year, month, day, hour, minute, second, period);
                rtc::setup(year, month, day, hour, minute, second, period, rtc::RtcSource::LSE);
                let response = eb_cmds::Response::SetTim(0);
                let (buf, len) = response.to_array();
                class.write_packet(&buf[..len]).await.unwrap();
            }
            Command::GetTim => {
                let date = rtc::get_date();
                let time = rtc::get_time();
                let response = eb_cmds::Response::GetTim(date.0, date.1, date.2, time.0, time.1, time.2);
                let (buf, len) = response.to_array();
                class.write_packet(&buf[..len]).await.unwrap();
            }
            Command::GetPic(id) => {
                let mut buf = [0; 64];
                buf[0] = 0x02;
                let mut pic_buf = [0; 512];
                let start_block = (id + IMG_START_BLOCK) * IMG_SIZE;
                sd.read_single_block_async(&mut pic_buf, start_block).await.unwrap();
                // get the end of picture
                let pic_end = ((pic_buf[0] as u32) << 24)
                    | ((pic_buf[1] as u32) << 16)
                    | ((pic_buf[2] as u32) << 8)
                    | (pic_buf[3] as u32);
                let block_count: u32 = ((pic_end + 512 - 1) / 512) as u32;
                let mut ordinal = 0;
                let mut send_len: usize;
                let mut res: eb_cmds::Response;
                let mut start = 16;
                loop {
                    if start >= pic_buf.len() {
                        break;
                    }
                    (ordinal, send_len, res) =
                        eb_cmds::Response::pic_res_from_data(id, ordinal, &pic_buf[start..]);
                    if send_len == 0 {
                        break;
                    }
                    start += send_len;

                    let (buf_tmp, len) = res.to_array();
                    class.write_packet(&buf_tmp[0..len]).await.unwrap();
                    // Timer::after(Duration::from_millis(100)).await;
                    // LED_BLUE.toggle();
                }
                // LED_GREEN.toggle();
                for block in 1..block_count {
                    // sd.read_single_block(&mut buf, start_block + block).unwrap();
                    // let mut pic_buf = [0; 512]; // why without this line, the program not work?
                    sd.read_single_block_async(&mut pic_buf, start_block + block).await.unwrap();
                    start = 0;
                    loop {
                        if start >= pic_buf.len() {
                            break;
                        }
                        (ordinal, send_len, res) =
                            eb_cmds::Response::pic_res_from_data(id, ordinal, &pic_buf[start..]);
                        if send_len == 0 {
                            break;
                        }
                        start += send_len;
                        let (buf_tmp, len) = res.to_array();
                        class.write_packet(&buf_tmp[0..len]).await.unwrap();
                    }
                }
            }
            Command::GetPicNum => {
                let mut buf = [0u8; 512];
                sd.read_single_block_async(&mut buf, SIZE_BLOCK).await.unwrap();
                let num = ((buf[0] as u32) << 24)
                    | ((buf[1] as u32) << 16)
                    | ((buf[2] as u32) << 8)
                    | (buf[3] as u32);
                // ebcmd::Response::GetPicNum(num)
                let res = eb_cmds::Response::GetPicNum(num);
                let (buf, len) = res.to_array();
                class.write_packet(&buf[0..len]).await.unwrap();
            }
            Command::ClearPic => {}
        }

        // let ret = select(class.read_packet(&mut buf), class.write_packet(&buf)).await;
        // class.write_packet(&buf[0..n]).await.unwrap();
    }
}
