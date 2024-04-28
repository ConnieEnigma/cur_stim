#![feature(noop_waker)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::panic::PanicInfo;
use aligned::Aligned;
use defmt::println;

use defmt_rtt as _;
use eb_cmds::Command;
use embassy_executor::Spawner;

use u5_lib::{
    *,
    usb_otg_hs::{ control_pipe::setup_process, mod_new::{cdc_acm_ep2_read, cdc_acm_ep2_write} },
    gpio::{SDMMC2_CMD_PD7, SDMMC2_D0_PB14},
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

fn setup_camera() -> (gpio::GpioPort, i2c::I2c) {
    let cam_down = gpio::PD13;
    let rst = gpio::PD12;
    cam_down.setup();
    rst.setup();
    clock::set_mco(
        gpio::GPIO_MCO_PA8,
        clock::Mcosel::HSI48,
        clock::Mcopre::DIV2,
    ); // clock. which use PA8 as clock output
    let mut i2c = i2c::I2c::new(i2c::I2cConfig::new(
        2,
        100_000,
        gpio::I2C2_SDA_PF0,
        gpio::I2C2_SCL_PF1,
    )).unwrap();
    delay_ms(1);
    cam_down.set_low();
    rst.set_high();
    delay_ms(10);
    // camera::setup_camera(&mut i2c);
    defmt::info!("start setup camera");
    u5_lib::drivers::ov5640::setup_camera(&mut i2c, &cam_down, &rst);
    // cam_down.set_high();
    (cam_down, i2c)
}


fn setup_leds() -> (gpio::GpioPort, gpio::GpioPort, gpio::GpioPort) {
    let green: gpio::GpioPort = gpio::PD14;
    let orange: gpio::GpioPort = gpio::PD15;
    let blue: gpio::GpioPort = gpio::PD10;
    green.setup();
    orange.setup();
    blue.setup();
    (green, orange, blue)
}

fn setup_sd() -> sdmmc::SdInstance {
    let mut sd = sdmmc::SdInstance::new(sdmmc::SDMMC2);
    sd.init(
        gpio::SDMMC2_CK_PD6,
        SDMMC2_CMD_PD7,
        SDMMC2_D0_PB14,
        SDMMC2_D1_PB15,
        SDMMC2_D2_PB3,
        SDMMC2_D3_PB4,
        SDMMC2_D4_PB8,
        SDMMC2_D5_PB9,
        SDMMC2_D6_PC6,
        SDMMC2_D7_PC7,
    );
    sd
}

fn set_dcmi() -> dcmi::DcmiPort {
    let dcmi = dcmi::DCMI;

    dcmi.init(
        gpio::DCMI_D0_PA9, // todo: update the pin
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

static mut PIC_BUF: Aligned<aligned::A4,[u8; 1_600_000]> = Aligned([0u8; 1_600_000]);
// static mut pic_buf: Aligned<[u8; 300_000_000]> = Aligned([0; 300_000_000]);

// #[embassy_executor::main]
#[task]
async fn async_main(spawner: Spawner) {
    clock::init_clock(false, true, 26_000_000, true, clock::ClockFreqs::KernelFreq4Mhz);
    // low_power::no_deep_sleep_request();

    // let (green, blue, blue) = setup_leds();
    let (green, orange, _blue) = setup_leds();
    //
    let (cam_down, mut i2c, sd, dcmi) =
    //     // todo: check functions, one of them may take too much time.
        clock::hclk_request(clock::ClockFreqs::KernelFreq160Mhz, || {
            let sd = setup_sd();
            defmt::info!("sd init finished!");
            let (cam_down, i2c) = setup_camera();
            defmt::info!("camera init finished!");
            let dcmi = set_dcmi();
            cam_down.set_high();
            (cam_down, i2c, sd, dcmi)
        });
    spawner.spawn(pwr::vddusb_monitor_up()).unwrap();
    spawner.spawn(setup_process()).unwrap();
    spawner.spawn(usb_task()).unwrap();
    spawner.spawn(btn()).unwrap();
    // green.set_high();
    // blue.set_high();
    // orange.set_high();
    // loop {
    //     exti::EXTI2_PB2.wait_for_raising().await;
    //     green.toggle();
    // }

    let mut power_on = false;
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
        green.toggle();
        // deep sleep is not allowed
        clock::hclk_request_async(clock::ClockFreqs::KernelFreq160Mhz, || async {
            low_power::run_no_deep_sleep_async(|| async {
                unsafe {
                    defmt::info!("start capture, ########################");
                    camera::capture(&cam_down, &mut i2c, &dcmi, &mut PIC_BUF[..]).await;
                    defmt::info!("finish capture, ########################");
                    camera::save_picture(&mut PIC_BUF[..], &sd).await;
                    defmt::debug!("finish save picture, ##################");
                    orange.toggle();
                }
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


// todo: these should be rewrite
const IMG_START_BLOCK: u32 = 10;
const IMG_SIZE: u32 = 2000;
// 2000 block = 2000 * 512 = 1M
const SIZE_BLOCK: u32 = 1; // first block store the number of image files

#[embassy_executor::task]
async unsafe fn usb_task() {
    defmt::info!("start usb handler");
    let sd = setup_sd();

    loop {
        // todo: in read function, we need to wait for usbepen to be set.
        let (ret, len) = cdc_acm_ep2_read().await;
        // cdc_acm_ep2_write(&ret[0..len]).await;
        // continue;
        let command = eb_cmds::Command::from_array(&ret[..]);
        if command.is_err() {
            continue;
        }
        let command = command.unwrap();

        match command {
            Command::SetTim(year, month, day, hour, minute, second, period) => {
                // rtc::set_time(year, month, day, hour, minute, second, period);
                rtc::setup(
                    year,
                    month,
                    day,
                    hour,
                    minute,
                    second,
                    period,
                    rtc::RtcSource::LSE,
                );
                let response = eb_cmds::Response::SetTim(0);
                let (buf, len) = response.to_array();
                // class.write_packet(&buf[..len]).await.unwrap();
                cdc_acm_ep2_write(&buf[0..len]).await;
            }
            Command::GetTim => {
                let date = rtc::get_date();
                let time = rtc::get_time();
                let response =
                    eb_cmds::Response::GetTim(date.0, date.1, date.2, time.0, time.1, time.2);
                let (buf, len) = response.to_array();
                // class.write_packet(&buf[..len]).await.unwrap();

                cdc_acm_ep2_write(&buf[0..len]).await;
            }
            Command::GetPic(id) => {
                let start_block = (id + IMG_START_BLOCK) * IMG_SIZE;
                let _ = sd.read_single_block_async(&mut PIC_BUF[..], start_block).await.unwrap();

                // pic_buf[0] = (pic_end >> 24) as u8;
                // pic_buf[1] = ((pic_end >> 16) & 0xff) as u8;
                // pic_buf[2] = ((pic_end >> 8) & 0xff) as u8;
                // pic_buf[3] = (pic_end & 0xff) as u8;
                // get the picture length from the first 4 bytes
                let pic_end = ((PIC_BUF[0] as u32) << 24)
                    | ((PIC_BUF[1] as u32) << 16)
                    | ((PIC_BUF[2] as u32) << 8)
                    | (PIC_BUF[3] as u32);
                let _ = sd.read_multiple_blocks_async(&mut PIC_BUF[..], start_block, IMG_SIZE).await;
                // only allow to send 30k data each time
                for i in 0..50 {
                    let begin = i * 30_000;
                    let begin = core::cmp::max(16, begin);
                    let mut end = (i + 1) * 30_000;
                    if end > pic_end as usize {
                        end = pic_end as usize;
                    }
                    cdc_acm_ep2_write(&PIC_BUF[begin..end]).await;
                    if end == pic_end as usize {
                        break;
                    }
                }
            }
            Command::GetPicNum => {
                let mut buf:Aligned<aligned::A4, [u8; 512]> = Aligned([0u8; 512]);
                sd.read_single_block_async(&mut buf[..], SIZE_BLOCK).await.unwrap();
                let num = ((buf[0] as u32) << 24)
                    | ((buf[1] as u32) << 16)
                    | ((buf[2] as u32) << 8)
                    | (buf[3] as u32);
                // ebcmd::Response::GetPicNum(num)
                let res = eb_cmds::Response::GetPicNum(num);
                let (buf, len) = res.to_array();
                let mut buf_align:Aligned<aligned::A4, [u8; 64]> = Aligned([0u8; 64]);
                for i in 0..len {
                    buf_align[i] = buf[i];
                }
                // cdc_acm_ep2_write(&buf[0..len]).await;
                cdc_acm_ep2_write(&buf_align[..len]).await;
            }
            Command::ClearPic => {
                let mut buf:Aligned<aligned::A4, [u8; 512]> = Aligned([0u8; 512]);
                sd.write_single_block_async(&mut buf[..], SIZE_BLOCK).await;

            }
        }
    }
}
