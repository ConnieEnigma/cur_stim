#![feature(noop_waker)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt_rtt as _;
use embassy_executor::Spawner;
use libm;
mod utils;

use u5_lib::{
    clock::{self, delay_ms, delay_s, delay_us},
    com_interface::ComInterface,
    exti,
    gpio::{self, GpioPort,  TIM1_CH2_PA9, TIM1_CH3_PA10, TIM3_CH1_PA6},
    i2c::{self, I2c},
    low_power::{no_deep_sleep_request, Executor},
    task,
    tim::{Config, TIM1, TIM3},
    *,
};

//use tim::{Config, TIM1};

fn i2c_init() -> (I2c, I2c) {
    let i2c_config_plus = i2c::I2cConfig::new(1, 100_000, gpio::I2C1_SCL_PB6, gpio::I2C1_SDA_PB3);
    let i2c_plus = I2c::new(i2c_config_plus).unwrap();
    let i2c_config_minus = i2c::I2cConfig::new(2, 100_000, gpio::I2C2_SCL_PB13, gpio::I2C2_SDA_PB14);
    let i2c_minus = I2c::new(i2c_config_minus).unwrap();
    (i2c_plus, i2c_minus)
}
fn switch_led_setup() -> ( gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort,
    gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort, gpio::GpioPort){
    let red: gpio::GpioPort = gpio::PB7;
    let green: gpio::GpioPort = gpio::PB8;
    let s0: gpio::GpioPort = gpio::PB15;
    let s1: gpio::GpioPort = gpio::PA9;
    let s2: gpio::GpioPort = gpio::PA10;
    let s3: gpio::GpioPort = gpio::PB4;
    let s4: gpio::GpioPort = gpio::PB5;
    let s5: gpio::GpioPort = gpio::PA5;
    // let s6: gpio::GpioPort = gpio::PA6;
    let s7: gpio::GpioPort = gpio::PA4;
    let s8: gpio::GpioPort = gpio::PA3;
    let chopper_clk: gpio::GpioPort = gpio::PA7;
    
    
    green.setup();
    red.setup();
    s0.setup();
    s1.setup();
    s2.setup();
    s3.setup();
    s4.setup();
    s5.setup();
    //s6.setup();
    s7.setup();
    s8.setup();
    chopper_clk.setup();
    (green, red, s0, s1, s2, s3, s4, s5, s7, s8, chopper_clk)

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
    clock::init_clock(true, true, 16_000_000, true, clock::ClockFreqs::KernelFreq16Mhz);
    unsafe {
        no_deep_sleep_request();
    }
    //TIM1_CH2_PA9.setup(); //s1 
    //TIM1_CH3_PA10.setup(); // s2 
    TIM3_CH1_PA6.setup(); // s6. chopper frequency. 
    let _ = TIM1.init(Config::default());
    let _ = TIM3.init(Config::default());
    // TIM1.set_pwm(1, 160, 80);
    //TIM1.set_pwm(2, 16000, 1600);  // (2, 16000, 1600) 1kHz 100us pulse. (2, 8000, 800) 2kHz 50us pulse. (2, 3200, 320) 5kHz 20us pulse. (2, 1600, 160) 10kHz 10us pulse. 
    //TIM1.set_pwm(3, 16000, 4000);
    TIM3.set_pwm(1, 16, 8);
    // TIM3.set_pwm(2, 160, 80);
    //TIM1.enable_output(2);
    //TIM1.enable_output(3);
    TIM3.enable_output(1);
    //TIM3.enable_output(2);
    clock::set_mco(
        gpio::GPIO_MCO_PA8,
        clock::Mcosel::HSE,
        clock::Mcopre::DIV8,
    ); // clock. which use PA8 as clock output

    defmt::info!("setup led finished!");
    let (green, red, s0,s1, s2, s3, s4, s5, s7, s8, chopper_clk) = switch_led_setup();
    s0.set_low(); // capacitor connection
    s1.set_low(); //pos DAC connection
    s2.set_low(); //neg DAC connection
    s3.set_high(); // pos DAC & capacitor -> Vstm 
    s4.set_high(); //neg DAC power
    s5.set_high  (); //pos DAC power 
    //s6.set_low(); //chopper switch
    s7.set_high(); // chopper and filter power
    s8.set_low(); // 1.5v DC connection
    chopper_clk.set_low();

    let Ipos = 0.5;
    let Ipos_f64 = Ipos as f64;
    let Ineg = -1.0;
    let Ipos_hex = utils::cur_coding(Ipos);
    let Ineg_hex = utils::cur_coding(Ineg);


    //let Ineg = 1.0; //1.0mA is the negative current
    //let product2 = Ineg * num1;
    //let Ineg_hex = libm::round(product2) as u8;
    //defmt::info!("Ineg is {}", Ipos_hex);


    let (mut i2c_plus, mut i2c_minus) = i2c_init();
    for i in 0..4{
        //i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, 0xA8]);
        i2c_send(&mut i2c_plus, POS_DAC_1_ADDR, [DAC_REG_BASE + i, Ipos_hex]);
    }
    for i in 0..4{     
        //i2c_send(&mut i2c_plus, POS_DAC_2_ADDR, [DAC_REG_BASE + i, 0xA8]);     
        i2c_send(&mut i2c_plus, POS_DAC_2_ADDR, [DAC_REG_BASE + i, Ipos_hex]);
    }
    for i in 0..4 {
        i2c_send(&mut i2c_minus, NEG_DAC_1_ADDR, [DAC_REG_BASE + i, Ineg_hex]);
        //i2c_send(&mut i2c_minus, NEG_DAC_1_ADDR, [DAC_REG_BASE + i, 50]);
    }
    for i in 0..4 {
        i2c_send(&mut i2c_minus, NEG_DAC_2_ADDR, [DAC_REG_BASE + i, Ineg_hex]);
        //i2c_send(&mut i2c_minus, NEG_DAC_2_ADDR, [DAC_REG_BASE + i, 50]);
    }
    defmt::info!("i2c finished!");


    let tmp_adc = adc::ADC1;
    tmp_adc.init();
    let adc_pin = gpio::ADC1_IN5_PA0;
    adc_pin.setup();
    let mut counter = 0;
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
    delay_s(1);
    let mut adc_values = [0.0; 5];
    let mut adc_indexer = 0;
    let mut adc_sum = 0.0;
    let mut vbase = 0.0;
    let mut vbase_sum = 0.0;

    //measure electrode voltage at start
    let res = tmp_adc.start_conversion_sw(5);
    let vbase = res as f64 * vref;
    vbase_sum -= adc_values[adc_indexer]; 
    adc_values[adc_indexer] = vbase; 
    vbase_sum += vbase;
    let vbase_avg = vbase_sum/5.0;
    defmt::info!("Vabse is {}", vbase_avg);

    //measure Rp before stimulation begin
    for i in 0..6{
        s1.set_high();
        delay_ms(5);
        let res = tmp_adc.start_conversion_sw(5);
        let vpos = res as f64 * vref;
        adc_sum -= adc_values[adc_indexer]; 
        adc_values[adc_indexer] = vpos; 
        adc_sum += vpos;
        defmt::info!("Vtotal is {}", adc_sum);
    }
    let Ipos_f64:f64 = Ipos as f64;
    let mut R_total = (adc_sum - vbase_avg)/Ipos_f64 * 1000.0; 
    defmt::info!("Rtotal is: {}", R_total);
    s1.set_low();

    loop {
        //s2.set_high();
        s2.set_high();
        delay_us(100);
        //s2.set_low();
        s2.set_low();
        delay_us(10);
        s1.set_high();
        delay_us(200);
        s1.set_low();
        delay_us(690);
    
        counter += 1;
        if counter >= 5000 {
            TIM1_CH2_PA9.setup();
            TIM1.set_pwm(2, 1600, 1580);
            TIM1.enable_output(2);
            let res = tmp_adc.start_conversion_sw(5); 
            let vpos = res as f64 * vref;
            adc_sum -= adc_values[adc_indexer]; 
            adc_values[adc_indexer] = vpos; 
            adc_sum += vpos; 
            defmt::info!("adc average value: {}", adc_sum/5.0);
            
            let Rs = (adc_sum/5.0 - vbase_avg) * 1000.0 / Ipos_f64;
            let Rp = R_total - Rs;
            defmt::info!("Rs is {}", Rs);
            defmt::info!("Rp is {}", Rp);
            // let C0 = 1e-6;
            
            // let tauRC =  (Rs * Rp * Cp)/(Rs + Rp);
            // let V0 = Ipos* t1/C0;
            // let I0 = V0/(Rs + Rp);
            // let base = 1 - Ineg*t1*1e-3/(I0*tauRC);
            // let t2 = -tauRC*base.ln();

            //defmt::info!("adc average value: {}", adc_sum/5.0);
            // if adc_sum / 5.0 > 1.6 {
            //     red.set_high();
            // }
            // else {
            //     red.set_low();
            // }
            s1.setup();
            s2.setup();
            adc_indexer += 1;
            adc_indexer %= 5;
            counter = 0;
        }
        green.toggle();
        //delay_s(3);
        // red.toggle();
        //delay_ms(1);
        //defmt::info!("toggle leds");
    }
}


#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        spawner.spawn(async_main(spawner)).unwrap();
    });
}

