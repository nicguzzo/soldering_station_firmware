#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]


use defmt::*;
use embassy::blocking_mutex::raw::ThreadModeRawMutex;
use embassy::mutex::Mutex;
use embassy::executor::Spawner;
use embassy::time::{Delay, Duration, Timer};
use embassy_stm32::adc::Adc;
use embassy_stm32::Peripherals;
use embassy_stm32::peripherals::I2C2;
use embassy_stm32::gpio::{Pin,AnyPin, Output, Level, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt;
use embassy_stm32::time::Hertz;

use defmt_rtt as _; 
//use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::iso_8859_1::FONT_10X20;
// global logger
use panic_probe as _;

use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use arrform::{arrform, ArrForm};
use thermocouple::{prelude::*, KType};
use rotary_encoder_hal::{Direction, Rotary};
mod thermistor;
static MAX_TEMP:i32= 500;

struct Station{
    temperature:i32,
    setpoint:i32,
    tc_millivolts:u32,
    th_millivolts:u32,
    cj_temperature:f32,
}

static STATION: Mutex<ThreadModeRawMutex, Station> = Mutex::new(Station{
    temperature:0,
    setpoint:350,
    tc_millivolts:0,
    th_millivolts: 0,
    cj_temperature: 0.0,
});

#[embassy::main]
async fn main(spawner: Spawner, p: Peripherals) {
    info!("main");
    
    //ADC
    let mut adc = Adc::new(p.ADC1, &mut Delay);
    let mut adc_pin1 = p.PA0;
    let mut adc_pin2 = p.PA1;
    let mut vref = adc.enable_vref(&mut Delay);
    let vdd=adc.calibrate(&mut vref);
    
    //DISPLAY
    let irq = interrupt::take!(I2C2_EV);
    let i2c = I2c::new(p.I2C2, p.PB10, p.PB11,Hertz(400_000));
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();
    display.init().unwrap();
    info!("spawn display task");
    
    spawner.spawn(display_task(display)).unwrap();
    const SAMPLES:u32=100;
    const GAIN:f32=50.0;
    const TC_AREF:f32=460.0;//tc biasing voltage in mv
    let thermocouple = KType::new();
    
    loop {
        
        let mut acc=0;
        for _a in 0..SAMPLES {                        
            let tt:u32 =adc.read(&mut adc_pin1) as u32;
            acc+=tt;
        }
        let mv=adc.to_millivolts((acc/SAMPLES)as u16);
        let th_mv=mv as u32;
        let temp_cj=thermistor::temperature(mv as u32,vdd);

        acc=0;
        for _a in 0..SAMPLES {                        
            let tt:u32 =adc.read(&mut adc_pin2) as u32;
            acc+=tt;
        }
        let mv=(((adc.to_millivolts((acc/SAMPLES)as u16)as f32)-TC_AREF)/GAIN);
        let tc_mv=mv as u32;
        let cj_mv=thermocouple.sense_voltage(Celsius(temp_cj));
        let temperature: Celsius = thermocouple.sense_temperature(mv.millivolts()+cj_mv);        
        
        {//update values to display
            let mut station=STATION.lock().await; 
            station.temperature=temperature.0 as i32;
            station.tc_millivolts=tc_mv;
            station.th_millivolts=th_mv;
            station.cj_temperature=temp_cj;
        }
                
        
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy::task]
async fn display_task(
    mut display:
    Ssd1306<
        I2CInterface<
            I2c<'static, I2C2>
        >, 
        DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>) {

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let text_style2 = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(BinaryColor::On)
        .build();

    
    display.flush().unwrap();
    Timer::after(Duration::from_millis(1000)).await;
    loop {
        display.clear();        
        {
            Text::with_baseline("Soldering Station", Point::zero(), text_style, Baseline::Top).draw(&mut display).unwrap();
            let station=STATION.lock().await;
                    
            let af = arrform!(30, "{}°/{}°",station.temperature,station.setpoint);
            Text::with_baseline(af.as_str(), Point::new(0,16), text_style2, Baseline::Top)
            .draw(&mut display).unwrap();
        
            let af = arrform!(30, "cjt {}° - {} mV ",station.cj_temperature as u32,station.tc_millivolts);
            Text::with_baseline(af.as_str(), Point::new(0,52), text_style, Baseline::Top)
            .draw(&mut display).unwrap();
        }
        display.flush().unwrap();

        Timer::after(Duration::from_millis(30)).await;
       
    }
}
