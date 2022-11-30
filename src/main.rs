#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy::blocking_mutex::raw::ThreadModeRawMutex;
use embassy::mutex::Mutex;
use embassy::executor::Spawner;
use embassy::time::{Delay, Duration, Timer, Instant};
use embassy_stm32::adc::Adc;
use embassy_stm32::Peripherals;
use embassy_stm32::peripherals::{I2C2, PA15, PC13, PB3, PB5, PB6, PB4,PB9};
use embassy_stm32::gpio::{Pin,AnyPin, Output, Level, Speed,Input, Pull};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt;
use embassy_stm32::time::Hertz;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::pwm::{simple_pwm::SimplePwm, Channel};
use defmt::*;
use defmt_rtt as _; 

// global logger
use panic_probe as _;

use embedded_graphics::{
    mono_font::{iso_8859_1::{FONT_6X10,FONT_10X20}, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text,Alignment},
};
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use arrform::{arrform, ArrForm};
use thermocouple::{prelude::*, KType};
//use rotary_encoder_hal::{Direction, Rotary};
use pid::Pid;

mod thermistor;

const MAX_TEMP:i32= 500;

struct Station{
    enabled:bool,
    temperature:i32,
    setpoint:i32,
    tc_millivolts:u16,
    th_millivolts:u32,
    cj_temperature:f32,
    duty:u16,
    duty_p:u16,
    duty_f:f32,
    duty_max:f32,
    using_pid:bool
}

static STATION: Mutex<ThreadModeRawMutex, Station> = Mutex::new(Station{
    enabled:false,
    temperature:0,
    setpoint:350,
    tc_millivolts:0,
    th_millivolts: 0,
    cj_temperature: 0.0,
    duty:0,
    duty_p:0,
    duty_f:0.0,
    duty_max:0.0,
    using_pid:false
});

const GAIN:f32=100.7;// calculated

#[embassy::main]
async fn main(spawner: Spawner, p: Peripherals) {
    //info!("main");
    const SAMPLES_TH:u32=10;
    const SAMPLES_TC:u32=30;
    
    let thermocouple = KType::new();
    //PWM
    let mut pwm = SimplePwm::new_1ch_2(p.TIM4, p.PB9, Hertz(10000));
    let pwm_max = pwm.get_max_duty();
    pwm.set_duty(Channel::Ch4, 0);
    pwm.enable(Channel::Ch4);
    
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
    //info!("spawn display task");
    
    spawner.spawn(display_task(display)).unwrap();
    
    let led = Output::new(p.PC13, Level::High, Speed::Low);
    spawner.spawn(blink(led)).unwrap();
    
    //inputs
    
    //let enc_a = Input::new(p.PA15, Pull::None);
    //let enc_a = ExtiInput::new(enc_a, p.EXTI15);
    //let enc_b = Input::new(p.PB3, Pull::None);
    //let enc_b = ExtiInput::new(enc_b, p.EXTI3);
    //spawner.spawn(input_task1(enc_a)).unwrap();
    //spawner.spawn(input_task2(enc_b)).unwrap();
    let btn1 = Input::new(p.PB4, Pull::Up);
    //let btn1 = ExtiInput::new(btn1, p.EXTI4);
    let btn2 = Input::new(p.PB5, Pull::Up);
    //let btn2 = ExtiInput::new(btn2, p.EXTI5);
    let btn3 = Input::new(p.PB6, Pull::Up);
    //let btn3 = ExtiInput::new(btn3, p.EXTI6);
    
    spawner.spawn(btn1_task(btn1,btn2,btn3)).unwrap();
    
    //PID
    let lim=(pwm_max-1) as f32;
    let mut setpoint={
        let mut station=STATION.lock().await;
        station.duty_f=lim;
        station.duty_max=lim;
        station.setpoint
    };
    
    let mut pid = Pid::new(10.0, 0.01, 120.0, lim, lim, lim, lim, setpoint as f32);
    
    let mut temperature: Celsius=Celsius(0.0);
    //let mut tunning:bool=true;
    loop {        
        let mut acc=0;
        for _a in 0..SAMPLES_TH {                        
            let tt:u32 =adc.read(&mut adc_pin1) as u32;
            acc+=tt;
        }
        let mv=adc.to_millivolts((acc/SAMPLES_TH)as u16);
        let temp_cj=thermistor::temperature(mv as u32,vdd);

        let ignore_pid=temperature.0< (setpoint as f32 -10.0);
        //if !ignore_pid {
            //pwm.set_duty(Channel::Ch4, 0);
            //Timer::after(Duration::from_millis(10)).await;//
        //}        
        acc=0;
        for _a in 0..SAMPLES_TC {                        
            let tt:u32 =adc.read(&mut adc_pin2) as u32;
            acc+=tt;
        }        
        //if !ignore_pid {
            //pwm.set_duty(Channel::Ch4, duty);
        //}
        let mut tc_mv=adc.to_millivolts((acc/SAMPLES_TC)as u16);
        if tc_mv>=12 {
            tc_mv-=12;
        }
        let mv=(tc_mv as f32)/GAIN;
        
        let cj_mv=thermocouple.sense_voltage(Celsius(temp_cj));
        temperature = thermocouple.sense_temperature(mv.millivolts()+cj_mv);
        
        
        let enabled={
            let station=STATION.lock().await;
            station.enabled
        };
        
        let duty=if enabled {
            {
                let station=STATION.lock().await;
                pid.setpoint=station.setpoint as f32;
                setpoint=station.setpoint;
            }
            if !ignore_pid {
                let mut output = (pid.next_control_output(temperature.0).output) as u16;
                if output >=pwm_max {
                    output=pwm_max-1;
                }    
                output
            }else{
                pwm_max-1
            }
        }else{
            0
        };
        
        pwm.set_duty(Channel::Ch4, duty);
        {//update values to display
            let mut station=STATION.lock().await;
            station.temperature=temperature.0 as i32;
            station.tc_millivolts=tc_mv;
            station.th_millivolts=(mv*1000.0) as u32;
            station.cj_temperature=temp_cj;            
            station.duty=duty;
            station.using_pid=!ignore_pid;
            station.duty_p=((((duty+1) as f32)/lim)*100.0) as u16;
        }
        Timer::after(Duration::from_millis(5)).await;
    }
}

#[embassy::task]
async fn blink(
    mut led:Output<'static, PC13>,
){
    loop{
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy::task]
async fn btn1_task(
    btn1:Input<'static, PB4>,
    btn2:Input<'static, PB5>,
    btn3:Input<'static, PB6>
)
{
    const DEBOUNCE:u64=10;
    loop
    {
        if btn1.is_low() {//debounce
            Timer::after(Duration::from_millis(DEBOUNCE)).await;
            if btn1.is_low() {
                let mut station=STATION.lock().await;
                station.enabled=!station.enabled;
            }
        }    
        if btn2.is_low() {//debounce
            Timer::after(Duration::from_millis(DEBOUNCE)).await;
            if btn2.is_low() {
                let mut station=STATION.lock().await;
                if station.setpoint<MAX_TEMP {
                    station.setpoint+=1;
                }
            }
        }
        if btn3.is_low() {//debounce
            Timer::after(Duration::from_millis(DEBOUNCE)).await;
            if btn3.is_low() {
                let mut station=STATION.lock().await;
                if station.setpoint>0 {
                    station.setpoint-=1;
                }
            }
        }
        Timer::after(Duration::from_millis(4)).await;
    }
}

type Disp=Ssd1306<I2CInterface<I2c<'static, I2C2>>,DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;

#[embassy::task]
async fn display_task(
    mut display:Disp
    ) 
{
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let text_style2 = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(BinaryColor::On)
        .build();
    display.clear();  
    Text::with_alignment("Starting", Point::new(64,32), text_style, Alignment::Center).
            draw(&mut display).unwrap();
    display.flush().unwrap();
    //Timer::after(Duration::from_millis(1000)).await;
    
    let mut af = ArrForm::<64>::new();    
    loop {
        display.clear();        
        {
            Text::with_alignment("Soldering Station", Point::new(64,10), text_style, Alignment::Center).
            draw(&mut display).unwrap();
            let station=STATION.lock().await;
            let enabled=if station.enabled{"On"}else{"Off"};
            let using_pid=if station.using_pid{"Pid"}else{"*"};
            
            af.format(format_args!("{}°/{}°",station.temperature,station.setpoint)).unwrap();
            Text::with_alignment(
                af.as_str(), 
                Point::new(64,32), text_style2, Alignment::Center)
            .draw(&mut display).unwrap();
        
            af.format(format_args!("{}-{}° {} {}% {}",
            enabled,
            station.cj_temperature as u32,
            station.tc_millivolts,
            //station.th_millivolts,
            station.duty_p,
            using_pid
            )).unwrap();
            /*af.format(format_args!("{} {} {} {} - {}",
            enabled,
            (station.kp *100.0) as u32,
            (station.ki *100.0) as u32,
            (station.kd *100.0) as u32,
            (station.duty_f) as u32
            )).unwrap();*/
            Text::with_baseline(
                af.as_str(), 
                Point::new(0,52), text_style, Baseline::Top)
            .draw(&mut display).unwrap();
        }
        display.flush().unwrap();

        Timer::after(Duration::from_millis(10)).await;
       
    }
}
