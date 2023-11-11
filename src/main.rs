// sudo adduser <user> dialout
// sudo chmod a+rw /dev/ttyACM0
//
// https://medium.com/@rajeshpachaikani/connect-esp32-to-wifi-with-rust-7d12532f539b
// https://github.com/esp-rs/std-training/blob/main/intro/http-server/examples/http_server.rs
// https://esp-rs.github.io/book/
// https://github.com/esp-rs/esp-idf-hal/blob/master/src/ledc.rs

use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use embedded_svc::{
    http::server::{HandlerError, Request},
    http::Method,
    io::Write,
};
use esp_idf_hal::delay::Delay;
use esp_idf_hal::{gpio::*, peripherals::Peripherals};
use esp_idf_hal::adc::config::Config as adc_Config;
use esp_idf_hal::adc::*;
// use esp_idf_hal::adc::attenuation;
//use esp_idf_hal::adc::{AdcChannelDriver, AdcDriver, Attenuation};
use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver};
use esp_idf_hal::units::*;
use esp_idf_svc::http::server::Configuration as SVC_Configuration;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    http::server::{EspHttpConnection, EspHttpServer},
    nvs::EspDefaultNvsPartition,
    wifi::EspWifi,
};
use esp_idf_sys as _;
use querystring;
use serde_json;
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    thread,
    thread::sleep,
    time::Duration,
};

use device::{Action, Device};
use encoder::Encoder;

fn main() {
    esp_idf_sys::link_patches(); //Needed for esp32-rs
    println!("Entered Main function!");
    let peripherals = Peripherals::take().unwrap();
    let sys_loop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().unwrap();


    let light_1 = Arc::new(Mutex::new(Device {
        name: "bed light".to_string(),
        action: Action::Off,
        available_actions: Vec::from([
            Action::On,
            Action::Off,
            Action::Up,
            Action::Down,
            Action::Set,
        ]),
        default_target: 3,
        duty_cycles: [0, 6, 12, 25, 50, 96],
        target: 0,
        freq_kHz: 1,
//        freq_Hz: 100.Hz().into(),
    }));

    // let freq_khz = { (1. /light_1.lock().unwrap().period_ms as f64 * 1000.).round() as usize };
    let driver_1 = Arc::new(Mutex::new(LedcDriver::new(
        peripherals.ledc.channel0,
        LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::new().frequency(100.Hz().into()),
        ).unwrap(),
        peripherals.pins.gpio7,
    ).unwrap()));

    // roof_vent Manager
    thread::spawn(move || {
        let voltages = [680, 1360, 2040, 2720, 3000, 50000];
        let mut pin_a = peripherals.pins.gpio5;
        let mut pin_b = peripherals.pins.gpio6;
        let encoder = Encoder::new(peripherals.pcnt0, &mut pin_a, &mut pin_b).unwrap();
        let mut last_value = 0;
        loop {
            {
                let value = encoder.get_value().unwrap();
                if value < last_value {
                    println!("-- {value}");
                    last_value = value;
                } else if value > last_value {
                    println!("+ {value}");
                    last_value = value;
                }
            }
            Delay::delay_ms(200);
        }
    });

    loop {
        sleep(Duration::new(1000, 0));
    }
}

#[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
// esp-idf encoder implementation using v4 pcnt api
mod encoder {
    use std::cmp::min;
    use std::sync::atomic::AtomicI64;
    use std::sync::atomic::Ordering;
    use std::sync::Arc;

    use esp_idf_hal::gpio::AnyInputPin;
    use esp_idf_hal::gpio::InputPin;
    use esp_idf_hal::pcnt::*;
    use esp_idf_hal::peripheral::Peripheral;
    use esp_idf_sys::EspError;

    const LOW_LIMIT: i16 = -100;
    const HIGH_LIMIT: i16 = 100;

    pub struct Encoder<'d> {
        unit: PcntDriver<'d>,
        approx_value: Arc<AtomicI64>,
    }

    impl<'d> Encoder<'d> {
        pub fn new<PCNT: Pcnt>(
            pcnt: impl Peripheral<P = PCNT> + 'd,
            pin_a: impl Peripheral<P = impl InputPin> + 'd,
            pin_b: impl Peripheral<P = impl InputPin> + 'd,
        ) -> Result<Self, EspError> {
            let mut unit = PcntDriver::new(
                pcnt,
                Some(pin_a),
                Some(pin_b),
                Option::<AnyInputPin>::None,
                Option::<AnyInputPin>::None,
            )?;
            unit.channel_config(
                PcntChannel::Channel0,
                PinIndex::Pin0,
                PinIndex::Pin1,
                &PcntChannelConfig {
                    lctrl_mode: PcntControlMode::Reverse,
                    hctrl_mode: PcntControlMode::Keep,
                    pos_mode: PcntCountMode::Decrement,
                    neg_mode: PcntCountMode::Increment,
                    counter_h_lim: HIGH_LIMIT,
                    counter_l_lim: LOW_LIMIT,
                },
            )?;
            unit.channel_config(
                PcntChannel::Channel1,
                PinIndex::Pin1,
                PinIndex::Pin0,
                &PcntChannelConfig {
                    lctrl_mode: PcntControlMode::Reverse,
                    hctrl_mode: PcntControlMode::Keep,
                    pos_mode: PcntCountMode::Increment,
                    neg_mode: PcntCountMode::Decrement,
                    counter_h_lim: HIGH_LIMIT,
                    counter_l_lim: LOW_LIMIT,
                },
            )?;

            unit.set_filter_value(min(10 * 80, 1023))?;
            unit.filter_enable()?;

            let approx_value = Arc::new(AtomicI64::new(0));
            // unsafe interrupt code to catch the upper and lower limits from the encoder
            // and track the overflow in `value: Arc<AtomicI64>` - I plan to use this for
            // a wheeled robot's odomerty
            unsafe {
                let approx_value = approx_value.clone();
                unit.subscribe(move |status| {
                    let status = PcntEventType::from_repr_truncated(status);
                    if status.contains(PcntEvent::HighLimit) {
                        approx_value.fetch_add(HIGH_LIMIT as i64, Ordering::SeqCst);
                    }
                    if status.contains(PcntEvent::LowLimit) {
                        approx_value.fetch_add(LOW_LIMIT as i64, Ordering::SeqCst);
                    }
                })?;
            }
            unit.event_enable(PcntEvent::HighLimit)?;
            unit.event_enable(PcntEvent::LowLimit)?;
            unit.counter_pause()?;
            unit.counter_clear()?;
            unit.counter_resume()?;

            Ok(Self { unit, approx_value })
        }

        pub fn get_value(&self) -> Result<i64, EspError> {
            let value =
                self.approx_value.load(Ordering::Relaxed) + self.unit.get_counter_value()? as i64;
            Ok(value)
        }
    }
}
