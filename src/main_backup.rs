//! PCNT decoding a rotary encoder
//!
//! To try this out, connect a rotary encoder to pins 5 and 6, the common should be grounded
//!
//! Note that PCNT only track a singed 16bit value.  We use interrupts to detect a LOW and HIGH
//! threshold and track how much that accounts for and provide an i64 value result

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
use esp_idf_hal::adc::config::Config as adc_Config;
use esp_idf_hal::adc::*;
use esp_idf_hal::delay::Delay;
use esp_idf_hal::{gpio::*, peripherals::Peripherals};
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
    time::{Duration, Instant},
};

use device::{Action, Device};
use encoder::Encoder;

#[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
fn main() {
    //    use anyhow::Context;

    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    println!("setup pins");
    let peripherals = Peripherals::take().unwrap();
    let sys_loop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().unwrap();

    let mut wifi_driver = EspWifi::new(peripherals.modem, sys_loop, Some(nvs)).unwrap();
    wifi_driver
        .set_configuration(&Configuration::Client(ClientConfiguration {
            ssid: "ATT5p3XSm9".into(),
            password: "46b9ap5na4iu".into(),
            ..Default::default()
        }))
        .unwrap();
    wifi_driver.start().unwrap();
    wifi_driver.connect().unwrap();
    while !wifi_driver.is_connected().unwrap() {
        let config = wifi_driver.get_configuration().unwrap();
        println!("Waiting for station {:?}", config);
    }
    println!("Should be connected now");

    let lights = Arc::new(Mutex::new(Vec::from([
        Device {
            name: "bedroom light".to_string(),
            action: Action::Off,
            available_actions: Vec::from([
                Action::On,
                Action::Off,
                Action::Up,
                Action::Down,
                Action::Set,
            ]),
            default_target: 3,
            duty_cycles: [0, 2, 4, 8, 16, 32, 64, 96],
            target: 0,
            freq_Hz: 1000,
        },
        Device {
            name: "kitchen light".to_string(),
            action: Action::Off,
            available_actions: Vec::from([
                Action::On,
                Action::Off,
                Action::Up,
                Action::Down,
                Action::Set,
            ]),
            default_target: 3,
            duty_cycles: [0, 2, 4, 8, 16, 32, 64, 96],
            target: 0,
            freq_Hz: 1000,
        },
    ])));
    let light_1 = Arc::new(Mutex::new(Device {
        name: "kitchen light".to_string(),
        action: Action::Off,
        available_actions: Vec::from([
            Action::On,
            Action::Off,
            Action::Up,
            Action::Down,
            Action::Set,
        ]),
        default_target: 3,
        duty_cycles: [0, 2, 4, 8, 16, 32, 64, 96],
        target: 0,
        freq_Hz: 1000,
    }));

    let light_2 = Arc::new(Mutex::new(Device {
        name: "bedroom light".to_string(),
        action: Action::Off,
        available_actions: Vec::from([
            Action::On,
            Action::Off,
            Action::Up,
            Action::Down,
            Action::Set,
        ]),
        default_target: 3,
        duty_cycles: [0, 2, 4, 8, 16, 32, 64, 96],
        target: 0,
        freq_Hz: 1000,
    }));

    let light_1_clone = light_1.clone();
    let light_2_clone = light_2.clone();
    let lights_clone = lights.clone();
    thread::spawn(move || {
        println!("setup encoder");
        let mut pin_a_1 = peripherals.pins.gpio5;
        let mut pin_b_1 = peripherals.pins.gpio6;
        let mut pin_a_2 = peripherals.pins.gpio7;
        let mut pin_b_2 = peripherals.pins.gpio8;
        let mut pins = Vec::from([
        (peripherals.pins.gpio5, peripherals.pins.gpio6),
        (peripherals.pins.gpio7, peripherals.pins.gpio8),
        ]);
        let encoder_1 = Encoder::new(peripherals.pcnt0, &mut pin_a_1, &mut pin_b_1).unwrap();
        let encoder_2 = Encoder::new(peripherals.pcnt1, &mut pin_a_2, &mut pin_b_2).unwrap();
        let encoders = Vec::from([
            Encoder::new(peripherals.pcnt0, &mut pin_a_1, &mut pin_b_1).unwrap(),
            Encoder::new(peripherals.pcnt1, &mut pin_a_2, &mut pin_b_2).unwrap(),
        ]);
        let mut last_encoder_value_1 = 0i64;
        let mut last_encoder_time_1 = Instant::now();
        let mut last_encoder_value_2 = 0i64;
        let mut last_encoder_time_2 = Instant::now();
        let mut 
        loop {
            let encoder_value = encoder_1.get_value().unwrap();
            if encoder_value != last_encoder_value_1 {
                let current_time = Instant::now();
                let time_since_last_check = current_time.duration_since(last_encoder_time_1);
                if time_since_last_check > Duration::from_millis(100) {
                    {
                        let mut light = light_1_clone.lock().unwrap();
                        if encoder_value > last_encoder_value_1 {
                            let _ = light.take_action(Action::Up, None);
                        } else {
                            let _ = light.take_action(Action::Down, None);
                        }
                    }
                    last_encoder_time_1 = Instant::now();
                }
                last_encoder_value_1 = encoder_value;
            }
            let encoder_value = encoder_2.get_value().unwrap();
            if encoder_value != last_encoder_value_2 {
                let current_time = Instant::now();
                let time_since_last_check = current_time.duration_since(last_encoder_time_2);
                if time_since_last_check > Duration::from_millis(100) {
                    {
                        let mut light = light_2_clone.lock().unwrap();
                        if encoder_value > last_encoder_value_2 {
                            let _ = light.take_action(Action::Up, None);
                        } else {
                            let _ = light.take_action(Action::Down, None);
                        }
                    }
                    last_encoder_time_2 = Instant::now();
                }
                last_encoder_value_2 = encoder_value;
            }
            Delay::delay_ms(100u32);
        }
    });

    let light_1_clone = light_1.clone();
    let light_2_clone = light_2.clone();
    thread::spawn(move || {
        let mut driver_1 = LedcDriver::new(
            peripherals.ledc.channel0,
            LedcTimerDriver::new(
                peripherals.ledc.timer0,
                &TimerConfig::new().frequency({ light_1_clone.lock().unwrap() }.freq_Hz.Hz()),
            )
            .unwrap(),
            peripherals.pins.gpio9,
        )
        .unwrap();
        let mut driver_2 = LedcDriver::new(
            peripherals.ledc.channel1,
            LedcTimerDriver::new(
                peripherals.ledc.timer1,
                &TimerConfig::new().frequency({ light_2_clone.lock().unwrap() }.freq_Hz.Hz()),
            )
            .unwrap(),
            peripherals.pins.gpio10,
        )
        .unwrap();
        let max_duty_1 = driver_1.get_max_duty();
        let max_duty_2 = driver_2.get_max_duty();
        loop {
            {
                let light_1 = light_1_clone.lock().unwrap();
                let light_2 = light_2_clone.lock().unwrap();

                let duty_cycle = light_1.get_duty_cycle() * max_duty_1 / 100;
                let _ = driver_1.set_duty(duty_cycle);

                let duty_cycle = light_2.get_duty_cycle() * max_duty_2 / 100;
                let _ = driver_2.set_duty(duty_cycle);
            }
            Delay::delay_ms(100u32);
        }
    });

    let mut server = EspHttpServer::new(&SVC_Configuration::default()).unwrap();
    server
        .fn_handler("/", Method::Get, |request| {
            println!("thing recieved!!!!!!!!!!!!!!!!!!!");
            println!("request uri: {}", request.uri());
            let mut response = request.into_ok_response()?;
            response.write_all("payload!!!!!".as_bytes())?;
            Ok(())
        })
        .unwrap();
    let light_1_clone = light_1.clone();
    let path = { light_1.lock().unwrap() }.name.clone().replace(" ", "%20");
    server
        .fn_handler(
            format!("/{path}_status").as_str(),
            Method::Get,
            move |request| {
                println!("Devices777777777777777777777777777777!!!!!!!!!!!!!!!!");
                let light_1 = light_1_clone.lock().unwrap().clone();
                let payload = serde_json::json!({
                            light_1.name.clone(): light_1,
                });
                let mut response = request.into_ok_response()?;
                response.write_all(payload.to_string().as_bytes())?;
                Ok(())
            },
        )
        .unwrap();
    let light_2_clone = light_2.clone();
    let path = { light_2.lock().unwrap() }.name.clone().replace(" ", "%20");
    server
        .fn_handler(
            format!("/{path}_status").as_str(),
            Method::Get,
            move |request| {
                println!("Devices777777777777777777777777777777!!!!!!!!!!!!!!!!");
                let light_2 = light_2_clone.lock().unwrap().clone();
                let payload = serde_json::json!({
                            light_2.name.clone(): light_2,
                });
                let mut response = request.into_ok_response()?;
                response.write_all(payload.to_string().as_bytes())?;
                Ok(())
            },
        )
        .unwrap();
    let light_1_clone = light_1.clone();
    let light_2_clone = light_2.clone();
    server
        .fn_handler("/devices", Method::Get, move |request| {
            println!("Devices777777777777777777777777777777!!!!!!!!!!!!!!!!");
            let light_1 = light_1_clone.lock().unwrap().clone();
            let light_2 = light_2_clone.lock().unwrap().clone();
            let payload = serde_json::json!({
                        light_1.name.clone(): light_1,
                        light_2.name.clone(): light_2,
            });
            let mut response = request.into_ok_response()?;
            response.write_all(payload.to_string().as_bytes())?;
            Ok(())
        })
        .unwrap();
    loop {
        println!(
            "IP info: {:?}",
            wifi_driver.sta_netif().get_ip_info().unwrap()
        );
        sleep(Duration::new(10, 0));
    }
}

#[cfg(not(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3))))]
fn main() {
    use esp_idf_hal::delay::FreeRtos;
    println!("pcnt peripheral not supported on this device!");
    loop {
        FreeRtos::delay_ms(100u32);
    }
}
