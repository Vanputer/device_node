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

include!("creds.inc");

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
            ssid: SSID.into(),
            password: PASSWORD.into(),
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
            updated: true,
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
            updated: true,
        },
    ])));

    // Update any light devices that need updating (if a knob's turned or anything)
    let lights_clone = lights.clone();
    thread::spawn(move || {
        println!("setup encoder");
        let mut pin_a_1 = peripherals.pins.gpio5;
        let mut pin_b_1 = peripherals.pins.gpio6;
        let mut pin_a_2 = peripherals.pins.gpio7;
        let mut pin_b_2 = peripherals.pins.gpio8;
        let mut encoders = Vec::from([
            Encoder::new(peripherals.pcnt0, &mut pin_a_1, &mut pin_b_1).unwrap(),
            Encoder::new(peripherals.pcnt1, &mut pin_a_2, &mut pin_b_2).unwrap(),
        ]);
        let mut last_encoder_values = Vec::from([0i64, 0i64]);
        let mut last_encoder_times = Vec::from([Instant::now(), Instant::now()]);
        loop {
            {
                let mut lights = lights_clone.lock().unwrap();
                for (((light, encoder), last_encoder_time), last_encoder_value) in lights
                    .iter_mut()
                    .zip(encoders.iter_mut())
                    .zip(last_encoder_times.iter_mut())
                    .zip(last_encoder_values.iter_mut())
                {
                    light_update(light, encoder, last_encoder_time, last_encoder_value);
                }
            }
            Delay::delay_ms(100u32);
        }
    });

    // Update duty cycles if/when needed (if a light Device has been updated)
    let lights_clone = lights.clone();
    thread::spawn(move || {
        let freqs: Vec<_> = {
            lights_clone
                .lock()
                .unwrap()
                .iter()
                .map(|l| l.freq_Hz.Hz())
                .collect()
        };
        let mut drivers = Vec::from([
            LedcDriver::new(
                peripherals.ledc.channel0,
                LedcTimerDriver::new(
                    peripherals.ledc.timer0,
                    &TimerConfig::new().frequency(freqs[0]),
                )
                .unwrap(),
                peripherals.pins.gpio9,
            )
            .unwrap(),
            LedcDriver::new(
                peripherals.ledc.channel1,
                LedcTimerDriver::new(
                    peripherals.ledc.timer1,
                    &TimerConfig::new().frequency(freqs[1]),
                )
                .unwrap(),
                peripherals.pins.gpio10,
            )
            .unwrap(),
        ]);
        let mut max_duty = Vec::with_capacity(drivers.len());
        for driver in drivers.iter() {
            max_duty.push(driver.get_max_duty());
        }
        loop {
            {
                for ((light, driver), max_duty) in lights_clone
                    .lock()
                    .unwrap()
                    .iter_mut()
                    .zip(drivers.iter_mut())
                    .zip(max_duty.iter())
                {
                    if light.updated {
                        let duty_cycle = light.get_duty_cycle() * max_duty / 100;
                        let _ = driver.set_duty(duty_cycle);
                        light.updated = false;
                    }
                }
            }
            Delay::delay_ms(100u32);
        }
    });

    let mut server = EspHttpServer::new(&SVC_Configuration::default()).unwrap();
    server
        .fn_handler("/", Method::Get, |request| {
            let mut response = request.into_ok_response()?;
            response.write_all("payload!!!!!".as_bytes())?;
            Ok(())
        })
        .unwrap();
    let lights_clone = lights.clone();
    server
        .fn_handler("/status", Method::Get, move |request| {
            if &request.uri().len() < &8_usize {
                let _ = exit_early(request, "Bad Status command given", 422);
                return Ok(());
            }
            let query = &request.uri()[8..].to_string().to_lowercase();
            let query: HashMap<_, _> = querystring::querify(query).into_iter().collect();
            match query.get("device") {
                Some(d) => {
                    let d = d.replace("%20", " ");
                    for light in lights_clone.lock().unwrap().iter() {
                        if light.name == d {
                            let mut response = request.into_ok_response()?;
                            let _ = response.write_all(&light.to_json().into_bytes()[..]);
                            return Ok(());
                        }
                    }
                    let _ = exit_early(request, "Device name not found", 422);
                    return Ok(());
                }
                None => {
                    let _ = exit_early(request, "No Device name given", 422);
                    return Ok(());
                }
            }
        })
        .unwrap();
    let lights_clone = lights.clone();
    server
        .fn_handler("/devices", Method::Get, move |request| {
            let mut lights = HashMap::new();
            {
                for light in lights_clone.lock().unwrap().iter() {
                    lights.insert(light.name.clone(), light.clone());
                }
            }
            let payload = serde_json::json!(lights);
            let mut response = request.into_ok_response()?;
            response.write_all(payload.to_string().as_bytes())?;
            Ok(())
        })
        .unwrap();
    let lights_clone = lights.clone();
    server
        .fn_handler("/command", Method::Get, move |request| {
            if &request.uri().len() < &9_usize {
                let _ = exit_early(request, "Bad Command given", 422);
                return Ok(());
            }
            let query = &request.uri()[9..].to_string();
            let query: HashMap<_, _> = querystring::querify(query).into_iter().collect();
            let device = match query.get("device") {
                Some(d) => {
                    let mut found = None;
                    for light in lights_clone.lock().unwrap().iter() {
                        if d.replace("%20", " ").to_lowercase() == light.name {
                            found = Some(light.name.clone());
                            break;
                        }
                    }
                    if found.is_none() {
                        let _ = exit_early(request, "Bad Device name given", 422);
                        return Ok(());
                    }
                    found.unwrap()
                }
                None => {
                    let _ = exit_early(request, "Device field not given", 422);
                    return Ok(());
                }
            };
            let action = match query.get("action") {
                Some(a) => match Action::from_str(&a.to_lowercase()) {
                    Ok(a) => a,
                    Err(_) => {
                        let _ = exit_early(request, "Bad Action name given", 422);
                        return Ok(());
                    }
                },
                None => {
                    let _ = exit_early(request, "No Action given", 422);
                    return Ok(());
                }
            };
            let target: Option<usize> = match query.get("target") {
                Some(t_text) => match t_text.parse::<usize>() {
                    Ok(t_num) => {
                        if t_num > 7 {
                            let _ = exit_early(request, "Target must be >= 0 & < 8", 422);
                            return Ok(());
                        }
                        Some(t_num)
                    }
                    Err(_) => {
                        if t_text != &"" {
                            let _ = exit_early(request, "Bad Target given", 422);
                            return Ok(());
                        }
                        None
                    }
                },
                None => None,
            };
            for light in lights.lock().unwrap().iter_mut() {
                if light.name == device {
                    let _ = light.take_action(action, target);
                }
            }
            let mut response = request.into_ok_response()?;
            response.write_all("Command!!!!!".as_bytes())?;
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

fn light_update(
    light: &mut Device,
    encoder: &mut Encoder,
    last_encoder_time: &mut Instant,
    last_encoder_value: &mut i64,
) {
    let encoder_value = encoder.get_value().unwrap();
    if encoder_value != *last_encoder_value {
        let current_time = Instant::now();
        let time_since_last_check = current_time.duration_since(*last_encoder_time);
        if time_since_last_check > Duration::from_millis(100) {
            {
                if encoder_value > *last_encoder_value {
                    let _ = light.take_action(Action::Up, None);
                    light.updated = true;
                } else {
                    let _ = light.take_action(Action::Down, None);
                    light.updated = true;
                }
            }
            *last_encoder_time = Instant::now();
        }
        *last_encoder_value = encoder_value;
    }
}

fn exit_early<'a>(
    request: Request<&mut EspHttpConnection<'a>>,
    message: &str,
    code: u16,
) -> Result<(), HandlerError> {
    let mut response = request.into_status_response(code)?;
    let _ = response.write_all(message.as_bytes());
    Ok(())
}

#[cfg(not(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3))))]
fn main() {
    use esp_idf_hal::delay::FreeRtos;
    println!("pcnt peripheral not supported on this device!");
    loop {
        FreeRtos::delay_ms(100u32);
    }
}
