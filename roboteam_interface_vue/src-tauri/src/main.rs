#![cfg_attr(
all(not(debug_assertions), target_os = "windows"),
windows_subsystem = "windows"
)]


extern crate core;

use std::sync::mpsc;
use std::thread::sleep;
use std::time::{Duration, Instant};
use prost::Message as ProtoMessage;
use tauri::Manager;
use zmq::{Message as ZMQMsg, Socket};

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/proto.rs"));
}

fn main() {
    println!("Start");
    let ctx = zmq::Context::new();

    let subscriber = ctx.socket(zmq::SUB)
        .expect("could not initialize socket object");

    subscriber
        .connect("tcp://127.0.0.1:5558")
        .expect("could not connect to publisher");

    subscriber.set_subscribe(b"").expect("failed subscribing");
    // proto::State::
    println!("Connected");


    tauri::Builder::default()
        .setup(|app| {
            let app_handle = app.handle();
            let interval = Duration::from_millis(10);
            let mut next_time = Instant::now() + interval;

            tauri::async_runtime::spawn(async move {
                loop {
                    if let Ok(msg) = subscriber.recv_msg(0) {
                        let state = proto::State::decode(msg.as_ref()).expect("TODO: panic message");

                        if next_time - Instant::now() <= Duration::from_millis(0) {
                            emit_state_to_js(state, &app_handle);
                            next_time += interval;
                        }
                    }

                }
            });

            Ok(())
        })
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}

fn emit_state_to_js<R: tauri::Runtime>(message: proto::State, manager: &impl Manager<R>) {
    manager
        .emit_all("new_state", message.last_seen_world)
        .unwrap();
}
