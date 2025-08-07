mod mission;
mod mission_scheduler;
mod cmission;
mod pymission;
mod mission_example;
mod concurrent_mission_example;

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

use std::vec;
use std::{collections::VecDeque};
use std::time::Duration;


use pyo3::{ffi::c_str};

use crate::{
    mission::{Mission}, mission_scheduler::MissionScheduler
};

fn main() {
    let pytest = c_str!(include_str!("pymission_example.py"));
    let pymission_example = pymission::get_mission_from(pytest, c_str!("pymission_example.py"));

    let cmission_example;
    unsafe {
        let cmission_ptr = cmission_example_create();
        cmission_example = *Box::from_raw(cmission_ptr as *mut Mission);
    }

    let foo = mission_example::new();
    let bar = concurrent_mission_example::new();

    let mission_list = VecDeque::from(vec![
        pymission_example,
        cmission_example,
        foo
    ]);
    let conc_mission_list = VecDeque::from(vec![
        bar
    ]);

    let scheduler = MissionScheduler::start();
    scheduler.append(mission_list);
    scheduler.conc_append(conc_mission_list);
    let data = scheduler.get_data();

    scheduler.run();
    while ! scheduler.is_waiting() {
        println!("{:#?}", data);
        std::thread::sleep(Duration::from_secs(1));
    }
    scheduler.stop();


}
