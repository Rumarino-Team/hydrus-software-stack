mod mission;
mod mission_scheduler;
mod cmission;
mod pymission;
mod mission_example;
mod concurrent_mission_example;
mod ros_mission;

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

use std::{collections::VecDeque};
use std::time::{Duration, Instant};


use pyo3::{ffi::c_str};

use crate::mission_scheduler::MissionBox;
use crate::{
    mission::{CommonMission}, mission_scheduler::MissionScheduler
};


fn main() {
    let pytest = c_str!(include_str!("pymission_example.py"));
    let pymission_example = pymission::get_mission_from(pytest, c_str!("pymission_example.py"));

    let cmission_example;
    unsafe {
        let cmission_ptr = cmission_example_create();
        cmission_example = *Box::from_raw(cmission_ptr as *mut CommonMission);
    }

    let foo = mission_example::new();
    let bar = concurrent_mission_example::new();

    let mission_list: [MissionBox; 3] = [
        Box::new(pymission_example),
        Box::new(cmission_example),
        Box::new(foo),
    ];
    let mission_list = VecDeque::from(mission_list);
    let conc_mission_list: [MissionBox; 1] = [
        Box::new(bar)
    ];
    let conc_mission_list = VecDeque::from(conc_mission_list);

    let mut scheduler = MissionScheduler::start();
    scheduler.append(mission_list);
    scheduler.conc_append(conc_mission_list);
    let _data = scheduler.get_data();

    let start = Instant::now();
    scheduler.run();
    while start.elapsed() < Duration::from_secs(15) {
        scheduler.ros_spin();
    }
    scheduler.stop();


}
