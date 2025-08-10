#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]


use std::thread::sleep;

use mission_planner_c::{CMission};
use mission_planner::mission_scheduler::{MissionScheduler};
use mission_planner::mission::Mission;

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

fn main() {
    let mission: Box<dyn Mission + Send>;
    unsafe {
        let mission_ptr = cmission_example_create();
        let mission_box = Box::from_raw(mission_ptr as *mut CMission);
        mission = mission_box;
    }

    let scheduler = MissionScheduler::start();
    scheduler.push_back(mission);
    scheduler.run();
    while ! scheduler.is_waiting() {
        sleep(std::time::Duration::from_millis(100));
    }
    scheduler.stop();
}
