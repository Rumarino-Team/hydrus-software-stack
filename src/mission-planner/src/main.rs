mod mission;
mod mission_scheduler;
mod cmission;
mod mission_example;
mod concurrent_mission_example;

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

use std::collections::VecDeque;
use std::time::Duration;


use crate::{
    mission::Mission,
    cmission::CMission,
    mission_scheduler::{MissionScheduler, MissionBox},
    mission_example::ExampleMission,
    concurrent_mission_example::ConcExampleMission,
};

fn main() {
    let cmission_example: Box<dyn Mission + Send>;
    unsafe {
        let cmission_ptr = cmission_example_create();
        cmission_example = Box::from_raw(cmission_ptr as *mut CMission);
    }

    let foo: MissionBox = Box::new(ExampleMission::new());
    let bar: MissionBox = Box::new(ConcExampleMission::new());

    let mission_list = VecDeque::from(vec![
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
