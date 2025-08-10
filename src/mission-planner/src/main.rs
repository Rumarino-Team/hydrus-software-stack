mod mission;
mod mission_example;
mod mission_scheduler;
mod concurrent_mission_example;

use std::collections::VecDeque;
use std::time::Duration;

use crate::{
    mission_scheduler::{MissionScheduler, MissionBox},
    mission_example::ExampleMission,
    concurrent_mission_example::ConcExampleMission,
};

fn main() {
    let foo: MissionBox = Box::new(ExampleMission::new());
    let bar: MissionBox = Box::new(ConcExampleMission::new());

    let mission_list = VecDeque::from(vec![
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
