use dashmap::DashMap;

use std::thread::sleep;
use crate::mission::{CommonMission, MissionResult, Mission, MissionHashMap, Task};

pub struct ExampleMission {
    common: CommonMission,
}

fn example(data: &MissionHashMap) -> MissionResult {
    println!("Hello world!");
    Err(false)
}

fn repair_example(data: &MissionHashMap) -> MissionResult {
    println!("Requested flag");
    data.insert("flag_request".to_string(), "true".to_string());

    while ! data.contains_key("flag") {
        sleep(std::time::Duration::from_millis(100));
    }
    println!("Got flag!");

    Ok(())
}

impl ExampleMission {
    pub fn new() -> Self {
        let name = "example-mission";
        let task = Task::new("example-task".to_string(), Some(example), Some(repair_example));
        let mut task_list: Vec<Task> = Vec::new();
        task_list.push(task);
        let mut repair_task_list: Vec<Task> = Vec::new();
        Self {
            common: CommonMission::new(name.to_string(), &mut task_list, &mut repair_task_list)
        }
    }
}

impl Mission for ExampleMission {
    fn run(&self, data: &DashMap<String, String>) -> MissionResult {
        self.common.run(data)
    }

    fn name(&self) -> &String {
        &self.common.name
    }

}
