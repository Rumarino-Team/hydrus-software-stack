use std::thread::sleep;
use crate::mission::{Mission, Task, MissionResult, MissionHashMap, RustTask};

fn example(_data: &MissionHashMap) -> MissionResult {
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

pub fn new() -> Mission {
    let name = "example-mission".to_string();
    let task = RustTask::new("example-task".to_string(), Some(example), Some(repair_example));
    let task_list: Vec<Box<dyn Task>> = vec![
        Box::new(task)
    ];

    Mission { name, task_list }
}
