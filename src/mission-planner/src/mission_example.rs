use std::thread::sleep;
use crate::mission::{Mission, Task, MissionResult, MissionData, RustTask};
use std::sync::atomic::Ordering;

fn example(_data: &MissionData) -> MissionResult {
    println!("Hello from Rust!");
    Err(false)
}

fn repair_example(data: &MissionData) -> MissionResult {
    println!("Requested flag");
    data.example_flag_request.store(true, Ordering::Relaxed);

    while ! data.example_flag.load(Ordering::Relaxed) {
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
