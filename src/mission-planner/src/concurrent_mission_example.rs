use std::sync::atomic::Ordering;

use crate::mission::{Mission, MissionResult, MissionData, RustTask, Task};

fn conc_example(data: &MissionData) -> MissionResult {
    if data.example_flag_request.load(Ordering::Relaxed) {
        data.example_flag.store(true, Ordering::Relaxed);
        println!("Wrote flag!");
    }
    Ok(())
}

pub fn new() -> Mission {
    let name = "concurrent-mission-example".to_string();
    let task = RustTask::new("conc-example-task".to_string(), Some(conc_example), None);
    let task_list: Vec<Box<dyn Task>> =  vec![
        Box::new(task)
    ];
    Mission { name, task_list }
}

