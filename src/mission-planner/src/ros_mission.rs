use crate::mission::{Mission, Task, MissionResult, MissionData, RustTask};

fn ros_example(_data: &MissionData) -> MissionResult {
    println!("Ros mission invoked!");
    Ok(())
}

pub fn new() -> Mission {
    let name = "ros-example-mission".to_string();
    let task = RustTask::new("ros-example-task".to_string(), Some(ros_example), None);
    let task_list: Vec<Box<dyn Task>> = vec![
        Box::new(task)
    ];

    Mission { name, task_list }
}
