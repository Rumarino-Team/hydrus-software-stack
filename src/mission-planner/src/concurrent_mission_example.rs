use crate::mission::{Mission, MissionResult, MissionHashMap, RustTask, Task};

fn conc_example(data: &MissionHashMap) -> MissionResult {
    if data.contains_key("flag_request") {
        println!("Wrote flag");
        data.insert("flag".to_string(), "true".to_string());
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

