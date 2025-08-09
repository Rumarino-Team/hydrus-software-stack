use crate::mission::{CommonMission, MissionResult, Mission, MissionHashMap, Task};

pub struct ConcExampleMission {
    common: CommonMission,
}

fn conc_example(data: &MissionHashMap) -> MissionResult {
    if data.contains_key("flag_request") {
        println!("Wrote flag");
        data.insert("flag".to_string(), "true".to_string());
    }
    Ok(())
}

impl ConcExampleMission {
    pub fn new() -> Self {
        let name = "example-mission";
        let task = Task::new("conc-example-task".to_string(), Some(conc_example), None);
        let mut task_list: Vec<Task> = Vec::new();
        task_list.push(task);
        let mut repair_task_list: Vec<Task> = Vec::new();
        Self {
            common: CommonMission::new(name.to_string(), &mut task_list, &mut repair_task_list)
        }
    }
}

impl Mission for ConcExampleMission {
    fn run(&self, data: &MissionHashMap) -> MissionResult {
        self.common.run(data)
    }

    fn name(&self) -> &String {
        &self.common.name
    }
}
