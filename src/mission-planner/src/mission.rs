use std::sync::atomic::AtomicBool;
pub type MissionResult = Result<(), bool>;

#[derive(Debug)]
#[pyo3::pyclass]
pub struct MissionData {
    pub example_flag : AtomicBool,
    pub example_flag_request : AtomicBool,
}

impl MissionData {
    pub fn new() -> Self {
        MissionData {
            example_flag: AtomicBool::new(false),
            example_flag_request: AtomicBool::new(false),
        }
    }
}

pub trait Task : Send + Sync {
    fn run(&self, data: &MissionData) -> MissionResult;
    fn repair_run(&self, data: &MissionData) -> MissionResult;
    fn name(&self) -> &String;
}

pub struct RustTask {
    pub name: String,
    func: Option<fn(&MissionData) -> MissionResult>,
    repair_func: Option<fn(&MissionData) -> MissionResult>
}

fn run_with(func: Option<fn(&MissionData) -> MissionResult>, data: &MissionData) -> MissionResult {
    let Some(func) = func else {
        return Err(false);
    };
    func(data)
}

impl RustTask {
    pub fn new(name: String, func: Option< fn(&MissionData) -> MissionResult>,
    repair_func: Option<fn(&MissionData)-> MissionResult>) -> RustTask {
        RustTask {
            name,
            func,
            repair_func,
        }
    }
}

impl Task for RustTask {
    fn run(&self, data: &MissionData) -> MissionResult {
        run_with(self.func, data)
    }
    fn repair_run(&self, data: &MissionData) -> MissionResult {
        run_with(self.repair_func, data)
    }
    fn name(&self) -> &String {
        &self.name
    }
}

pub struct Mission {
    pub name: String,
    pub task_list: Vec<Box<dyn Task>>,
}

impl Mission {
    pub fn run(&self, data: &MissionData) -> MissionResult {
        if self.task_list.is_empty() {
            return Ok(())
        }

        let mut res = Ok(());
        for task in &self.task_list {
            let task_res = task.run(data);
            res = match task_res {
                Ok(_) => task_res,
                Err(skip) => {
                    if skip {
                        task_res
                    } else {
                        task.repair_run(data)
                    }
                },

            };
            if let Err(skip) = res {
                if ! skip {
                    break
                }
            }
        }
        res
    }

    pub fn name(&self) -> &String {
        &self.name
    }
}
