use dashmap::DashMap;

pub type MissionHashMap = DashMap<String, String>;
pub type MissionResult = Result<(), bool>;

pub trait Task : Send + Sync {
    fn run(&self, data: &MissionHashMap) -> MissionResult;
    fn repair_run(&self, data: &MissionHashMap) -> MissionResult;
    fn name(&self) -> &String;
}

pub struct RustTask {
    pub name: String,
    func: Option<fn(&MissionHashMap) -> MissionResult>,
    repair_func: Option<fn(&MissionHashMap) -> MissionResult>
}

fn run_with(func: Option<fn(&MissionHashMap) -> MissionResult>, data: &MissionHashMap) -> MissionResult {
    let Some(func) = func else {
        return Err(false);
    };
    func(data)
}

impl RustTask {
    pub fn new(name: String, func: Option< fn(&MissionHashMap) -> MissionResult>,
    repair_func: Option<fn(&MissionHashMap)-> MissionResult>) -> RustTask {
        RustTask {
            name,
            func,
            repair_func,
        }
    }
}

impl Task for RustTask {
    fn run(&self, data: &MissionHashMap) -> MissionResult {
        run_with(self.func, data)
    }
    fn repair_run(&self, data: &MissionHashMap) -> MissionResult {
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
    pub fn run(&self, data: &DashMap<String, String>) -> MissionResult {
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
