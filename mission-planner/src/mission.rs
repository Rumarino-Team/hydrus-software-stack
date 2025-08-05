use dashmap::DashMap;

pub type MissionHashMap = DashMap<String, String>;
pub type MissionResult = Result<(), bool>;

pub struct Task {
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

impl Task {
    pub fn new(name: String, func: Option< fn(&MissionHashMap) -> MissionResult>,
    repair_func: Option<fn(&MissionHashMap)-> MissionResult>) -> Task {
        Task {
            name,
            func,
            repair_func,
        }
    }
}

pub struct CommonMission {
    pub name: String,
    task_list: Vec<Task>,
    safety_task_list: Vec<Task>,
}

impl CommonMission {
    pub fn new(name: String, task_list: &mut Vec<Task>, repair_task_list: &mut Vec<Task>) -> Self {
        let mut common = CommonMission {
            name,
            task_list: Vec::new(),
            safety_task_list: Vec::new(),
        };
        common.task_list.append(task_list);
        common.safety_task_list.append(repair_task_list);
        common
    }

    pub fn run(&self, data: &DashMap<String, String>) -> MissionResult {
        if self.task_list.is_empty() {
            return Ok(())
        }

        let mut res = Ok(());
        for task in &self.task_list {
            let task_res = run_with(task.func, data);
            res = match task_res {
                Ok(_) => task_res,
                Err(skip) => {
                    if skip {
                        task_res
                    } else {
                        run_with(task.repair_func, data)
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
}

pub trait Mission : Send + Sync {
    fn run(&self, data: &DashMap<String, String>) -> MissionResult;
    fn name(&self) -> &String;
}