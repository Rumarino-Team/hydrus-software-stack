use mission_planner::mission::{Mission, MissionResult, MissionHashMap};
use std::ffi::{c_char, CStr};

#[repr(C)]
pub enum CMissionResult {
    Ok,
    Err,
    ErrSkip
}

#[repr(C)]
pub enum OptionFunction<T> {
    Some(T),
    None
}

pub struct MissionMapPtr;


type CTaskFunc = unsafe extern "C" fn(data: *mut MissionMapPtr) -> CMissionResult;

pub struct CTask {
    name: String,
    task_func: OptionFunction<CTaskFunc>,
    repair_task_func: OptionFunction<CTaskFunc>,
}

#[unsafe(no_mangle)]
pub extern "C" fn ctask_create(name_ptr: *const c_char, task_func: OptionFunction<CTaskFunc>, repair_task_func: OptionFunction<CTaskFunc>) -> *mut CTask {

    let name;
    unsafe {
        let temp = CStr::from_ptr(name_ptr);
        name = temp.to_str().expect("Failed to get string literal!");
    };
    let name = name.to_string();
    let ctask = CTask {
        name,
        task_func,
        repair_task_func
    };
    let ctask_box = Box::new(ctask);
    Box::into_raw(ctask_box)

}

fn run_with(func: &OptionFunction<CTaskFunc>, data: &MissionHashMap) -> CMissionResult {
    let OptionFunction::Some(func) = func else {
        return CMissionResult::Err
    };
    let data_ptr = Box::new(data);
    let data_ptr = Box::into_raw(data_ptr) as *mut MissionMapPtr;
    unsafe { func(data_ptr) }
}

pub struct CMission {
    name: String,
    task_list: Vec<Box<CTask>>,
}

impl CMission {
    fn new(name: String, task_list: Vec<Box<CTask>>) -> Self {
        CMission {
            name: name,
            task_list
        }
    }
}

impl Mission for CMission {
    fn run(&self, data: &MissionHashMap) -> MissionResult {
        let mut res = CMissionResult::Err;
        for task in &self.task_list {
            let task_res = run_with(&task.task_func, data);
            res = match task_res {
                CMissionResult::Ok | CMissionResult::ErrSkip => task_res,
                CMissionResult::Err => {
                    let task_res = run_with(&task.repair_task_func, data);
                    task_res
                }
            };
            if let CMissionResult::Err = res {
                break
            }
        }
        match res {
            CMissionResult::Ok => Ok(()),
            CMissionResult::Err => Err(false),
            CMissionResult::ErrSkip => Err(true),
        }
    }

    fn name(&self) -> &String {
        &self.name
    }

}

pub struct CMissionPtr;

#[unsafe(no_mangle)]
//TODO: Change to array
pub extern "C" fn cmission_create(name_ptr: *const c_char, task_array: *mut CTask, size: isize) -> *mut CMissionPtr {
    let mut task_list = Vec::new();
    for i in 0..size {
        unsafe {
            let task = task_array.offset(i);
            let task = Box::from_raw(task);
            task_list.push(task);

        }
    }

    let name;
    unsafe {
        let temp = CStr::from_ptr(name_ptr);
        name = temp.to_str().expect("Failed to get string literal!");
    };
    let c_mission = CMission::new(name.to_string(), task_list);
    let mission_box = Box::new(c_mission);
    Box::into_raw(mission_box) as *mut CMissionPtr
}


#[cfg(test)]
mod tests {
    //use super::*;

    //TODO: Write tests

}
