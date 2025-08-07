use crate::mission::{CommonMission, MissionHashMap, MissionResult, Task};
use std::ffi::{c_char, CStr};

#[repr(C)]
#[allow(unused)]
pub enum CMissionResult {
    Ok,
    Err,
    ErrSkip
}

#[repr(C)]
#[allow(unused)]
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

fn run_with(func: &OptionFunction<CTaskFunc>, data: &MissionHashMap) -> MissionResult {
    let OptionFunction::Some(func) = func else {
        return Err(false)
    };
    let data_ptr = Box::new(data);
    let data_ptr = Box::into_raw(data_ptr) as *mut MissionMapPtr;
    let res = unsafe { func(data_ptr) };
    match res {
        CMissionResult::Ok => Ok(()),
        CMissionResult::Err => Err(false),
        CMissionResult::ErrSkip => Err(true),
    }
}

impl Task for CTask {
    fn run(&self, data: &MissionHashMap) -> MissionResult {
        run_with(&self.task_func, data)
    }
    fn repair_run(&self, data: &MissionHashMap) -> MissionResult {
        run_with(&self.repair_task_func, data)
    }
    fn name(&self) -> &String {
        &self.name
    }

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

pub struct CMissionPtr;

#[unsafe(no_mangle)]
/* It is also possible to create the Mission in Rust using CTasks, but this was made for ease of use, especially
   if there are a lot of CTasks */
pub extern "C" fn cmission_create(name_ptr: *const c_char, task_array: *mut CTask, size: isize) -> *mut CMissionPtr {
    let mut task_list: Vec<Box<dyn Task>> = Vec::new();
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
    let name = name.to_string();
    let mission = CommonMission {
        name,
        task_list
    };
    let mission_box = Box::new(mission);
    Box::into_raw(mission_box) as *mut CMissionPtr
}


#[cfg(test)]
mod tests {
    //use super::*;

    //TODO: Write tests

}
