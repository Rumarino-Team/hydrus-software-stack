use std::ffi::CStr;

use crate::mission::{CommonMission, Task, MissionHashMap, MissionResult};

use pyo3::{ffi::c_str, prelude::*, types::IntoPyDict, PyResult, Python};

pub struct PyTask {
    name: String,
    pytask_obj: PyObject
}

impl PyTask {
    //Assume pytask
    pub fn new(pytask_obj: PyObject) -> Self {
        let res = Python::with_gil(|py| -> PyResult<String> {
            let name = pytask_obj.getattr(py, "name")?.to_string();
            Ok(name)
        });
        let Ok(name) = res else {
            panic!("Failed to get name. Error:\n{:#?}", res);
        };
        Self {
            name,
            pytask_obj
        }
    }

    fn run_with(&self, option: &str, data: &MissionHashMap) -> MissionResult {
        let res = Python::with_gil(|py| -> PyResult<()> {
            let task_obj = self.pytask_obj.as_ref();
            //TODO: This implies read-only access
            let dict = data.clone().into_py_dict(py)?;
            let args = (dict, );
            task_obj.call_method1(py, option, args)?;
            Ok(())
        });
        match res {
            Ok(_) => Ok(()),
            Err(pyerr) => {
                println!("Python error: {:#?}", pyerr);
                Err(false)
            }
        }
        
    }
}

impl Task for PyTask {
    fn run(&self, data: &MissionHashMap) -> MissionResult {
        //TODO: Handle errors from Python and not simply assume everything is fine
        self.run_with("run", data)
    }

    fn repair_run(&self, data: &MissionHashMap) -> MissionResult {
        self.run_with("repair_run", data)
    }

    fn name(&self) -> &String {
        &self.name
    }

}

pub fn get_mission_from(file: &CStr, file_name: &CStr) -> CommonMission {
    let pytask = c_str!(include_str!("pymission.py"));

    let res = Python::with_gil(|py| -> PyResult<(String, Vec<PyObject>)> {
        PyModule::from_code(py, pytask, c_str!("pymission.py"), c_str!("pymission"))?;
        let pymission: PyObject = PyModule::from_code(py, file, file_name, c_str!(""))?
            .getattr("new")?
            .into();
        let pymission = pymission.call0(py)?;
        let name = pymission.getattr(py, "name")?.to_string();
        let task_list: PyObject = pymission.getattr(py, "task_list")?.into();
        let task_list: Vec<PyObject> = task_list.extract(py)?;
        Ok((name, task_list))
    });

    let Ok((name, task_list_py)) = res else {
        let file_name = file_name.to_str().expect("Failed to get string literal from filename");
        panic!("Failed to load mission from {}, error:\n {:#?}", file_name, res);
        
    };

    let mut task_list: Vec<Box<dyn Task>> = Vec::new();
    for pytask in task_list_py {
        let task = PyTask::new(pytask);
        task_list.push(Box::new(task));
    }

    CommonMission {
        name,
        task_list
    }
}
