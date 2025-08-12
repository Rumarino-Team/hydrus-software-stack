use dashmap::DashMap;
use crate::mission::{Mission, MissionHashMap};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, sleep};

pub type MissionVec = VecDeque<Mission>;
struct MissionThreadData {
    mission_list: Arc<Mutex<MissionVec>>,
    conc_mission_list: Arc<Mutex<MissionVec>>,
    mission_data: Arc<DashMap<String, String>>,
    run: AtomicBool,
    stop: AtomicBool,
    waiting: AtomicBool,

}

impl MissionThreadData {
    fn new() -> Self {
        Self {
            mission_list: Arc::new(Mutex::new(VecDeque::new())),
            conc_mission_list: Arc::new(Mutex::new(VecDeque::new())),
            mission_data: Arc::new(DashMap::new()),
            run: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            waiting: AtomicBool::new(false),
        }
    }

    fn with_mission_list(&self, func: impl FnOnce(&mut MissionVec) -> Option<Mission>, is_concurrent: bool) -> Option<Mission> {
        let mut guard = if is_concurrent {
            self.conc_mission_list.try_lock().expect("Concurrent mission lock is poisoned!")
        } else {
            self.mission_list.try_lock().expect("Mission lock is poisoned!")
        };
        func(&mut guard)
    }

    pub fn pop_front(&self) -> Option<Mission> {
        let func = move |mission_list: &mut VecDeque<Mission>| {
            mission_list.pop_front()
        };
        self.with_mission_list(func, false)
    }
}
pub struct MissionScheduler {
    normal_handle: thread::JoinHandle<()>,
    concurrent_handle: thread::JoinHandle<()>,
    scheduler_data : Arc<MissionThreadData>,
}

impl MissionScheduler {
    fn new(normal_handle: thread::JoinHandle<()>, concurrent_handle: thread::JoinHandle<()>, scheduler_data: Arc<MissionThreadData>) -> Self {
        Self {
            normal_handle,
            concurrent_handle,
            scheduler_data,
        }
    }

    #[allow(unused)]
    pub fn push_back(&self, mission: Mission) {
        let func = move |mission_list: &mut VecDeque<Mission>| {
            mission_list.push_back(mission);
            None
        };
        self.scheduler_data.with_mission_list(func, false);
    }

    #[allow(unused)]
    pub fn conc_push_back(&self, mission: Mission) {
        let func = move |mission_list: &mut VecDeque<Mission>| {
            mission_list.push_back(mission);
            None
        };
        self.scheduler_data.with_mission_list(func, true);
    }

    #[allow(unused)]
    pub fn append(&self, mut mission_vec: MissionVec) {
        let func = move |mission_list: &mut VecDeque<Mission>| {
            mission_list.append(&mut mission_vec);
            None
        };
        self.scheduler_data.with_mission_list(func, false);
    }

    #[allow(unused)]
    pub fn conc_append(&self, mut mission_vec: MissionVec) {
        let func = move |mission_list: &mut VecDeque<Mission>| {
            mission_list.append(&mut mission_vec);
            None
        };
        self.scheduler_data.with_mission_list(func, true);
    }

    pub fn get_data(&self) -> Arc<MissionHashMap> {
        self.scheduler_data.mission_data.clone()
    }

    pub fn is_waiting(&self) -> bool {
        self.scheduler_data.waiting.load(Ordering::Relaxed)
    }

    // pub fn concurrent_append(&mut self, mission: &mut Vec<Box<dyn Mission<'static> + Send >>) {
    //     self.concurrent_mission_list.append(mission);
    // }

    pub fn start() -> Self {
        let scheduler_data = Arc::new(MissionThreadData::new());

        let scheduler_data_normal = scheduler_data.clone();
        let normal_func = move || {
            let mut stop = false;
            let data = &scheduler_data_normal.mission_data;

            while ! stop {
                let run = scheduler_data_normal.run.load(Ordering::Relaxed);
                stop = scheduler_data_normal.stop.load(Ordering::Relaxed);
                if ! run {
                    sleep(std::time::Duration::from_millis(1));
                    continue
                }


                let mission = scheduler_data_normal.pop_front();
                let Some(mission) = mission else {
                    println!("Waiting for missions...");
                    scheduler_data_normal.waiting.store(true, Ordering::Relaxed);
                    sleep(std::time::Duration::from_secs(3));
                    continue;
                };
                let res = mission.run(data);
                match res {
                    Ok(_) => (),
                    Err(skip) => {
                        if skip {
                            println!("{} mission skipped!", mission.name());
                        }
                        else {
                            println!("{} mission failed!", mission.name());
                            scheduler_data_normal.stop.store(true, Ordering::Relaxed);
                            stop = true;
                        }
                    }
                };


                sleep(std::time::Duration::from_millis(1));
            }
        };

        let scheduler_data_conc = scheduler_data.clone();
        let concurrent_func = move || {
            let mut stop = false;
            let conc_mission_list = scheduler_data_conc.conc_mission_list.clone();
            let data = &scheduler_data_conc.mission_data;
            while ! stop {
                let run = scheduler_data_conc.run.load(Ordering::Relaxed);
                stop = scheduler_data_conc.stop.load(Ordering::Relaxed);
                if ! run {
                    sleep(std::time::Duration::from_millis(1));
                    continue
                }

                let getter = conc_mission_list
                    .try_lock()
                    .expect("Concurrent mission lock is poisoned!");
                for mission in &*getter {
                    let res = mission.run(data);
                    match res {
                        Ok(_) => (),
                        Err(skip) => {
                            if skip {
                                println!("{} mission skipped!", mission.name());
                            }
                            else {
                                println!("{} mission failed!", mission.name());
                                scheduler_data_conc.stop.store(true, Ordering::Relaxed);
                                stop = true;
                            }
                        }
                    };
                };

                sleep(std::time::Duration::from_millis(100));
            }
        };
        let normal_handle = thread::spawn(normal_func);
        let conc_handle = thread::spawn(concurrent_func);
        MissionScheduler::new(normal_handle, conc_handle, scheduler_data.clone())
    }

    pub fn run(&self) {
        self.scheduler_data.run.store(true, Ordering::Relaxed);
    }

    pub fn stop(self) {
        self.scheduler_data.stop.store(true, Ordering::Relaxed);
        self.normal_handle.join().expect("Failed to join mission handle!");
        self.concurrent_handle.join().expect("Failed to join concurrent mission handle!");
    }
}
