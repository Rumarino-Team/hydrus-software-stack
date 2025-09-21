use futures::StreamExt;
use futures::executor::ThreadPool;
use r2r::{Node, QosProfile, sensor_msgs, std_msgs};
use crate::mission::{Mission, MissionData};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, sleep};
use std::time::Duration;
use crate::ros_mission;

pub type MissionBox = Box<dyn Mission>;
pub type MissionVec = VecDeque<MissionBox>;
struct MissionThreadData {
    mission_list: Arc<Mutex<MissionVec>>,
    conc_mission_list: Arc<Mutex<MissionVec>>,
    mission_data: Arc<MissionData>,
    run: AtomicBool,
    stop: AtomicBool,
    waiting: AtomicBool,
}

impl MissionThreadData {
    fn new() -> Self {
        Self {
            mission_list: Arc::new(Mutex::new(VecDeque::new())),
            conc_mission_list: Arc::new(Mutex::new(VecDeque::new())),
            mission_data: Arc::new(MissionData::new()),
            run: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            waiting: AtomicBool::new(false),
        }
    }

    fn with_mission_list(&self, func: impl FnOnce(&mut MissionVec) -> Option<MissionBox>, is_concurrent: bool) -> Option<MissionBox> {
        let mut guard = if is_concurrent {
            self.conc_mission_list.try_lock().expect("Concurrent mission lock is poisoned!")
        } else {
            self.mission_list.try_lock().expect("Mission lock is poisoned!")
        };
        func(&mut guard)
    }

    pub fn pop_front(&self) -> Option<Box<dyn Mission>> {
        let func = move |mission_list: &mut MissionVec| {
            mission_list.pop_front()
        };
        self.with_mission_list(func, false)
    }

    pub fn push_back(&self, mission : impl Mission + 'static) -> Option<MissionBox> {
        let func = move |mission_list: &mut MissionVec| {
            mission_list.push_back(Box::new(mission));
            None
        };
        self.with_mission_list(func, false)
    }
}


pub struct MissionScheduler {
    normal_handle: thread::JoinHandle<()>,
    concurrent_handle: thread::JoinHandle<()>,
    node: Node,
    pool: ThreadPool,
    scheduler_data : Arc<MissionThreadData>,
}

impl MissionScheduler {
    fn new(normal_handle: thread::JoinHandle<()>, concurrent_handle: thread::JoinHandle<()>, node: Node, scheduler_data: Arc<MissionThreadData>) -> Self {
        Self {
            normal_handle,
            concurrent_handle,
            node,
            scheduler_data,
            pool: ThreadPool::new().expect("Failed to create ThreadPool"),
        }
    }

    #[allow(unused)]
    pub fn push_back(&self, mission: impl Mission + 'static) {
        let func = move |mission_list: &mut VecDeque<MissionBox>| {
            mission_list.push_back(Box::new(mission));
            None
        };
        self.scheduler_data.with_mission_list(func, false);
    }

    #[allow(unused)]
    pub fn conc_push_back(&self, mission: impl Mission + 'static) {
        let func = move |mission_list: &mut MissionVec| {
            mission_list.push_back(Box::new(mission));
            None
        };
        self.scheduler_data.with_mission_list(func, true);
    }

    #[allow(unused)]
    pub fn append(&self, mut mission_vec: MissionVec) {
        let func = move |mission_list: &mut MissionVec| {
            mission_list.append(&mut mission_vec);
            None
        };
        self.scheduler_data.with_mission_list(func, false);
    }

    #[allow(unused)]
    pub fn conc_append(&self, mut mission_vec: MissionVec) {
        let func = move |mission_list: &mut MissionVec| {
            mission_list.append(&mut mission_vec);
            None
        };
        self.scheduler_data.with_mission_list(func, true);
    }

    pub fn get_data(&self) -> Arc<MissionData> {
        self.scheduler_data.mission_data.clone()
    }

    #[allow(unused)]
    pub fn is_waiting(&self) -> bool {
        self.scheduler_data.waiting.load(Ordering::Relaxed)
    }

    // pub fn concurrent_append(&mut self, mission: &mut Vec<Box<dyn Mission<'static> + Send >>) {
    //     self.concurrent_mission_list.append(mission);
    // }

    fn run_ros_topics(&mut self) {
        //TODO: We should not have this hardcoded
        let mut example_sub = self.node
            .subscribe::<sensor_msgs::msg::Image>("/camera/image", QosProfile::default())
            .expect("Failed to create example subscriber!");
        let example_pub= self.node
            .create_publisher::<std_msgs::msg::String>("/example", QosProfile::default())
            .expect("Failed to create example publisher!");

        let scheduler_data = self.scheduler_data.clone();
        let example_subscriber_func = async move {
            while ! scheduler_data.stop.load(Ordering::Relaxed) {
                match example_sub.next().await {
                    Some(msg) => {
                        let mission = ros_mission::new(msg);
                        scheduler_data.push_back(mission);
                    }
                    None => break,
                }
            }
        };

        let scheduler_data = self.scheduler_data.clone();
        let example_publisher_func = async move {
            let mut counter = 0;
            let mut stop = false;
            while ! stop {
                let msg = std_msgs::msg::String {
                    data: format!("{}", counter),
                };
                example_pub.publish(&msg).expect("Failed to publish example!");
                counter += 1;
                stop = scheduler_data.stop.load(Ordering::Relaxed);
                //Should we use a ros timer instead?
                sleep(Duration::from_secs(1));
                //This should probably go on another thread
            }
        };
    
        self.pool.spawn_ok(example_publisher_func);
        self.pool.spawn_ok(example_subscriber_func);
    }
   
    pub fn start() -> Self {

        let scheduler_data_orig = Arc::new(MissionThreadData::new());


        let scheduler_data = scheduler_data_orig.clone();
        let normal_func = move || {
            let mut stop = false;
            let data = &scheduler_data.mission_data;

            while ! stop {
                let run = scheduler_data.run.load(Ordering::Relaxed);
                stop = scheduler_data.stop.load(Ordering::Relaxed);
                if ! run {
                    sleep(std::time::Duration::from_millis(1));
                    continue
                }


                let mission = scheduler_data.pop_front();
                let Some(mission) = mission else {
                    println!("Waiting for missions...");
                    scheduler_data.waiting.store(true, Ordering::Relaxed);
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
                            scheduler_data.stop.store(true, Ordering::Relaxed);
                            stop = true;
                        }
                    }
                };


                sleep(std::time::Duration::from_millis(1));
            }
        };

        let scheduler_data = scheduler_data_orig.clone();
        let concurrent_func = move || {
            let mut stop = false;
            let conc_mission_list = scheduler_data.conc_mission_list.clone();
            let data = &scheduler_data.mission_data;
            while ! stop {
                let run = scheduler_data.run.load(Ordering::Relaxed);
                stop = scheduler_data.stop.load(Ordering::Relaxed);
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
                                scheduler_data.stop.store(true, Ordering::Relaxed);
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
        let ctx = r2r::Context::create().expect("Failed to create r2r context!");
        let node = r2r::Node::create(ctx, "mission_scheduler", "namespace")
            .expect("Failed to get Node!");
        MissionScheduler::new(normal_handle, conc_handle, node, scheduler_data_orig)
    }

    pub fn run(&mut self) {
        self.run_ros_topics();

        self.scheduler_data.run.store(true, Ordering::Relaxed);
    }

    pub fn stop(self) {
        self.scheduler_data.stop.store(true, Ordering::Relaxed);
        self.normal_handle.join().expect("Failed to join mission handle!");
        self.concurrent_handle.join().expect("Failed to join concurrent mission handle!");
    }

    pub fn ros_spin(&mut self) {
        self.node.spin_once(Duration::from_millis(100));
    }
}
