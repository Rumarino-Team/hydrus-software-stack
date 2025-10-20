mod mission;
mod mission_scheduler;
mod cmission;
mod pymission;
mod mission_example;
mod concurrent_mission_example;
mod ros_mission;

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::thread::sleep;
use std::{collections::VecDeque};
use std::time::{Duration, Instant};

use futures::prelude::*;
use futures::future::BoxFuture;
use pyo3::{ffi::c_str};
use r2r::{Node, QosProfile, sensor_msgs, std_msgs};

use crate::mission_scheduler::{MissionBox, MissionThreadData};
use crate::{
    mission::{CommonMission}, mission_scheduler::MissionScheduler
};

fn add_ros_topics(scheduler: &MissionScheduler) -> Node {
    let ctx = r2r::Context::create().expect("Failed to create r2r context!");
    let mut node = r2r::Node::create(ctx, "mission_planner", "namespace")
        .expect("Failed to get Node!");

    node
}

fn main() {

}

#[cfg(test)]
mod tests {
    use super::*;

    fn add_example_ros_topics(scheduler: &MissionScheduler) -> Node {
        let ctx = r2r::Context::create().expect("Failed to create r2r context!");
        let mut node = r2r::Node::create(ctx, "mission_planner", "namespace")
            .expect("Failed to get Node!");
        let mut example_sub = node
            .subscribe::<sensor_msgs::msg::Image>("/camera/image", QosProfile::default())
            .expect("Failed to create example subscriber!");
        let example_pub= node
            .create_publisher::<std_msgs::msg::String>("/example", QosProfile::default())
            .expect("Failed to create example publisher!");

        let example_subscriber_func =
        |thread_data : Arc<MissionThreadData>| {
            let scheduler_data = thread_data.clone();  
            let pin: BoxFuture<'static, ()> = Box::pin(async move {
                while ! scheduler_data.stop.load(Ordering::Relaxed) {
                    match example_sub.next().await {
                        Some(msg) => {
                            let mission = ros_mission::new(msg);
                            scheduler_data.push_back(mission);
                        }
                        None => break,
                    }
                }
            });
            pin
        };

        let example_publisher_func =
        |thread_data : Arc<MissionThreadData>| {
            let scheduler_data = thread_data.clone();
            let pin: BoxFuture<'static, ()> = Box::pin(async move {
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
            });
            pin
        };

        scheduler.add_async_thread(example_publisher_func);
        scheduler.add_async_thread(example_subscriber_func);
        node
    }

    #[test]
    fn main_test() -> Result<(), String> {
        let pytest = c_str!(include_str!("pymission_example.py"));
        let pymission_example = pymissfion::get_mission_from(pytest, c_str!("pymission_example.py"));

        let cmission_example;
        unsafe {
            let cmission_ptr = cmission_example_create();
            cmission_example = *Box::from_raw(cmission_ptr as *mut CommonMission);
        }

        let foo = mission_example::new();
        let bar = concurrent_mission_example::new();

        let mission_list: [MissionBox; 3] = [
            Box::new(pymission_example),
            Box::new(cmission_example),
            Box::new(foo),
        ];
        let mission_list = VecDeque::from(mission_list);
        let conc_mission_list: [MissionBox; 1] = [
            Box::new(bar)
        ];
        let conc_mission_list = VecDeque::from(conc_mission_list);

        let scheduler = MissionScheduler::start();
        scheduler.append(mission_list);
        scheduler.conc_append(conc_mission_list);
        let _data = scheduler.get_data();

        let start = Instant::now();
        scheduler.run();
        let mut node = add_example_ros_topics(&scheduler);
        while start.elapsed() < Duration::from_secs(15) {
            node.spin_once(Duration::from_millis(100));
        }
        scheduler.stop();
        Ok(())


    }
}
