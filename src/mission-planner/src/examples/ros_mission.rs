use crate::mission::{Mission, MissionResult, MissionData};
use cv_bridge::CvImage;
use opencv::highgui;
use r2r::sensor_msgs::msg::Image;

struct ExampleImageMission {
    name: String,
    image: Image,
}

impl Mission for ExampleImageMission {
    fn name(&self) -> &String {
        return &self.name
    }
    fn run(&self, data: &MissionData) -> MissionResult {
        ros_example(&self.image, data)
    }
    
}

fn ros_example(image: &Image, _data: &MissionData) -> MissionResult {

    let mut cv_image = CvImage::from_imgmsg(image.clone()).expect("Failed to get cvimage!");
    let mat = match cv_image.as_cvmat() {
        Ok(mat) => mat,
        Err(err) => {
            println!("Error getting mat: {}", err);
            return Err(false)
        }
    };

    // let scalar =opencv::core::Scalar::new(0.0, 0.0, 0.0, 0.0);
    // let mat = opencv::core::Mat::new_rows_cols_with_default(1024, 1024, CV_8UC1, scalar)
    //     .unwrap();

    let window = "foo";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE).unwrap();
    highgui::imshow(window, &mat).unwrap();
    highgui::wait_key(1).unwrap();

    Ok(())
}

pub fn new(image: Image) -> impl Mission {
    let name = "ros-example-mission".to_string();

    ExampleImageMission { name, image }
}
