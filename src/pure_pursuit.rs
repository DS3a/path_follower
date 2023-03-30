use anyhow::{Error, Result};
use nav_msgs::msg::Odometry as OdometryMsg;
use nav_msgs::msg::Path as PathMsg;
use rclrs;
use std::env;
use std::sync::{Arc, Mutex};
use std::thread;

use dynamics_solver;
mod path;

use nalgebra::Vector2;

// dt = lookahead time
static dt: f64 = 0.5;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "pratham_path_follower")?;

    let path_ptr: Arc<Mutex<Option<path::Path>>> = Arc::new(Mutex::new(Option::<path::Path>::None));
    let odom_ptr: Arc<Mutex<Option<OdometryMsg>>> =
        Arc::new(Mutex::new(Option::<OdometryMsg>::None));

    let path_subscription_path_ptr = Arc::clone(&path_ptr);
    let path_subscription_odom_ptr = Arc::clone(&odom_ptr);
    let path_subscription = node.create_subscription(
        "/path_local",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: PathMsg| {
            println!("received path");
            /*
             * TODO
             * store reference odom
             * store the path vector DONE
             */
            // *path_subscription_path_ptr.lock().unwrap() = Some(msg);
            let path_to_follow = path::Path::new(msg);
            path_to_follow.find_closest_point(Vector2::new(0.2f64, 0f64));
            *path_subscription_path_ptr.lock().unwrap() = Some(path_to_follow);
            if let Some(odom) = &*path_subscription_odom_ptr.lock().unwrap() {
                // TODO set reference odom
            } else {
                println!("The odom pointer is None, please make sure the odometry topic is being published to")
            }
        },
    );

    let odom_subscription_odom_ptr = Arc::clone(&odom_ptr);
    let odom_subscription = node.create_subscription(
        "/Odometry/local",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: OdometryMsg| {
            *odom_subscription_odom_ptr.lock().unwrap() = Some(msg);
        },
    );

    thread::spawn(move || {
        /*
         * TODO follow the path
         *
         */
    });

    loop {
        rclrs::spin(&node)?;
    }
}
