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
use nalgebra::{Quaternion, UnitQuaternion};

// dt = lookahead time
static DT: f64 = 0.5;


fn main() -> Result<(), Error> {

    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "pratham_path_follower")?;

    let path_ptr: Arc<Mutex<Option<path::Path>>> = Arc::new(Mutex::new(Option::<path::Path>::None));
    let odom_ptr: Arc<Mutex<Option<OdometryMsg>>> =
        Arc::new(Mutex::new(Option::<OdometryMsg>::None));

    let reference_odom_ptr: Arc<Mutex<Option<OdometryMsg>>> =
        Arc::new(Mutex::new(Option::<OdometryMsg>::None));


    let path_subscription_path_ptr = Arc::clone(&path_ptr);
    let path_subscription_odom_ptr = Arc::clone(&odom_ptr);
    let path_subscription_reference_odom_ptr = Arc::clone(&reference_odom_ptr);
    let _path_subscription = node.create_subscription(
        "/path_local",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: PathMsg| {
            println!("received path");
            let path_to_follow = path::Path::new(msg);
            // path_to_follow.find_closest_point(Vector2::new(0.2f64, 0f64));
            *path_subscription_path_ptr.lock().unwrap() = Some(path_to_follow);
            if let Some(odom) = &*path_subscription_odom_ptr.lock().unwrap() {
                // set reference odom
                *path_subscription_reference_odom_ptr.lock().unwrap() = Some(odom.clone());
            } else {
                println!("The odom pointer is None, please make sure the odometry topic is being published to")
            }
        },
    );

    let odom_subscription_odom_ptr = Arc::clone(&odom_ptr);
    let odom_subscription_path_ptr = Arc::clone(&path_ptr);
    let odom_subscription_reference_odom_ptr = Arc::clone(&reference_odom_ptr);
    let _odom_subscription = node.create_subscription(
        "/Odometry/local",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: OdometryMsg| {

            // calulate the current state with respect to the current reference frame.
            // The reference frame is the position of base_link when the path was generated
            let mut current_state = dynamics_solver::differential_drive::State::new(0f64, 0f64, 0f64);
            current_state.update_linear_velocity(msg.twist.twist.linear.x);
            current_state.update_angular_velocity(msg.twist.twist.angular.z);

            *odom_subscription_odom_ptr.lock().unwrap() = Some(msg.clone());
            let reference_odom = &mut *odom_subscription_reference_odom_ptr.lock().unwrap();
            match reference_odom {
                Some(ref_odom) => {
                    let ref_orientation = Quaternion::new(ref_odom.pose.pose.orientation.w,
                                                          ref_odom.pose.pose.orientation.x,
                                                          ref_odom.pose.pose.orientation.y,
                                                          ref_odom.pose.pose.orientation.z);
                    let ref_euler_angles = UnitQuaternion::from_quaternion(ref_orientation).euler_angles();
                    let ref_yaw = ref_euler_angles.2;

                    let current_orientation = Quaternion::new(msg.pose.pose.orientation.w,
                                                          msg.pose.pose.orientation.x,
                                                          msg.pose.pose.orientation.y,
                                                          msg.pose.pose.orientation.z);
                    let current_euler_angles = UnitQuaternion::from_quaternion(current_orientation).euler_angles();
                    let current_yaw = current_euler_angles.2;

                    current_state.set_state(msg.pose.pose.position.x - ref_odom.pose.pose.position.x,
                                            msg.pose.pose.position.y - ref_odom.pose.pose.position.y,
                                            current_yaw - ref_yaw);
                },
                None => {
                    println!("The reference odom has not been set, i.e., the path callback has not been called, is the path being published?");
                    // setting reference odom to current odom for debugging purposes
                    *reference_odom = Some(msg);
                }
            }

            std::mem::drop(reference_odom);
            println!("The current state is : {:?}", &current_state);

            // now to find the closest point in the path to the the current state
            let path_to_follow_opt = & *odom_subscription_path_ptr.lock().unwrap();
            match path_to_follow_opt {
                Some(path_to_follow) => {
                    let (closest_point, idx) = path_to_follow.find_closest_point(Vector2::new(current_state.x, current_state.y));
                    println!("The closest point is {}, and it is in the {} place in the path vector", &closest_point, &idx);

                    if idx == path_to_follow.get_path_size() - 1 {
                        println!("The goal has been reached");
                    } else {

                    }
                },
                None => {
                    println!("The path is empty, is it being published?");
                },
            }
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
