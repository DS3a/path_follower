use anyhow::{Error, Result};
use geometry_msgs::msg::Twist as TwistMsg;
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
static DT: f64 = 0.95492;
static DEVIATION_THRESHOLD: f64 = 0.3; // in meters
static LOOKAHEAD_DISCOUNT_FACTOR: f64 = 1.5;

static DEACCELERATION_DECAY: f64 = 0.5;

static MIN_LIN_X: f64 = 0.443;
static MAX_LIN_X: f64 = 1.0;

static MIN_LIN_X_FOR_ROT: f64 = 0.2235;

static MAX_ANG_Z: f64 = 1.0471975;

static HARSH_GAIN: f64 = 1.0;
static MEDIUM_GAIN: f64 = 0.5;
static DAMPED_GAIN: f64 = 0.25;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "pratham_path_follower")?;

    let path_ptr: Arc<Mutex<Option<path::Path>>> = Arc::new(Mutex::new(Option::<path::Path>::None));
    let odom_ptr: Arc<Mutex<Option<OdometryMsg>>> =
        Arc::new(Mutex::new(Option::<OdometryMsg>::None));

    let reference_odom_ptr: Arc<Mutex<Option<OdometryMsg>>> =
        Arc::new(Mutex::new(Option::<OdometryMsg>::None));

    let cmd_vel_publisher_ptr = Arc::new(Mutex::new(
        node.create_publisher::<TwistMsg>("/cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?,
    ));

    let max_lin_x_ptr = Arc::new(Mutex::new(MAX_LIN_X));
    let min_lin_x_ptr = Arc::new(Mutex::new(MIN_LIN_X));

    let max_ang_z_ptr = Arc::new(Mutex::new(MAX_ANG_Z));

    let path_subscription_path_ptr = Arc::clone(&path_ptr);
    let path_subscription_odom_ptr = Arc::clone(&odom_ptr);
    let path_subscription_reference_odom_ptr = Arc::clone(&reference_odom_ptr);
    let _path_subscription = node.create_subscription(
        "/path_local",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: PathMsg| {
            // println!("received path");
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
    let odom_subscription_max_ang_z_ptr = Arc::clone(&max_ang_z_ptr);
    let odom_subscription_max_lin_x_ptr = Arc::clone(&max_lin_x_ptr);
    let odom_subscription_min_lin_x_ptr = Arc::clone(&min_lin_x_ptr);
    let odom_subscription_cmd_vel_publisher_ptr = Arc::clone(&cmd_vel_publisher_ptr);
    let _odom_subscription = node.create_subscription(
        "/odometry/filtered",
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
                    *reference_odom = Some(msg.clone());
                }
            }

            std::mem::drop(reference_odom);
            // println!("The current state is : {:?}", &current_state);

            let mut vel_cmd = TwistMsg::default();

            // now to find the closest point in the path to the the current state
            let path_to_follow_opt = & *odom_subscription_path_ptr.lock().unwrap();
            match path_to_follow_opt {
                Some(path_to_follow) => {
                    println!("The twist msg is {:?}", vel_cmd);
                    match path_to_follow.get_path_deviation(Vector2::new(current_state.x, current_state.y)) {
                        Some(deviation) => {
                            if deviation > DEVIATION_THRESHOLD {
                                println!("The current deviation is high");
                                vel_cmd.linear.x = MIN_LIN_X_FOR_ROT;
                                vel_cmd.angular.z = HARSH_GAIN * deviation;
                            } else {
                                /*
                                 * TODO
                                 * propagate the state
                                 * check if the propagated state is in the path
                                 ** if it is, then check deviation and stuff, and adjust the angular velocity with medium/low gain
                                 ** if it is not,
                                 */

                                let mut propagated_state = current_state.clone().propagate(DT);
                                match path_to_follow.get_propagtation_deviation(Vector2::new(current_state.x, current_state.y),
                                                                                Vector2::new(propagated_state.x, propagated_state.y)) {
                                    Some(deviation) => {
                                        if deviation > DEVIATION_THRESHOLD * LOOKAHEAD_DISCOUNT_FACTOR {
                                            println!("The future deviation is high");
                                            vel_cmd.linear.x = *odom_subscription_min_lin_x_ptr.lock().unwrap();
                                            vel_cmd.angular.z = MEDIUM_GAIN * deviation;
                                        } else {
                                            println!("The future deviation is okay");
                                            vel_cmd.angular.z = DAMPED_GAIN * deviation;
                                            let min_lin_x = *odom_subscription_min_lin_x_ptr.lock().unwrap();
                                            let max_lin_x = *odom_subscription_max_lin_x_ptr.lock().unwrap();

                                            vel_cmd.linear.x = min_lin_x + (max_lin_x - min_lin_x) * deviation.abs() / (DEVIATION_THRESHOLD * LOOKAHEAD_DISCOUNT_FACTOR);

                                            std::mem::drop(min_lin_x);
                                            std::mem::drop(max_lin_x);
                                        }
                                    },
                                    None => { // if the propagated state is near/beyond the goal; start deceleration
                                        vel_cmd.linear.x = &msg.twist.twist.linear.x * DEACCELERATION_DECAY;
                                        vel_cmd.angular.z = &msg.twist.twist.angular.z * DEACCELERATION_DECAY;
                                    }
                                }

                            }
                        }, None => {
                            println!("The goal has been reached, sending stop command");
                        }
                    }
                },
                None => {
                    println!("The path is empty, is it being published?, sending stop command");
                },
            }

            let max_lin_x = *odom_subscription_max_lin_x_ptr.lock().unwrap();
            let max_ang_z = *odom_subscription_max_ang_z_ptr.lock().unwrap();

            if vel_cmd.linear.x.abs() > max_lin_x {
                vel_cmd.linear.x = max_lin_x * vel_cmd.linear.x/vel_cmd.linear.x.abs();
                std::mem::drop(max_lin_x);
            }

            if vel_cmd.angular.z.abs() > max_ang_z {
                vel_cmd.angular.z = max_ang_z * vel_cmd.angular.z/vel_cmd.angular.z.abs();
                std::mem::drop(max_ang_z);
            }

            odom_subscription_cmd_vel_publisher_ptr.lock().unwrap().publish(vel_cmd).unwrap();
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
