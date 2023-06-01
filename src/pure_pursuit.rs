use anyhow::{Error, Result};
use geometry_msgs::msg::Twist as TwistMsg;
use nav_msgs::msg::Odometry as OdometryMsg;
use nav_msgs::msg::Path as PathMsg;
use rclrs;
use std::env;
use std::sync::{Arc, Mutex};
use std::thread;
use std_msgs::msg::Bool as BoolMsg;

use dynamics_solver;
mod path;

use nalgebra::Vector2;
use nalgebra::{Quaternion, UnitQuaternion};

// dt = lookahead time
// The amount of time that the controller server will propagate ...
// the current state to check the future/lookahead deviation
static DT: f64 = 3.3;

// The deviation threshold is the first threshold...
// It is compared to the deviation of the current state of the robot from the path
// If the current deviation is greater than the deviation threshold,
//      it will move at linear_x = MIN_LIN_X_FOR_ROT
//      angular_z will be a proportional control system with gain = HARSH_GAIN
// Else, it will propagate the current state to the future by time DT to
// predict the future state and check the future deviation/lookahead deviation
static DEVIATION_THRESHOLD: f64 = 0.015; // in meters

// If the controller server decides the the current deviation is acceptable and propagates the state to the future,
// It will compare the future deviation to DEVATION_THRESHOLD*LOOKAHEAD_DISCOUNT_FACTOR instead of DEVATION_THRESHOLD alone
static LOOKAHEAD_DISCOUNT_FACTOR: f64 = 1.15;

// Before checking the linear displacement of the robot from the path, it'll check the angular deviation from the path.
// and apply proportional control with ANGULAR_GAIN to the angular_z with 0 linear_x
static ANGULAR_DEVIATION_THRESHOLD: f64 = 1.3; // 60 degrees

// junk parameter, let it be, no need to tune this
static DEACCELERATION_DECAY: f64 = 0.5;


// minimum LINEAR_X that the robot is allowed to move at
static MIN_LIN_X: f64 = 0.48;

// maximum LINEAR_X that the robot is allowed to move at
static MAX_LIN_X: f64 = 0.63;

// the linear_x of the robot for the harsh gain condition, the name is immaterial
static MIN_LIN_X_FOR_ROT: f64 = 0.3;

// the maximum angular velocity to prescibe to the control systems
static MAX_ANG_Z: f64 = 1.0471975;

static ANGULAR_GAIN: f64 = 1.5;

static HARSH_GAIN: f64 = -6.8;
static MEDIUM_GAIN: f64 = -4.8;
static DAMPED_GAIN: f64 = -3.8;

/*
 * Angular gain is employed on angular_z of the current angular_deviation is greater than ANGULAR_DEVIATION_THRESHOLD
 *      In this case, the linear_x of the robot is 0
 * Harsh gain is employed on the angular_z of the robot if the current deviation is greater than DEVIATION_THRESHOLD
 *      In this case, the linear_x of the robot will be MIN_LIN_X_FOR_ROT
 * Medium gain is employed if the current deivation is fine, and the future deviation is greater than LOOKAHEAD_DISCOUNT_FACTOR*DEVIATION_THRESHOLD
 *      In this case, the linear_x of the robot will be MIN_LIN_X
 * Damped gain is emplyed if the current deviation ii fine, and the future deviation as well.
 *      In this case, the linear_x of the robot will be decided by this formula:
 *             vel_cmd.linear.x = min_lin_x + (max_lin_x - min_lin_x) * deviation.abs() / (DEVIATION_THRESHOLD * LOOKAHEAD_DISCOUNT_FACTOR);
 */


fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "pratham_path_follower")?;

    let path_ptr: Arc<Mutex<Option<path::Path>>> = Arc::new(Mutex::new(Option::<path::Path>::None));
    let path_found_ptr: Arc<Mutex<Option<bool>>> = Arc::new(Mutex::new(Option::<bool>::None));
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
    println!("creating the path subscription");
    let _path_subscription = node.create_subscription(
        "/path_local",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: PathMsg| {
            // println!("received path");
            let path_to_follow = path::Path::new(msg.clone());
            // path_to_follow.find_closest_point(Vector2::new(0.2f64, 0f64));

            *path_subscription_path_ptr.lock().unwrap() = Some(path_to_follow);
            if let Some(odom) = &*path_subscription_odom_ptr.lock().unwrap() {
                // set reference odom
                println!("setting the reference odom as {:?}", &odom.pose.pose.position);
                *path_subscription_reference_odom_ptr.lock().unwrap() = Some(odom.clone());
            } else {
                println!("The odom pointer is None, please make sure the odometry topic is being published to");
            }
        },
    );

    let path_found_subscription_path_found_ptr = Arc::clone(&path_found_ptr);
    println!("creating the path_found subscription");
    let _path_found_subscription = node.create_subscription(
        "/path_found",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: BoolMsg| {
            *path_found_subscription_path_found_ptr.lock().unwrap() = Some(msg.data);
        },
    );

    let odom_subscription_odom_ptr = Arc::clone(&odom_ptr);
    let odom_subscription_path_ptr = Arc::clone(&path_ptr);
    let odom_subscription_path_found_ptr = Arc::clone(&path_found_ptr);
    let odom_subscription_reference_odom_ptr = Arc::clone(&reference_odom_ptr);
    let odom_subscription_max_ang_z_ptr = Arc::clone(&max_ang_z_ptr);
    let odom_subscription_max_lin_x_ptr = Arc::clone(&max_lin_x_ptr);
    let odom_subscription_min_lin_x_ptr = Arc::clone(&min_lin_x_ptr);
    let odom_subscription_cmd_vel_publisher_ptr = Arc::clone(&cmd_vel_publisher_ptr);
    println!("Creating the odom subscription");
    let _odom_subscription = node.create_subscription(
        "/odometry/filtered",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: OdometryMsg| {

            println!("Received odom");
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
            let path_found_opt = & *odom_subscription_path_found_ptr.lock().unwrap();
            let path_to_follow_opt = & *odom_subscription_path_ptr.lock().unwrap();

            match path_found_opt {
                None => {
                    println!("The path is empty, is it being published?, sending stop command");
                },
                Some(path_found) => {
                    if *path_found {
                        match path_to_follow_opt {
                            Some(path_to_follow) => {
                                println!("The current odom is {:?}", &msg.pose.pose.position);
                                match path_to_follow.get_path_deviation(Vector2::new(current_state.x, current_state.y)) {
                                    Some((deviation, angular_deviation)) => {
                                        if angular_deviation.abs() > ANGULAR_DEVIATION_THRESHOLD {
                                            println!("The angular deviation is too high");
                                            vel_cmd.linear.x = 0.0;
                                            vel_cmd.angular.z = ANGULAR_GAIN * angular_deviation;
                                        } else {
                                            if deviation > DEVIATION_THRESHOLD {
                                                println!("The current deviation is high {}", &deviation);
                                                vel_cmd.linear.x = MIN_LIN_X_FOR_ROT;
                                                vel_cmd.angular.z = HARSH_GAIN * deviation;
                                            } else {
                                                println!("the current deviation ({}), is lower than the threshold, propagating the state", &deviation);
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
                    } else {
                        println!("The path planner is unable to find path, executing recovery behaviour");
                        vel_cmd.linear.x = -0.2;
                    }
                }
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
