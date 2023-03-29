use anyhow::{Result, Error};
use rclrs;
use nav_msgs::msg::Path as PathMsg;
use nav_msgs::msg::Odometry as OdometryMsg;
use std::thread;
use std::env;
use std::sync::{Arc, Mutex};

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "pratham_path_follower")?;

    let path_ptr: Arc<Mutex<Option<PathMsg>>> = Arc::new(Mutex::new(Option::<PathMsg>::None));

    let path_subscription_path_ptr = Arc::clone(&path_ptr);
    let path_subscription = node
        .create_subscription("/path_local",
                             rclrs::QOS_PROFILE_DEFAULT,
                             move |msg: PathMsg| {
                                 /*
                                 println!("received path of type {:?}", &msg.poses.len());
                                 println!("received path begin {:?}", &msg.poses[0]);
                                 println!("received path end {:?}", &msg.poses[&msg.poses.len()-3]);
                                 println!("received path end {:?}", &msg.poses[&msg.poses.len()-2]);
                                 println!("received path end {:?}\n\n\n", &msg.poses[&msg.poses.len()-1]);
                                 */
                                 println!("received path");
                                 *path_subscription_path_ptr.lock().unwrap() = Some(msg);
                             },);

    loop {
        rclrs::spin(&node)?;
    }
    Ok(())
}
