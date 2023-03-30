use nalgebra::Vector2;
use nav_msgs::msg::Path as PathMsg;

fn get_dist(a: Vector2<f64>, b: Vector2<f64>) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).powf(0.5)
}

pub struct Path {
    path_vector: Vec<Vector2<f64>>,
}

impl Path {
    pub fn new(path_msg: PathMsg) -> Self {
        let mut path_vector = Vec::<Vector2<f64>>::new();
        for pose_index in 0..(path_msg.poses.len()) {
            path_vector.push(Vector2::new(
                path_msg.poses[pose_index].pose.position.x,
                path_msg.poses[pose_index].pose.position.y,
            ));
        }

        Self { path_vector }
    }

    pub fn find_closest_point(&self, point: Vector2<f64>) -> (Vector2<f64>, usize) {
        let mut min_dist: Option<f64> = None;
        let mut min_dist_ind: usize = 0;

        for pose_index in 0..(self.path_vector.len()) {
            match &mut min_dist {
                Some(dist_) => {
                    let dist = get_dist(point, self.path_vector[pose_index]);
                    if *dist_ > dist {
                        min_dist = Some(dist);
                        min_dist_ind = pose_index;
                    }
                }
                None => {
                    min_dist = Some(get_dist(point, self.path_vector[pose_index]));
                }
            }
        }

        println!(
            "the minimum distance is {:?}, and the index is {}",
            min_dist, min_dist_ind
        );

        (self.path_vector[min_dist_ind], min_dist_ind)
    }
}
