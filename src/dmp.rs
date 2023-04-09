
use std::iter::zip;

const BASE_REWARD: f32 = 100.0;

pub struct DMP {
    n: i32,
    reward_trajectory: Vec<(f64,f64,f32)>
}

impl DMP {
    pub fn new(number_gaussians: i32, yellow_cones: Vec<(f64,f64)>, blue_cones: Vec<(f64,f64)>) -> DMP {
        let mut r_trajectory: Vec<(f64,f64,f32)> = vec![];
        let mut i: i32 = 0;
        for (b_cone, y_cone) in zip(yellow_cones,blue_cones) {
            let x = (b_cone.0 + y_cone.0)/2.0;
            let y = (b_cone.1 + y_cone.1)/2.0;
            let reward: f32 = i as f32 + BASE_REWARD;
            r_trajectory.push((x,y,reward));

        }
        DMP{
            n: number_gaussians,
            reward_trajectory: r_trajectory 
        }
    }

    pub fn get_reward(x:f32, y:f32) -> f32 {
        return 0.0
    }

    
}

pub struct PD_Controller {
    Kp: f32,
    Kd: f32,
    last_error: f32

}

impl PD_Controller {

    pub fn new(p:f32, d:f32) -> PD_Controller {
        PD_Controller {
            Kp: p,
            Kd: d,
            last_error: 0.0
        }
    }

    pub fn compute(&mut self, state: f32, goal: f32) -> f32 {
        let error = state-goal;
        let error_diff = error - self.last_error;
        self.last_error = error;

        return self.Kp * error + self.Kd * error_diff;
    }
}