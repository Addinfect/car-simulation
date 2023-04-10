
use std::iter::zip;

const BASE_REWARD: f32 = 100.0;

#[derive(Clone)]

pub struct DMP {
    n: i32,
    reward_trajectory: Vec<(f64,f64,f32)>,
    variance: f32,
    weights: Vec<f32>,
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
        let mut weight_vec: Vec<f32> = vec![0.0;number_gaussians as usize];
        DMP{
            n: number_gaussians,
            reward_trajectory: r_trajectory,
            variance: 1.0,
            weights: weight_vec,
        }
    }

    pub fn get_reward(pos: (f32,f32) ) -> f32 {
        return 0.0
    }

    fn base_function(self,x: f32, c:f32) -> f32 {
        // variance = n_base_functions / c
        return (f32::powf(x-c,2.0)* -(self.n as f32 / c)).exp();
    }

    pub fn set_weights(&mut self,  weights: Vec<f32>) {
        if weights.len() == self.weights.len() {
            self.weights = weights;
        }
    }

    fn forcing_function(self, goal: f32, state: f32) -> Vec<f32> {
        let mut y_vec: Vec<f32> = vec![];
        for i in 0..1000 {
            let x = i as f32 *0.001;
            let mut base_sum = 0.0;
            let mut base_sum_weights = 0.0;
            let n_base_functions = self.n.clone();
            for n in 0..n_base_functions {
                let c = 1.0/(n_base_functions as f32);
                let t = self.clone().base_function(x, c*((n+1) as f32));
                base_sum_weights += t*self.weights[n as usize];
                base_sum += t; //*x*(goal-state);
            }

            y_vec.push( (base_sum_weights/base_sum) * x * (goal-state));
        }
        return y_vec;
    }

    pub fn generate_trajectory(self, goal: f32, state: f32) -> Vec<f32> {
        return self.forcing_function(goal, state);
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