

#[derive(Copy, Clone)]
pub struct Car {
    pos_x:f32,
    pos_y:f32,
    direction: f32,
    view_angle: f32,
    view_range: f32,
    pub velocity: f32,
    pub steering_angle: f32,
    max_angle: f32,
    b: f32,
    a: f32,
    traveled_distance: f32,
}

impl Car {
    pub fn new(x:f32, y:f32, direction: f32) -> Car {
        Car {
            pos_x: x,
            pos_y: y,
            direction: direction,
            view_angle: 160.0,
            view_range: 20.0,
            velocity: 0.0,
            steering_angle: 0.0,         // -1.0 < angle < 1.0
            max_angle: std::f32::consts::PI/8.0,        // angle @ steering_angle 1.0 / -1.0
            b: 2.0,                         // distance between left anf right wheel
            a: 2.0,                          // distance between front and rear axis
            traveled_distance: 0.0,
        }
    }
}

impl Car {
    pub fn update_car_position (&mut self) {
        let old_x: f32 = self.pos_x;
        let old_y: f32 = self.pos_y;
        if self.steering_angle.abs() > 0.01 {
            let r = (self.steering_angle*self.max_angle+std::f32::consts::PI/2.0).tan()*self.a;    // radius of curve or between car and icc
            let alpha = self.velocity/(r+self.b/2.0);                                    // angle of traveled distance per tick

            let len = ((alpha/2.0).tan()*(r+self.b/2.0))*2.0;     // eukled dist between actual and next position
            let beta = self.direction + alpha;                    // angle to next position

            // update position
            self.pos_x += beta.cos()*len;
            self.pos_y += beta.sin()*len;
            self.direction += alpha;
            /*if self.direction < 0.0 {
                self.direction += std::f32::consts::PI*2.0;
            } else if self.direction > std::f32::consts::PI*2.0 {
                self.direction -= std::f32::consts::PI*2.0;
            }*/

        } else {    // straight movement
            self.pos_x += self.direction.cos()*self.velocity;
            self.pos_y += self.direction.sin()*self.velocity;
        }
        self.traveled_distance += f32::sqrt((old_x-self.pos_x).powf(2.0)+(old_y-self.pos_y).powf(2.0));
    }

    pub fn get_position(self) -> (f32,f32,f32) {
        return (self.pos_x, self.pos_y, self.direction);
    }

    pub fn scan_cones(self, blue_cones: Vec<(f64,f64)>, yellow_cones: Vec<(f64,f64)>) -> (Vec<(f64,f64)>,Vec<(f64,f64)>) {
        let mut detected_blue: Vec<(f64,f64)> = vec![];
        let mut detected_yellow: Vec<(f64,f64)> = vec![];
        //println!("{}", self.direction);
        for blue in blue_cones {
            if self.is_cone_in_range((blue.0 as f32, blue.1 as f32)) {
                detected_blue.push(blue);
            } 
        }
        for yellow in yellow_cones {
            if self.is_cone_in_range((yellow.0 as f32, yellow.1 as f32)) {
                detected_yellow.push(yellow);
            } 
        }
        return (detected_blue,detected_yellow)
    }

    fn is_cone_in_range(self, cone: (f32,f32)) -> bool {
        let x_diff = self.pos_x-cone.0;
        let y_diff = self.pos_y-cone.1;
        let distance = f32::sqrt(f32::powf(x_diff,2.0) + f32::powf(y_diff,2.0));
        if distance <= self.view_range {
            return self.is_cone_in_angle(y_diff,x_diff);
        }
        return false;
    }

    fn is_cone_in_angle(self, y_diff: f32, x_diff:f32) -> bool {
        let radiant = y_diff.atan2(x_diff) + std::f32::consts::PI*2.0;
        let car_direction = self.direction % (std::f32::consts::PI*2.0);
        let min_rad = car_direction - ((self.view_angle*std::f32::consts::PI)/180.0)/2.0 + std::f32::consts::PI;
        let max_rad = car_direction + ((self.view_angle*std::f32::consts::PI)/180.0)/2.0 + std::f32::consts::PI;
        if (min_rad < radiant && radiant < max_rad) ||
         (min_rad < (radiant - std::f32::consts::PI*2.0) &&  (radiant - std::f32::consts::PI*2.0) < max_rad) || 
         (min_rad < (radiant + std::f32::consts::PI*2.0) &&  (radiant + std::f32::consts::PI*2.0) < max_rad)  {
            return true;
        } 
        return false;
    }

    pub fn get_view_edge(self) -> Vec<(f64,f64)> {
        let n = 20;
        let mut points: Vec<(f64,f64)> = vec![];
        let min_rad = self.direction - ((self.view_angle*std::f32::consts::PI)/180.0)/2.0;
        let max_rad = self.direction + ((self.view_angle*std::f32::consts::PI)/180.0)/2.0;
        for i in 0..n {
            let d = (self.view_range /(n as f32))*(i as f32);
            let min_x = (min_rad.cos()*d + self.pos_x) as f64;
            let min_y = (min_rad.sin()*d + self.pos_y) as f64;
            let max_x = (max_rad.cos()*d + self.pos_x) as f64;
            let max_y = (max_rad.sin()*d + self.pos_y) as f64;

            points.push((min_x,min_y));
            points.push((max_x,max_y));
            let angle_dx = (((self.view_angle*std::f32::consts::PI)/180.0)) /(n as f32);
            let g = angle_dx*(i as f32);
            let grad_x = ((min_rad + g).cos()*self.view_range + self.pos_x) as f64;
            let grad_y = ((min_rad + g).sin()*self.view_range + self.pos_y) as f64;
            points.push((grad_x,grad_y));

        }
        return points;
    
    }
     pub fn get_traveled_distance(self) -> f32 {
        return self.traveled_distance;
     }
}


