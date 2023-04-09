use std::iter::zip;
use std::vec;
use macroquad::prelude as mq;


use rand::prelude::*;

const METER_TO_PXL: f32 = 5.0;      // One meter has the size of 5 pixel

use crate::car::Car;
use crate::dmp::DMP;
use crate::dmp::PD_Controller;

pub mod car;
pub mod dmp;

#[derive(Copy, Clone)]

struct screen {
    x: f32,
    y: f32,
    zoom: f32,
    mouse_pressed: bool,
    m_pressed_x: f32,
    m_pressed_y: f32
}

#[macroquad::main("BasicShapes")]
async fn main() {

    let (blue_cones,yellow_cones,orange_cones) = generate_cone_lists();


    let mut display = screen {
        x: mq::screen_width()/2.0,
        y: mq::screen_height()/2.0,
        zoom: 1.0,
        mouse_pressed: false,
        m_pressed_x: 0.0,
        m_pressed_y: 0.0
    };

    let mut race_car = Car::new((orange_cones[0].0+orange_cones[1].0) as f32 /2.0, 
                                        (orange_cones[0].1+orange_cones[1].1) as f32 /2.0,
                                3.0*std::f32::consts::PI/2.0);

    let mut steering_controller = PD_Controller::new(1.0,1.0);
    let mut steering_goal: f32 = 0.0;

    loop {
        
        mq::clear_background(mq::DARKGRAY);
        let (detected_blue, detected_yellow) = race_car.scan_cones(blue_cones.clone(), yellow_cones.clone());
        for cone in detected_blue.iter() {
            draw_cone(*cone,0.35,mq::BLUE, display);
        }
        for cone in detected_yellow.iter() {
            draw_cone(*cone,0.35,mq::YELLOW, display);
        }
        for cone in orange_cones.iter() {
            draw_cone(*cone,0.35,mq::ORANGE, display);
        }
        let (_,_,car_direction) = race_car.get_position();
        race_car.steering_angle = steering_controller.compute(car_direction, steering_goal);
        if race_car.steering_angle > 1.0 {
            race_car.steering_angle = 1.0;
        } else if race_car.steering_angle < -1.0 {
            race_car.steering_angle = -1.0;
        }
        println!("steering angle:{}, car_direction:{}", race_car.steering_angle, car_direction);
        race_car.update_car_position();
        draw_car(race_car, display);
        let edge_points = race_car.get_view_edge();
        for cone in edge_points.iter() {
            draw_cone(*cone,0.35,mq::GREEN, display);
        }
        // Mouse Actions
        let (mouse_x, mouse_y) = mq::mouse_position();
        let (_, mouse_wheel) = mq::mouse_wheel();
        if mouse_wheel > 0.0 {
            display.zoom =  display.zoom+0.1;
        } else if mouse_wheel < 0.0 {
            display.zoom =  display.zoom-0.1;
        }
        if mq::is_mouse_button_down(mq::MouseButton::Left) {
            if display.mouse_pressed {
                display.x += mouse_x-display.m_pressed_x;
                display.y += mouse_y-display.m_pressed_y;
            }
            display.m_pressed_x = mouse_x;
            display.m_pressed_y = mouse_y;
            display.mouse_pressed = true;
        }
        else {
            display.mouse_pressed = false;
        }

        // Manual Actions
        if mq::is_key_down(mq::KeyCode::Right) {
            steering_goal += -0.01;
        } else if mq::is_key_down(mq::KeyCode::Left) {
            steering_goal += 0.01;
        } 
        if race_car.steering_angle > 1.0 {
            race_car.steering_angle = 1.0;
        } else if race_car.steering_angle < -1.0 {
            race_car.steering_angle = -1.0;
        }
        if mq::is_key_pressed(mq::KeyCode::Down) {
            race_car.velocity -= 0.1;
        }
        if mq::is_key_pressed(mq::KeyCode::Up) {
            race_car.velocity += 0.1;
        }




        mq::next_frame().await
    }

}


fn draw_cone(cone: (f64,f64), r: f32, color: mq::Color, sc: screen) {
    mq::draw_circle((mq::screen_width()/2.0 + (cone.0 as f32)*METER_TO_PXL)*sc.zoom+sc.x, (mq::screen_height()/2.0 + (cone.1 as f32)*METER_TO_PXL)*sc.zoom+sc.y, r*sc.zoom * METER_TO_PXL , color);
}

fn draw_car(car: Car, sc: screen) {
    let (car_x,car_y,car_direction) = car.get_position();
    let x = (mq::screen_width()/2.0 + car_x * METER_TO_PXL ) * sc.zoom + sc.x;
    let y = (mq::screen_height()/2.0 + car_y * METER_TO_PXL ) * sc.zoom + sc.y;
    
    let v1 = mq::Vec2 {
        x: x+(car_direction.cos()*2.0)*( METER_TO_PXL * sc.zoom),
        y: y+(car_direction.sin()*2.0)*( METER_TO_PXL * sc.zoom)
    };  // (mq::screen_width()/2.0 + (car.pos_x as f32)*3.0)*sc.zoom+sc.x,
    let v2 = mq::Vec2 {
        x: x+(-(car_direction+std::f32::consts::PI/2.0).cos()*0.75)*( METER_TO_PXL * sc.zoom),
        y: y+(-(car_direction+std::f32::consts::PI/2.0).sin()*0.75) * (METER_TO_PXL * sc.zoom)
    }; 
    let v3 = mq::Vec2 {
        x: x+((car_direction+std::f32::consts::PI/2.0).cos()*0.75)*( METER_TO_PXL * sc.zoom),
        y: y+((car_direction+std::f32::consts::PI/2.0).sin()*0.75) * (METER_TO_PXL * sc.zoom)
    }; 
    mq::draw_triangle(v1, v2,v3, mq::RED);
}


fn generate_cone_lists() -> (Vec<(f64,f64)>,Vec<(f64,f64)>,Vec<(f64,f64)>) {
    
    let center = (0.,0.);
    let r1 = 40.0;
    let r2 = 40.0;
    let n: u32 = 8;
    let (p, pitch) = get_circle_samples(center, r1, r2, n);
    let mut pitch_points: Vec<(f64,f64)> = vec![];

    for ((x,y),pit) in zip(p.clone(), pitch.clone()) {
        pitch_points.push((x+pit.cos()*0.5,y+pit.sin()*0.5));
        print!("{}",pit);
    }

    let mut bezier: Vec<(f64, f64)> = vec![];
    for i in 0..n-1 {
        let mut bezier_points = bezier_curve(&p[i as usize], pitch[i as usize],&p[(i+1) as usize], pitch[(i+1) as usize], 100);
        bezier.append(&mut bezier_points);
    }
    let mut bezier_points = bezier_curve(&p[(n-1) as usize], pitch[(n-1) as usize],&p[0], pitch[0], 100);
    bezier.append(&mut bezier_points);



    let mut last_point: (f64,f64) = (0.0,0.0);  //point[i-1]
    let mut last_cone: (f64,f64) = (0.0,0.0);
    let mut blue_cones: Vec<(f64, f64)> = vec![];
    let mut yellow_cones: Vec<(f64, f64)> = vec![];
    let mut orange_cones: Vec<(f64, f64)> = vec![];
    orange_cones.push((bezier[0].0-1.5,bezier[0].1));
    orange_cones.push((bezier[0].0+1.5,bezier[0].1));
    for point in bezier.iter() {
        if get_eukled_distance(last_cone,*point) > 4.0 {
            if !((last_cone.0 == 0.0) && (last_cone.1 == 0.0 )) {
                let mut radiant = f64::atan2(last_point.1-point.1, last_point.0-point.0)+ std::f64::consts::PI/2.0;
                yellow_cones.push((point.0+radiant.cos()*1.5, point.1+radiant.sin()*1.5));
                radiant = radiant - std::f64::consts::PI;
                blue_cones.push((point.0+radiant.cos()*1.5, point.1+radiant.sin()*1.5));    
            }
            last_cone = *point;

        }
        last_point = *point;
    }
    return (blue_cones, yellow_cones,orange_cones);

}

fn get_eukled_distance(p1: (f64,f64), p2:(f64,f64)) -> f64 {
    return (f64::powf(p1.0-p2.0,2.0)+f64::powf(p1.1-p2.1,2.0)).sqrt()
}

fn get_circle_samples(center:(f64,f64), r1: f64, r2:f64, n:u32) -> (Vec<(f64, f64)>, Vec<f64>){
    let mut points: Vec<(f64, f64)>= vec![];
    let mut pitch: Vec<f64>= vec![];

    for i in 0..n {
        let diff = std::f64::consts::PI*2.0/f64::from(n);
        let factor = 0.2;
        let mut rng =  rand::thread_rng();
        let rad = f64::from(i)*diff;
        let x = (r1*rad.cos()+center.0) + rng.gen_range(-r1*factor..r1*factor);
        let y = (r2*rad.sin()+center.1) + rng.gen_range(-r2*factor..r2*factor);
        points.push((x,y));
        pitch.push(rad+std::f64::consts::PI/2.0);
    }
    return (points, pitch);
}

fn bezier_curve(s_point: &(f64,f64), s_rad:f64, e_point: &(f64,f64), e_rad: f64, n:u32)-> Vec<(f64,f64)> {
    let mut points: Vec<(f64,f64)> = vec![];
    let dx: f64 = 1.0/f64::from(n);
    let factor = 10.0;
    let s_support: (f64,f64)  = (s_point.0+s_rad.cos()*factor,s_point.1+s_rad.sin()*factor);
    let e_support: (f64,f64)  = (e_point.0+(e_rad-std::f64::consts::PI).cos()*factor,e_point.1+(e_rad-std::f64::consts::PI).sin()*factor);
    for i in 0..n {
        let t = dx*f64::from(i);
        let x:f64 = s_point.0 + t*(-3.0*s_point.0+3.0*s_support.0) + t*t*( 3.0* s_point.0-6.0*s_support.0+ 3.0*e_support.0) + t*t*t* ( -s_point.0+ 3.0*s_support.0-3.0*e_support.0+e_point.0);
        let y:f64 = s_point.1 + t*(-3.0*s_point.1+3.0*s_support.1) + t*t*( 3.0* s_point.1-6.0*s_support.1+ 3.0*e_support.1) + t*t*t* ( -s_point.1+ 3.0*s_support.1-3.0*e_support.1+e_point.1);
        points.push((x,y));
    }


    return points;

}