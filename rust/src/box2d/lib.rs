#![crate_id = "box2drs"]

#![allow(non_camel_case_types)]
#![feature(link_args)]
#[cfg(main_test)]
use std::mem;

pub mod math;
pub mod ffi;
pub mod dynamics;
pub mod shapes;
pub mod settings;
pub mod common;

#[cfg(main_test)]
fn main () {
    println!("Rust box2d test...");

    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    let time_step = 1.0 / 60.0;
    let velocity_iterations = 6;
    let position_iterations = 2;


    let gravity = math::Vec2 { x:0.0, y:-10.0 };
    let mut world = dynamics::World::new(gravity);

    assert_eq!(world.get_body_count(), 0);
    assert_eq!(world.get_gravity(), gravity);

    let shape = shapes::PolygonShape::box_shape(2.0, 3.0);

    let mut body_def = dynamics::BodyDef::default();
    body_def.body_type = dynamics::DYNAMIC_BODY;
    body_def.position = math::Vec2 {x:0.0, y:4.0};
    body_def.linear_velocity = math::Vec2 {x:1.0, y:4.0};

    let mut b1 = world.create_body(&body_def);

    let mut fixture_def = dynamics::FixtureDef::default(&shape as &shapes::Shape);
    // Set the box density to be non-zero, so it will be dynamic.
    fixture_def.density = 1.0;
    fixture_def.friction = 0.3;
    b1.create_fixture(&fixture_def);

    assert_eq!(world.get_body_count(), 1);

    for _ in range(0, 60) {
        world.step(time_step, velocity_iterations, position_iterations);
        let pos = b1.get_position();
        println!("b1: [x:{}, y:{}]", pos.x, pos.y);
    }

    world.destroy_body(&mut b1);
    assert_eq!(world.get_body_count(), 0);

    let sz = mem::size_of::<dynamics::BodyDef>();
    println!("size_of: {}", sz);

    // ...
    // RAII takes car of destroying the world
}
