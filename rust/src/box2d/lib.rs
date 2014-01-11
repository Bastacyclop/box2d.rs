#[link(name="cbox2d", kind="static")]

pub mod math;
pub mod ffi;
pub mod dynamics;
pub mod shapes;
pub mod settings;
pub mod common;


fn main () {
    println("Rust box2d test...");
    let gravity = math::Vec2 { x:0.0, y:-1.0 };
    let mut world = dynamics::World::new(gravity);

    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    let time_step = 1.0f32 / 60.0f32;
    let velocity_iterations = 6;
    let position_iterations = 2;

    println("a");
    assert_eq!(world.get_body_count(), 0);
    println("b");
    assert_eq!(world.get_gravity(), gravity);
    println("c");

    let mut body_def = dynamics::BodyDef::default();
    let mut b1 = world.create_body(&body_def);

    let shape = shapes::PolygonShape::box_shape(2.0, 3.0);

    println("d");
    assert_eq!(world.get_body_count(), 1);
    println("e");

    for i in range(0, 60) {
        println(" -- step");
        world.step(time_step, velocity_iterations, position_iterations);
        let pos = b1.get_position();
        println!("b1: [x:{}, y:{}]", pos.x, pos.y);
    }

    world.destroy_body(&mut b1);
    assert_eq!(world.get_body_count(), 0);
    // ...
    // RAII takes car of destroying the world
}
