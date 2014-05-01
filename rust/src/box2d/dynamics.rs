
use std::cast;
use math::{Vec2};
use ffi;
use shapes;
use common;

pub type BodyType = i32;
pub static STATIC_BODY: BodyType    = 0 as BodyType;
pub static KINEMATIC_BODY: BodyType = 1 as BodyType;
pub static DYNAMIC_BODY: BodyType   = 2 as BodyType;

pub type JointType = i32;
pub static UNKNOWN_JOINT: JointType      = 0 as JointType;
pub static REVOLUTE_JOINT: JointType     = 1 as JointType;
pub static PROSMATIC_JOINT: JointType    = 2 as JointType;
pub static DISTANCE_JOINT: JointType     = 3 as JointType;
pub static PULLEY_JOINT: JointType       = 4 as JointType;
pub static MOUSE_JOINT: JointType        = 5 as JointType;
pub static GEAR_JOINT: JointType         = 6 as JointType;
pub static WHEEL_JOINT: JointType        = 7 as JointType;
pub static WELD_JOINT: JointType         = 8 as JointType;
pub static FRICTION_JOINT: JointType     = 9 as JointType;
pub static ROPE_JOINT: JointType         = 10 as JointType;

pub struct World {
    pub ptr: *ffi::box2d_World,
}

pub struct Body {
    pub ptr: *ffi::box2d_Body,
}

pub struct Fixture {
    pub ptr: *ffi::box2d_Fixture,
}

pub struct Joint {
    pub ptr: *ffi::box2d_Joint,
}

pub struct Contact {
    pub ptr: *ffi::box2d_Contact,
}

pub struct BodyDef {
    /// The body type: static, kinematic, or dynamic.
    /// Note: if a dynamic body would have zero mass, the mass is set to one.
    pub body_type: BodyType,

    /// The world position of the body. Avoid creating bodies at the origin
    /// since this can lead to many overlapping shapes.
    pub position: Vec2,

    /// The world angle of the body in radians.
    pub angle: f32,

    /// The linear velocity of the body's origin in world co-ordinates.
    pub linear_velocity: Vec2,

    /// The angular velocity of the body.
    pub angular_velocity: f32,

    /// Linear damping is use to reduce the linear velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    pub linear_damping: f32,

    /// Angular damping is use to reduce the angular velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    pub angular_damping: f32,

    /// Set this flag to false if this body should never fall asleep. Note that
    /// this increases CPU usage.
    pub allow_sleep: bool,

    /// Is this body initially awake or sleeping?
    pub awake: bool,

    /// Should this body be prevented from rotating? Useful for characters.
    pub fixed_rotation: bool,

    /// Is this a fast moving body that should be prevented from tunneling through
    /// other moving bodies? Note that all bodies are prevented from tunneling through
    /// kinematic and static bodies. This setting is only considered on dynamic bodies.
    /// @warning You should use this flag sparingly since it increases processing time.
    pub bullet: bool,

    /// Does this body start out active?
    pub active: bool,

    /// Use this to store application specific body data.
    pub user_data: *ffi::box2d_UserData,

    /// Scale the gravity applied to this body.
    pub gravity_scale : f32,
}

impl BodyDef {
    pub fn default() -> BodyDef {
        unsafe {
            return BodyDef {
                user_data: cast::transmute(0),
                position: Vec2 {x: 0.0, y:0.0},
                angle: 0.0,
                linear_velocity: Vec2 {x:0.0, y:0.0},
                angular_velocity: 0.0,
                linear_damping: 0.0,
                angular_damping: 0.0,
                allow_sleep: true,
                awake: true,
                fixed_rotation: false,
                bullet: false,
                body_type: STATIC_BODY,
                active: true,
                gravity_scale: 1.0,
            };
        }
    }
}

pub struct JointDef {
    /// The joint type is set automatically for concrete joint types.
    pub joint_type: JointType,

    /// Use this to attach application specific data to your joints.
    pub user_data: common::UserData,

    /// The first attached body.
    pub body_a: *ffi::box2d_Body,

    /// The second attached body.
    pub body_b: *ffi::box2d_Body,

    /// Set this flag to true if the attached bodies should collide.
    pub collide_connected: bool,
}

pub struct Filter {
    /// The collision category bits. Normally you would just set one bit.
    pub category_bits: u16,

    /// The collision mask bits. This states the categories that this
    /// shape would accept for collision.
    pub mask_bits: i16,

    /// Collision groups allow a certain group of objects to never collide (negative)
    /// or always collide (positive). Zero means no collision group. Non-zero group
    /// filtering always wins against the mask bits.
    pub group_index: i16,
}

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
pub struct FixtureDef<'l> {
    /// The shape, this must be set. The shape will be cloned, so you
    /// can create the shape on the stack.
    pub shape: &'l ffi::box2d_Shape,

    /// Use this to store application specific fixture data.
    pub userData: *ffi::box2d_UserData,

    /// The friction coefficient, usually in the range [0,1].
    pub friction: f32,

    /// The restitution (elasticity) usually in the range [0,1].
    pub restitution: f32,

    /// The density, usually in kg/m^2.
    pub density: f32,

    /// A sensor shape collects contact information but never generates a collision
    /// response.
    pub is_sensor: bool,

    /// Contact filtering data.
    pub filter: Filter,
}

impl<'l> FixtureDef<'l> {
    pub fn default<'l>(s: &'l shapes::Shape) -> FixtureDef<'l> {
        unsafe {
            return FixtureDef {
                shape: cast::transmute(s.handle()),
                userData: cast::transmute(s.handle()), // TODO placeholder
                friction: 0.0,
                restitution: 0.0,
                density: 0.0,
                is_sensor: false,
                filter: Filter {
                    category_bits: 0x0001,
                    mask_bits: 0xFFFF,
                    group_index: 0,
                },
            };
        }
    }
}

impl Drop for World {
    fn drop(&mut self) {
        unsafe {
            ffi::box2d_World_Destroy(self.ptr);
        }
    }
}

impl World {
    pub fn new(gravity: Vec2) -> World {
        unsafe {
            return World { ptr: ffi::box2d_World_Create(&gravity) };
        }
    }

    pub fn set_destruction_listener(&self, listener: *ffi::box2d_DestructionListener) {
        unsafe {
            ffi::box2d_World_SetDestructionListener(self.ptr, listener);
        }
    }
    pub fn set_contact_filter(&mut self, filter: *ffi::box2d_ContactFilter) {
        unsafe {
            ffi::box2d_World_SetContactFilter(self.ptr, filter);
        }
    }
    pub fn set_contact_listener(&mut self, listener: *ffi::box2d_ContactListener) {
        unsafe {
            ffi::box2d_World_SetContactListener(self.ptr, listener);
        }
    }
    pub fn set_debug_draw(&mut self, debug_draw: *ffi::box2d_Draw) {
        unsafe {
            ffi::box2d_World_SetDebugDraw(self.ptr, debug_draw);
        }
    }
    pub fn create_body(&mut self, def: &BodyDef) -> Body {
        unsafe {
            return Body { ptr: ffi::box2d_World_CreateBody(self.ptr, cast::transmute(def)) };
        }
    }
    pub fn create_joint(&mut self, def: &JointDef) -> Joint {
        unsafe {
            return Joint { ptr: ffi::box2d_World_CreateJoint(self.ptr, cast::transmute(def)) };
        }
    }
    pub fn destroy_joint(&mut self, joint: &mut Joint) {
        unsafe {
            ffi::box2d_World_DestroyJoint(self.ptr, joint.ptr);
        }
    }
    pub fn destroy_body(&mut self, body: &mut Body) {
        unsafe {
            ffi::box2d_World_DestroyBody(self.ptr, body.ptr);
        }
    }
    pub fn step(&mut self, time_step: f32, velocity_iterations: i32, position_iterations: i32) {
        unsafe {
            ffi::box2d_World_Step(self.ptr, time_step, velocity_iterations, position_iterations);
        }
    }
    pub fn clear_forces(&mut self) {
        unsafe {
            ffi::box2d_World_ClearForces(self.ptr);
        }
    }
    pub fn draw_debug_data(&self) {
        unsafe {
            ffi::box2d_World_DrawDebugData(self.ptr);
        }
    }
    pub fn query_aabb(&self, cb: *ffi::box2d_QueryCallback, aabb: *ffi::box2d_AABB) {
        unsafe {
            ffi::box2d_World_QueryAABB(self.ptr, cb, aabb);
        }
    }
    pub fn ray_cast(&self, cb: *ffi::box2d_RayCastCallback, p1: Vec2, p2: Vec2) {
        unsafe {
            ffi::box2d_World_RayCast(self.ptr, cb, &p1, &p2);
        }
    }
    pub fn get_body_list(&self) -> Body {
        unsafe {
            return Body{ptr:ffi::box2d_World_GetBodyList(self.ptr)};
        }
    }
    pub fn get_joint_list(&self) -> Joint {
        unsafe {
            return Joint {ptr:ffi::box2d_World_GetJointList(self.ptr)};
        }
    }
    pub fn get_contact_list(&self) -> Contact {
        unsafe {
            return Contact { ptr:ffi::box2d_World_GetContactList(self.ptr) };
        }
    }
    pub fn set_allow_sleeping(&mut self, flag: bool) {
        unsafe {
            ffi::box2d_World_SetAllowSleeping(self.ptr, if flag {1} else {0});
        }
    }
    pub fn get_allow_sleeping(&self) -> bool {
        unsafe {
            return ffi::box2d_World_GetAllowSleeping(self.ptr) != 0;
        }
    }
    pub fn set_warm_starting(&mut self, flag: bool) {
        unsafe {
            ffi::box2d_World_SetWarmStarting(self.ptr, if flag {1} else {0});
        }
    }
    pub fn get_warm_starting(&self) -> bool {
        unsafe {
            return ffi::box2d_World_GetWarmStarting(self.ptr) != 0;
        }
    }
    pub fn set_continuous_Physics(&mut self, flag: bool) {
        unsafe {
            ffi::box2d_World_SetContinuousPhysics(self.ptr, if flag {1} else {0});
        }
    }
    pub fn get_continuous_Physics(&self) -> bool {
        unsafe {
            return ffi::box2d_World_GetContinuousPhysics(self.ptr) != 0;
        }
    }
    pub fn set_sub_stepping(&mut self, flag: bool) {
        unsafe {
            ffi::box2d_World_SetSubStepping(self.ptr, if flag {1} else {0});
        }
    }
    pub fn get_sub_stepping(&self) -> bool {
        unsafe {
            return ffi::box2d_World_GetSubStepping(self.ptr) != 0;
        }
    }
    pub fn get_proxy_count(&self) -> i32 {
        unsafe {
            return ffi::box2d_World_GetProxyCount(self.ptr);
        }
    }
    pub fn get_body_count(&self) -> i32 {
        unsafe {
            return ffi::box2d_World_GetBodyCount(self.ptr);
        }
    }
    pub fn get_joint_count(&self) -> i32 {
        unsafe {
            return ffi::box2d_World_GetJointCount(self.ptr);
        }
    }
    pub fn get_contact_count(&self) -> i32 {
        unsafe {
            return ffi::box2d_World_GetContactCount(self.ptr);
        }
    }
    pub fn get_tree_height(&self) -> i32 {
        unsafe {
            return ffi::box2d_World_GetTreeHeight(self.ptr);
        }
    }
    pub fn get_tree_balance(&self) -> i32 {
        unsafe {
            return ffi::box2d_World_GetTreeBalance(self.ptr);
        }
    }
    pub fn get_tree_quality(&self) -> f32 {
        unsafe {
            return ffi::box2d_World_GetTreeQuality(self.ptr);
        }
    }
    pub fn set_gravity(&mut self, gravity: Vec2) {
        unsafe {
            ffi::box2d_World_SetGravity(self.ptr, &gravity);
        }
    }
    pub fn get_gravity(&self) -> Vec2 {
        unsafe {
            return ffi::box2d_World_GetGravity(self.ptr);
        }
    }
    pub fn is_locked(&self) -> bool {
        unsafe {
            return ffi::box2d_World_IsLocked(self.ptr) != 0;
        }
    }
    pub fn set_auto_clear_forces(&mut self, flag: bool) {
        unsafe {
            ffi::box2d_World_SetAutoClearForces(self.ptr, if flag {1} else {0});
        }
    }
    pub fn get_auto_clear_forces(&self) -> bool {
        unsafe {
            return ffi::box2d_World_GetAutoClearForces(self.ptr) != 0;
        }
    }
    pub fn get_contact_manager(&self) -> *ffi::box2d_ContactManager {
        unsafe {
            return ffi::box2d_World_GetContactManager(self.ptr);
        }
    }
    pub fn get_profile(&self) -> *ffi::box2d_Profile {
        unsafe {
            return ffi::box2d_World_GetProfile(self.ptr);
        }
    }
    pub fn dump(&self) {
        unsafe {
            ffi::box2d_World_Dump(self.ptr);
        }
    }
} // World

impl Body {
    pub fn create_fixture(&self, def: &FixtureDef) -> Fixture {
        unsafe {
            return Fixture { ptr: ffi::box2d_Body_CreateFixture(self.ptr, cast::transmute(def)) };
        }
    }
    pub fn create_fixture_with_shape(&self, shape: &ffi::box2d_Shape, density: f32) -> Fixture {
        unsafe {
            return Fixture { ptr: ffi::box2d_Body_CreateFixture_shape(self.ptr, cast::transmute(shape), density) };
        }
    }
    pub fn destroy_fixture(&self, fixture: Fixture) {
        unsafe {
            ffi::box2d_Body_DestroyFixture(self.ptr, fixture.ptr);
        }
    }
    pub fn set_transform(&self, pos: Vec2, angle: f32) {
        unsafe {
            ffi::box2d_Body_SetTransform(self.ptr, cast::transmute(&pos), angle);
        }
    }
    pub fn get_transform(&self) -> *ffi::box2d_Transform {
        unsafe {
            return ffi::box2d_Body_GetTransform(self.ptr);
        }
    }
    pub fn get_position(&self) -> Vec2 {
        unsafe {
            return *ffi::box2d_Body_GetPosition(self.ptr);
        }
    }
    pub fn get_angle(&self) -> f32 {
        unsafe {
            return ffi::box2d_Body_GetAngle(self.ptr);
        }
    }
    pub fn get_world_center(&self) -> Vec2 {
        unsafe {
            return *ffi::box2d_Body_GetWorldCenter(self.ptr);
        }
    }
    pub fn get_local_center(&self) -> Vec2 {
        unsafe {
            return *ffi::box2d_Body_GetLocalCenter(self.ptr);
        }
    }
    pub fn set_linear_velocity(&self, v: Vec2) {
        unsafe {
            ffi::box2d_Body_SetLinearVelocity(self.ptr, cast::transmute(&v));
        }
    }
    pub fn get_linear_velocity(&self) -> Vec2 {
        unsafe {
            return ffi::box2d_Body_GetLinearVelocity(self.ptr);
        }
    }
    pub fn set_angular_velocity(&self, omega: f32) {
        unsafe {
            ffi::box2d_Body_SetAngularVelocity(self.ptr, omega);
        }
    }
    pub fn get_angular_velocity(&self) -> f32 {
        unsafe {
            return ffi::box2d_Body_GetAngularVelocity(self.ptr);
        }
    }
    pub fn apply_force(&self, force: Vec2, point: Vec2) {
        unsafe {
            ffi::box2d_Body_ApplyForce(self.ptr, cast::transmute(&force), cast::transmute(&point));
        }
    }
    pub fn apply_force_to_center(&self, force: Vec2) {
        unsafe {
            ffi::box2d_Body_ApplyForceToCenter(self.ptr, cast::transmute(&force));
        }
    }
    pub fn apply_torque(&self, torque: f32) {
        unsafe {
            ffi::box2d_Body_ApplyTorque(self.ptr, torque);
        }
    }
    pub fn apply_linear_impulse(&self, impulse: Vec2, p: Vec2) {
        unsafe {
            ffi::box2d_Body_ApplyLinearImpulse(self.ptr, cast::transmute(&impulse), cast::transmute(&p));
        }
    }
    pub fn apply_angular_impulse(&self, impulse: f32) {
        unsafe {
            ffi::box2d_Body_ApplyAngularImpulse(self.ptr, impulse);
        }
    }
    pub fn get_mass(&self) -> f32 {
        unsafe {
            return ffi::box2d_Body_GetMass(self.ptr);
        }
    }
    pub fn get_inertia(&self) -> f32 {
        unsafe {
            return ffi::box2d_Body_GetInertia(self.ptr);
        }
    }
    pub fn get_mass_data(&self, data: *ffi::box2d_MassData) {
        unsafe {
            ffi::box2d_Body_GetMassData(self.ptr, data);
        }
    }
    pub fn set_mass_data(&self, data: *ffi::box2d_MassData) {
        unsafe {
            ffi::box2d_Body_SetMassData(self.ptr, data);
        }
    }
    pub fn reset_mass_data(&self) {
        unsafe {
            ffi::box2d_Body_ResetMassData(self.ptr);
        }
    }
    pub fn get_world_point(&self, p: Vec2) -> Vec2 {
        unsafe {
            return ffi::box2d_Body_GetWorldPoint(self.ptr, cast::transmute(&p));
        }
    }
    pub fn get_world_vector(&self, v: Vec2) -> Vec2 {
        unsafe {
            return ffi::box2d_Body_GetWorldVector(self.ptr, cast::transmute(&v));
        }
    }
    pub fn get_local_point(&self, p: Vec2) -> Vec2 {
        unsafe {
            return ffi::box2d_Body_GetLocalPoint(self.ptr, cast::transmute(&p));
        }
    }
    pub fn get_local_vector(&self, v: Vec2) -> Vec2 {
        unsafe {
            return ffi::box2d_Body_GetLocalVector(self.ptr, cast::transmute(&v));
        }
    }
    pub fn get_linear_velocity_from_world_point(&self, p: Vec2) -> Vec2 {
        unsafe {
            return ffi::box2d_Body_GetLinearVelocityFromWorldPoint(self.ptr, cast::transmute(&p));
        }
    }
    pub fn get_linear_velocity_from_local_point(&self, p: Vec2) -> Vec2 {
        unsafe {
            return ffi::box2d_Body_GetLinearVelocityFromLocalPoint(self.ptr, cast::transmute(&p));
        }
    }
    pub fn get_linear_damping(&self) -> f32 {
        unsafe {
            return ffi::box2d_Body_GetLinearDamping(self.ptr);
        }
    }
    pub fn set_linear_damping(&self, linear_damping: f32) {
        unsafe {
            ffi::box2d_Body_SetLinearDamping(self.ptr, linear_damping);
        }
    }
    pub fn get_angular_damping(&self) -> f32 {
        unsafe {
            return ffi::box2d_Body_GetAngularDamping(self.ptr);
        }
    }
    pub fn set_angular_damping(&self, angular_damping: f32) {
        unsafe {
            ffi::box2d_Body_SetAngularDamping(self.ptr, angular_damping);
        }
    }
    pub fn get_gravity_scale(&self) -> f32 {
        unsafe {
            return ffi::box2d_Body_GetGravityScale(self.ptr);
        }
    }
    pub fn set_gravity_scale(&self, scale: f32) {
        unsafe {
            ffi::box2d_Body_SetGravityScale(self.ptr, scale);
        }
    }
    pub fn set_type(&self, body_type: BodyType) {
        unsafe {
            ffi::box2d_Body_SetType(self.ptr, body_type);
        }
    }
    pub fn get_type(&self) -> BodyType {
        unsafe {
            return ffi::box2d_Body_GetType(self.ptr);
        }
    }
    pub fn set_bullet(&self, flag: bool) {
        unsafe {
            ffi::box2d_Body_SetBullet(self.ptr, ffi::box2d_bool(flag));
        }
    }
    pub fn is_bullet(&self) -> bool {
        unsafe {
            return ffi::box2d_Body_IsBullet(self.ptr) != 0;
        }
    }
    pub fn set_sleeping_allowed(&self, flag: bool) {
        unsafe {
            ffi::box2d_Body_SetSleepingAllowed(self.ptr, ffi::box2d_bool(flag));
        }
    }
    pub fn is_sleeping_allowed(&self) -> bool {
        unsafe {
            return ffi::box2d_Body_IsSleepingAllowed(self.ptr) != 0;
        }
    }
    pub fn set_awake(&self, flag: bool) {
        unsafe {
            ffi::box2d_Body_SetAwake(self.ptr, ffi::box2d_bool(flag));
        }
    }
    pub fn is_awake(&self) -> bool {
        unsafe {
            return ffi::box2d_Body_IsAwake(self.ptr) != 0;
        }
    }
    pub fn set_active(&self, flag: bool) {
        unsafe {
            ffi::box2d_Body_SetActive(self.ptr, ffi::box2d_bool(flag));
        }
    }
    pub fn is_active(&self) -> bool {
        unsafe {
            return ffi::box2d_Body_IsActive(self.ptr) != 0;
        }
    }
    pub fn set_fixed_rotation(&self, flag: bool) {
        unsafe {
            ffi::box2d_Body_SetFixedRotation(self.ptr, ffi::box2d_bool(flag));
        }
    }
    pub fn is_fixed_rotation(&self) -> bool {
        unsafe {
            return ffi::box2d_Body_IsFixedRotation(self.ptr) != 0;
        }
    }
    pub fn get_fixture_list(&self) -> Fixture {
        unsafe {
            return Fixture { ptr: ffi::box2d_Body_GetFixtureList(self.ptr) };
        }
    }
    pub fn get_joint_list(&self) -> *ffi::box2d_JointEdge {
        unsafe {
            return ffi::box2d_Body_GetJointList(self.ptr);
        }
    }
    pub fn get_contact_list(&self) -> *ffi::box2d_ContactEdge {
        unsafe {
            return ffi::box2d_Body_GetContactList(self.ptr);
        }
    }
    pub fn get_next(&self) -> Body {
        unsafe {
            return Body { ptr: ffi::box2d_Body_GetNext(self.ptr) };
        }
    }
    pub fn get_user_data(&self) {
        unsafe {
            return ffi::box2d_Body_GetUserData(self.ptr);
        }
    }
    pub fn set_user_data(&self, data: *ffi::box2d_UserData) {
        unsafe {
            ffi::box2d_Body_SetUserData(self.ptr, data);
        }
    }
    pub fn get_world(&self) -> World {
        unsafe {
            return World { ptr:ffi::box2d_Body_GetWorld(self.ptr) };
        }
    }
    pub fn dump(&self) {
        unsafe {
            ffi::box2d_Body_Dump(self.ptr);
        }
    }
} // World

impl Fixture {
    pub fn get_type(&self) -> shapes::ShapeType {
        unsafe {
            return ffi::box2d_Fixture_GetType(self.ptr) as shapes::ShapeType;
        }
    }
    pub fn get_shape(&self) -> *ffi::box2d_Shape {
        unsafe {
            return ffi::box2d_Fixture_GetShape(self.ptr);
        }
    }
    pub fn set_sensor(&self, sensor: bool) {
        unsafe {
            ffi::box2d_Fixture_SetSensor(self.ptr, ffi::box2d_bool(sensor));
        }
    }
    pub fn is_sensor(&self) -> bool {
        unsafe {
            return ffi::box2d_Fixture_IsSensor(self.ptr) != 0;
        }
    }
    pub fn set_filter_data(&self, filter: *ffi::box2d_Filter) {
        unsafe {
            ffi::box2d_Fixture_SetFilterData(self.ptr, filter);
        }
    }
    pub fn get_filter_data(&self) -> *ffi::box2d_Filter {
        unsafe {
            return ffi::box2d_Fixture_GetFilterData(self.ptr);
        }
    }
    pub fn refilter(&self) {
        unsafe {
            ffi::box2d_Fixture_Refilter(self.ptr);
        }
    }
    pub fn get_body(&self) -> Body {
        unsafe {
            return Body { ptr: ffi::box2d_Fixture_GetBody(self.ptr) };
        }
    }
    pub fn get_next(&self) -> Fixture {
        unsafe {
            return Fixture { ptr: ffi::box2d_Fixture_GetNext(self.ptr) };
        }
    }
    pub fn get_user_data(&self) -> *ffi::box2d_UserData {
        unsafe {
            return ffi::box2d_Fixture_GetUserData(self.ptr);
        }
    }
    pub fn set_user_data(&self, data: *ffi::box2d_UserData) {
        unsafe {
            ffi::box2d_Fixture_SetUserData(self.ptr, data);
        }
    }
    pub fn test_point(&self,  p: Vec2) -> bool {
        unsafe {
            return ffi::box2d_Fixture_TestPoint(self.ptr, cast::transmute(&p)) != 0;
        }
    }
    pub fn ray_cast(&self, output: *ffi::box2d_RayCastOutput, input: *ffi::box2d_RayCastInput, child_index: i32) -> bool {
        unsafe {
            return ffi::box2d_Fixture_RayCast(self.ptr, output, input, child_index) != 0;
        }
    }
    pub fn get_mass_data(&self, data: &ffi::box2d_MassData) {
        unsafe {
            return ffi::box2d_Fixture_GetMassData(self.ptr, cast::transmute(data));
        }
    }
    pub fn set_density(&self, density: f32) {
        unsafe {
            ffi::box2d_Fixture_SetDensity(self.ptr, density);
        }
    }
    pub fn get_density(&self) -> f32 {
        unsafe {
            return ffi::box2d_Fixture_GetDensity(self.ptr);
        }
    }
    pub fn get_friction(&self) -> f32 {
        unsafe {
            return ffi::box2d_Fixture_GetFriction(self.ptr);
        }
    }
    pub fn set_friction(&self, friction: f32) {
        unsafe {
            ffi::box2d_Fixture_SetFriction(self.ptr, friction);
        }
    }
    pub fn get_restitution(&self) -> f32 {
        unsafe {
            return ffi::box2d_Fixture_GetRestitution(self.ptr);
        }
    }
    pub fn set_restitution(&self, restitution: f32) {
        unsafe {
            ffi::box2d_Fixture_SetRestitution(self.ptr, restitution);
        }
    }
    pub fn get_aabb(&self, child_index: i32) -> *ffi::box2d_AABB {
        unsafe {
            return ffi::box2d_Fixture_GetAABB(self.ptr, child_index);
        }
    }
    pub fn dump(&self, body_index: i32) {
        unsafe {
            ffi::box2d_Fixture_Dump(self.ptr, body_index);
        }
    }
} // Fixture
