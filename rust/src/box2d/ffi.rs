#[link(name="cbox2d", kind="static")]

use std::cast;

static PI: f32 = 3.14159265359;
static MAX_MANIFOLD_POINTS: i32 = 2;
static MAX_POLYGON_VERTICES: i32 = 8;
static AABB_EXTENSION: f32 = 0.1;
static AABB_MULTIPLIER: f32 = 2.0;
static LINEAR_STOP: f32 = 0.005;
static ANGULAR_STOP: f32 = (2.0 / 180.0 * PI);
static POLYGON_RADIUS: f32 = (2.0 * LINEAR_STOP);
static MAX_SUBSTEPS: i32 = 8;

#[deriving(Eq, Ord)]
struct Vec2 {
    x: f32,
    y: f32,
}

#[deriving(Eq, Ord)]
struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}

type box2d_bool = u8;
fn box2d_bool(b: bool) -> box2d_bool { if b {1} else {0} }

struct box2d_Filter;
struct box2d_PolygonShape;
struct box2d_BodyDef;
struct box2d_FixtureDef;
struct box2d_Joint;
struct box2d_JointEdge;
struct box2d_World;
struct box2d_Shape;
struct box2d_Body;
struct box2d_Fixture;
struct box2d_Transform;
struct box2d_RayCastInput;
struct box2d_RayCastOutput;
struct box2d_AABB;
struct box2d_MassData;
struct box2d_ContactEdge;
struct box2d_ContactListener;
struct box2d_ContactFilter;
struct box2d_DestructionListener;
struct box2d_Contact;
struct box2d_ContactManager;
struct box2d_Profile;
struct box2d_Draw;
struct box2d_RayCastCallback;
struct box2d_QueryCallback;

struct box2d_UserData;
struct box2d_BodyType;

type BodyType = i32;
static STATIC_BODY: BodyType    = 0 as BodyType;
static KINEMATIC_BODY: BodyType = 1 as BodyType;
static DYNAMIC_BODY: BodyType   = 2 as BodyType;

type JointType = i32;
static UNKNOWN_JOINT: JointType      = 0 as JointType;
static REVOLUTE_JOINT: JointType     = 1 as JointType;
static PROSMATIC_JOINT: JointType    = 2 as JointType;
static DISTANCE_JOINT: JointType     = 3 as JointType;
static PULLEY_JOINT: JointType       = 4 as JointType;
static MOUSE_JOINT: JointType        = 5 as JointType;
static GEAR_JOINT: JointType         = 6 as JointType;
static WHEEL_JOINT: JointType        = 7 as JointType;
static WELD_JOINT: JointType         = 8 as JointType;
static FRICTION_JOINT: JointType     = 9 as JointType;
static ROPE_JOINT: JointType         = 10 as JointType;

type ShapeType = i32;
static CIRCLE_SHAPE: ShapeType  = 0 as ShapeType;
static EDGE_SHAPE: ShapeType    = 1 as ShapeType;
static POLYGON_SHAPE: ShapeType = 2 as ShapeType;
static CHAIN_SHAPE: ShapeType   = 3 as ShapeType;


type UserData = *i32;

struct BodyDef {
    /// The body type: static, kinematic, or dynamic.
    /// Note: if a dynamic body would have zero mass, the mass is set to one.
    body_type: BodyType,

    /// The world position of the body. Avoid creating bodies at the origin
    /// since this can lead to many overlapping shapes.
    position: Vec2,

    /// The world angle of the body in radians.
    angle: f32,

    /// The linear velocity of the body's origin in world co-ordinates.
    linear_velocity: Vec2,

    /// The angular velocity of the body.
    angular_velocity: f32,

    /// Linear damping is use to reduce the linear velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    linear_damping: f32,

    /// Angular damping is use to reduce the angular velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    angular_damping: f32,

    /// Set this flag to false if this body should never fall asleep. Note that
    /// this increases CPU usage.
    allow_sleep: bool,

    /// Is this body initially awake or sleeping?
    awake: bool,

    /// Should this body be prevented from rotating? Useful for characters.
    fixed_rotation: bool,

    /// Is this a fast moving body that should be prevented from tunneling through
    /// other moving bodies? Note that all bodies are prevented from tunneling through
    /// kinematic and static bodies. This setting is only considered on dynamic bodies.
    /// @warning You should use this flag sparingly since it increases processing time.
    bullet: bool,

    /// Does this body start out active?
    active: bool,

    /// Use this to store application specific body data.
    user_data: Option<UserData>,

    /// Scale the gravity applied to this body.
    gravity_scale : f32,
}

impl BodyDef {
    fn default() -> BodyDef {
        return BodyDef {
            user_data: None,
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

struct JointDef {
    /// The joint type is set automatically for concrete joint types.
    joint_type: JointType,

    /// Use this to attach application specific data to your joints.
    user_data: *box2d_UserData,

    /// The first attached body.
    body_a: *box2d_Body,

    /// The second attached body.
    body_b: *box2d_Body,

    /// Set this flag to true if the attached bodies should collide.
    collide_connected: bool,
}

struct Filter {
    /// The collision category bits. Normally you would just set one bit.
    category_bits: u16,

    /// The collision mask bits. This states the categories that this
    /// shape would accept for collision.
    mask_bits: i16,

    /// Collision groups allow a certain group of objects to never collide (negative)
    /// or always collide (positive). Zero means no collision group. Non-zero group
    /// filtering always wins against the mask bits.
    group_index: i16,
}

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
struct FixtureDef<'l> {
    /// The shape, this must be set. The shape will be cloned, so you
    /// can create the shape on the stack.
    shape: &'l box2d_Shape,

    /// Use this to store application specific fixture data.
    userData: Option<UserData>,

    /// The friction coefficient, usually in the range [0,1].
    friction: f32,

    /// The restitution (elasticity) usually in the range [0,1].
    restitution: f32,

    /// The density, usually in kg/m^2.
    density: f32,

    /// A sensor shape collects contact information but never generates a collision
    /// response.
    is_sensor: bool,

    /// Contact filtering data.
    filter: Filter,
}

impl<'l> FixtureDef<'l> {
    fn default<'l>(s: &'l box2d_Shape) -> FixtureDef<'l> {
        return FixtureDef {
            shape: s,
            userData: None,
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

struct PolygonShape {
    centroid: Vec2,
    vertices: [Vec2, ..MAX_POLYGON_VERTICES],
    normals: [Vec2, ..MAX_POLYGON_VERTICES],
    vertex_count: i32,
}

impl PolygonShape {
    fn default() -> PolygonShape {
        return PolygonShape {
            centroid: Vec2 {x:0.0, y:0.0},
            vertices: [
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
            ],
            normals: [
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
            ],
            vertex_count: 0,
        }
    }
    fn box_shape(w:f32, h:f32) -> PolygonShape {
        let result = PolygonShape::default();
        unsafe {
            box2d_PolygonShape_SetAsBox(cast::transmute(&result), w, h);
        }
        return result;
    }
}

extern {
    // b2World
    fn box2d_World_Create(gravity: *Vec2) -> *box2d_World;
    fn box2d_World_Destroy(world: *box2d_World);
    fn box2d_World_SetDestructionListener(this: *box2d_World, listener: *box2d_DestructionListener);
    fn box2d_World_SetContactFilter(this: *box2d_World, filter: *box2d_ContactFilter);
    fn box2d_World_SetContactListener(this: *box2d_World, listener: *box2d_ContactListener);
    fn box2d_World_SetDebugDraw(this: *box2d_World, debugdraw: *box2d_Draw);
    fn box2d_World_CreateBody(this: *box2d_World, def: *box2d_BodyDef) -> *box2d_Body;
    fn box2d_World_CreateJoint(this: *box2d_World, def: *JointDef) -> *box2d_Joint;
    fn box2d_World_DestroyJoint(this: *box2d_World, joint: *box2d_Joint);
    fn box2d_World_DestroyBody(this: *box2d_World, body: *box2d_Body);
    fn box2d_World_Step(this: *box2d_World, timeStep: f32, velocityIterations: i32, positionIterations: i32);
    fn box2d_World_ClearForces(this: *box2d_World);
    fn box2d_World_DrawDebugData(this: *box2d_World);
    fn box2d_World_QueryAABB(this: *box2d_World, cb: *box2d_QueryCallback, aabb: *box2d_AABB);
    fn box2d_World_RayCast(this: *box2d_World, cb: *box2d_RayCastCallback, p1: *Vec2, p2: *Vec2);
    fn box2d_World_GetBodyList(this: *box2d_World) -> *box2d_Body;
    fn box2d_World_GetJointList(this: *box2d_World) -> *box2d_Joint;
    fn box2d_World_GetContactList(this: *box2d_World) -> *box2d_Contact;
    fn box2d_World_SetAllowSleeping(this: *box2d_World, flag: box2d_bool);
    fn box2d_World_GetAllowSleeping(this: *box2d_World) -> box2d_bool;
    fn box2d_World_SetWarmStarting(this: *box2d_World, flag: box2d_bool);
    fn box2d_World_GetWarmStarting(this: *box2d_World) -> box2d_bool;
    fn box2d_World_SetContinuousPhysics(this: *box2d_World, flag: box2d_bool);
    fn box2d_World_GetContinuousPhysics(this: *box2d_World) -> box2d_bool;
    fn box2d_World_SetSubStepping(this: *box2d_World, flag: box2d_bool);
    fn box2d_World_GetSubStepping(this: *box2d_World) -> box2d_bool;
    fn box2d_World_GetProxyCount(this: *box2d_World) -> i32;
    fn box2d_World_GetBodyCount(this: *box2d_World) -> i32;
    fn box2d_World_GetJointCount(this: *box2d_World) -> i32;
    fn box2d_World_GetContactCount(this: *box2d_World) -> i32;
    fn box2d_World_GetTreeHeight(this: *box2d_World) -> i32;
    fn box2d_World_GetTreeBalance(this: *box2d_World) -> i32;
    fn box2d_World_GetTreeQuality(this: *box2d_World) -> f32;
    fn box2d_World_SetGravity(this: *box2d_World, gravity: *Vec2);
    fn box2d_World_GetGravity(this: *box2d_World) -> Vec2;
    fn box2d_World_IsLocked(this: *box2d_World) -> box2d_bool;
    fn box2d_World_SetAutoClearForces(this: *box2d_World, flag: box2d_bool);
    fn box2d_World_GetAutoClearForces(this: *box2d_World) -> box2d_bool;
    fn box2d_World_GetContactManager(this: *box2d_World) -> *box2d_ContactManager;
    fn box2d_World_GetProfile(this: *box2d_World) -> *box2d_Profile;
    fn box2d_World_Dump(this: *box2d_World);
    // b2Body
    fn box2d_Body_CreateFixture(this: *box2d_Body, def: *box2d_FixtureDef) -> *box2d_Fixture;
    fn box2d_Body_CreateFixture_shape(this: *box2d_Body, shape: *box2d_Shape, density: f32) -> *box2d_Fixture;
    fn box2d_Body_DestroyFixture(this: *box2d_Body, fixture: *box2d_Fixture);
    fn box2d_Body_SetTransform(this: *box2d_Body, position: *Vec2, angle: f32);
    fn box2d_Body_GetTransform(this: *box2d_Body) -> *box2d_Transform;
    fn box2d_Body_GetPosition(this: *box2d_Body) -> *Vec2;
    fn box2d_Body_GetAngle(this: *box2d_Body) -> f32;
    fn box2d_Body_GetWorldCenter(this: *box2d_Body) -> *Vec2;
    fn box2d_Body_GetLocalCenter(this: *box2d_Body) -> *Vec2;
    fn box2d_Body_SetLinearVelocity(this: *box2d_Body, v: *Vec2);
    fn box2d_Body_GetLinearVelocity(this: *box2d_Body) -> Vec2;
    fn box2d_Body_SetAngularVelocity(this: *box2d_Body, omega: f32);
    fn box2d_Body_GetAngularVelocity(this: *box2d_Body) -> f32;
    fn box2d_Body_ApplyForce(this: *box2d_Body, forece: *Vec2, point: *Vec2);
    fn box2d_Body_ApplyForceToCenter(this: *box2d_Body, force: *Vec2);
    fn box2d_Body_ApplyTorque(this: *box2d_Body, torque: f32);
    fn box2d_Body_ApplyLinearImpulse(this: *box2d_Body, impulse: *Vec2, point: *Vec2);
    fn box2d_Body_ApplyAngularImpulse(this: *box2d_Body, impulse: f32);
    fn box2d_Body_GetMass(this: *box2d_Body) -> f32;
    fn box2d_Body_GetInertia(this: *box2d_Body) -> f32;
    fn box2d_Body_GetMassData(this: *box2d_Body, data: *box2d_MassData);
    fn box2d_Body_SetMassData(this: *box2d_Body, data: *box2d_MassData);
    fn box2d_Body_ResetMassData(this: *box2d_Body);
    fn box2d_Body_GetWorldPoint(this: *box2d_Body, localPoint: *Vec2) -> Vec2;
    fn box2d_Body_GetWorldVector(this: *box2d_Body, localVector: *Vec2) -> Vec2;
    fn box2d_Body_GetLocalPoint(this: *box2d_Body, worldPoint: *Vec2) -> Vec2;
    fn box2d_Body_GetLocalVector(this: *box2d_Body, worldVector: *Vec2) -> Vec2;
    fn box2d_Body_GetLinearVelocityFromWorldPoint(this: *box2d_Body, worldPoint: *Vec2) -> Vec2;
    fn box2d_Body_GetLinearVelocityFromLocalPoint(this: *box2d_Body, localPoint: *Vec2) -> Vec2;
    fn box2d_Body_GetLinearDamping(this: *box2d_Body) -> f32;
    fn box2d_Body_SetLinearDamping(this: *box2d_Body, linearDamping: f32);
    fn box2d_Body_GetAngularDamping(this: *box2d_Body) -> f32;
    fn box2d_Body_SetAngularDamping(this: *box2d_Body, angularDamping: f32);
    fn box2d_Body_GetGravityScale(this: *box2d_Body) -> f32;
    fn box2d_Body_SetGravityScale(this: *box2d_Body, scale: f32);
    fn box2d_Body_SetType(this: *box2d_Body, bodyType: BodyType);
    fn box2d_Body_GetType(this: *box2d_Body) -> BodyType;
    fn box2d_Body_SetBullet(this: *box2d_Body, flag: box2d_bool);
    fn box2d_Body_IsBullet(this: *box2d_Body) -> box2d_bool;
    fn box2d_Body_SetSleepingAllowed(this: *box2d_Body, flag: box2d_bool);
    fn box2d_Body_IsSleepingAllowed(this: *box2d_Body) -> box2d_bool;
    fn box2d_Body_SetAwake(this: *box2d_Body, flag: box2d_bool);
    fn box2d_Body_IsAwake(this: *box2d_Body) -> box2d_bool;
    fn box2d_Body_SetActive(this: *box2d_Body, flag: box2d_bool);
    fn box2d_Body_IsActive(this: *box2d_Body) -> box2d_bool;
    fn box2d_Body_SetFixedRotation(this: *box2d_Body, flag: box2d_bool);
    fn box2d_Body_IsFixedRotation(this: *box2d_Body) -> box2d_bool;
    fn box2d_Body_GetFixtureList(this: *box2d_Body) -> *box2d_Fixture;
    fn box2d_Body_GetJointList(this: *box2d_Body) -> *box2d_JointEdge;
    fn box2d_Body_GetContactList(this: *box2d_Body) -> *box2d_ContactEdge;
    fn box2d_Body_GetNext(this: *box2d_Body) -> *box2d_Body;
    fn box2d_Body_GetUserData(this: *box2d_Body);
    fn box2d_Body_SetUserData(this: *box2d_Body, data: *box2d_UserData);
    fn box2d_Body_GetWorld(this: *box2d_Body) -> *box2d_World;
    fn box2d_Body_Dump(this: *box2d_Body);
    // b2Shape
    fn box2d_Shape_GetType(this: *box2d_Shape) -> int; // TODO int
    fn box2d_Shape_GetChildCount(this: *box2d_Shape) -> i32;
    fn box2d_Shape_TestPoint(this: *box2d_Shape,  xf: *box2d_Transform, p: *Vec2) -> box2d_bool;
    fn box2d_Shape_RayCast(this: *box2d_Shape, output: *box2d_RayCastOutput,  input: *box2d_RayCastInput, transform: *box2d_Transform, childIndex: i32) -> box2d_bool;
    fn box2d_Shape_ComputeAABB(this: *box2d_Shape, aabb: *box2d_AABB, xf: *box2d_Transform, childIndex: i32);
    fn box2d_Shape_ComputeMass(this: *box2d_Shape, massData: *box2d_MassData, density: f32);
    // b2PolygonShape
    fn box2d_PolygonShape_Create() -> box2d_PolygonShape;
    fn box2d_PolygonShape_Upcast(s: *box2d_PolygonShape) -> *box2d_Shape;
    fn box2d_PolygonShape_GetChildCount(this: *box2d_PolygonShape) -> i32;
    fn box2d_PolygonShape_Set(this: *box2d_PolygonShape,  points: *Vec2, count: i32);
    fn box2d_PolygonShape_SetAsBox(this: *box2d_PolygonShape, hx: f32, hy: f32);
    fn box2d_PolygonShape_SetAsBox_2(this: *box2d_PolygonShape, hx: f32, hy: f32, center: *Vec2, angle: f32);
    fn box2d_PolygonShape_GetVertexCount(this: *box2d_PolygonShape) -> i32;
    fn box2d_PolygonShape_GetVertex(this: *box2d_PolygonShape, index: i32) -> *Vec2;
    // b2Fixture
    fn box2d_FixtureDef_create() -> box2d_FixtureDef;
    fn box2d_Fixture_GetType(this: *box2d_Fixture) -> ShapeType;
    fn box2d_Fixture_GetShape(this: *box2d_Fixture) -> *box2d_Shape;
    fn box2d_Fixture_SetSensor(this: *box2d_Fixture, sensor: box2d_bool);
    fn box2d_Fixture_IsSensor(this: *box2d_Fixture) -> box2d_bool;
    fn box2d_Fixture_SetFilterData(this: *box2d_Fixture, filter: *box2d_Filter);
    fn box2d_Fixture_GetFilterData(this: *box2d_Fixture) -> *box2d_Filter;
    fn box2d_Fixture_Refilter(this: *box2d_Fixture);
    fn box2d_Fixture_GetBody(this: *box2d_Fixture) -> *box2d_Body;
    fn box2d_Fixture_GetNext(this: *box2d_Fixture) -> *box2d_Fixture;
    fn box2d_Fixture_GetNext_(this: *box2d_Fixture) -> *box2d_Fixture;
    fn box2d_Fixture_GetUserData(this: *box2d_Fixture) -> *box2d_UserData;
    fn box2d_Fixture_SetUserData(this: *box2d_Fixture, data: *box2d_UserData);
    fn box2d_Fixture_TestPoint(this: *box2d_Fixture,  p: *Vec2) -> box2d_bool;
    fn box2d_Fixture_RayCast(this: *box2d_Fixture, output: *box2d_RayCastOutput, input: *box2d_RayCastInput, childIndex: i32) -> box2d_bool;
    fn box2d_Fixture_GetMassData(this: *box2d_Fixture, massData: *box2d_MassData);
    fn box2d_Fixture_SetDensity(this: *box2d_Fixture, density: f32);
    fn box2d_Fixture_GetDensity(this: *box2d_Fixture) -> f32;
    fn box2d_Fixture_GetFriction(this: *box2d_Fixture) -> f32;
    fn box2d_Fixture_SetFriction(this: *box2d_Fixture, friction: f32);
    fn box2d_Fixture_GetRestitution(this: *box2d_Fixture) -> f32;
    fn box2d_Fixture_SetRestitution(this: *box2d_Fixture, restitution: f32);
    fn box2d_Fixture_GetAABB(this: *box2d_Fixture, childIndex: i32) -> *box2d_AABB;
    fn box2d_Fixture_Dump(this: *box2d_Fixture, bodyIndex: i32);
    // b2Joint
    fn box2d_Joint_GetType(this: *box2d_Joint) -> i32;
    fn box2d_Joint_GetBodyA(this: *box2d_Joint) -> *box2d_Body;
    fn box2d_Joint_GetBodyB(this: *box2d_Joint) -> *box2d_Body;
    fn box2d_Joint_GetAnchorA(this: *box2d_Joint) -> Vec2;
    fn box2d_Joint_GetAnchorB(this: *box2d_Joint) -> Vec2;
    fn box2d_Joint_GetReactionForce(this: *box2d_Joint, inv_dt: f32) -> Vec2;
    fn box2d_Joint_GetReactionTorque(this: *box2d_Joint, inv_dt: f32) -> f32;
    fn box2d_Joint_GetNext(this: *box2d_Joint) -> *box2d_Joint;
    fn box2d_Joint_GetUserData(this: *box2d_Joint) -> *box2d_UserData;
    fn box2d_Joint_SetUserData(this: *box2d_Joint, data: box2d_UserData);
    fn box2d_Joint_IsActive(this: *box2d_Joint) -> box2d_bool;
    fn box2d_Joint_GetCollideConnected(this: *box2d_Joint) -> box2d_bool;
    fn box2d_Joint_Dump(this: *box2d_Joint);
}


struct World {
    ptr: *box2d_World,
}

struct Joint {
    ptr: *box2d_Joint,
}

struct Body {
    ptr: *box2d_Body,
}

struct Fixture {
    ptr: *box2d_Fixture,
}

struct Contact {
    ptr: *box2d_Contact,
}

impl Drop for World {
    fn drop(&mut self) {
        unsafe {
            box2d_World_Destroy(self.ptr);
        }
    }
}

impl World {
    fn new(gravity: Vec2) -> World {
        unsafe {
            let w: *box2d_World = box2d_World_Create(&gravity);
            return World { ptr:w };
        }
    }

    fn set_destruction_listener(&self, listener: *box2d_DestructionListener) {
        unsafe {
            box2d_World_SetDestructionListener(self.ptr, listener);
        }
    }
    fn set_contact_filter(&mut self, filter: *box2d_ContactFilter) {
        unsafe {
            box2d_World_SetContactFilter(self.ptr, filter);
        }
    }
    fn set_contact_listener(&mut self, listener: *box2d_ContactListener) {
        unsafe {
            box2d_World_SetContactListener(self.ptr, listener);
        }
    }
    fn set_debug_draw(&mut self, debug_draw: *box2d_Draw) {
        unsafe {
            box2d_World_SetDebugDraw(self.ptr, debug_draw);
        }
    }
    fn create_body(&mut self, def: &BodyDef) -> Body {
        unsafe {
            return Body { ptr: box2d_World_CreateBody(self.ptr, cast::transmute(def)) };
        }
    }
    fn create_joint(&mut self, def: &JointDef) -> Joint {
        unsafe {
            return Joint { ptr: box2d_World_CreateJoint(self.ptr, cast::transmute(def)) };
        }
    }
    fn destroy_joint(&mut self, joint: &mut Joint) {
        unsafe {
            box2d_World_DestroyJoint(self.ptr, joint.ptr);
        }
    }
    fn destroy_body(&mut self, body: &mut Body) {
        unsafe {
            box2d_World_DestroyBody(self.ptr, body.ptr);
        }
    }
    fn step(&mut self, time_step: f32, velocity_iterations: i32, position_iterations: i32) {
        unsafe {
            box2d_World_Step(self.ptr, time_step, velocity_iterations, position_iterations);
        }
    }
    fn clear_forces(&mut self) {
        unsafe {
            box2d_World_ClearForces(self.ptr);
        }
    }
    fn draw_debug_data(&self) {
        unsafe {
            box2d_World_DrawDebugData(self.ptr);
        }
    }
    fn query_aabb(&self, cb: *box2d_QueryCallback, aabb: *box2d_AABB) {
        unsafe {
            box2d_World_QueryAABB(self.ptr, cb, aabb);
        }
    }
    fn ray_cast(&self, cb: *box2d_RayCastCallback, p1: Vec2, p2: Vec2) {
        unsafe {
            box2d_World_RayCast(self.ptr, cb, &p1, &p2);
        }
    }
    fn get_body_list(&self) -> Body {
        unsafe {
            return Body{ptr:box2d_World_GetBodyList(self.ptr)};
        }
    }
    fn get_joint_list(&self) -> Joint {
        unsafe {
            return Joint {ptr:box2d_World_GetJointList(self.ptr)};
        }
    }
    fn get_contact_list(&self) -> Contact {
        unsafe {
            return Contact { ptr:box2d_World_GetContactList(self.ptr) };
        }
    }
    fn set_allow_sleeping(&mut self, flag: bool) {
        unsafe {
            box2d_World_SetAllowSleeping(self.ptr, if flag {1} else {0});
        }
    }
    fn get_allow_sleeping(&self) -> bool {
        unsafe {
            return box2d_World_GetAllowSleeping(self.ptr) != 0;
        }
    }
    fn set_warm_starting(&mut self, flag: bool) {
        unsafe {
            box2d_World_SetWarmStarting(self.ptr, if flag {1} else {0});
        }
    }
    fn get_warm_starting(&self) -> bool {
        unsafe {
            return box2d_World_GetWarmStarting(self.ptr) != 0;
        }
    }
    fn set_continuous_Physics(&mut self, flag: bool) {
        unsafe {
            box2d_World_SetContinuousPhysics(self.ptr, if flag {1} else {0});
        }
    }
    fn get_continuous_Physics(&self) -> bool {
        unsafe {
            return box2d_World_GetContinuousPhysics(self.ptr) != 0;
        }
    }
    fn set_sub_stepping(&mut self, flag: bool) {
        unsafe {
            box2d_World_SetSubStepping(self.ptr, if flag {1} else {0});
        }
    }
    fn get_sub_stepping(&self) -> bool {
        unsafe {
            return box2d_World_GetSubStepping(self.ptr) != 0;
        }
    }
    fn get_proxy_count(&self) -> i32 {
        unsafe {
            return box2d_World_GetProxyCount(self.ptr);
        }
    }
    fn get_body_count(&self) -> i32 {
        unsafe {
            return box2d_World_GetBodyCount(self.ptr);
        }
    }
    fn get_joint_count(&self) -> i32 {
        unsafe {
            return box2d_World_GetJointCount(self.ptr);
        }
    }
    fn get_contact_count(&self) -> i32 {
        unsafe {
            return box2d_World_GetContactCount(self.ptr);
        }
    }
    fn get_tree_height(&self) -> i32 {
        unsafe {
            return box2d_World_GetTreeHeight(self.ptr);
        }
    }
    fn get_tree_balance(&self) -> i32 {
        unsafe {
            return box2d_World_GetTreeBalance(self.ptr);
        }
    }
    fn get_tree_quality(&self) -> f32 {
        unsafe {
            return box2d_World_GetTreeQuality(self.ptr);
        }
    }
    fn set_gravity(&mut self, gravity: Vec2) {
        unsafe {
            box2d_World_SetGravity(self.ptr, &gravity);
        }
    }
    fn get_gravity(&self) -> Vec2 {
        unsafe {
            return box2d_World_GetGravity(self.ptr);
        }
    }
    fn is_locked(&self) -> bool {
        unsafe {
            return box2d_World_IsLocked(self.ptr) != 0;
        }
    }
    fn set_auto_clear_forces(&mut self, flag: bool) {
        unsafe {
            box2d_World_SetAutoClearForces(self.ptr, if flag {1} else {0});
        }
    }
    fn get_auto_clear_forces(&self) -> bool {
        unsafe {
            return box2d_World_GetAutoClearForces(self.ptr) != 0;
        }
    }
    fn get_contact_manager(&self) -> *box2d_ContactManager {
        unsafe {
            return box2d_World_GetContactManager(self.ptr);
        }
    }
    fn get_profile(&self) -> *box2d_Profile {
        unsafe {
            return box2d_World_GetProfile(self.ptr);
        }
    }
    fn dump(&self) {
        unsafe {
            box2d_World_Dump(self.ptr);
        }
    }
}

impl Fixture {
    fn get_type(&self) -> ShapeType {
        unsafe {
            return box2d_Fixture_GetType(self.ptr) as ShapeType;
        }
    }
    fn get_shape(&self) -> *box2d_Shape {
        unsafe {
            return box2d_Fixture_GetShape(self.ptr);
        }
    }
    fn set_sensor(&self, sensor: bool) {
        unsafe {
            box2d_Fixture_SetSensor(self.ptr, box2d_bool(sensor));
        }
    }
    fn is_sensor(&self) -> bool {
        unsafe {
            return box2d_Fixture_IsSensor(self.ptr) != 0;
        }
    }
    fn set_filter_data(&self, filter: *box2d_Filter) {
        unsafe {
            box2d_Fixture_SetFilterData(self.ptr, filter);
        }
    }
    fn get_filter_data(&self) -> *box2d_Filter {
        unsafe {
            return box2d_Fixture_GetFilterData(self.ptr);
        }
    }
    fn refilter(&self) {
        unsafe {
            box2d_Fixture_Refilter(self.ptr);
        }
    }
    fn get_body(&self) -> Body {
        unsafe {
            return Body { ptr: box2d_Fixture_GetBody(self.ptr) };
        }
    }
    fn get_next(&self) -> Fixture {
        unsafe {
            return Fixture { ptr: box2d_Fixture_GetNext(self.ptr) };
        }
    }
    fn get_user_data(&self) -> *box2d_UserData {
        unsafe {
            return box2d_Fixture_GetUserData(self.ptr);
        }
    }
    fn set_user_data(&self, data: *box2d_UserData) {
        unsafe {
            box2d_Fixture_SetUserData(self.ptr, data);
        }
    }
    fn test_point(&self,  p: Vec2) -> bool {
        unsafe {
            return box2d_Fixture_TestPoint(self.ptr, cast::transmute(&p)) != 0;
        }
    }
    fn ray_cast(&self, output: *box2d_RayCastOutput, input: *box2d_RayCastInput, child_index: i32) -> bool {
        unsafe {
            return box2d_Fixture_RayCast(self.ptr, output, input, child_index) != 0;
        }
    }
    fn get_mass_data(&self, data: &box2d_MassData) {
        unsafe {
            return box2d_Fixture_GetMassData(self.ptr, cast::transmute(data));
        }
    }
    fn set_density(&self, density: f32) {
        unsafe {
            box2d_Fixture_SetDensity(self.ptr, density);
        }
    }
    fn get_density(&self) -> f32 {
        unsafe {
            return box2d_Fixture_GetDensity(self.ptr);
        }
    }
    fn get_friction(&self) -> f32 {
        unsafe {
            return box2d_Fixture_GetFriction(self.ptr);
        }
    }
    fn set_friction(&self, friction: f32) {
        unsafe {
            box2d_Fixture_SetFriction(self.ptr, friction);
        }
    }
    fn get_restitution(&self) -> f32 {
        unsafe {
            return box2d_Fixture_GetRestitution(self.ptr);
        }
    }
    fn set_restitution(&self, restitution: f32) {
        unsafe {
            box2d_Fixture_SetRestitution(self.ptr, restitution);
        }
    }
    fn get_aabb(&self, child_index: i32) -> *box2d_AABB {
        unsafe {
            return box2d_Fixture_GetAABB(self.ptr, child_index);
        }
    }
    fn dump(&self, body_index: i32) {
        unsafe {
            box2d_Fixture_Dump(self.ptr, body_index);
        }
    }
}

fn main () {
    println("Rust box2d test...");
    let gravity = Vec2 { x:0.0, y:-1.0 };
    let mut world = World::new(gravity);

    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    let time_step = 1.0f32 / 60.0f32;
    let velocity_iterations = 6;
    let position_iterations = 2;

    assert_eq!(world.get_body_count(), 0);
    assert_eq!(world.get_gravity(), gravity);

    let mut body_def = BodyDef::default();
    let mut b1 = world.create_body(&body_def);

    let shape = PolygonShape::box_shape(2.0, 3.0);

    assert_eq!(world.get_body_count(), 1);

    for i in range(0, 60) {
        println(" -- step");
        world.step(time_step, velocity_iterations, position_iterations);
    }

    world.destroy_body(&mut b1);
    assert_eq!(world.get_body_count(), 0);
    // ...
    // RAII takes car of destroying the world
}