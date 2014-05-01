
#ifndef C_BOX2D_BODY
#define C_BOX2D_BODY

#include "Box2D/Common/c_box2d_settings.h"
#include "Box2D/Dynamics/Joints/c_box2d_joint.h"
#include "Box2D/Dynamics/c_box2d_fixture.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum box2d_BodyType {
    box2d_staticBody = 0,
    box2d_kinematicBody,
    box2d_dynamicBody
} box2d_BodyType;

typedef struct box2d_BodyDef {
    /// The body type: static, kinematic, or dynamic.
    /// Note: if a dynamic body would have zero mass, the mass is set to one.
    box2d_BodyType type;

    /// The world position of the body. Avoid creating bodies at the origin
    /// since this can lead to many overlapping shapes.
    box2d_Vec2 position;

    /// The world angle of the body in radians.
    float32 angle;

    /// The linear velocity of the body's origin in world co-ordinates.
    box2d_Vec2 linearVelocity;

    /// The angular velocity of the body.
    float32 angularVelocity;

    /// Linear damping is use to reduce the linear velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    float32 linearDamping;

    /// Angular damping is use to reduce the angular velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    float32 angularDamping;

    /// Set this flag to false if this body should never fall asleep. Note that
    /// this increases CPU usage.
    box2d_bool allowSleep;

    /// Is this body initially awake or sleeping?
    box2d_bool awake;

    /// Should this body be prevented from rotating? Useful for characters.
    box2d_bool fixedRotation;

    /// Is this a fast moving body that should be prevented from tunneling through
    /// other moving bodies? Note that all bodies are prevented from tunneling through
    /// kinematic and static bodies. This setting is only considered on dynamic bodies.
    /// @warning You should use this flag sparingly since it increases processing time.
    box2d_bool bullet;

    /// Does this body start out active?
    box2d_bool active;

    /// Use this to store application specific body data.
    void* userData;

    /// Scale the gravity applied to this body.
    float32 gravityScale;
} box2d_BodyDef;

/// This constructor sets the body definition default values.
box2d_BodyDef box2d_BodyDef_Create();

/// Creates a fixture and attach it to this body. Use this function if you need
/// to set some fixture parameters, like friction. Otherwise you can create the
/// fixture directly from a shape.
/// If the density is non-zero, this function automatically updates the mass of the body.
/// Contacts are not created until the next time step.
/// @param def the fixture definition.
/// @warning This function is locked during callbacks.
box2d_Fixture* box2d_Body_CreateFixture(box2d_Body* self, const box2d_FixtureDef* def);

/// Creates a fixture from a shape and attach it to this body.
/// This is a convenience function. Use box2d_FixtureDef if you need to set parameters
/// like friction, restitution, user data, or filtering.
/// If the density is non-zero, this function automatically updates the mass of the body.
/// @param shape the shape to be cloned.
/// @param density the shape density (set to zero for static bodies).
/// @warning This function is locked during callbacks.
box2d_Fixture* box2d_Body_CreateFixture_shape(box2d_Body* self, const box2d_Shape* shape, float32 density);

/// Destroy a fixture. This removes the fixture from the broad-phase and
/// destroys all contacts associated with this fixture. This will
/// automatically adjust the mass of the body if the body is dynamic and the
/// fixture has positive density.
/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
/// @param fixture the fixture to be removed.
/// @warning This function is locked during callbacks.
void box2d_Body_DestroyFixture(box2d_Body* self, box2d_Fixture* fixture);

/// Set the position of the body's origin and rotation.
/// Manipulating a body's transform may cause non-physical behavior.
/// Note: contacts are updated on the next call to box2d_World::Step.
/// @param position the world position of the body's local origin.
/// @param angle the world rotation in radians.
void box2d_Body_SetTransform(box2d_Body* self, const box2d_Vec2* position, float32 angle);

/// Get the body transform for the body's origin.
/// @return the world transform of the body's origin.
const box2d_Transform* box2d_Body_GetTransform(const box2d_Body* self);

/// Get the world body origin position.
/// @return the world position of the body's origin.
const box2d_Vec2* box2d_Body_GetPosition(const box2d_Body* self);

/// Get the angle in radians.
/// @return the current world rotation angle in radians.
float32 box2d_Body_GetAngle(const box2d_Body* self);

/// Get the world position of the center of mass.
const box2d_Vec2* box2d_Body_GetWorldCenter(const box2d_Body* self);

/// Get the local position of the center of mass.
const box2d_Vec2* box2d_Body_GetLocalCenter(const box2d_Body* self);

/// Set the linear velocity of the center of mass.
/// @param v the new linear velocity of the center of mass.
void box2d_Body_SetLinearVelocity(box2d_Body* self, const box2d_Vec2* v);

/// Get the linear velocity of the center of mass.
/// @return the linear velocity of the center of mass.
const box2d_Vec2 box2d_Body_GetLinearVelocity(const box2d_Body* self);

/// Set the angular velocity.
/// @param omega the new angular velocity in radians/second.
void box2d_Body_SetAngularVelocity(box2d_Body* self, float32 omega);

/// Get the angular velocity.
/// @return the angular velocity in radians/second.
float32 box2d_Body_GetAngularVelocity(const box2d_Body* self);

/// Apply a force at a world point. If the force is not
/// applied at the center of mass, it will generate a torque and
/// affect the angular velocity. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param point the world position of the point of application.
/// @param wake also wake up the body
void box2d_Body_ApplyForce(box2d_Body* self,
                           const box2d_Vec2* force,
                           const box2d_Vec2* point
                           , box2d_bool wake);

/// Apply a force to the center of mass. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param wake also wake up the body
void box2d_Body_ApplyForceToCenter(box2d_Body* self, const box2d_Vec2* force, box2d_bool wake);

/// Apply a torque. This affects the angular velocity
/// without affecting the linear velocity of the center of mass.
/// This wakes up the body.
/// @param torque about the z-axis (out of the screen), usually in N-m.
/// @param wake also wake up the body
void box2d_Body_ApplyTorque(box2d_Body* self, float32 torque, box2d_bool wake);

/// Apply an impulse at a point. This immediately modifies the velocity.
/// It also modifies the angular velocity if the point of application
/// is not at the center of mass. This wakes up the body.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param point the world position of the point of application.
/// @param wake also wake up the body
void box2d_Body_ApplyLinearImpulse(box2d_Body* self, const box2d_Vec2* impulse, const box2d_Vec2* point, box2d_bool wake);

/// Apply an angular impulse.
/// @param impulse the angular impulse in units of kg*m*m/s
/// @param wake also wake up the body
void box2d_Body_ApplyAngularImpulse(box2d_Body* self, float32 impulse, box2d_bool wake);

/// Get the total mass of the body.
/// @return the mass, usually in kilograms (kg).
float32 box2d_Body_GetMass(const box2d_Body* self);

/// Get the rotational inertia of the body about the local origin.
/// @return the rotational inertia, usually in kg-m^2.
float32 box2d_Body_GetInertia(const box2d_Body* self);

/// Get the mass data of the body.
/// @return a struct containing the mass, inertia and center of the body.
void box2d_Body_GetMassData(const box2d_Body* self, box2d_MassData* data);

/// Set the mass properties to override the mass properties of the fixtures.
/// Note that this changes the center of mass position.
/// Note that creating or destroying fixtures can also alter the mass.
/// This function has no effect if the body isn't dynamic.
/// @param massData the mass properties.
void box2d_Body_SetMassData(box2d_Body* self, const box2d_MassData* data);

/// This resets the mass properties to the sum of the mass properties of the fixtures.
/// This normally does not need to be called unless you called SetMassData to override
/// the mass and you later want to reset the mass.
void box2d_Body_ResetMassData(box2d_Body* self);

/// Get the world coordinates of a point given the local coordinates.
/// @param localPoint a point on the body measured relative the the body's origin.
/// @return the same point expressed in world coordinates.
box2d_Vec2 box2d_Body_GetWorldPoint(const box2d_Body* self, const box2d_Vec2* localPoint);

/// Get the world coordinates of a vector given the local coordinates.
/// @param localVector a vector fixed in the body.
/// @return the same vector expressed in world coordinates.
box2d_Vec2 box2d_Body_GetWorldVector(const box2d_Body* self, const box2d_Vec2* localVector);

/// Gets a local point relative to the body's origin given a world point.
/// @param a point in world coordinates.
/// @return the corresponding local point relative to the body's origin.
box2d_Vec2 box2d_Body_GetLocalPoint(const box2d_Body* self, const box2d_Vec2* worldPoint);

/// Gets a local vector given a world vector.
/// @param a vector in world coordinates.
/// @return the corresponding local vector.
box2d_Vec2 box2d_Body_GetLocalVector(const box2d_Body* self, const box2d_Vec2* worldVector);

/// Get the world linear velocity of a world point attached to this body.
/// @param a point in world coordinates.
/// @return the world velocity of a point.
box2d_Vec2 box2d_Body_GetLinearVelocityFromWorldPoint(const box2d_Body* self, const box2d_Vec2* worldPoint);

/// Get the world velocity of a local point.
/// @param a point in local coordinates.
/// @return the world velocity of a point.
box2d_Vec2 box2d_Body_GetLinearVelocityFromLocalPoint(const box2d_Body* self, const box2d_Vec2* localPoint);

/// Get the linear damping of the body.
float32 box2d_Body_GetLinearDamping(const box2d_Body* self);

/// Set the linear damping of the body.
void box2d_Body_SetLinearDamping(box2d_Body* self, float32 linearDamping);

/// Get the angular damping of the body.
float32 box2d_Body_GetAngularDamping(const box2d_Body* self);

/// Set the angular damping of the body.
void box2d_Body_SetAngularDamping(box2d_Body* self, float32 angularDamping);

/// Get the gravity scale of the body.
float32 box2d_Body_GetGravityScale(const box2d_Body* self);

/// Set the gravity scale of the body.
void box2d_Body_SetGravityScale(box2d_Body* self, float32 scale);

/// Set the type of this body. This may alter the mass and velocity.
void box2d_Body_SetType(box2d_Body* self, int type);

/// Get the type of this body.
box2d_BodyType box2d_Body_GetType(const box2d_Body* self);

/// Should this body be treated like a bullet for continuous collision detection?
void box2d_Body_SetBullet(box2d_Body* self, box2d_bool flag);

/// Is this body treated like a bullet for continuous collision detection?
box2d_bool box2d_Body_IsBullet(const box2d_Body* self);

/// You can disable sleeping on this body. If you disable sleeping, the
/// body will be woken.
void box2d_Body_SetSleepingAllowed(box2d_Body* self, box2d_bool flag);

/// Is this body allowed to sleep
box2d_bool box2d_Body_IsSleepingAllowed(const box2d_Body* self);

/// Set the sleep state of the body. A sleeping body has very
/// low CPU cost.
/// @param flag set to true to wake the body, false to put it to sleep.
void box2d_Body_SetAwake(box2d_Body* self, box2d_bool flag);

/// Get the sleeping state of this body.
/// @return true if the body is awake.
box2d_bool box2d_Body_IsAwake(const box2d_Body* self);

/// Set the active state of the body. An inactive body is not
/// simulated and cannot be collided with or woken up.
/// If you pass a flag of true, all fixtures will be added to the
/// broad-phase.
/// If you pass a flag of false, all fixtures will be removed from
/// the broad-phase and all contacts will be destroyed.
/// Fixtures and joints are otherwise unaffected. You may continue
/// to create/destroy fixtures and joints on inactive bodies.
/// Fixtures on an inactive body are implicitly inactive and will
/// not participate in collisions, ray-casts, or queries.
/// Joints connected to an inactive body are implicitly inactive.
/// An inactive body is still owned by a box2d_World object and remains
/// in the body list.
void box2d_Body_SetActive(box2d_Body* self, box2d_bool flag);

/// Get the active state of the body.
box2d_bool box2d_Body_IsActive(const box2d_Body* self);

/// Set this body to have fixed rotation. This causes the mass
/// to be reset.
void box2d_Body_SetFixedRotation(box2d_Body* self, box2d_bool flag);

/// Does this body have fixed rotation?
box2d_bool box2d_Body_IsFixedRotation(const box2d_Body* self);

/// Get the list of all fixtures attached to this body.
box2d_Fixture* box2d_Body_GetFixtureList(box2d_Body* self);
const box2d_Fixture* box2d_Body_GetFixtureList_const(const box2d_Body* self);

/// Get the list of all joints attached to this body.
box2d_JointEdge* box2d_Body_GetJointList(box2d_Body* self);
const box2d_JointEdge* box2d_Body_GetJointList_const(const box2d_Body* self);

/// Get the list of all contacts attached to this body.
/// @warning this list changes during the time step and you may
/// miss some collisions if you don't use box2d_ContactListener.
box2d_ContactEdge* box2d_Body_GetContactList(box2d_Body* self);
const box2d_ContactEdge* box2d_Body_GetContactList_const(const box2d_Body* self);

/// Get the next body in the world's body list.
box2d_Body* box2d_Body_GetNext(box2d_Body* self);
const box2d_Body* box2d_Body_GetNext_const(const box2d_Body* self);

/// Get the user data pointer that was provided in the body definition.
void* box2d_Body_GetUserData(const box2d_Body* self);

/// Set the user data. Use this to store your application specific data.
void box2d_Body_SetUserData(box2d_Body* self, void* data);

/// Get the parent world of this body.
box2d_World* box2d_Body_GetWorld(box2d_Body* self);
const box2d_World* box2d_Body_GetWorld_const(const box2d_Body* self);

/// Dump this body to a log file
void box2d_Body_Dump(box2d_Body* self);

#ifdef __cplusplus
} // extern C
#endif

#endif
