
extern "C" {

/// Creates a fixture and attach it to this body. Use this function if you need
/// to set some fixture parameters, like friction. Otherwise you can create the
/// fixture directly from a shape.
/// If the density is non-zero, this function automatically updates the mass of the body.
/// Contacts are not created until the next time step.
/// @param def the fixture definition.
/// @warning This function is locked during callbacks.
b2Fixture* box2d_Body_CreateFixture(b2Body* self, const b2FixtureDef* def) {
    return self->CreateFixture(def);
}

/// Creates a fixture from a shape and attach it to this body.
/// This is a convenience function. Use b2FixtureDef if you need to set parameters
/// like friction, restitution, user data, or filtering.
/// If the density is non-zero, this function automatically updates the mass of the body.
/// @param shape the shape to be cloned.
/// @param density the shape density (set to zero for static bodies).
/// @warning This function is locked during callbacks.
b2Fixture* box2d_Body_CreateFixture_shape(b2Body* self, const b2Shape* shape, float32 density) {
    return self->CreateFixture(shape, density);
}

/// Destroy a fixture. This removes the fixture from the broad-phase and
/// destroys all contacts associated with this fixture. This will
/// automatically adjust the mass of the body if the body is dynamic and the
/// fixture has positive density.
/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
/// @param fixture the fixture to be removed.
/// @warning This function is locked during callbacks.
void box2d_Body_DestroyFixture(b2Body* self, b2Fixture* fixture) {
    self->DestroyFixture(fixture);
}

/// Set the position of the body's origin and rotation.
/// Manipulating a body's transform may cause non-physical behavior.
/// Note: contacts are updated on the next call to b2World::Step.
/// @param position the world position of the body's local origin.
/// @param angle the world rotation in radians.
void box2d_Body_SetTransform(b2Body* self, const b2Vec2* position, float32 angle) {
    self->SetTransform(*position, angle);
}

/// Get the body transform for the body's origin.
/// @return the world transform of the body's origin.
const b2Transform* box2d_Body_GetTransform(const b2Body* self) {
    return &self->GetTransform();
}

/// Get the world body origin position.
/// @return the world position of the body's origin.
const b2Vec2* box2d_Body_GetPosition(const b2Body* self) {
    return &self->GetPosition();
}

/// Get the angle in radians.
/// @return the current world rotation angle in radians.
float32 box2d_Body_GetAngle(const b2Body* self) {
    return self->GetAngle();
}

/// Get the world position of the center of mass.
const b2Vec2* box2d_Body_GetWorldCenter(const b2Body* self) {
    return &self->GetWorldCenter();
}

/// Get the local position of the center of mass.
const b2Vec2* box2d_Body_GetLocalCenter(const b2Body* self) {
    return &self->GetLocalCenter();
}

/// Set the linear velocity of the center of mass.
/// @param v the new linear velocity of the center of mass.
void box2d_Body_SetLinearVelocity(b2Body* self, const b2Vec2* v) {
    self->SetLinearVelocity(*v);
}

/// Get the linear velocity of the center of mass.
/// @return the linear velocity of the center of mass.
const b2Vec2 box2d_Body_GetLinearVelocity(const b2Body* self) {
    return self->GetLinearVelocity();
}

/// Set the angular velocity.
/// @param omega the new angular velocity in radians/second.
void box2d_Body_SetAngularVelocity(b2Body* self, float32 omega) {
    self->SetAngularVelocity(omega);
}

/// Get the angular velocity.
/// @return the angular velocity in radians/second.
float32 box2d_Body_GetAngularVelocity(const b2Body* self) {
    return self->GetAngularVelocity();
}

/// Apply a force at a world point. If the force is not
/// applied at the center of mass, it will generate a torque and
/// affect the angular velocity. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param point the world position of the point of application.
/// @param wake also wake up the body
void box2d_Body_ApplyForce(b2Body* self, const b2Vec2* force, const b2Vec2* point, bool wake) {
    self->ApplyForce(*force, *point/*, wake*/);
}

/// Apply a force to the center of mass. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param wake also wake up the body
void box2d_Body_ApplyForceToCenter(b2Body* self, const b2Vec2* force, bool wake) {
    self->ApplyForceToCenter(*force/*, wake*/);
}

/// Apply a torque. This affects the angular velocity
/// without affecting the linear velocity of the center of mass.
/// This wakes up the body.
/// @param torque about the z-axis (out of the screen), usually in N-m.
/// @param wake also wake up the body
void box2d_Body_ApplyTorque(b2Body* self, float32 torque, bool wake) {
    self->ApplyTorque(torque/*, wake*/);
}

/// Apply an impulse at a point. This immediately modifies the velocity.
/// It also modifies the angular velocity if the point of application
/// is not at the center of mass. This wakes up the body.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param point the world position of the point of application.
/// @param wake also wake up the body
void box2d_Body_ApplyLinearImpulse(b2Body* self, const b2Vec2* impulse, const b2Vec2* point, bool wake) {
    self->ApplyLinearImpulse(*impulse, *point/*, wake*/);
}

/// Apply an angular impulse.
/// @param impulse the angular impulse in units of kg*m*m/s
/// @param wake also wake up the body
void box2d_Body_ApplyAngularImpulse(b2Body* self, float32 impulse, bool wake) {
    self->ApplyAngularImpulse(impulse/*, wake*/);
}

/// Get the total mass of the body.
/// @return the mass, usually in kilograms (kg).
float32 box2d_Body_GetMass(const b2Body* self) {
    return self->GetMass();
}

/// Get the rotational inertia of the body about the local origin.
/// @return the rotational inertia, usually in kg-m^2.
float32 box2d_Body_GetInertia(const b2Body* self) {
    return self->GetInertia();
}

/// Get the mass data of the body.
/// @return a struct containing the mass, inertia and center of the body.
void box2d_Body_GetMassData(const b2Body* self, b2MassData* data) {
    self->GetMassData(data);
}

/// Set the mass properties to override the mass properties of the fixtures.
/// Note that this changes the center of mass position.
/// Note that creating or destroying fixtures can also alter the mass.
/// This function has no effect if the body isn't dynamic.
/// @param massData the mass properties.
void box2d_Body_SetMassData(b2Body* self, const b2MassData* data) {
    self->SetMassData(data);
}

/// This resets the mass properties to the sum of the mass properties of the fixtures.
/// This normally does not need to be called unless you called SetMassData to override
/// the mass and you later want to reset the mass.
void box2d_Body_ResetMassData(b2Body* self) {
    self->ResetMassData();
}

/// Get the world coordinates of a point given the local coordinates.
/// @param localPoint a point on the body measured relative the the body's origin.
/// @return the same point expressed in world coordinates.
b2Vec2 box2d_Body_GetWorldPoint(const b2Body* self, const b2Vec2* localPoint) {
    return self->GetWorldPoint(*localPoint);
}

/// Get the world coordinates of a vector given the local coordinates.
/// @param localVector a vector fixed in the body.
/// @return the same vector expressed in world coordinates.
b2Vec2 box2d_Body_GetWorldVector(const b2Body* self, const b2Vec2* localVector) {
    return self->GetWorldVector(*localVector);
}

/// Gets a local point relative to the body's origin given a world point.
/// @param a point in world coordinates.
/// @return the corresponding local point relative to the body's origin.
b2Vec2 box2d_Body_GetLocalPoint(const b2Body* self, const b2Vec2* worldPoint) {
    return self->GetLocalPoint(*worldPoint);
}

/// Gets a local vector given a world vector.
/// @param a vector in world coordinates.
/// @return the corresponding local vector.
b2Vec2 box2d_Body_GetLocalVector(const b2Body* self, const b2Vec2* worldVector) {
    return self->GetLocalVector(*worldVector);
}

/// Get the world linear velocity of a world point attached to this body.
/// @param a point in world coordinates.
/// @return the world velocity of a point.
b2Vec2 box2d_Body_GetLinearVelocityFromWorldPoint(const b2Body* self, const b2Vec2* worldPoint) {
    return self->GetLinearVelocityFromWorldPoint(*worldPoint);
}

/// Get the world velocity of a local point.
/// @param a point in local coordinates.
/// @return the world velocity of a point.
b2Vec2 box2d_Body_GetLinearVelocityFromLocalPoint(const b2Body* self, const b2Vec2* localPoint) {
    return self->GetLinearVelocityFromLocalPoint(*localPoint);
}

/// Get the linear damping of the body.
float32 box2d_Body_GetLinearDamping(const b2Body* self) {
    return self->GetLinearDamping();
}

/// Set the linear damping of the body.
void box2d_Body_SetLinearDamping(b2Body* self, float32 linearDamping) {
    self->SetLinearDamping(linearDamping);
}

/// Get the angular damping of the body.
float32 box2d_Body_GetAngularDamping(const b2Body* self) {
    return self->GetAngularDamping();
}

/// Set the angular damping of the body.
void box2d_Body_SetAngularDamping(b2Body* self, float32 angularDamping) {
    self->SetAngularDamping(angularDamping);
}

/// Get the gravity scale of the body.
float32 box2d_Body_GetGravityScale(const b2Body* self) {
    return self->GetGravityScale();
}

/// Set the gravity scale of the body.
void box2d_Body_SetGravityScale(b2Body* self, float32 scale) {
    self->SetGravityScale(scale);
}

/// Set the type of this body. This may alter the mass and velocity.
void box2d_Body_SetType(b2Body* self, int type) {
    self->SetType(static_cast<b2BodyType>(type));
}

/// Get the type of this body.
int box2d_Body_GetType(const b2Body* self) {
    return self->GetType();
}

/// Should this body be treated like a bullet for continuous collision detection?
void box2d_Body_SetBullet(b2Body* self, bool flag) {
    self->SetBullet(flag);
}

/// Is this body treated like a bullet for continuous collision detection?
bool box2d_Body_IsBullet(const b2Body* self) {
    return self->IsBullet();
}

/// You can disable sleeping on this body. If you disable sleeping, the
/// body will be woken.
void box2d_Body_SetSleepingAllowed(b2Body* self, bool flag) {
    self->SetSleepingAllowed(flag);
}

/// Is this body allowed to sleep
bool box2d_Body_IsSleepingAllowed(const b2Body* self) {
    return self->IsSleepingAllowed();
}

/// Set the sleep state of the body. A sleeping body has very
/// low CPU cost.
/// @param flag set to true to wake the body, false to put it to sleep.
void box2d_Body_SetAwake(b2Body* self, bool flag) {
    self->SetAwake(flag);
}

/// Get the sleeping state of this body.
/// @return true if the body is awake.
bool box2d_Body_IsAwake(const b2Body* self) {
    return self->IsAwake();
}

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
/// An inactive body is still owned by a b2World object and remains
/// in the body list.
void box2d_Body_SetActive(b2Body* self, bool flag) {
    self->SetActive(flag);
}

/// Get the active state of the body.
bool box2d_Body_IsActive(const b2Body* self) {
    return self->IsActive();
}

/// Set this body to have fixed rotation. This causes the mass
/// to be reset.
void box2d_Body_SetFixedRotation(b2Body* self, bool flag) {
    return self->SetFixedRotation(flag);
}

/// Does this body have fixed rotation?
bool box2d_Body_IsFixedRotation(const b2Body* self) {
    return self ->IsFixedRotation();
}

/// Get the list of all fixtures attached to this body.
b2Fixture* box2d_Body_GetFixtureList(b2Body* self) {
    return self->GetFixtureList();
}
const b2Fixture* box2d_Body_GetFixtureList_const(const b2Body* self) {
    return self->GetFixtureList();
}

/// Get the list of all joints attached to this body.
b2JointEdge* box2d_Body_GetJointList(b2Body* self) {
    return self->GetJointList();
}
const b2JointEdge* box2d_Body_GetJointList_const(const b2Body* self) {
    return self->GetJointList();
}

/// Get the list of all contacts attached to this body.
/// @warning this list changes during the time step and you may
/// miss some collisions if you don't use b2ContactListener.
b2ContactEdge* box2d_Body_GetContactList(b2Body* self) {
    return self->GetContactList();
}
const b2ContactEdge* box2d_Body_GetContactList_const(const b2Body* self) {
    return self->GetContactList();
}

/// Get the next body in the world's body list.
b2Body* box2d_Body_GetNext(b2Body* self) {
    return self->GetNext();
}
const b2Body* box2d_Body_GetNext_const(const b2Body* self) {
    return self->GetNext();
}

/// Get the user data pointer that was provided in the body definition.
void* box2d_Body_GetUserData(const b2Body* self) {
    return self->GetUserData();
}

/// Set the user data. Use this to store your application specific data.
void box2d_Body_SetUserData(b2Body* self, void* data) {
    self->SetUserData(data);
}

/// Get the parent world of this body.
b2World* box2d_Body_GetWorld(b2Body* self) {
    return self->GetWorld();
}
const b2World* box2d_Body_GetWorld_const(const b2Body* self) {
    return self->GetWorld();
}

/// Dump this body to a log file
void box2d_Body_Dump(b2Body* self) {
    self->Dump();
}

} // extern C