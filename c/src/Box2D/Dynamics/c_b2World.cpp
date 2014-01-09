extern "C"
{

/// Construct a world object.
/// @param gravity the world gravity vector.
b2World* box2d_World_ctor(const b2Vec2* gravity) {
    return new b2World(*gravity);
}

/// Destruct the world. All physics entities are destroyed and all heap memory is released.
void box2d_World_dtor(b2World* self) {
    delete self;
}

void box2d_World_SetDestructionListener(b2World* self, b2DestructionListener* listener) {
    self->SetDestructionListener(listener);
}

/// Register a contact filter to provide specific control over collision.
/// Otherwise the default filter is used (b2_defaultFilter). The listener is
/// owned by you and must remain in scope. 
void box2d_World_SetContactFilter(b2World* self, b2ContactFilter* filter) {
    self->SetContactFilter(filter);
}

/// Register a contact event listener. The listener is owned by you and must
/// remain in scope.
void box2d_World_SetContactListener(b2World* self, b2ContactListener* listener) {
    self->SetContactListener(listener);
}

/// Register a routine for debug drawing. The debug draw functions are called
/// inside with b2World::DrawDebugData method. The debug draw object is owned
/// by you and must remain in scope.
void box2d_World_SetDebugDraw(b2World* self, b2Draw* debugDraw) {
    self->SetDebugDraw(debugDraw);
}

/// Create a rigid body given a definition. No reference to the definition
/// is retained.
/// @warning This function is locked during callbacks.
b2Body* box2d_World_CreateBody(b2World* self, const b2BodyDef* def) {
    return self->CreateBody(def);
}

/// Destroy a rigid body given a definition. No reference to the definition
/// is retained. This function is locked during callbacks.
/// @warning This automatically deletes all associated shapes and joints.
/// @warning This function is locked during callbacks.
void box2d_World_DestroyBody(b2World* self, b2Body* body) {
    self->DestroyBody(body);
}

/// Create a joint to constrain bodies together. No reference to the definition
/// is retained. This may cause the connected bodies to cease colliding.
/// @warning This function is locked during callbacks.
b2Joint* box2d_World_CreateJoint(b2World* self, const b2JointDef* def) {
    return self->CreateJoint(def);
}

/// Destroy a joint. This may cause the connected bodies to begin colliding.
/// @warning This function is locked during callbacks.
void box2d_World_DestroyJoint(b2World* self, b2Joint* joint) {
    self->DestroyJoint(joint);
}

/// Take a time step. This performs collision detection, integration,
/// and constraint solution.
/// @param timeStep the amount of time to simulate, this should not vary.
/// @param velocityIterations for the velocity constraint solver.
/// @param positionIterations for the position constraint solver.
void box2d_World_Step(b2World* self,
            float32 timeStep,
            int32 velocityIterations,
            int32 positionIterations) {
    self->Step(timeStep, velocityIterations, positionIterations);
}

/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
/// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
/// a fixed sized time step under a variable frame-rate.
/// When you perform sub-stepping you will disable auto clearing of forces and instead call
/// ClearForces after all sub-steps are complete in one pass of your game loop.
/// @see SetAutoClearForces
void box2d_World_ClearForces(b2World* self) {
    self->ClearForces();
}

/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
void box2d_World_DrawDebugData(b2World* self) {
    self->DrawDebugData();
}

/// Query the world for all fixtures that potentially overlap the
/// provided AABB.
/// @param callback a user implemented callback class.
/// @param aabb the query box.
void box2d_World_QueryAABB(const b2World* self, b2QueryCallback* callback, const b2AABB* aabb) {
    self->QueryAABB(callback, *aabb);
}

/// Ray-cast the world for all fixtures in the path of the ray. Your callback
/// controls whether you get the closest point, any point, or n-points.
/// The ray-cast ignores shapes that contain the starting point.
/// @param callback a user implemented callback class.
/// @param point1 the ray starting point
/// @param point2 the ray ending point
void box2d_World_RayCast(const b2World* self, b2RayCastCallback* callback, const b2Vec2* point1, const b2Vec2* point2) {
    self->RayCast(callback, *point1, *point2);
}

/// Get the world body list. With the returned body, use b2Body::GetNext to get
/// the next body in the world list. A NULL body indicates the end of the list.
/// @return the head of the world body list.
b2Body* box2d_World_GetBodyList(b2World* self) {
    return self->GetBodyList();
}
//const b2Body* box2d_World_GetBodyList() const;

/// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
/// the next joint in the world list. A NULL joint indicates the end of the list.
/// @return the head of the world joint list.
b2Joint* box2d_World_GetJointList(b2World* self) {
    return self->GetJointList();
}
//const b2Joint* box2d_World_GetJointList() const;

/// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
/// the next contact in the world list. A NULL contact indicates the end of the list.
/// @return the head of the world contact list.
/// @warning contacts are created and destroyed in the middle of a time step.
/// Use b2ContactListener to avoid missing contacts.
b2Contact* box2d_World_GetContactList(b2World* self) {
    return self->GetContactList();
}
//const b2Contact* box2d_World_GetContactList() const;

/// Enable/disable sleep.
void box2d_World_SetAllowSleeping(b2World* self, bool flag) {
    self->SetAllowSleeping(flag);
}

bool box2d_World_GetAllowSleeping(const b2World* self) {
    return self->GetAllowSleeping();
}

/// Enable/disable warm starting. For testing.
void box2d_World_SetWarmStarting(b2World* self, bool flag) {
    self->SetWarmStarting(flag);
}
bool box2d_World_GetWarmStarting(const b2World* self) {
    return self->GetWarmStarting();
}

/// Enable/disable continuous physics. For testing.
void box2d_World_SetContinuousPhysics(b2World* self, bool flag) {
    self->SetContinuousPhysics(flag);
}
bool box2d_World_GetContinuousPhysics(const b2World* self) {
    return self->GetContinuousPhysics();
}

/// Enable/disable single stepped continuous physics. For testing.
void box2d_World_SetSubStepping(b2World* self, bool flag) {
    self->SetSubStepping(flag);
}
bool box2d_World_GetSubStepping(const b2World* self) {
    return self->GetSubStepping();
}

/// Get the number of broad-phase proxies.
int32 box2d_World_GetProxyCount(const b2World* self) {
    return self->GetProxyCount();
}

/// Get the number of bodies.
int32 box2d_World_GetBodyCount(const b2World* self) {
    return self->GetBodyCount();
}

/// Get the number of joints.
int32 box2d_World_GetJointCount(const b2World* self) {
    return self->GetJointCount();
}

/// Get the number of contacts (each may have 0 or more contact points).
int32 box2d_World_GetContactCount(const b2World* self) {
    return self->GetContactCount();
}

/// Get the height of the dynamic tree.
int32 box2d_World_GetTreeHeight(const b2World* self) {
    return self->GetTreeHeight();
}

/// Get the balance of the dynamic tree.
int32 box2d_World_GetTreeBalance(const b2World* self) {
    return self->GetTreeBalance();
}

/// Get the quality metric of the dynamic tree. The smaller the better.
/// The minimum is 1.
float32 box2d_World_GetTreeQuality(const b2World* self) {
    return self->GetTreeQuality();
}

/// Change the global gravity vector.
void box2d_World_SetGravity(b2World* self, const b2Vec2* gravity) {
    self->SetGravity(*gravity);
}

/// Get the global gravity vector.
b2Vec2 box2d_World_GetGravity(const b2World* self) {
    return self->GetGravity();
}

/// Is the world locked (in the middle of a time step).
bool box2d_World_IsLocked(const b2World* self) {
    return self->IsLocked();
}

/// Set flag to control automatic clearing of forces after each time step.
void box2d_World_SetAutoClearForces(b2World* self, bool flag) {
    self->SetAutoClearForces(flag);
}

/// Get the flag that controls automatic clearing of forces after each time step.
bool box2d_World_GetAutoClearForces(const b2World* self) {
    return self->GetAutoClearForces();
}

/// Shift the world origin. Useful for large worlds.
/// The body shift formula is: position -= newOrigin
/// @param newOrigin the new origin with respect to the old origin
void box2d_World_ShiftOrigin(b2World* self, const b2Vec2* newOrigin) {
    self->ShiftOrigin(*newOrigin);
}

/// Get the contact manager for testing.
const b2ContactManager* box2d_World_GetContactManager(const b2World* self) {
    return &self->GetContactManager();
}

/// Get the current profile.
const b2Profile* box2d_World_GetProfile(const b2World* self) {
    return &self->GetProfile();
}

/// Dump the world into the log file.
/// @warning this should be called outside of a time step.
void box2d_World_Dump(b2World* self) {
    self->Dump();
}

} // extern C

