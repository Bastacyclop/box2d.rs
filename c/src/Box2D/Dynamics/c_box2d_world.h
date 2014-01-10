#ifndef C_BOX2D_WORLD
#define C_BOX2D_WORLD

#include "Box2D/Common/c_box2d_math.h"
#include "Box2D/Dynamics/Joints/c_box2d_joint.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Construct a world object.
/// @param gravity the world gravity vector.
box2d_World* box2d_World_Create(const box2d_Vec2* gravity);

/// Destruct the world. All physics entities are destroyed and all heap memory is released.
void box2d_World_Destroy(box2d_World* self);

void box2d_World_SetDestructionListener(box2d_World* self, box2d_DestructionListener* listener);

/// Register a contact filter to provide specific control over collision.
/// Otherwise the default filter is used (box2d__defaultFilter). The listener is
/// owned by you and must remain in scope. 
void box2d_World_SetContactFilter(box2d_World* self, box2d_ContactFilter* filter);

/// Register a contact event listener. The listener is owned by you and must
/// remain in scope.
void box2d_World_SetContactListener(box2d_World* self, box2d_ContactListener* listener);

/// Register a routine for debug drawing. The debug draw functions are called
/// inside with box2d_World::DrawDebugData method. The debug draw object is owned
/// by you and must remain in scope.
void box2d_World_SetDebugDraw(box2d_World* self, box2d_Draw* debugDraw);

/// Create a rigid body given a definition. No reference to the definition
/// is retained.
/// @warning This function is locked during callbacks.
box2d_Body* box2d_World_CreateBody(box2d_World* self, const box2d_BodyDef* def);

/// Destroy a rigid body given a definition. No reference to the definition
/// is retained. This function is locked during callbacks.
/// @warning This automatically deletes all associated shapes and joints.
/// @warning This function is locked during callbacks.
void box2d_World_DestroyBody(box2d_World* self, box2d_Body* body);

/// Create a joint to constrain bodies together. No reference to the definition
/// is retained. This may cause the connected bodies to cease colliding.
/// @warning This function is locked during callbacks.
box2d_Joint* box2d_World_CreateJoint(box2d_World* self, const box2d_JointDef* def);

/// Destroy a joint. This may cause the connected bodies to begin colliding.
/// @warning This function is locked during callbacks.
void box2d_World_DestroyJoint(box2d_World* self, box2d_Joint* joint);

/// Take a time step. This performs collision detection, integration,
/// and constraint solution.
/// @param timeStep the amount of time to simulate, this should not vary.
/// @param velocityIterations for the velocity constraint solver.
/// @param positionIterations for the position constraint solver.
void box2d_World_Step(box2d_World* self,
                      float32 timeStep,
                      int32 velocityIterations,
                      int32 positionIterations);

/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
/// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
/// a fixed sized time step under a variable frame-rate.
/// When you perform sub-stepping you will disable auto clearing of forces and instead call
/// ClearForces after all sub-steps are complete in one pass of your game loop.
/// @see SetAutoClearForces
void box2d_World_ClearForces(box2d_World* self);

/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
void box2d_World_DrawDebugData(box2d_World* self);

/// Query the world for all fixtures that potentially overlap the
/// provided AABB.
/// @param callback a user implemented callback class.
/// @param aabb the query box.
void box2d_World_QueryAABB(const box2d_World* self, box2d_QueryCallback* callback, const box2d_AABB* aabb);

/// Ray-cast the world for all fixtures in the path of the ray. Your callback
/// controls whether you get the closest point, any point, or n-points.
/// The ray-cast ignores shapes that contain the starting point.
/// @param callback a user implemented callback class.
/// @param point1 the ray starting point
/// @param point2 the ray ending point
void box2d_World_RayCast(const box2d_World* self, box2d_RayCastCallback* callback, const box2d_Vec2* point1, const box2d_Vec2* point2);

/// Get the world body list. With the returned body, use box2d_Body::GetNext to get
/// the next body in the world list. A NULL body indicates the end of the list.
/// @return the head of the world body list.
box2d_Body* box2d_World_GetBodyList(box2d_World* self);
const box2d_Body* box2d_World_GetBodyList_const(const box2d_World* self);

/// Get the world joint list. With the returned joint, use box2d_Joint::GetNext to get
/// the next joint in the world list. A NULL joint indicates the end of the list.
/// @return the head of the world joint list.
box2d_Joint* box2d_World_GetJointList(box2d_World* self);
const box2d_Joint* box2d_World_GetJointList_const(const box2d_World* self);

/// Get the world contact list. With the returned contact, use box2d_Contact::GetNext to get
/// the next contact in the world list. A NULL contact indicates the end of the list.
/// @return the head of the world contact list.
/// @warning contacts are created and destroyed in the middle of a time step.
/// Use box2d_ContactListener to avoid missing contacts.
box2d_Contact* box2d_World_GetContactList(box2d_World* self);
const box2d_Contact* box2d_World_GetContactList_const(const box2d_World* self);

/// Enable/disable sleep.
void box2d_World_SetAllowSleeping(box2d_World* self, box2d_bool flag);
box2d_bool box2d_World_GetAllowSleeping(const box2d_World* self);

/// Enable/disable warm starting. For testing.
void box2d_World_SetWarmStarting(box2d_World* self, box2d_bool flag);
box2d_bool box2d_World_GetWarmStarting(const box2d_World* self);

/// Enable/disable continuous physics. For testing.
void box2d_World_SetContinuousPhysics(box2d_World* self, box2d_bool flag);
box2d_bool box2d_World_GetContinuousPhysics(const box2d_World* self);

/// Enable/disable single stepped continuous physics. For testing.
void box2d_World_SetSubStepping(box2d_World* self, box2d_bool flag);
box2d_bool box2d_World_GetSubStepping(const box2d_World* self);

/// Get the number of broad-phase proxies.
int32 box2d_World_GetProxyCount(const box2d_World* self);

/// Get the number of bodies.
int32 box2d_World_GetBodyCount(const box2d_World* self);

/// Get the number of joints.
int32 box2d_World_GetJointCount(const box2d_World* self);

/// Get the number of contacts (each may have 0 or more contact points).
int32 box2d_World_GetContactCount(const box2d_World* self);

/// Get the height of the dynamic tree.
int32 box2d_World_GetTreeHeight(const box2d_World* self);

/// Get the balance of the dynamic tree.
int32 box2d_World_GetTreeBalance(const box2d_World* self);

/// Get the quality metric of the dynamic tree. The smaller the better.
/// The minimum is 1.
float32 box2d_World_GetTreeQuality(const box2d_World* self);

/// Change the global gravity vector.
void box2d_World_SetGravity(box2d_World* self, const box2d_Vec2* gravity);

/// Get the global gravity vector.
box2d_Vec2 box2d_World_GetGravity(const box2d_World* self);

/// Is the world locked (in the middle of a time step).
box2d_bool box2d_World_IsLocked(const box2d_World* self);

/// Set flag to control automatic clearing of forces after each time step.
void box2d_World_SetAutoClearForces(box2d_World* self, box2d_bool flag);

/// Get the flag that controls automatic clearing of forces after each time step.
box2d_bool box2d_World_GetAutoClearForces(const box2d_World* self);
/*
/// Shift the world origin. Useful for large worlds.
/// The body shift formula is: position -= newOrigin
/// @param newOrigin the new origin with respect to the old origin
void box2d_World_ShiftOrigin(box2d_World* self, const box2d_Vec2* newOrigin) {
    self->ShiftOrigin(*newOrigin);
}
*/
/// Get the contact manager for testing.
const box2d_ContactManager* box2d_World_GetContactManager(const box2d_World* self);

/// Get the current profile.
const box2d_Profile* box2d_World_GetProfile(const box2d_World* self);

/// Dump the world into the log file.
/// @warning this should be called outside of a time step.
void box2d_World_Dump(box2d_World* self);

#ifdef __cplusplus
} // extern C
#endif
#endif