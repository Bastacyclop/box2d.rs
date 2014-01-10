
#include "Box2D/Dynamics/Joints/c_box2d_joint.h"

box2d_JointDef box2d_JointDef_Create()
{
    box2d_JointDef result;
    result.type = e_box2d_unknownJoint;
    result.userData = NULL;
    result.bodyA = NULL;
    result.bodyB = NULL;
    result.collideConnected = false;
    return result;
}

/// Get the type of the concrete joint.
int32 box2d_Joint_GetType(const box2d_Joint* self) {
    return self->GetType();
}

/// Get the first body attached to this joint.
box2d_Body* box2d_Joint_GetBodyA(box2d_Joint* self) {
    return self->GetBodyA();
}

/// Get the second body attached to this joint.
box2d_Body* box2d_Joint_GetBodyB(box2d_Joint* self) {
    return self->GetBodyB();
}

/// Get the anchor point on bodyA in world coordinates.
/// virtual
box2d_Vec2 box2d_Joint_GetAnchorA(const box2d_Joint* self) {
    b2Vec2 tmp =  self->GetAnchorA();
    return *cast(&tmp);
}

/// Get the anchor point on bodyB in world coordinates.
/// virtual
box2d_Vec2 box2d_Joint_GetAnchorB(const box2d_Joint* self) {
    b2Vec2 tmp = self->GetAnchorB();
    return *cast(&tmp);
}

/// Get the reaction force on bodyB at the joint anchor in Newtons.
/// virtual
box2d_Vec2 box2d_Joint_GetReactionForce(const box2d_Joint* self, float32 inv_dt) {
    b2Vec2 tmp = self->GetReactionForce(inv_dt);
    return *cast(&tmp);
}

/// Get the reaction torque on bodyB in N*m.
/// virtual
float32 box2d_Joint_GetReactionTorque(const box2d_Joint* self, float32 inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

/// Get the next joint the world joint list.
box2d_Joint* box2d_Joint_GetNext(box2d_Joint* self) {
    return self->GetNext();
}
const box2d_Joint* box2d_Joint_GetNext_const(const box2d_Joint* self) {
    return self->GetNext();
}

/// Get the user data pointer.
void* box2d_Joint_GetUserData(const box2d_Joint* self) {
    return self->GetUserData();
}

/// Set the user data pointer.
void box2d_Joint_SetUserData(box2d_Joint* self, void* data) {
    self->SetUserData(data);
}

/// Short-cut function to determine if either body is inactive.
box2d_bool box2d_Joint_IsActive(const box2d_Joint* self) {
    return self->IsActive();
}

/// Get collide connected.
/// Note: modifying the collide connect flag won't work correctly because
/// the flag is only checked when fixture AABBs begin to overlap.
box2d_bool box2d_Joint_GetCollideConnected(const box2d_Joint* self) {
    return self->GetCollideConnected();
}

/// Dump this joint to the log file.
void box2d_Joint_Dump(box2d_Joint* self) {
    return self->Dump();
}

