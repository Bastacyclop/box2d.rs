#ifndef C_BOX2D_JOINT_H
#define C_BOX2D_JOINT_H

#ifdef __cplusplus
extern "C" {
#endif
typedef enum box2d_JointType
{
    e_box2d_unknownJoint,
    e_box2d_revoluteJoint,
    e_box2d_prismaticJoint,
    e_box2d_distanceJoint,
    e_box2d_pulleyJoint,
    e_box2d_mouseJoint,
    e_box2d_gearJoint,
    e_box2d_wheelJoint,
    e_box2d_weldJoint,
    e_box2d_frictionJoint,
    e_box2d_ropeJoint
} box2d_JointType;

typedef enum box2d_LimitState
{
    e_box2d_inactiveLimit,
    e_box2d_atLowerLimit,
    e_box2d_atUpperLimit,
    e_box2d_equalLimits
} box2d_LimitState;

/*
typedef struct box2d_Jacobian
{
    box2d_Vec2 linear;
    float32 angularA;
    float32 angularB;
} box2d_Jacobian;

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
typedef struct box2d_JointEdge
{
    box2d_Body* other;          ///< provides quick access to the other body attached.
    box2d_Joint* joint;         ///< the joint
    box2d_JointEdge* prev;      ///< the previous joint edge in the body's joint list
    box2d_JointEdge* next;      ///< the next joint edge in the body's joint list
} box2d_JointEdge;
*/
/// Joint definitions are used to construct joints.
typedef struct box2d_JointDef
{
    /// The joint type is set automatically for concrete joint types.
    box2d_JointType type;

    /// Use this to attach application specific data to your joints.
    void* userData;

    /// The first attached body.
    box2d_Body* bodyA;

    /// The second attached body.
    box2d_Body* bodyB;

    /// Set this flag to true if the attached bodies should collide.
    box2d_bool collideConnected;
} box2d_JointDef;

box2d_JointDef box2d_JointDef_Create();


/// Get the type of the concrete joint.
int32 box2d_Joint_GetType(const box2d_Joint* self);

/// Get the first body attached to this joint.
box2d_Body* box2d_Joint_GetBodyA(box2d_Joint* self);

/// Get the second body attached to this joint.
box2d_Body* box2d_Joint_GetBodyB(box2d_Joint* self);

/// Get the anchor point on bodyA in world coordinates.
/// virtual
box2d_Vec2 box2d_Joint_GetAnchorA(const box2d_Joint* self);

/// Get the anchor point on bodyB in world coordinates.
/// virtual
box2d_Vec2 box2d_Joint_GetAnchorB(const box2d_Joint* self);

/// Get the reaction force on bodyB at the joint anchor in Newtons.
/// virtual
box2d_Vec2 box2d_Joint_GetReactionForce(const box2d_Joint* self, float32 inv_dt);

/// Get the reaction torque on bodyB in N*m.
/// virtual
float32 box2d_Joint_GetReactionTorque(const box2d_Joint* self, float32 inv_dt);

/// Get the next joint the world joint list.
box2d_Joint* box2d_Joint_GetNext(box2d_Joint* self);
const box2d_Joint* box2d_Joint_GetNext_const(const box2d_Joint* self);

/// Get the user data pointer.
void* box2d_Joint_GetUserData(const box2d_Joint* self);

/// Set the user data pointer.
void box2d_Joint_SetUserData(box2d_Joint* self, void* data);

/// Short-cut function to determine if either body is inactive.
box2d_bool box2d_Joint_IsActive(const box2d_Joint* self);

/// Get collide connected.
/// Note: modifying the collide connect flag won't work correctly because
/// the flag is only checked when fixture AABBs begin to overlap.
box2d_bool box2d_Joint_GetCollideConnected(const box2d_Joint* self);

/// Dump this joint to the log file.
void box2d_Joint_Dump(box2d_Joint* self);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
