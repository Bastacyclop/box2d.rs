
#ifndef C_BOX2D_SETTINGS
#define C_BOX2D_SETTINGS


#ifdef __cplusplus
extern "C" {
#endif

typedef signed char int8;
typedef signed short int16;
typedef signed int int32;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef float float32;
typedef double float64;
typedef int8 box2d_bool;

struct b2World;
struct b2Body;
struct b2Fixture;
struct b2AABB;
struct b2RaycastInput;
struct b2RaycastOutput;
struct b2DestructionListener;
struct b2Shape;
struct b2PolygonShape;
struct b2MassData;
struct b2Transform;
struct b2Joint;
struct b2JointEdge;
struct b2ContactEdge;
struct b2ContactFilter;
struct b2ContactListener;
struct b2DestructionListener;
struct b2Contact;
struct b2ContactManager;
struct b2Profile;
struct b2Draw;
struct b2QueryCallback;
struct b2RayCastCallback;

struct box2d_Vec2;
struct box2d_Vec3;
struct box2d_Mat2;
struct box2d_Mat3;
struct box2d_Filter;
struct box2d_BodyDef;
struct box2d_FixtureDef;
struct box2d_JointDef;

typedef struct b2Joint box2d_Joint;
typedef struct b2JointEdge box2d_JointEdge;
typedef struct b2World box2d_World;
typedef struct b2Shape box2d_Shape;
typedef struct b2Body box2d_Body;
typedef struct b2Fixture box2d_Fixture;
typedef struct b2Transform box2d_Transform;
typedef struct b2RayCastInput box2d_RayCastInput;
typedef struct b2RayCastOutput box2d_RayCastOutput;
typedef struct b2AABB box2d_AABB;
typedef struct b2MassData box2d_MassData;
typedef struct b2ContactEdge box2d_ContactEdge;
typedef struct b2ContactListener box2d_ContactListener;
typedef struct b2ContactFilter box2d_ContactFilter;
typedef struct b2DestructionListener box2d_DestructionListener;
typedef struct b2Contact box2d_Contact;
typedef struct b2ContactManager box2d_ContactManager;
typedef struct b2Profile box2d_Profile;
typedef struct b2Draw box2d_Draw;
typedef struct b2RayCastCallback box2d_RayCastCallback;
typedef struct b2QueryCallback box2d_QueryCallback;
typedef struct b2PolygonShape box2d_PolygonShape;

#ifndef NULL
#define NULL            0
#endif
#define b2_maxFloat     FLT_MAX
#define b2_epsilon      FLT_EPSILON
#define b2_pi           3.14159265359f

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
#define b2_maxManifoldPoints    2

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
#define b2_maxPolygonVertices   8

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
#define b2_aabbExtension        0.1f

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
#define b2_aabbMultiplier       2.0f

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
#define b2_linearSlop           0.005f

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
#define b2_angularSlop          (2.0f / 180.0f * b2_pi)

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
#define b2_polygonRadius        (2.0f * b2_linearSlop)

/// Maximum number of sub-steps per contact in continuous physics simulation.
#define b2_maxSubSteps          8


// Dynamics

/// Maximum number of contacts to be handled to solve a TOI impact.
#define b2_maxTOIContacts           32

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
#define b2_velocityThreshold        1.0f

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
#define b2_maxLinearCorrection      0.2f

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
#define b2_maxAngularCorrection     (8.0f / 180.0f * b2_pi)

/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
#define b2_maxTranslation           2.0f
#define b2_maxTranslationSquared    (b2_maxTranslation * b2_maxTranslation)

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
#define b2_maxRotation              (0.5f * b2_pi)
#define b2_maxRotationSquared       (b2_maxRotation * b2_maxRotation)

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
#define b2_baumgarte                0.2f
#define b2_toiBaugarte              0.75f


// Sleep

/// The time that a body must be still before it will go to sleep.
#define b2_timeToSleep              0.5f

/// A body cannot sleep if its linear velocity is above this tolerance.
#define b2_linearSleepTolerance     0.01f

/// A body cannot sleep if its angular velocity is above this tolerance.
#define b2_angularSleepTolerance    (2.0f / 180.0f * b2_pi)

#ifdef __cplusplus
} // extern C
#endif

#endif
