
#ifndef C_BOX2D_SHAPE
#define C_BOX2D_SHAPE

#include "Box2D/Common/c_box2d_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Clone the concrete shape using the provided allocator.
/// virtual
//box2d_Shape* box2d_Shape_Clone(const box2d_Shape* self, b2BlockAllocator* allocator);

/// Get the type of this shape. You can use this to down cast to the concrete shape.
/// @return the shape type.
int box2d_Shape_GetType(const box2d_Shape* self);

/// Get the number of child primitives.
/// virtual
int32 box2d_Shape_GetChildCount(const box2d_Shape* Self);

/// Test a point for containment in this shape. This only works for convex shapes.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// virtual
box2d_bool box2d_Shape_TestPoint(const box2d_Shape* self, const box2d_Transform* xf, const box2d_Vec2* p);

/// Cast a ray against a child shape.
/// @param output the ray-cast results.
/// @param input the ray-cast input parameters.
/// @param transform the transform to be applied to the shape.
/// @param childIndex the child shape index
/// virtual
box2d_bool box2d_Shape_RayCast(const box2d_Shape* self,
	                           box2d_RayCastOutput* output,
							   const box2d_RayCastInput* input,
                               const box2d_Transform* transform, int32 childIndex);

/// Given a transform, compute the associated axis aligned bounding box for a child shape.
/// @param aabb returns the axis aligned box.
/// @param xf the world transform of the shape.
/// @param childIndex the child shape
/// virtual
void box2d_Shape_ComputeAABB(const box2d_Shape* self, box2d_AABB* aabb, const box2d_Transform* xf, int32 childIndex);

/// Compute the mass properties of this shape using its dimensions and density.
/// The inertia tensor is computed about the local origin.
/// @param massData returns the mass data for this shape.
/// @param density the density in kilograms per meter squared.
/// virtual
void box2d_Shape_ComputeMass(const box2d_Shape* self, box2d_MassData* massData, float32 density);

#ifdef __cplusplus
} // extern C
#endif

#endif
