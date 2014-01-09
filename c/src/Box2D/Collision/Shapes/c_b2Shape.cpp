extern "C" {


/// Clone the concrete shape using the provided allocator.
/// virtual
//b2Shape* box2d_Shape_Clone(const b2Shape* self, b2BlockAllocator* allocator) {
//    return self->Clone(allocator);
//}

/// Get the type of this shape. You can use this to down cast to the concrete shape.
/// @return the shape type.
int box2d_Shape_GetType(const b2Shape* self) {
    return self->GetType();
}

/// Get the number of child primitives.
/// virtual
int32 box2d_Shape_GetChildCount(const b2Shape* Self) {
    return Self->GetChildCount();
}

/// Test a point for containment in this shape. This only works for convex shapes.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// virtual
bool box2d_Shape_TestPoint(const b2Shape* self, const b2Transform* xf, const b2Vec2* p) {
    return self->TestPoint(*xf, *p);
}

/// Cast a ray against a child shape.
/// @param output the ray-cast results.
/// @param input the ray-cast input parameters.
/// @param transform the transform to be applied to the shape.
/// @param childIndex the child shape index
/// virtual
bool box2d_Shape_RayCast(const b2Shape* self, b2RayCastOutput* output, const b2RayCastInput& input,
                    const b2Transform* transform, int32 childIndex) {
    return self->RayCast(output, input, *transform, childIndex);
}

/// Given a transform, compute the associated axis aligned bounding box for a child shape.
/// @param aabb returns the axis aligned box.
/// @param xf the world transform of the shape.
/// @param childIndex the child shape
/// virtual
void box2d_Shape_ComputeAABB(const b2Shape* self, b2AABB* aabb, const b2Transform* xf, int32 childIndex) {
    self->ComputeAABB(aabb, *xf, childIndex);
}

/// Compute the mass properties of this shape using its dimensions and density.
/// The inertia tensor is computed about the local origin.
/// @param massData returns the mass data for this shape.
/// @param density the density in kilograms per meter squared.
/// virtual
void box2d_Shape_ComputeMass(const b2Shape* self, b2MassData* massData, float32 density) {
    self->ComputeMass(massData, density);
}

} // extern C