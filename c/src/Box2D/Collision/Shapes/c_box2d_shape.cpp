extern "C" {

/// Clone the concrete shape using the provided allocator.
/// virtual
//box2d_Shape* box2d_Shape_Clone(const box2d_Shape* self, b2BlockAllocator* allocator) {
//    return self->Clone(allocator);
//}

int box2d_Shape_GetType(const box2d_Shape* self) {
    return self->GetType();
}

int32 box2d_Shape_GetChildCount(const box2d_Shape* Self) {
    return Self->GetChildCount();
}

box2d_bool box2d_Shape_TestPoint(const box2d_Shape* self, const box2d_Transform* xf, const box2d_Vec2* p) {
    return self->TestPoint(*xf, *cast(p));
}

box2d_bool box2d_Shape_RayCast(const box2d_Shape* self,
							   box2d_RayCastOutput* output,
							   const box2d_RayCastInput* input,
                    		   const box2d_Transform* transform, int32 childIndex) {
    return self->RayCast(output, *input, *transform, childIndex);
}

void box2d_Shape_ComputeAABB(const box2d_Shape* self, box2d_AABB* aabb, const box2d_Transform* xf, int32 childIndex) {
    self->ComputeAABB(aabb, *xf, childIndex);
}

void box2d_Shape_ComputeMass(const box2d_Shape* self, box2d_MassData* massData, float32 density) {
    self->ComputeMass(massData, density);
}

} // extern C
