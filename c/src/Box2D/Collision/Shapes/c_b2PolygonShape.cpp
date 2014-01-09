#include "c_b2Shape.cpp"

extern "C" {

/// @see b2Shape::GetChildCount
int32 box2d_PolygonShape_GetChildCount(const b2PolygonShape* self) {
    return self->GetChildCount();
}

/// Create a convex hull from the given array of local points.
/// The count must be in the range [3, b2_maxPolygonVertices].
/// @warning the points may be re-ordered, even if they form a convex polygon
/// @warning collinear points are handled but not removed. Collinear points
/// may lead to poor stacking behavior.
void box2d_PolygonShape_Set(b2PolygonShape* self, const b2Vec2* points, int32 count) {
    self->Set(points, count);
}

/// Build vertices to represent an axis-aligned box centered on the local origin.
/// @param hx the half-width.
/// @param hy the half-height.
void box2d_PolygonShape_SetAsBox(b2PolygonShape* self, float32 hx, float32 hy) {
    self->SetAsBox(hx, hy);
}

/// Build vertices to represent an oriented box.
/// @param hx the half-width.
/// @param hy the half-height.
/// @param center the center of the box in local coordinates.
/// @param angle the rotation of the box in local coordinates.
void box2d_PolygonShape_SetAsBox_2(b2PolygonShape* self, float32 hx, float32 hy, const b2Vec2* center, float32 angle) {
    self->SetAsBox(hx, hy, *center, angle);
}

/// Get the vertex count.
int32 box2d_PolygonShape_GetVertexCount(const b2PolygonShape* self) {
    return self->GetVertexCount();
}

/// Get a vertex by index.
const b2Vec2* box2d_PolygonShape_GetVertex(const b2PolygonShape* self, int32 index) {
    return &self->GetVertex(index);
}

/// Validate convexity. This is a very time consuming operation.
/// @returns true if valid
bool box2d_PolygonShape_Validate(const b2PolygonShape* self) {
    return self->Validate();
}

} // extern C