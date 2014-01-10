#include "Box2D/Collision/Shapes/c_box2d_polygonshape.h"

extern "C" {

box2d_Shape* box2d_PolygonShape_Upcast(box2d_PolygonShape* s) {
	return static_cast<b2Shape*>(reinterpret_cast<b2PolygonShape*>(s));
}

const box2d_Shape* box2d_PolygonShape_Upcast_const(const box2d_PolygonShape* s) {
	return static_cast<const b2Shape*>(reinterpret_cast<const b2PolygonShape*>(s));
}

box2d_PolygonShape box2d_PolygonShape_Create() {
	b2PolygonShape tmp;
	return *cast(&tmp);
}

int32 box2d_PolygonShape_GetChildCount(const box2d_PolygonShape* self) {
    return cast(self)->GetChildCount();
}

void box2d_PolygonShape_Set(box2d_PolygonShape* self, const box2d_Vec2* points, int32 count) {
    cast(self)->Set(cast(points), count);
}

void box2d_PolygonShape_SetAsBox(box2d_PolygonShape* self, float32 hx, float32 hy) {
    cast(self)->SetAsBox(hx, hy);
}

void box2d_PolygonShape_SetAsBox_2(box2d_PolygonShape* self, float32 hx, float32 hy, const box2d_Vec2* center, float32 angle) {
    cast(self)->SetAsBox(hx, hy, *cast(center), angle);
}

int32 box2d_PolygonShape_GetVertexCount(const box2d_PolygonShape* self) {
    return cast(self)->GetVertexCount();
}

const box2d_Vec2* box2d_PolygonShape_GetVertex(const box2d_PolygonShape* self, int32 index) {
    return cast(&cast(self)->GetVertex(index));
}

/*
/// Validate convexity. This is a very time consuming operation.
/// @returns true if valid
box2d_bool box2d_PolygonShape_Validate(const box2d_PolygonShape* self) {
    return self->Validate();
}
*/

} // extern C