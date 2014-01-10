
#ifndef C_BOX2D_POLYGONSHAPE
#define C_BOX2D_POLYGONSHAPE

#ifdef __cplusplus
extern "C" {
#endif

typedef struct box2d_PolygonShape {
    box2d_Vec2 m_centroid;
    box2d_Vec2 m_vertices[b2_maxPolygonVertices];
    box2d_Vec2 m_normals[b2_maxPolygonVertices];
    int32 m_vertexCount;
} box2d_PolygonShape;

box2d_PolygonShape box2d_PolygonShape_Create();

/// @see b2Shape::GetChildCount
int32 box2d_PolygonShape_GetChildCount(const box2d_PolygonShape* self);

/// Create a convex hull from the given array of local points.
/// The count must be in the range [3, b2_maxPolygonVertices].
/// @warning the points may be re-ordered, even if they form a convex polygon
/// @warning collinear points are handled but not removed. Collinear points
/// may lead to poor stacking behavior.
void box2d_PolygonShape_Set(box2d_PolygonShape* self, const box2d_Vec2* points, int32 count);

/// Build vertices to represent an axis-aligned box centered on the local origin.
/// @param hx the half-width.
/// @param hy the half-height.
void box2d_PolygonShape_SetAsBox(box2d_PolygonShape* self, float32 hx, float32 hy);

/// Build vertices to represent an oriented box.
/// @param hx the half-width.
/// @param hy the half-height.
/// @param center the center of the box in local coordinates.
/// @param angle the rotation of the box in local coordinates.
void box2d_PolygonShape_SetAsBox_2(box2d_PolygonShape* self, float32 hx, float32 hy, const box2d_Vec2* center, float32 angle);

/// Get the vertex count.
int32 box2d_PolygonShape_GetVertexCount(const box2d_PolygonShape* self);

/// Get a vertex by index.
const box2d_Vec2* box2d_PolygonShape_GetVertex(const box2d_PolygonShape* self, int32 index);

/*
/// Validate convexity. This is a very time consuming operation.
/// @returns true if valid
box2d_bool box2d_PolygonShape_Validate(const box2d_PolygonShape* self);
*/

#ifdef __cplusplus
} // extern C
#endif

#endif
