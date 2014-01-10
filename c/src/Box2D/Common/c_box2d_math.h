#ifndef C_BOX2D_MATH
#define C_BOX2D_MATH

#ifdef __cplusplus
extern "C" {
#endif

typedef struct box2d_Vec2 { float32 x, y; } box2d_Vec2;
typedef struct box2d_Vec3 { float32 x, y, z; } box2d_Vec3;

/// A 2-by-2 matrix. Stored in column-major order.
struct box2d_Mat22
{
	box2d_Vec2 ex, ey;
};

/// A 3-by-3 matrix. Stored in column-major order.
struct box2d_Mat33
{
	box2d_Vec3 ex, ey, ez;
};

#ifdef __cplusplus
} // extern "C"
#endif

#endif
