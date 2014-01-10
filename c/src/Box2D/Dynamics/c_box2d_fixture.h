#ifndef C_BOX2D_FIXTURE
#define C_BOX2D_FIXTURE

#ifdef __cplusplus
extern "C" {
#endif

/// This holds contact filtering data.
typedef struct box2d_Filter
{
    /// The collision category bits. Normally you would just set one bit.
    uint16 categoryBits;

    /// The collision mask bits. This states the categories that this
    /// shape would accept for collision.
    uint16 maskBits;

    /// Collision groups allow a certain group of objects to never collide (negative)
    /// or always collide (positive). Zero means no collision group. Non-zero group
    /// filtering always wins against the mask bits.
    int16 groupIndex;
} box2d_Filter;

/// The constructor sets the default fixture definition values.
box2d_Filter box2d_Filter_create();

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
typedef struct box2d_FixtureDef
{
    /// The shape, this must be set. The shape will be cloned, so you
    /// can create the shape on the stack.
    box2d_Shape* shape;

    /// Use this to store application specific fixture data.
    void* userData;

    /// The friction coefficient, usually in the range [0,1].
    float32 friction;

    /// The restitution (elasticity) usually in the range [0,1].
    float32 restitution;

    /// The density, usually in kg/m^2.
    float32 density;

    /// A sensor shape collects contact information but never generates a collision
    /// response.
    box2d_bool isSensor;

    /// Contact filtering data.
    box2d_Filter filter;
} box2d_FixtureDef;

/// The constructor sets the default fixture definition values.
box2d_FixtureDef box2d_FixtureDef_create();

/// Get the type of the child shape. You can use this to down cast to the concrete shape.
/// @return the shape type.
int box2d_Fixture_GetType(const box2d_Fixture* self);

/// Get the child shape. You can modify the child shape, however you should not change the
/// number of vertices because this will crash some collision caching mechanisms.
/// Manipulating the shape may lead to non-physical behavior.
box2d_Shape* box2d_Fixture_GetShape(box2d_Fixture* self);
const box2d_Shape* box2d_Fixture_GetShape_const(const box2d_Fixture* self);

/// Set if this fixture is a sensor.
void box2d_Fixture_SetSensor(box2d_Fixture* self, box2d_bool sensor);

/// Is this fixture a sensor (non-solid)?
/// @return the true if the shape is a sensor.
box2d_bool box2d_Fixture_IsSensor(const box2d_Fixture* self);

/// Set the contact filtering data. This will not update contacts until the next time
/// step when either parent body is active and awake.
/// This automatically calls Refilter.
void box2d_Fixture_SetFilterData(box2d_Fixture* self, const box2d_Filter* filter);

/// Get the contact filtering data.
const box2d_Filter* box2d_Fixture_GetFilterData(const box2d_Fixture* self);

/// Call this if you want to establish collision that was previously disabled by box2d_ContactFilter::ShouldCollide.
void box2d_Fixture_Refilter(box2d_Fixture* self);

/// Get the parent body of this fixture. This is NULL if the fixture is not attached.
/// @return the parent body.
box2d_Body* box2d_Fixture_GetBody(box2d_Fixture* self);
const box2d_Body* box2d_Fixture_GetBody_const(const box2d_Fixture* self);

/// Get the next fixture in the parent body's fixture list.
/// @return the next shape.
box2d_Fixture* box2d_Fixture_GetNext(box2d_Fixture* self);
const box2d_Fixture* box2d_Fixture_GetNext_const(const box2d_Fixture* self);

/// Get the user data that was assigned in the fixture definition. Use this to
/// store your application specific data.
void* box2d_Fixture_GetUserData(const box2d_Fixture* self);

/// Set the user data. Use this to store your application specific data.
void box2d_Fixture_SetUserData(box2d_Fixture* self, void* data);

/// Test a point for containment in this fixture.
/// @param p a point in world coordinates.
box2d_bool box2d_Fixture_TestPoint(const box2d_Fixture* self, const box2d_Vec2* p);

/// Cast a ray against this shape.
/// @param output the ray-cast results.
/// @param input the ray-cast input parameters.
box2d_bool box2d_Fixture_RayCast(const box2d_Fixture* self, box2d_RayCastOutput* output, const box2d_RayCastInput* input, int32 childIndex);

/// Get the mass data for this fixture. The mass data is based on the density and
/// the shape. The rotational inertia is about the shape's origin. This operation
/// may be expensive.
void box2d_Fixture_GetMassData(const box2d_Fixture* self, box2d_MassData* massData);

/// Set the density of this fixture. This will _not_ automatically adjust the mass
/// of the body. You must call box2d_Body::ResetMassData to update the body's mass.
void box2d_Fixture_SetDensity(box2d_Fixture* self, float32 density);

/// Get the density of this fixture.
float32 box2d_Fixture_GetDensity(const box2d_Fixture* self);

/// Get the coefficient of friction.
float32 box2d_Fixture_GetFriction(const box2d_Fixture* self);

/// Set the coefficient of friction. This will _not_ change the friction of
/// existing contacts.
void box2d_Fixture_SetFriction(box2d_Fixture* self, float32 friction);

/// Get the coefficient of restitution.
float32 box2d_Fixture_GetRestitution(const box2d_Fixture* self);

/// Set the coefficient of restitution. This will _not_ change the restitution of
/// existing contacts.
void SetRestitution(box2d_Fixture* self, float32 restitution);

/// Get the fixture's AABB. This AABB may be enlarge and/or stale.
/// If you need a more accurate AABB, compute it using the shape and
/// the body transform.
const box2d_AABB* box2d_Fixture_GetAABB(const box2d_Fixture* self, int32 childIndex);

/// Dump this fixture to the log file.
void box2d_Fixture_Dump(box2d_Fixture* self, int32 bodyIndex);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
