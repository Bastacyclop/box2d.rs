
#include "Box2D/Dynamics/c_box2d_fixture.h"

extern "C"
{

box2d_Filter box2d_Filter_create()
{
    box2d_Filter result;
    result.categoryBits = 0x0001;
    result.maskBits = 0xFFFF;
    result.groupIndex = 0;
    return result;
}

box2d_FixtureDef box2d_FixtureDef_create()
{
    box2d_FixtureDef result;
    result.shape = NULL;
    result.userData = NULL;
    result.friction = 0.2f;
    result.restitution = 0.0f;
    result.density = 0.0f;
    result.isSensor = false;
    result.filter = box2d_Filter_create();
    return result;
}

int box2d_Fixture_GetType(const box2d_Fixture* self) {
    return self->GetType();
}

box2d_Shape* box2d_Fixture_GetShape(box2d_Fixture* self) {
    return self->GetShape();
}
const box2d_Shape* box2d_Fixture_GetShape_const(const box2d_Fixture* self) {
    return self->GetShape();
}

void box2d_Fixture_SetSensor(box2d_Fixture* self, box2d_bool sensor) {
    self->SetSensor(sensor);
}

box2d_bool box2d_Fixture_IsSensor(const box2d_Fixture* self) {
    return self->IsSensor();
}

void box2d_Fixture_SetFilterData(box2d_Fixture* self, const box2d_Filter* filter) {
    self->SetFilterData(*cast(filter));
}

const box2d_Filter* box2d_Fixture_GetFilterData(const box2d_Fixture* self) {
    return cast(&self->GetFilterData());
}

void box2d_Fixture_Refilter(box2d_Fixture* self) {
    self->Refilter();
}

box2d_Body* box2d_Fixture_GetBody(box2d_Fixture* self) {
    return self->GetBody();
}
const box2d_Body* box2d_Fixture_GetBody_const(const box2d_Fixture* self){
    return self->GetBody();
}

box2d_Fixture* box2d_Fixture_GetNext(box2d_Fixture* self) {
    return self->GetNext();
}
const box2d_Fixture* box2d_Fixture_GetNext_const(const box2d_Fixture* self) {
    return self->GetNext();
}

void* box2d_Fixture_GetUserData(const box2d_Fixture* self) {
    return self->GetUserData();
}

void box2d_Fixture_SetUserData(box2d_Fixture* self, void* data) {
    self->SetUserData(data);
}

box2d_bool box2d_Fixture_TestPoint(const box2d_Fixture* self, const box2d_Vec2* p) {
    return self->TestPoint(*cast(p));
}

box2d_bool box2d_Fixture_RayCast(const box2d_Fixture* self, box2d_RayCastOutput* output, const box2d_RayCastInput* input, int32 childIndex) {
    return self->RayCast(output, *input, childIndex);
}

void box2d_Fixture_GetMassData(const box2d_Fixture* self, box2d_MassData* massData) {
    self->GetMassData(massData);
}

void box2d_Fixture_SetDensity(box2d_Fixture* self, float32 density) {
    self->SetDensity(density);
}

float32 box2d_Fixture_GetDensity(const box2d_Fixture* self) {
    return self->GetDensity();
}

float32 box2d_Fixture_GetFriction(const box2d_Fixture* self) {
    return self->GetFriction();
}

void box2d_Fixture_SetFriction(box2d_Fixture* self, float32 friction) {
    self->SetFriction(friction);
}

float32 box2d_Fixture_GetRestitution(const box2d_Fixture* self) {
    return self->GetRestitution();
}

void box2d_Fixture_SetRestitution(box2d_Fixture* self, float32 restitution) {
    self->SetRestitution(restitution);
}

const box2d_AABB* box2d_Fixture_GetAABB(const box2d_Fixture* self, int32 childIndex) {
    return &self->GetAABB(childIndex);
}

void box2d_Fixture_Dump(box2d_Fixture* self, int32 bodyIndex) {
    self->Dump(bodyIndex);
}

}