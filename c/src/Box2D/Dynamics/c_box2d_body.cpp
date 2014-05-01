
#include "Box2D/Dynamics/c_box2d_body.h"

extern "C" {

box2d_BodyDef box2d_BodyDef_Create()
{
    box2d_BodyDef result;
    result.userData = NULL;
    result.position.x = 0.0f;
    result.position.y = 0.0f;
    result.angle = 0.0f;
    result.linearVelocity.x = 0.0f;
    result.linearVelocity.y = 0.0f;
    result.angularVelocity = 0.0f;
    result.linearDamping = 0.0f;
    result.angularDamping = 0.0f;
    result.allowSleep = true;
    result.awake = true;
    result.fixedRotation = false;
    result.bullet = false;
    result.type = box2d_staticBody;
    result.active = true;
    result.gravityScale = 1.0f;
    return result;
}

box2d_Fixture* box2d_Body_CreateFixture(box2d_Body* self, const box2d_FixtureDef* def) {
    return self->CreateFixture(cast(def));
}

box2d_Fixture* box2d_Body_CreateFixture_shape(box2d_Body* self, const box2d_Shape* shape, float32 density) {
    return self->CreateFixture(shape, density);
}

void box2d_Body_DestroyFixture(box2d_Body* self, box2d_Fixture* fixture) {
    self->DestroyFixture(fixture);
}

void box2d_Body_SetTransform(box2d_Body* self, const box2d_Vec2* position, float32 angle) {
    self->SetTransform(*cast(position), angle);
}

const box2d_Transform* box2d_Body_GetTransform(const box2d_Body* self) {
    return &self->GetTransform();
}

const box2d_Vec2* box2d_Body_GetPosition(const box2d_Body* self) {
    return cast(&self->GetPosition());
}

float32 box2d_Body_GetAngle(const box2d_Body* self) {
    return self->GetAngle();
}

const box2d_Vec2* box2d_Body_GetWorldCenter(const box2d_Body* self) {
    return cast(&self->GetWorldCenter());
}

const box2d_Vec2* box2d_Body_GetLocalCenter(const box2d_Body* self) {
    return cast(&self->GetLocalCenter());
}

void box2d_Body_SetLinearVelocity(box2d_Body* self, const box2d_Vec2* v) {
    self->SetLinearVelocity(*cast(v));
}

const box2d_Vec2 box2d_Body_GetLinearVelocity(const box2d_Body* self) {
    b2Vec2 tmp = self->GetLinearVelocity();
    return *cast(&tmp);
}

void box2d_Body_SetAngularVelocity(box2d_Body* self, float32 omega) {
    self->SetAngularVelocity(omega);
}

float32 box2d_Body_GetAngularVelocity(const box2d_Body* self) {
    return self->GetAngularVelocity();
}

void box2d_Body_ApplyForce(box2d_Body* self, const box2d_Vec2* force,
                           const box2d_Vec2* point, box2d_bool wake) {
    self->ApplyForce(*cast(force), *cast(point), wake);
}

void box2d_Body_ApplyForceToCenter(box2d_Body* self, const box2d_Vec2* force, box2d_bool wake) {
    self->ApplyForceToCenter(*cast(force), wake);
}

void box2d_Body_ApplyTorque(box2d_Body* self, float32 torque, box2d_bool wake) {
    self->ApplyTorque(torque, wake);
}

void box2d_Body_ApplyLinearImpulse(box2d_Body* self, const box2d_Vec2* impulse, const box2d_Vec2* point, box2d_bool wake) {
    self->ApplyLinearImpulse(*cast(impulse), *cast(point), wake);
}

void box2d_Body_ApplyAngularImpulse(box2d_Body* self, float32 impulse, box2d_bool wake) {
    self->ApplyAngularImpulse(impulse, wake);
}

float32 box2d_Body_GetMass(const box2d_Body* self) {
    return self->GetMass();
}

float32 box2d_Body_GetInertia(const box2d_Body* self) {
    return self->GetInertia();
}

void box2d_Body_GetMassData(const box2d_Body* self, box2d_MassData* data) {
    self->GetMassData(data);
}

void box2d_Body_SetMassData(box2d_Body* self, const box2d_MassData* data) {
    self->SetMassData(data);
}

void box2d_Body_ResetMassData(box2d_Body* self) {
    self->ResetMassData();
}

box2d_Vec2 box2d_Body_GetWorldPoint(const box2d_Body* self, const box2d_Vec2* localPoint) {
    b2Vec2 tmp = self->GetWorldPoint(*cast(localPoint));
    return *cast(&tmp);
}

box2d_Vec2 box2d_Body_GetWorldVector(const box2d_Body* self, const box2d_Vec2* localVector) {
    b2Vec2 tmp = self->GetWorldVector(*cast(localVector));
    return *cast(&tmp);
}

box2d_Vec2 box2d_Body_GetLocalPoint(const box2d_Body* self, const box2d_Vec2* worldPoint) {
    b2Vec2 tmp = self->GetLocalPoint(*cast(worldPoint));
    return *cast(&tmp);
}

box2d_Vec2 box2d_Body_GetLocalVector(const box2d_Body* self, const box2d_Vec2* worldVector) {
    b2Vec2 tmp = self->GetLocalVector(*cast(worldVector));
    return *cast(&tmp);
}

box2d_Vec2 box2d_Body_GetLinearVelocityFromWorldPoint(const box2d_Body* self, const box2d_Vec2* worldPoint) {
    b2Vec2 tmp = self->GetLinearVelocityFromWorldPoint(*cast(worldPoint));
    return *cast(&tmp);
}

box2d_Vec2 box2d_Body_GetLinearVelocityFromLocalPoint(const box2d_Body* self, const box2d_Vec2* localPoint) {
    b2Vec2 tmp = self->GetLinearVelocityFromLocalPoint(*cast(localPoint));
    return *cast(&tmp);
}

float32 box2d_Body_GetLinearDamping(const box2d_Body* self) {
    return self->GetLinearDamping();
}

void box2d_Body_SetLinearDamping(box2d_Body* self, float32 linearDamping) {
    self->SetLinearDamping(linearDamping);
}

float32 box2d_Body_GetAngularDamping(const box2d_Body* self) {
    return self->GetAngularDamping();
}

void box2d_Body_SetAngularDamping(box2d_Body* self, float32 angularDamping) {
    self->SetAngularDamping(angularDamping);
}

float32 box2d_Body_GetGravityScale(const box2d_Body* self) {
    return self->GetGravityScale();
}

void box2d_Body_SetGravityScale(box2d_Body* self, float32 scale) {
    self->SetGravityScale(scale);
}

void box2d_Body_SetType(box2d_Body* self, int type) {
    self->SetType(static_cast<b2BodyType>(type));
}

box2d_BodyType box2d_Body_GetType(const box2d_Body* self) {
    return (box2d_BodyType)self->GetType();
}

void box2d_Body_SetBullet(box2d_Body* self, box2d_bool flag) {
    self->SetBullet(flag);
}

box2d_bool box2d_Body_IsBullet(const box2d_Body* self) {
    return self->IsBullet();
}

void box2d_Body_SetSleepingAllowed(box2d_Body* self, box2d_bool flag) {
    self->SetSleepingAllowed(flag);
}

box2d_bool box2d_Body_IsSleepingAllowed(const box2d_Body* self) {
    return self->IsSleepingAllowed();
}

void box2d_Body_SetAwake(box2d_Body* self, box2d_bool flag) {
    self->SetAwake(flag);
}

box2d_bool box2d_Body_IsAwake(const box2d_Body* self) {
    return self->IsAwake();
}

void box2d_Body_SetActive(box2d_Body* self, box2d_bool flag) {
    self->SetActive(flag);
}

box2d_bool box2d_Body_IsActive(const box2d_Body* self) {
    return self->IsActive();
}

void box2d_Body_SetFixedRotation(box2d_Body* self, box2d_bool flag) {
    return self->SetFixedRotation(flag);
}

box2d_bool box2d_Body_IsFixedRotation(const box2d_Body* self) {
    return self ->IsFixedRotation();
}

box2d_Fixture* box2d_Body_GetFixtureList(box2d_Body* self) {
    return self->GetFixtureList();
}
const box2d_Fixture* box2d_Body_GetFixtureList_const(const box2d_Body* self) {
    return self->GetFixtureList();
}

box2d_JointEdge* box2d_Body_GetJointList(box2d_Body* self) {
    return self->GetJointList();
}
const box2d_JointEdge* box2d_Body_GetJointList_const(const box2d_Body* self) {
    return self->GetJointList();
}

box2d_ContactEdge* box2d_Body_GetContactList(box2d_Body* self) {
    return self->GetContactList();
}
const box2d_ContactEdge* box2d_Body_GetContactList_const(const box2d_Body* self) {
    return self->GetContactList();
}

box2d_Body* box2d_Body_GetNext(box2d_Body* self) {
    return self->GetNext();
}
const box2d_Body* box2d_Body_GetNext_const(const box2d_Body* self) {
    return self->GetNext();
}

void* box2d_Body_GetUserData(const box2d_Body* self) {
    return self->GetUserData();
}

void box2d_Body_SetUserData(box2d_Body* self, void* data) {
    self->SetUserData(data);
}

box2d_World* box2d_Body_GetWorld(box2d_Body* self) {
    return self->GetWorld();
}
const box2d_World* box2d_Body_GetWorld_const(const box2d_Body* self) {
    return self->GetWorld();
}

void box2d_Body_Dump(box2d_Body* self) {
    self->Dump();
}

} // extern C
