struct b2Vec2;
struct b2BodyDef;
struct b2FixtureDef;
struct b2PolygonShape;
struct box2d_Vec2;
struct box2d_BodyDef;
struct box2d_FixtureDef;
struct box2d_PolygonShape;

box2d_Vec2* cast(b2Vec2* v) {
	return reinterpret_cast<box2d_Vec2*>(v);
}
const box2d_Vec2* cast(const b2Vec2* v) {
	return reinterpret_cast<const box2d_Vec2*>(v);
}

b2Vec2* cast(box2d_Vec2* v) {
	return reinterpret_cast<b2Vec2*>(v);
}
const b2Vec2* cast(const box2d_Vec2* v) {
	return reinterpret_cast<const b2Vec2*>(v);
}


box2d_BodyDef* cast(b2BodyDef* def) {
	return reinterpret_cast<box2d_BodyDef*>(def);
}
const box2d_BodyDef* cast(const b2BodyDef* def){
	return reinterpret_cast<const box2d_BodyDef*>(def);
}

b2BodyDef* cast(box2d_BodyDef* def) {
	return reinterpret_cast<b2BodyDef*>(def);
}
const b2BodyDef* cast(const box2d_BodyDef* def) {
	return reinterpret_cast<const b2BodyDef*>(def);
}


box2d_FixtureDef* cast(b2FixtureDef* def) {
	return reinterpret_cast<box2d_FixtureDef*>(def);
}
const box2d_FixtureDef* cast(const b2FixtureDef* def){
	return reinterpret_cast<const box2d_FixtureDef*>(def);
}

b2FixtureDef* cast(box2d_FixtureDef* def) {
	return reinterpret_cast<b2FixtureDef*>(def);
}
const b2FixtureDef* cast(const box2d_FixtureDef* def) {
	return reinterpret_cast<const b2FixtureDef*>(def);
}


box2d_JointDef* cast(b2JointDef* def) {
	return reinterpret_cast<box2d_JointDef*>(def);
}
const box2d_JointDef* cast(const b2JointDef* def){
	return reinterpret_cast<const box2d_JointDef*>(def);
}

b2JointDef* cast(box2d_JointDef* def) {
	return reinterpret_cast<b2JointDef*>(def);
}
const b2JointDef* cast(const box2d_JointDef* def) {
	return reinterpret_cast<const b2JointDef*>(def);
}


box2d_PolygonShape* cast(b2PolygonShape* def) {
	return reinterpret_cast<box2d_PolygonShape*>(def);
}
const box2d_PolygonShape* cast(const b2PolygonShape* def){
	return reinterpret_cast<const box2d_PolygonShape*>(def);
}

b2PolygonShape* cast(box2d_PolygonShape* def) {
	return reinterpret_cast<b2PolygonShape*>(def);
}
const b2PolygonShape* cast(const box2d_PolygonShape* def) {
	return reinterpret_cast<const b2PolygonShape*>(def);
}

box2d_Filter* cast(b2Filter* def) {
	return reinterpret_cast<box2d_Filter*>(def);
}
const box2d_Filter* cast(const b2Filter* def){
	return reinterpret_cast<const box2d_Filter*>(def);
}

b2Filter* cast(box2d_Filter* def) {
	return reinterpret_cast<b2Filter*>(def);
}
const b2Filter* cast(const box2d_Filter* def) {
	return reinterpret_cast<const b2Filter*>(def);
}
