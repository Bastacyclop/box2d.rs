use std::cast;
use math::Vec2;
use super::ffi;
use super::math;

pub type ShapeType = i32;
pub static CIRCLE_SHAPE: ShapeType  = 0 as ShapeType;
pub static EDGE_SHAPE: ShapeType    = 1 as ShapeType;
pub static POLYGON_SHAPE: ShapeType = 2 as ShapeType;
pub static CHAIN_SHAPE: ShapeType   = 3 as ShapeType;

pub trait Shape {
    fn handle(&self) -> *ffi::box2d_Shape;

    fn get_type(&self) -> ShapeType {
        unsafe {
            return ffi::box2d_Shape_GetType(self.handle());
        }
    }
    fn get_child_count(&self) -> i32 {
        unsafe {
            return ffi::box2d_Shape_GetChildCount(self.handle());
        }
    }
    fn test_point(&self,  xf: *ffi::box2d_Transform, p: math::Vec2) -> bool {
        unsafe {
            return ffi::box2d_Shape_TestPoint(self.handle(), xf, cast::transmute(&p)) != 0;
        }
    }
    fn ray_cast(&self, output: *ffi::box2d_RayCastOutput,  input: *ffi::box2d_RayCastInput, transform: *ffi::box2d_Transform, childIndex: i32) -> bool {
        unsafe {
            return ffi::box2d_Shape_RayCast(self.handle(), output, input, transform, childIndex) != 0;
        }
    }
    fn compute_aabb(&self, aabb: *ffi::box2d_AABB, xf: *ffi::box2d_Transform, child_index: i32) {
        unsafe {
            ffi::box2d_Shape_ComputeAABB(self.handle(), aabb, xf, child_index);
        }
    }
    fn compute_mass(&self, mass_data: *ffi::box2d_MassData, density: f32) {
        unsafe {
            ffi::box2d_Shape_ComputeMass(self.handle(), mass_data, density);
        }
    }
}

pub struct PolygonShape {
    ptr: *ffi::box2d_PolygonShape,
}

impl PolygonShape {
    pub fn default() -> PolygonShape {
        unsafe {
            return PolygonShape { ptr: ffi::box2d_PolygonShape_Create() };
        }
    }
    pub fn box_shape(w:f32, h:f32) -> PolygonShape {
        unsafe {
            let result = PolygonShape::default();
            ffi::box2d_PolygonShape_SetAsBox(result.ptr, w, h);
            return result;
        }
    }
}

impl Shape for PolygonShape {
    fn handle(&self) -> *ffi::box2d_Shape {
        unsafe {
            return ffi::box2d_PolygonShape_Upcast(self.ptr);
        }
    }
}

impl Drop for PolygonShape {
    fn drop(&mut self) {
        unsafe {
            ffi::box2d_PolygonShape_Destroy(self.ptr);
        }
    }
}
