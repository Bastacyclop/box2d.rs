use std::cast;
use math::{Vec2, Vec3};
use super::ffi;
use super::settings;

pub type ShapeType = i32;
pub static CIRCLE_SHAPE: ShapeType  = 0 as ShapeType;
pub static EDGE_SHAPE: ShapeType    = 1 as ShapeType;
pub static POLYGON_SHAPE: ShapeType = 2 as ShapeType;
pub static CHAIN_SHAPE: ShapeType   = 3 as ShapeType;

pub struct PolygonShape {
    centroid: Vec2,
    vertices: [Vec2, ..settings::MAX_POLYGON_VERTICES],
    normals: [Vec2, ..settings::MAX_POLYGON_VERTICES],
    vertex_count: i32,
}

impl PolygonShape {
    pub fn default() -> PolygonShape {
        return PolygonShape {
            centroid: Vec2 {x:0.0, y:0.0},
            vertices: [
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
            ],
            normals: [
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
                Vec2 {x:0.0, y:0.0},
            ],
            vertex_count: 0,
        }
    }
    pub fn box_shape(w:f32, h:f32) -> PolygonShape {
        let result = PolygonShape::default();
        unsafe {
            ffi::box2d_PolygonShape_SetAsBox(cast::transmute(&result), w, h);
        }
        return result;
    }
}
