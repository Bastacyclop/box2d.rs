use std::fmt;

#[deriving(Eq, Ord)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl fmt::Show for Vec2 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f.buf, "({}, {})", self.x, self.y)
    }
}


#[deriving(Eq, Ord)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl fmt::Show for Vec3 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f.buf, "({}, {}, {})", self.x, self.y, self.z)
    }
}
