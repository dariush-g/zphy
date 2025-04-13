use bevy::prelude::*;

pub struct Ray {
    start: Vec3,
    dir: Vec3,
}

impl Ray {
    pub fn new(start: Vec3, dir: Vec3) -> Self {
        Self { start, dir }
    }

    pub fn get_start(&self) -> Vec3 {
        self.start
    }

    pub fn get_dir(&self) -> Vec3 {
        self.dir
    }
}
