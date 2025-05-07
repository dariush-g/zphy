mod collider_systems;

use crate::bodies::Velocity;
use bevy::prelude::*;
use collider_systems::{detect_collisions, update_vertices};

pub struct CollisionPlugin;
impl Plugin for CollisionPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, (update_vertices, detect_collisions));
    }
}

#[derive(Debug)]
pub struct ContactInfo {
    pub normal: Vec3,
    pub penetration_depth: f32,
    pub contact_point_a: Vec3,
    pub a_vel: Velocity,
    pub contact_point_b: Vec3,
    pub b_vel: Velocity,
}

#[derive(Clone)]
pub struct ColliderVertexInfo {
    pub vertices: Vec<Vec3>,
}

impl ColliderVertexInfo {
    pub fn from_cuboid(center: &Vec3, half_size: &Vec3, rotation: &Quat) -> Self {
        let offsets = [
            Vec3::new(-half_size.x, -half_size.y, -half_size.z),
            Vec3::new(-half_size.x, -half_size.y, half_size.z),
            Vec3::new(-half_size.x, half_size.y, -half_size.z),
            Vec3::new(-half_size.x, half_size.y, half_size.z),
            Vec3::new(half_size.x, -half_size.y, -half_size.z),
            Vec3::new(half_size.x, -half_size.y, half_size.z),
            Vec3::new(half_size.x, half_size.y, -half_size.z),
            Vec3::new(half_size.x, half_size.y, half_size.z),
        ];

        let vertices = offsets
            .iter()
            .map(|offset| *center + *rotation * *offset)
            .collect();
        ColliderVertexInfo { vertices }
    }
}

#[derive(Clone)]
pub struct Collider {
    pub collider_shape: ColliderShape,
    pub center: Vec3,
    pub rotation: Quat,
    pub axes: [Vec3; 3],
    pub half_extents: Vec3,
    pub vertex_info: ColliderVertexInfo,
}

#[derive(PartialEq, Eq, Clone, Copy, Debug, Default)]
pub enum ColliderShape {
    #[default]
    Cuboid,
    Capsule,
    Sphere,
    Ellipsoid,
}

impl Collider {
    pub fn get_axes(&self, other: &Self) -> Vec<Vec3> {
        let mut axes = vec![];

        // 3 local axes from self
        axes.extend_from_slice(&self.axes);

        // 3 local axes from other
        axes.extend_from_slice(&other.axes);

        // 9 cross products
        for &a in &self.axes {
            for &b in &other.axes {
                let cross = a.cross(b);
                if cross.length_squared() > 1e-6 {
                    axes.push(cross.normalize());
                }
            }
        }

        axes
    }

    pub fn from_cuboid(half_size: Vec3, center: Vec3, rotation: Quat) -> Self {
        let axes = [rotation * Vec3::X, rotation * Vec3::Y, rotation * Vec3::Z];

        let vertex_info = ColliderVertexInfo::from_cuboid(&center, &half_size, &rotation);

        Self {
            collider_shape: ColliderShape::Cuboid,
            center,
            rotation,
            axes,
            half_extents: half_size,
            vertex_info,
        }
    }
}
