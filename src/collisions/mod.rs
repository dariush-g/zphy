use crate::bodies::{RigidbodyComponent, RigidbodyType, Velocity};
use bevy::prelude::*;

pub struct CollisionPlugin;
impl Plugin for CollisionPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, (update_vertices, detect_collisions));
    }
}

fn update_vertices(mut query: Query<(&mut RigidbodyComponent, &Transform), With<RigidbodyComponent>>) {
    for (mut body, transform) in query.iter_mut() {
        body.collider.center = transform.translation;

        //let rot: [f32; 3] = transform.rotation.to_euler(EulerRot::XYZ).into();
        body.collider.axes = [
            transform.rotation * Vec3::X,
            transform.rotation * Vec3::Y,
            transform.rotation * Vec3::Z,
        ];

        body.collider.vertex_info = ColliderVertexInfo::from_cuboid(
            &body.collider.center,
            &body.collider.half_extents,
            &transform.rotation,
        );
    }
}

fn rotation_to_align(from: Vec3, to: Vec3) -> Quat {
    let from = from.normalize();
    let to = to.normalize();
    let axis = from.cross(to);
    let angle = from.angle_between(to);
    if axis.length_squared() < 1e-6 {
        if from.dot(to) < 0.0 {
            // Rotate 180° around any perpendicular axis
            Quat::from_axis_angle(Vec3::X, std::f32::consts::PI)
        } else {
            Quat::IDENTITY
        }
    } else {
        Quat::from_axis_angle(axis.normalize(), angle)
    }
}

fn detect_collisions(mut query: Query<(&mut RigidbodyComponent, &mut Transform)>) {
    let mut items: Vec<_> = query.iter_mut().collect();

    for i in 0..items.len() {
        for j in (i + 1)..items.len() {
            let (left, right) = items.split_at_mut(j);
            let (ref mut body_a, ref mut transform_a) = left[i];
            let (ref mut body_b, ref mut transform_b) = right[0];

            if let Some(collision_data) = get_collision_info(
                &body_a.collider,
                &body_a.velocity,
                &body_b.collider,
                &body_b.velocity,
            ) {
                println!("Collision detected! Normal: {:?}, Penetration: {}", collision_data.normal, collision_data.penetration_depth);
                resolve_collision(body_a, transform_a, body_b, transform_b, &collision_data);
            }
        }
    }
}

fn resolve_collision(
    a: &mut RigidbodyComponent,
    tf_a: &mut Transform,
    b: &mut RigidbodyComponent,
    tf_b: &mut Transform,
    contact: &ContactInfo,
) {
    let total_inverse_mass = a.inverse_mass + b.inverse_mass;

    if total_inverse_mass == 0.0 {
        return;
    }

    let correction = contact.normal * (contact.penetration_depth / total_inverse_mass);

    let temp_vel = a.velocity.linear;

    if a.inverse_mass > 0.0 {
        //todo: if the vel of the rb is < 0.2, make it sleep so that it does not keep rubberbanding and jittering
        a.collider.center += correction * a.inverse_mass;
        a.velocity.linear = Vec3::ZERO;
        a.velocity.angular = Vec3::ZERO;
        a.velocity.linear += a.restitution * (contact.normal + temp_vel) * (1. - a.damping.linear)
            + contact.b_vel.linear;
        tf_a.translation = a.collider.center;
    }

    if b.rbt == RigidbodyType::Static && a.rbt == RigidbodyType::Dynamic {
        let local_down = Vec3::Y;
        let world_down = tf_a.rotation * local_down;
        let align_rotation: Vec3 = rotation_to_align(world_down, contact.normal)
            .to_euler(EulerRot::XYZ)
            .into();
        a.velocity.angular += align_rotation * a.inverse_mass * a.velocity.linear.length().max(6.5);
    }

    if b.inverse_mass > 0.0 {
        b.collider.center += correction * b.inverse_mass;
        b.velocity.linear = Vec3::ZERO;
        b.velocity.angular = Vec3::ZERO;
        b.velocity.linear += b.restitution * (contact.normal + temp_vel) * (1. - b.damping.linear)
            + contact.a_vel.linear;
        tf_b.translation = b.collider.center;
    }
}

fn project_collider(collider: &Collider, axis: Vec3) -> (f32, f32) {
    let mut min = f32::MAX;
    let mut max = f32::MIN;

    for vertex in &collider.vertex_info.vertices {
        let projection = axis.dot(*vertex);
        min = min.min(projection);
        max = max.max(projection);
    }

    (min, max)
}

fn get_overlap(min_a: f32, max_a: f32, min_b: f32, max_b: f32) -> f32 {
    (max_a.min(max_b) - min_a.max(min_b)).max(0.0)
}

fn get_collision_info(
    a: &Collider,
    a_vel: &Velocity,
    b: &Collider,
    b_vel: &Velocity,
) -> Option<ContactInfo> {
    let axes = a.get_axes(b);
    let mut min_overlap = f32::INFINITY;
    let mut collision_axis = Vec3::ZERO;

    for axis in axes {
        let axis = axis.normalize();
        let (min_a, max_a) = project_collider(a, axis);
        let (min_b, max_b) = project_collider(b, axis);

        let overlap = get_overlap(min_a, max_a, min_b, max_b);
        if overlap <= 0.0 {
            return None;
        }

        if overlap < min_overlap {
            min_overlap = overlap;
            collision_axis = axis;
        }
    }

    Some(ContactInfo {
        normal: collision_axis,
        penetration_depth: min_overlap,
        contact_point_a: a.center,
        contact_point_b: b.center,
        a_vel: *a_vel,
        b_vel: *b_vel,
    })
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
            axes,
            half_extents: half_size,
            vertex_info,
        }
    }
}
