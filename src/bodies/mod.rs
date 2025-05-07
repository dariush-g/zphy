use bevy::prelude::*;

use crate::collisions::{Collider, ColliderShape};

pub struct RigidBodyPlugin;

impl Plugin for RigidBodyPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, apply_forces);
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RigidbodyType {
    Static,
    Dynamic,
    Kinematic,
}

impl From<RigidbodyComponent> for RigidbodyType {
    fn from(value: RigidbodyComponent) -> Self {
        match value.rbt {
            RigidbodyType::Static => RigidbodyType::Static,
            RigidbodyType::Dynamic => RigidbodyType::Dynamic,
            RigidbodyType::Kinematic => RigidbodyType::Kinematic,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Velocity {
    pub linear: Vec3,
    pub angular: Vec3,
}

impl Velocity {
    pub fn new(linear: Vec3, angular: Vec3) -> Self {
        Self { linear, angular }
    }

    pub const ZERO: Self = Self {
        linear: Vec3::ZERO,
        angular: Vec3::ZERO,
    };
}

#[derive(Clone, Default, Eq, PartialEq)]
pub enum RigidBodyState {
    Asleep,
    #[default]
    Awake,
}

#[derive(Copy, Clone)]
pub struct Damping {
    pub linear: f32,
    pub angular: f32,
}

impl Default for Damping {
    fn default() -> Self {
        Self {
            linear: 0.05,
            angular: 0.05,
        }
    }
}

#[derive(Component, Clone)]
pub struct RigidbodyComponent {
    pub state: RigidBodyState,
    pub rbt: RigidbodyType,
    pub collider: Collider,
    pub velocity: Velocity,
    pub inverse_mass: f32,
    pub friction: f32,
    pub torque: Vec3,
    pub damping: Damping,
    pub inverse_inertia_tensor: Mat3,
    pub restitution: f32,
}

fn cube_inertia_tensor(mass: f32, size: Vec3) -> Mat3 {
    let w = size.x;
    let h = size.y;
    let d = size.z;

    let ix = (1.0 / 12.0) * mass * (h * h + d * d);
    let iy = (1.0 / 12.0) * mass * (w * w + d * d);
    let iz = (1.0 / 12.0) * mass * (w * w + h * h);

    Mat3::from_diagonal(Vec3::new(ix, iy, iz))
}

impl RigidbodyComponent {
    #[allow(clippy::too_many_arguments)]
    pub fn new_dynamic(
        mass: f32,
        collider: Collider,
        friction: f32,
        velocity: Vec3,
        angular_velocity: Vec3,
        torque: Vec3,
        damping: Damping,
        restitution: f32,
    ) -> Self {
        let mut inertia_tensor = Mat3::ZERO;
        if collider.collider_shape == ColliderShape::Cuboid {
            inertia_tensor = cube_inertia_tensor(mass, collider.half_extents * 2.);
        }

        Self {
            state: RigidBodyState::Awake,
            rbt: RigidbodyType::Dynamic,
            collider,
            inverse_mass: 1. / mass,
            friction,
            velocity: Velocity::new(velocity, angular_velocity),
            torque,
            damping,
            inverse_inertia_tensor: inertia_tensor.inverse(),
            restitution,
        }
    }

    pub fn new_static(collider: Collider) -> Self {
        Self {
            state: RigidBodyState::Awake,
            rbt: RigidbodyType::Static,
            inverse_mass: 0.,
            friction: 0.,
            velocity: Velocity::new(Vec3::ZERO, Vec3::ZERO),
            torque: Vec3::ZERO,
            damping: Damping::default(),
            inverse_inertia_tensor: Mat3::ZERO,
            restitution: 0.,
            collider,
        }
    }

    pub fn new_kinematic(mass: f32, collider: Collider) -> Self {
        Self {
            state: RigidBodyState::Awake,
            rbt: RigidbodyType::Kinematic,
            inverse_mass: 1. / mass,
            friction: 0.,
            collider,
            velocity: Velocity::ZERO,
            torque: Vec3::ZERO,
            damping: Damping::default(),
            inverse_inertia_tensor: Mat3::ZERO,
            restitution: 0.,
        }
    }

    pub fn get_inverse_inertia_world(&self, rotation: &Quat) -> Mat3 {
        let rot_mat = Mat3::from_quat(*rotation);
        rot_mat * self.inverse_inertia_tensor * rot_mat.transpose()
    }
}

fn apply_forces(mut query: Query<(&mut RigidbodyComponent, &mut Transform)>, time: Res<Time>) {
    let gravity = Vec3::new(0.0, -9.81, 0.0);

    for (mut body, mut transform) in query.iter_mut() {
        if body.rbt == RigidbodyType::Static {
            continue;
        }

        let linear_damping = body.damping.linear;
        let angular_damping = body.damping.angular;
        body.velocity.linear *= 1.0 - linear_damping;
        body.velocity.angular *= 1.0 - angular_damping;

        let inverse_mass = body.inverse_mass;
        body.velocity.linear += gravity * (1. / inverse_mass) * time.delta_secs();

        transform.translation += body.velocity.linear * time.delta_secs();

        let angular_speed = body.velocity.angular.length();
        if angular_speed > 0.01 {
            let rotation_axis = body.velocity.angular.normalize();
            let delta_rotation =
                Quat::from_axis_angle(rotation_axis, angular_speed * time.delta_secs());
            body.collider.rotation = (delta_rotation * body.collider.rotation).normalize();
        }

        body.collider.center = transform.translation;
        if body.rbt == RigidbodyType::Dynamic {
            transform.rotation = body.collider.rotation;
        }
    }
}
