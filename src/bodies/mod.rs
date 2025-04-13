use bevy::prelude::*;

use crate::collisions::Collider;

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

#[derive(Clone, Copy)]
pub struct Velocity {
    pub linear: Vec3,
    pub angular: Vec3,
}

impl Velocity {
    pub fn new(linear: Vec3, angular: Vec3) -> Self {
        Self { linear, angular }
    }
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
            linear: 0.0,
            angular: 0.0,
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
        let inertia_tensor = Mat3::IDENTITY;

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

    pub fn get_inverse_inertia_world(&self, rotation: &Quat) -> Mat3 {
        let rot_mat = Mat3::from_quat(*rotation);
        rot_mat * self.inverse_inertia_tensor * rot_mat.transpose()
    }
}

fn apply_forces(mut query: Query<(&mut RigidbodyComponent, &mut Transform)>, time: Res<Time>) {
    for (mut body, mut transform) in query.iter_mut() {
        let acceleration = -9.18 / body.inverse_mass;

        let linear_damping = body.damping.linear;
        let angular_damping = body.damping.angular;
        body.velocity.linear *= 1. - linear_damping;
        body.velocity.angular *= 1. - angular_damping;

        body.velocity.linear.y += acceleration * time.delta_secs();

        transform.translation += body.velocity.linear * time.delta_secs();
        transform.rotation = Quat::from_euler(
            EulerRot::XYZ,
            (body.torque.x + 1.) * body.velocity.angular.x * time.delta_secs(),
            (body.torque.y + 1.) * body.velocity.angular.y * time.delta_secs(),
            (body.torque.z + 1.) * body.velocity.angular.z * time.delta_secs(),
        ) * transform.rotation;
    }
}
