use crate::prelude::RigidbodyComponent;
use bevy::prelude::*;
use bevy::render::render_resource::ShaderType;

/// The main struct for a Joint that holds different types of constraints
#[derive(Component, Clone, Debug)]
pub struct Joint {
    pub member: JointMember,
    pub joint_type: JointType,
}

#[derive(Clone, Debug)]
pub enum JointType {
    BallSocket,
    Slider,
    Hinge,
}

#[derive(Clone, Debug)]
pub struct JointMember {
    pub entity: Entity,
    pub direction: Vec3,
    pub limits: MemberLimit<Vec3>,
}

impl JointMember {
    pub fn new(entity: Entity, direction: Vec3, limits: MemberLimit<Vec3>) -> Self {
        Self {
            entity,
            direction,
            limits,
        }
    }
}

#[derive(Clone, Debug)]
pub struct MemberLimit<N> {
    pub min: N,
    pub max: N,
}

impl MemberLimit<Vec3> {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        if min.size() < max.size() {
            panic!()
        }

        Self { min, max }
    }

    pub fn clamp_position(&self, position: Vec3, direction: Vec3) -> Vec3 {
        // First, project the position onto the direction
        let pos_along_dir = position.dot(direction);

        // Clamp the projected position between the min and max bounds
        let clamped_pos_along_dir =
            pos_along_dir.clamp(self.min.dot(direction), self.max.dot(direction));

        // Reconstruct the position, using the direction and the clamped value
        direction * clamped_pos_along_dir
    }
}

impl Joint {
    pub fn new(member: JointMember, joint_type: JointType) -> Self {
        Self { member, joint_type }
    }

    // Apply force to enforce the joint's constraint
    pub fn enforce(
        mut query: Query<(&mut Joint, &mut RigidbodyComponent, &mut Transform)>,
        time: Res<Time>,
    ) {
        if let Ok((joint, mut rb, mut transform)) = query.get_single_mut() {
            match joint.joint_type {
                JointType::BallSocket => joint.enforce_ball_socket(&mut rb, &mut transform),
                JointType::Slider => joint.enforce_slider(&mut rb, &mut transform),
                JointType::Hinge => joint.enforce_hinge(&mut rb, &mut transform),
            }
        }
    }

    fn enforce_ball_socket(&self, rb: &mut RigidbodyComponent, transform: &mut Transform) {
        // Ball-Socket constraint: No translation, only rotation allowed
        // This will enforce no positional movement but free rotation

        let direction = transform.translation; // assuming the direction is the position relative to the joint origin
        let limit = self.member.limits.clone(); // assuming all members have the same limit

        let clamped_position = limit.clamp_position(transform.translation, direction);
        transform.translation = clamped_position;
    }

    fn enforce_slider(&self, rb: &mut RigidbodyComponent, transform: &mut Transform) {
        // Slider constraint: Allow motion along one axis (e.g., along the X-axis), no rotation

        let direction = transform.translation; // Get position along the axis
        let limit = self.member.limits.clone(); // Assume limits

        let clamped_position = limit.clamp_position(transform.translation, direction);
        transform.translation = clamped_position;

        // Apply forces to enforce translation along direction
    }

    fn enforce_hinge(&self, rb: &mut RigidbodyComponent, transform: &mut Transform) {
        // Hinge constraint: Allow rotation around one axis, no translation along that axis
        rb.velocity.linear = Vec3::ZERO;

        let direction = transform.translation; // Axis for rotation
        let limit = self.member.limits.clone(); // Assume limits

        let clamped_position = limit.clamp_position(transform.translation, direction);
        println!("clamped joint!");
        transform.translation = clamped_position;
    }
}
