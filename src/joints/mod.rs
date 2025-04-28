pub mod joint_system;

use crate::bodies::RigidbodyComponent;
use bevy::prelude::*;

pub struct JointPlugin;

impl Plugin for JointPlugin {
    fn build(&self, app: &mut App) {}
}
