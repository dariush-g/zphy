pub mod joint_system;

use bevy::prelude::*;
use joint_system::*;

pub struct JointPlugin;

impl Plugin for JointPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, Joint::enforce);
    }
}
