use crate::joint::joint_test;
use bevy::prelude::*;
use zphy::prelude::ZphyPlugin;

pub mod joint;
pub mod rigidbodies;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, ZphyPlugin))
        .add_systems(Startup, joint_test)
        .run();
}
