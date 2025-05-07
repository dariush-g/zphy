use crate::rigidbodies::rigid_body_test;
use bevy::prelude::*;
use zphy::prelude::ZphyPlugin;

pub mod joint;
pub mod rigidbodies;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, ZphyPlugin))
        .add_systems(Startup, rigid_body_test)
        .run();
}
