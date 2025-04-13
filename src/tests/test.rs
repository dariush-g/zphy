use bevy::prelude::*;
use rigidbodies::rigid_body_test;
use zphy::{bodies::RigidBodyPlugin, collisions::CollisionPlugin};

pub mod rigidbodies;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, CollisionPlugin, RigidBodyPlugin))
        .add_systems(Startup, rigid_body_test)
        .run();
}
