use bevy::prelude::*;
use zphy::bodies::{Damping, RigidBodyPlugin, RigidbodyComponent};

pub fn rigid_body_test() {
    App::new()
        .add_plugins((DefaultPlugins, RigidBodyPlugin))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(5.0, 5.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(Vec3::ONE))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.7, 0.6))),
        RigidbodyComponent::new_dynamic(
            1.0,
            0.5,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
            Damping::default(),
            0.5,
        ),
    ));
}
