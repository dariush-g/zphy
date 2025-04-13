use bevy::{asset::RenderAssetUsages, prelude::*};
use zphy::{
    bodies::{Damping, RigidbodyComponent},
    collisions::Collider,
};

pub(crate) fn rigid_body_test(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn(Camera3d::default())
        .insert(Transform::from_xyz(5., 5., 5.).looking_at(Vec3::ZERO, Vec3::Y));

    let cuboid = Cuboid::new(1., 1., 1.);

    commands.spawn((
        RigidbodyComponent::new_dynamic(
            0.1,
            Collider::from_cuboid(
                cuboid.half_size,
                Vec3::ZERO,
                Quat::from_euler(EulerRot::XYZ, 0., 0., 0.),
            ),
            0.,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
            Damping::default(),
            0.,
        ),
        Mesh3d(meshes.add(cuboid)),
        MeshMaterial3d(materials.add(Color::WHITE)),
    ));

    let cuboid = Cuboid::new(10., 1., 10.);

    commands.spawn((
        RigidbodyComponent::new_dynamic(
            0.000001,
            Collider::from_cuboid(
                cuboid.half_size,
                Vec3::ZERO,
                Quat::from_euler(EulerRot::XYZ, 0., -10., 0.),
            ),
            0.,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
            Damping::default(),
            0.,
        ),
        Mesh3d(meshes.add(cuboid)),
        MeshMaterial3d(materials.add(Color::WHITE)),
        Transform::from_xyz(0., -10., 0.),
    ));
}
