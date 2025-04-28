use bevy::prelude::*;
use zphy::{
    bodies::{Damping, RigidbodyComponent},
    collisions::Collider,
};

#[derive(Component)]
pub struct X;

pub(crate) fn rigid_body_test(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn(Camera3d::default())
        .insert(Transform::from_xyz(8., 5., 8.).looking_at(Vec3::ZERO, Vec3::Y));

    let cuboid = Cuboid::new(1., 1., 1.);

    commands.spawn(PointLight::default());

    commands.spawn((
        X,
        RigidbodyComponent::new_dynamic(
            0.5,
            Collider::from_cuboid(
                cuboid.half_size,
                Vec3::ZERO,
                Quat::from_euler(EulerRot::XYZ, 0., 100., 0.),
            ),
            0.,
            Vec3::ZERO,
            Vec3::new(1., 0.3, 0.5),
            Vec3::ZERO,
            Damping::default(),
            0.0,
        ),
        Mesh3d(meshes.add(cuboid)),
        MeshMaterial3d(materials.add(Color::WHITE)),
    ));

    let cuboid = Cuboid::new(10., 1., 10.);

    let rotation = Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, std::f32::consts::FRAC_PI_4); // 45°
    let position = Vec3::new(0.0, -5.0, 0.0);

    commands.spawn((
        RigidbodyComponent::new_static(Collider::from_cuboid(cuboid.half_size, position, rotation)),
        Mesh3d(meshes.add(cuboid)),
        MeshMaterial3d(materials.add(Color::WHITE)),
        Transform {
            translation: position,
            rotation,
            ..Default::default()
        },
    ));
}
