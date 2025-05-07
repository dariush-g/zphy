use bevy::prelude::*;
use zphy::joints::joint_system::{Joint, JointMember, JointType, MemberLimit};
use zphy::prelude::*;

pub(crate) fn joint_test(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut mats: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(PointLight::default());
    commands
        .spawn(Camera3d::default())
        .insert(Transform::from_xyz(-5.0, 5.0, -5.0).looking_at(Vec3::ZERO, Vec3::Y));

    let cuboid = Cuboid::new(10., 1., 10.);
    let rotation = Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, 1.);
    let position = Vec3::new(0.0, -5.0, 0.0);

    commands.spawn((
        RigidbodyComponent::new_static(Collider::from_cuboid(cuboid.half_size, position, rotation)),
        Mesh3d(meshes.add(cuboid)),
        MeshMaterial3d(mats.add(Color::WHITE)),
        Transform {
            translation: position,
            rotation,
            ..Default::default()
        },
    ));

    // let cuboid = Cuboid::new(5., 1., 1.);
    // let rotation = Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, 0.);
    // let position = Vec3::new(0.0, 0.0, 0.0);
    //
    // let p = commands
    //     .spawn((
    //         RigidbodyComponent::new_dynamic(
    //             1.,
    //             Collider::from_cuboid(cuboid.half_size, position, rotation),
    //             0.,
    //             Vec3::ZERO,
    //             Vec3::ZERO,
    //             Vec3::ZERO,
    //             Damping::default(),
    //             0.,
    //         ),
    //         Mesh3d(meshes.add(cuboid)),
    //         MeshMaterial3d(mats.add(Color::WHITE)),
    //         Transform {
    //             translation: position,
    //             rotation,
    //             ..Default::default()
    //         },
    //     ))
    //     .id();

    let cuboid = Cuboid::new(5., 1., 1.);
    let rotation = Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, 1.);
    let position = Vec3::new(0.0, 0.0, -1.0);

    let p1 = commands
        .spawn((
            RigidbodyComponent::new_dynamic(
                2.,
                Collider::from_cuboid(cuboid.half_size, position, rotation),
                0.,
                Vec3::ZERO,
                Vec3::new(1., 0., 0.),
                Vec3::ZERO,
                Damping::default(),
                0.,
            ),
            Mesh3d(meshes.add(cuboid)),
            MeshMaterial3d(mats.add(Color::WHITE)),
            Transform {
                translation: position,
                ..Default::default()
            },
        ))
        .id();

    let joint = Joint::new(
        JointMember::new(p1, Vec3::ZERO, MemberLimit::new(-Vec3::Y, Vec3::Y)),
        JointType::Hinge,
    );
    commands.spawn(joint);
}
