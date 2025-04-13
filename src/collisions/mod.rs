use bevy::prelude::*;

use crate::bodies::RigidbodyComponent;

pub struct CollisionPlugin;
impl Plugin for CollisionPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, obb);
    }
}

fn obb(mut query: Query<(&RigidbodyComponent, &Transform)>) {
    for (rigidbody, transform) in query.iter() {



    }
}
