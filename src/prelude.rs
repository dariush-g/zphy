use bevy::app::App;
pub use crate::{bodies::*, collisions::*, rays::*};

pub struct ZphyPlugin;
impl bevy::app::Plugin for ZphyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((CollisionPlugin, RigidBodyPlugin));
    }
}