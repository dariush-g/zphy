pub use crate::{bodies::*, collisions::*, joints::*, rays::*};
use bevy::app::App;

pub struct ZphyPlugin;
impl bevy::app::Plugin for ZphyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((CollisionPlugin, RigidBodyPlugin, JointPlugin));
    }
}
