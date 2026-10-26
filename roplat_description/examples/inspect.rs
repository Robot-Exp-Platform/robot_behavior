//! Print the tree of each URDF given on the command line.
use roplat_description::RobotModel;

fn main() {
    for path in std::env::args().skip(1) {
        match RobotModel::from_urdf_file(&path) {
            Ok(m) => {
                let movable = m.joints.iter().filter(|j| j.is_movable()).count();
                println!(
                    "ok   {path}: {} links, {} joints ({movable} movable), root {}",
                    m.links.len(),
                    m.joints.len(),
                    m.root().name
                );
            }
            Err(e) => println!("FAIL {path}: {e}"),
        }
    }
}
