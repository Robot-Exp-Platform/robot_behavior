//! A pair of bodies keeps its own padding in place of the scene's, in
//! configuration checks and whole-motion checks alike.

use std::path::PathBuf;

use nalgebra::Isometry3;
use roplat_description::{Geometry, RobotModel};
use roplat_scene::{CollisionCheck, MOTION_TOLERANCE, PlanningScene, SceneError};

/// A planar two-link arm 0.25 m up (as in `scene.rs`): the forearm, 3 cm in
/// radius, runs from 0.5 to 0.9 m when straight; the finger is a 2 cm sphere
/// at 0.93 m.
const ARM: &str = r#"<robot name="toy">
  <link name="base"><collision><origin xyz="0 0 0.1"/><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/><origin xyz="0 0 0.25"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="1"/></joint>
  <link name="upper"><collision><origin xyz="0.25 0 0"/><geometry><box size="0.5 0.05 0.05"/></geometry></collision></link>
  <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/><origin xyz="0.5 0 0"/><axis xyz="0 0 1"/><limit lower="-3.1" upper="3.1" effort="10" velocity="1"/></joint>
  <link name="fore"><collision><origin xyz="0.2 0 0" rpy="0 1.5707963267948966 0"/><geometry><cylinder radius="0.03" length="0.4"/></geometry></collision></link>
  <joint name="wrist" type="fixed"><parent link="fore"/><child link="finger"/><origin xyz="0.43 0 0"/></joint>
  <link name="finger"><collision><geometry><sphere radius="0.02"/></geometry></collision></link>
</robot>"#;

const STRAIGHT: [f64; 2] = [0.0, 0.0];

/// Two posts beside the straight arm: `glass` 4 cm from the forearm's side,
/// `shelf` 4 cm from the finger.
fn scene() -> PlanningScene<2> {
    let model = RobotModel::from_urdf_str(ARM).unwrap();
    let mut scene = PlanningScene::new(model, ["shoulder", "elbow"], |name: &str| {
        PathBuf::from(name)
    })
    .unwrap();
    let post = Geometry::Box { size: [0.04, 0.04, 0.3] };
    // Forearm surface at y = 0.03; the glass's near face at y = 0.07.
    scene
        .add_object("glass", &post, Isometry3::translation(0.7, 0.09, 0.25))
        .unwrap();
    // Finger surface at x = 0.95; the shelf's near face at x = 0.99.
    scene
        .add_object("shelf", &post, Isometry3::translation(1.01, 0.0, 0.25))
        .unwrap();
    scene
}

fn pairs(scene: &PlanningScene<2>, q: &[f64; 2]) -> Vec<(String, String)> {
    scene
        .contacts(q)
        .into_iter()
        .map(|c| (c.first, c.second))
        .collect()
}

#[test]
fn a_pair_keeps_more_than_the_scene() {
    let mut scene = scene();
    assert!(scene.is_free(&STRAIGHT));
    // 5 cm around the glass only: the forearm, 4 cm away, is too close; the
    // finger, 4 cm from the shelf, is not.
    scene.set_pair_padding("glass", "fore", 0.05).unwrap();
    assert_eq!(
        pairs(&scene, &STRAIGHT),
        [("fore".to_string(), "glass".to_string())]
    );
    let contact = scene.first_contact(&STRAIGHT).unwrap();
    assert!((contact.distance - 0.04).abs() < 1e-9, "{contact:?}");
    // Either order names the pair.
    assert_eq!(scene.pair_padding("fore", "glass"), 0.05);
    assert_eq!(scene.pair_padding("finger", "shelf"), 0.0);
    // The scene's padding still applies elsewhere.
    scene.set_padding(0.045).unwrap();
    let both = pairs(&scene, &STRAIGHT);
    assert!(
        both.contains(&("finger".to_string(), "shelf".to_string())),
        "{both:?}"
    );
}

#[test]
fn a_pair_keeps_less_than_the_scene() {
    let mut scene = scene();
    scene.set_padding(0.05).unwrap();
    assert_eq!(pairs(&scene, &STRAIGHT).len(), 2);
    // The finger may pass the shelf at 1 cm.
    scene.set_pair_padding("shelf", "finger", 0.01).unwrap();
    assert_eq!(
        pairs(&scene, &STRAIGHT),
        [("fore".to_string(), "glass".to_string())]
    );
    assert!(scene.clear_pair_padding("finger", "shelf"));
    assert!(!scene.clear_pair_padding("finger", "shelf"));
    assert_eq!(pairs(&scene, &STRAIGHT).len(), 2);
}

#[test]
fn motions_keep_the_pair_padding() {
    let mut scene = scene();
    // Folding the elbow 0.3 rad towards the glass brings the forearm in.
    let (from, to) = ([0.0, -0.3], [0.0, 0.3]);
    let free_without = scene.motion_contact(&from, &to);
    scene.set_pair_padding("fore", "glass", 0.06).unwrap();
    let hit = scene.motion_contact(&from, &to).unwrap();
    assert_eq!(
        (hit.contact.first.as_str(), hit.contact.second.as_str()),
        ("fore", "glass")
    );
    assert!(
        hit.contact.distance <= 0.06 + MOTION_TOLERANCE + 1e-9,
        "{hit:?}"
    );
    // It stops earlier than a motion that only avoids touching would.
    if let Some(touch) = free_without {
        assert!(hit.fraction < touch.fraction, "{hit:?} {touch:?}");
    }
    // Every configuration before the reported fraction keeps the 6 cm.
    for k in 0..200 {
        let t = hit.fraction * k as f64 / 200.0;
        let q = [0.0, -0.3 + 0.6 * t];
        assert!(
            scene.is_free(&q),
            "contact at {t}: {:?}",
            scene.first_contact(&q)
        );
    }
}

#[test]
fn a_padding_must_be_finite_and_not_negative() {
    let mut scene = scene();
    for bad in [-0.01, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            scene.set_pair_padding("fore", "glass", bad),
            Err(SceneError::InvalidPadding(_))
        ));
    }
    assert_eq!(scene.pair_padding("fore", "glass"), 0.0);
}
