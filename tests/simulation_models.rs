use robot_behavior::{
    ModelFormat, ModelSource, RobotDescription, SimulationBackend, SimulationModel,
    select_simulation_model,
};

struct BulletLoader;
impl SimulationBackend for BulletLoader {
    const BACKEND_ID: &'static str = "bullet";
    const MODEL_FORMATS: &'static [ModelFormat] = &[ModelFormat::Urdf];
}

struct MujocoLoader;
impl SimulationBackend for MujocoLoader {
    const BACKEND_ID: &'static str = "mujoco";
    const MODEL_FORMATS: &'static [ModelFormat] = &[ModelFormat::Mjcf, ModelFormat::Urdf];
}

struct MultiBackendArm;
impl RobotDescription for MultiBackendArm {
    const URDF: Option<&'static str> = Some("legacy/arm.urdf");
    const SIMULATION_MODELS: &'static [SimulationModel] = &[
        SimulationModel::new(ModelFormat::Urdf, "portable/arm.urdf"),
        SimulationModel::new(ModelFormat::Mjcf, "mujoco/arm.xml").for_backend("mujoco"),
        SimulationModel::new(ModelFormat::Urdf, "bullet/arm.urdf").for_backend("bullet"),
    ];
}

struct LegacyArm;
impl RobotDescription for LegacyArm {
    const URDF: Option<&'static str> = Some("legacy/arm.urdf");
}

#[test]
fn device_backend_combinations_select_their_own_declared_models() {
    let bullet = BulletLoader::select_robot_model::<MultiBackendArm>().unwrap();
    let mujoco = MujocoLoader::select_robot_model::<MultiBackendArm>().unwrap();
    assert_eq!(bullet.model.path, "bullet/arm.urdf");
    assert_eq!(mujoco.model.path, "mujoco/arm.xml");
    assert_eq!(bullet.source, ModelSource::BackendSpecific);
    assert_eq!(mujoco.source, ModelSource::BackendSpecific);

    let other = select_simulation_model::<MultiBackendArm>("other", &[ModelFormat::Urdf]).unwrap();
    assert_eq!(other.model.path, "portable/arm.urdf");
    assert_eq!(other.source, ModelSource::Portable);
}

#[test]
fn existing_urdf_only_implementations_remain_usable() {
    let selected = BulletLoader::select_robot_model::<LegacyArm>().unwrap();
    assert_eq!(selected.model.format, ModelFormat::Urdf);
    assert_eq!(selected.model.path, "legacy/arm.urdf");
    assert_eq!(selected.source, ModelSource::LegacyUrdf);
}

#[test]
fn unsupported_format_does_not_invent_a_conversion_or_use_legacy_urdf() {
    let error =
        select_simulation_model::<LegacyArm>("mjcf-only", &[ModelFormat::Mjcf]).unwrap_err();
    assert_eq!(error.backend, "mjcf-only");
    assert_eq!(error.supported_formats, &[ModelFormat::Mjcf]);
    assert!(error.to_string().contains("LegacyArm"));
    assert!(error.to_string().contains("Mjcf"));
}

#[test]
fn format_match_cannot_bypass_a_backend_restriction() {
    struct Restricted;
    impl RobotDescription for Restricted {
        const SIMULATION_MODELS: &'static [SimulationModel] =
            &[SimulationModel::new(ModelFormat::Urdf, "other/arm.urdf").for_backend("other")];
    }
    assert!(BulletLoader::select_robot_model::<Restricted>().is_err());
}

#[test]
fn backend_format_preference_then_declaration_order_are_deterministic() {
    struct Portable;
    impl RobotDescription for Portable {
        const SIMULATION_MODELS: &'static [SimulationModel] = &[
            SimulationModel::new(ModelFormat::Urdf, "first.urdf"),
            SimulationModel::new(ModelFormat::Mjcf, "first.xml"),
            SimulationModel::new(ModelFormat::Mjcf, "second.xml"),
        ];
    }
    assert_eq!(
        MujocoLoader::select_robot_model::<Portable>()
            .unwrap()
            .model
            .path,
        "first.xml"
    );
    assert_eq!(
        BulletLoader::select_robot_model::<Portable>()
            .unwrap()
            .model
            .path,
        "first.urdf"
    );
}

#[test]
fn backend_specific_model_precedes_a_portable_model_of_a_preferred_format() {
    struct Adapted;
    impl RobotDescription for Adapted {
        const SIMULATION_MODELS: &'static [SimulationModel] = &[
            SimulationModel::new(ModelFormat::Mjcf, "portable.xml"),
            SimulationModel::new(ModelFormat::Urdf, "adapted.urdf").for_backend("mujoco"),
        ];
    }
    let selected = MujocoLoader::select_robot_model::<Adapted>().unwrap();
    assert_eq!(selected.model.path, "adapted.urdf");
    assert_eq!(selected.source, ModelSource::BackendSpecific);
}

#[test]
fn empty_loader_or_description_returns_an_error_without_panicking() {
    struct NoModel;
    impl RobotDescription for NoModel {}
    assert!(BulletLoader::select_robot_model::<NoModel>().is_err());
    assert!(select_simulation_model::<MultiBackendArm>("none", &[]).is_err());
}
