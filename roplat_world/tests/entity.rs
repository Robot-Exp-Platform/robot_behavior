//! URDF shapes become world entities with the entity's size conventions.
use roplat_description::Geometry;
use roplat_world::Entity;

#[test]
fn a_urdf_box_becomes_half_extents() {
    let urdf = Geometry::Box { size: [0.2, 0.4, 0.6] };
    let Entity::Box { half_extents } = Entity::from(&urdf) else {
        panic!("a box stays a box");
    };
    assert_eq!(half_extents, [0.1, 0.2, 0.3]);
}

#[test]
fn a_urdf_cylinder_length_becomes_its_height() {
    let urdf = Geometry::Cylinder { radius: 0.05, length: 0.3 };
    let Entity::Cylinder { radius, height } = Entity::from(&urdf) else {
        panic!("a cylinder stays a cylinder");
    };
    assert_eq!((radius, height), (0.05, 0.3));
}

#[test]
fn a_urdf_mesh_keeps_its_file_and_scale() {
    let urdf = Geometry::Mesh { filename: "meshes/link0.stl".into(), scale: [1.0, 1.0, 2.0] };
    let Entity::MeshFile { file, scale } = Entity::from(&urdf) else {
        panic!("a mesh stays a mesh file");
    };
    assert_eq!((file, scale), ("meshes/link0.stl", [1.0, 1.0, 2.0]));
}
