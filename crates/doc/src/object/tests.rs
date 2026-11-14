use super::*;

#[test]
fn ids_names_and_removal() {
    let mut d = Document::new();
    let a = d.add_mesh("scan", Mesh::default());
    let b = d.add_mesh("scan", Mesh::default());
    assert_ne!(a, b);
    assert_eq!(d.mesh(b).unwrap().name, "scan 2");
    assert_eq!(d.find_by_name("scan 2"), Some((b, ObjectKind::Mesh)));
    let r = d.add_region("region", b, vec![3, 1, 3]);
    assert_eq!(*d.region(r).unwrap().triangles, vec![1, 3]);
    assert!(d.remove(a));
    assert!(!d.remove(a));
    assert_eq!(d.summaries().len(), 2);
    assert!(d.rename(b, "part"));
    assert_eq!(d.kind_of(b), Some(ObjectKind::Mesh));
    assert_eq!(d.kind_of(r), Some(ObjectKind::Region));
    // removing the mesh removes its regions
    assert!(d.remove(b));
    assert!(d.summaries().is_empty());
}
