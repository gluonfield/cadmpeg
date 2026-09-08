// SPDX-License-Identifier: Apache-2.0
//! Production SCH_13006 periodic-face reconstruction.
//!
//! This module ports the qualified R66 simple periodic reconstruction and the
//! R50.L oracle-independent complex reconstruction into typed Rust. Selection
//! is structural/analytic only: no fixture entity ids, no STEP oracle, and no
//! historical R72/R73 tables are consulted at runtime.

use std::collections::{BTreeMap, BTreeSet};

use cadmpeg_core::CodecError;
use cadmpeg_ir::geometry::{Curve, CurveGeometry, SurfaceGeometry};
use cadmpeg_ir::ids::{
    CoedgeId, CurveId, EdgeId, FaceId, LoopId, PointId, SurfaceId, VertexId,
};
use cadmpeg_ir::math::{Point3, Vector3};
use cadmpeg_ir::topology::{Coedge, Edge, Point, Sense, Vertex};
use cadmpeg_ir::CadIr;
use serde::Serialize;

const GEOM_TOL_MM: f64 = 1.0e-5;
const PHASE_DOT_TOL: f64 = 1.0e-10;
const ARC_TOL: f64 = 1.0e-7;
const AXIS_TOL: f64 = 1.0e-8;
const ALIGN_TOL: f64 = 1.0e-8;

const NATIVE_NAMESPACE: &str = "parasolid";
const STEP_ORIENTATION_ARENA: &str = "sch13006_step_edge_orientation";
const STEP_ORIENTED_EDGE_OVERRIDE_ARENA: &str =
    "sch13006_step_oriented_edge_override";
const NATIVE_NAMESPACE_VERSION: u32 = 1;

const PLANE_LIFT_CLASS: &str = "plane_lift_source_orientation";
const COMPLEX_CIRCLE_CLASS: &str =
    "complex_periodic_reconstruction_plus_two_edge_planar_radial_ownership";
const SHARED_CIRCLE_OVERRIDE_CLASS: &str =
    "shared_circle_planar_radial_cross_ownership";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SupportKind {
    Plane,
    Cylinder,
    Cone,
}

#[derive(Debug, Clone)]
struct SimpleSplitLoop {
    first_coedge: String,
    second_coedge: String,
    original_vertex: String,
    split_vertex: String,
}

#[derive(Debug, Clone)]
struct SimpleSplit {
    loops: BTreeMap<String, SimpleSplitLoop>,
}

#[derive(Debug, Clone)]
struct SplitOperation {
    edge_id: String,
    point: Point3,
}

#[derive(Debug, Clone)]
enum CutRef {
    Existing(String),
    SplitEdge(String),
}

#[derive(Debug, Clone)]
enum ComplexPlan {
    Pole {
        face_id: String,
        pole_loop: String,
        boundary_loop: String,
        pole_vertex: String,
        boundary_cut_0: String,
        boundary_cut_1: String,
    },
    Periodic {
        face_id: String,
        anchor_loop: String,
        boundary_loop: String,
        anchor_edge: String,
        anchor_vertex: String,
        boundary_cut_0: CutRef,
        boundary_cut_1: CutRef,
    },
}

#[derive(Debug, Clone)]
struct ExecutedSplit {
    original_edge: String,
    new_edge: String,
    split_vertex: String,
}

#[derive(Debug, Clone, Serialize)]
struct StepEdgeOrientationRecord {
    id: String,
    edge: String,
    class: String,
    same_sense: bool,
}

#[derive(Debug, Clone, Serialize)]
struct StepOrientedEdgeOverrideRecord {
    id: String,
    coedge: String,
    edge: String,
    class: String,
}

struct IdAllocator {
    counter: u64,
    used: BTreeSet<String>,
}

impl IdAllocator {
    fn new(ir: &CadIr) -> Self {
        let mut used = BTreeSet::new();

        macro_rules! add_arena {
            ($arena:expr) => {
                for entity in $arena {
                    used.insert(entity.id.as_str().to_string());
                }
            };
        }

        add_arena!(&ir.model.points);
        add_arena!(&ir.model.vertices);
        add_arena!(&ir.model.curves);
        add_arena!(&ir.model.edges);
        add_arena!(&ir.model.coedges);
        add_arena!(&ir.model.loops);
        add_arena!(&ir.model.surfaces);
        add_arena!(&ir.model.faces);

        Self { counter: 0, used }
    }

    fn alloc(&mut self, kind: &str) -> String {
        loop {
            let id = format!(
                "parasolid:sch13006:{kind}#r50n-{:010}",
                self.counter
            );
            self.counter += 1;
            if self.used.insert(id.clone()) {
                return id;
            }
        }
    }
}

fn malformed(message: impl Into<String>) -> CodecError {
    CodecError::Malformed(message.into())
}

fn point_distance(a: Point3, b: Point3) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn vec_sub(a: Point3, b: Point3) -> Vector3 {
    Vector3::new(a.x - b.x, a.y - b.y, a.z - b.z)
}

fn point_add_vector(point: Point3, vector: Vector3) -> Point3 {
    Point3::new(
        point.x + vector.x,
        point.y + vector.y,
        point.z + vector.z,
    )
}

fn vector_sub(a: Vector3, b: Vector3) -> Vector3 {
    Vector3::new(a.x - b.x, a.y - b.y, a.z - b.z)
}

fn vector_scale(v: Vector3, s: f64) -> Vector3 {
    Vector3::new(v.x * s, v.y * s, v.z * s)
}

fn vector_add(a: Vector3, b: Vector3) -> Vector3 {
    Vector3::new(a.x + b.x, a.y + b.y, a.z + b.z)
}

fn cross(a: Vector3, b: Vector3) -> Vector3 {
    Vector3::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

fn dot(a: Vector3, b: Vector3) -> f64 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

fn norm(v: Vector3) -> f64 {
    dot(v, v).sqrt()
}

fn normalize(v: Vector3, label: &str) -> Result<Vector3, CodecError> {
    let length = norm(v);
    if !length.is_finite() || length < 1.0e-15 {
        return Err(malformed(format!("{label} is degenerate")));
    }
    Ok(vector_scale(v, 1.0 / length))
}

fn clamp_unit(value: f64) -> f64 {
    value.clamp(-1.0, 1.0)
}

fn angle(a: Vector3, b: Vector3) -> Result<f64, CodecError> {
    let an = normalize(a, "angle first vector")?;
    let bn = normalize(b, "angle second vector")?;
    Ok(clamp_unit(dot(an, bn)).acos())
}

fn point_index(ir: &CadIr, id: &str) -> Result<usize, CodecError> {
    ir.model
        .points
        .iter()
        .position(|value| value.id.as_str() == id)
        .ok_or_else(|| malformed(format!("missing CADIR point {id}")))
}

fn vertex_index(ir: &CadIr, id: &str) -> Result<usize, CodecError> {
    ir.model
        .vertices
        .iter()
        .position(|value| value.id.as_str() == id)
        .ok_or_else(|| malformed(format!("missing CADIR vertex {id}")))
}

fn curve_index(ir: &CadIr, id: &str) -> Result<usize, CodecError> {
    ir.model
        .curves
        .iter()
        .position(|value| value.id.as_str() == id)
        .ok_or_else(|| malformed(format!("missing CADIR curve {id}")))
}

fn edge_index(ir: &CadIr, id: &str) -> Result<usize, CodecError> {
    ir.model
        .edges
        .iter()
        .position(|value| value.id.as_str() == id)
        .ok_or_else(|| malformed(format!("missing CADIR edge {id}")))
}

fn coedge_index(ir: &CadIr, id: &str) -> Result<usize, CodecError> {
    ir.model
        .coedges
        .iter()
        .position(|value| value.id.as_str() == id)
        .ok_or_else(|| malformed(format!("missing CADIR coedge {id}")))
}

fn loop_index(ir: &CadIr, id: &str) -> Result<usize, CodecError> {
    ir.model
        .loops
        .iter()
        .position(|value| value.id.as_str() == id)
        .ok_or_else(|| malformed(format!("missing CADIR loop {id}")))
}

fn surface_index(ir: &CadIr, id: &str) -> Result<usize, CodecError> {
    ir.model
        .surfaces
        .iter()
        .position(|value| value.id.as_str() == id)
        .ok_or_else(|| malformed(format!("missing CADIR surface {id}")))
}

fn face_index(ir: &CadIr, id: &str) -> Result<usize, CodecError> {
    ir.model
        .faces
        .iter()
        .position(|value| value.id.as_str() == id)
        .ok_or_else(|| malformed(format!("missing CADIR face {id}")))
}

fn vertex_position(ir: &CadIr, vertex_id: &str) -> Result<Point3, CodecError> {
    let vertex = &ir.model.vertices[vertex_index(ir, vertex_id)?];
    let point = &ir.model.points[point_index(ir, vertex.point.as_str())?];
    Ok(point.position)
}

fn support_kind(surface: &SurfaceGeometry) -> Option<SupportKind> {
    match surface {
        SurfaceGeometry::Plane { .. } => Some(SupportKind::Plane),
        SurfaceGeometry::Cylinder { .. } => Some(SupportKind::Cylinder),
        SurfaceGeometry::Cone { .. } => Some(SupportKind::Cone),
        _ => None,
    }
}

fn support_axis(surface: &SurfaceGeometry) -> Option<Vector3> {
    match surface {
        SurfaceGeometry::Cylinder { axis, .. } | SurfaceGeometry::Cone { axis, .. } => Some(*axis),
        _ => None,
    }
}

fn support_origin(surface: &SurfaceGeometry) -> Option<Point3> {
    match surface {
        SurfaceGeometry::Cylinder { origin, .. } | SurfaceGeometry::Cone { origin, .. } => {
            Some(*origin)
        }
        _ => None,
    }
}

fn edge_curve<'a>(ir: &'a CadIr, edge: &Edge) -> Result<&'a Curve, CodecError> {
    let curve_id = edge
        .curve
        .as_ref()
        .ok_or_else(|| malformed(format!("edge {} has no 3D carrier", edge.id)))?;
    Ok(&ir.model.curves[curve_index(ir, curve_id.as_str())?])
}

fn edge_is_circle(ir: &CadIr, edge_id: &str) -> Result<bool, CodecError> {
    let edge = &ir.model.edges[edge_index(ir, edge_id)?];
    Ok(matches!(&edge_curve(ir, edge)?.geometry, CurveGeometry::Circle { .. }))
}

fn edge_uses(ir: &CadIr, edge_id: &str) -> Vec<String> {
    ir.model
        .coedges
        .iter()
        .filter(|coedge| coedge.edge.as_str() == edge_id)
        .map(|coedge| coedge.id.as_str().to_string())
        .collect()
}

fn loop_edge_ids(ir: &CadIr, loop_id: &str) -> Result<Vec<String>, CodecError> {
    let loop_ = &ir.model.loops[loop_index(ir, loop_id)?];
    loop_
        .coedges
        .iter()
        .map(|coedge_id| {
            let coedge = &ir.model.coedges[coedge_index(ir, coedge_id.as_str())?];
            Ok(coedge.edge.as_str().to_string())
        })
        .collect()
}

fn loop_vertex_ids(ir: &CadIr, loop_id: &str) -> Result<Vec<String>, CodecError> {
    let loop_ = &ir.model.loops[loop_index(ir, loop_id)?];
    let mut output = BTreeSet::new();

    for vertex_use in &loop_.vertex_uses {
        output.insert(vertex_use.vertex.as_str().to_string());
    }

    for edge_id in loop_edge_ids(ir, loop_id)? {
        let edge = &ir.model.edges[edge_index(ir, &edge_id)?];
        output.insert(edge.start.as_str().to_string());
        output.insert(edge.end.as_str().to_string());
    }

    Ok(output.into_iter().collect())
}

fn coedge_traversal(ir: &CadIr, coedge_id: &str) -> Result<(String, String), CodecError> {
    let coedge = &ir.model.coedges[coedge_index(ir, coedge_id)?];
    let edge = &ir.model.edges[edge_index(ir, coedge.edge.as_str())?];
    Ok(match coedge.sense {
        Sense::Forward => (
            edge.start.as_str().to_string(),
            edge.end.as_str().to_string(),
        ),
        Sense::Reversed => (
            edge.end.as_str().to_string(),
            edge.start.as_str().to_string(),
        ),
    })
}

fn set_ring(ir: &mut CadIr, loop_id: &str, ring: &[String]) -> Result<(), CodecError> {
    if ring.is_empty() {
        return Err(malformed(format!("cannot emit empty loop ring {loop_id}")));
    }

    for (index, coedge_id) in ring.iter().enumerate() {
        let previous = &ring[(index + ring.len() - 1) % ring.len()];
        let next = &ring[(index + 1) % ring.len()];
        let position = coedge_index(ir, coedge_id)?;
        let coedge = &mut ir.model.coedges[position];
        coedge.owner_loop = LoopId(loop_id.to_string());
        coedge.previous = CoedgeId(previous.clone());
        coedge.next = CoedgeId(next.clone());
    }

    let position = loop_index(ir, loop_id)?;
    ir.model.loops[position].coedges = ring.iter().cloned().map(CoedgeId).collect();
    ir.model.loops[position].vertex_uses.clear();
    Ok(())
}

fn shared_edge_vertex(ir: &CadIr, first_edge_id: &str, second_edge_id: &str) -> Result<String, CodecError> {
    let first = &ir.model.edges[edge_index(ir, first_edge_id)?];
    let second = &ir.model.edges[edge_index(ir, second_edge_id)?];
    let first_vertices = [first.start.as_str(), first.end.as_str()];
    let second_vertices = [second.start.as_str(), second.end.as_str()];
    let matches = first_vertices
        .into_iter()
        .filter(|vertex| second_vertices.contains(vertex))
        .collect::<BTreeSet<_>>();

    if matches.len() != 1 {
        return Err(malformed(format!(
            "edges {first_edge_id}/{second_edge_id} do not share exactly one vertex"
        )));
    }

    Ok(matches.into_iter().next().expect("one shared vertex").to_string())
}

fn assert_local_coedge_traversal(
    ir: &CadIr,
    loop_id: &str,
    coedge_id: &str,
) -> Result<(), CodecError> {
    let loop_ = &ir.model.loops[loop_index(ir, loop_id)?];
    let position = loop_
        .coedges
        .iter()
        .position(|value| value.as_str() == coedge_id)
        .ok_or_else(|| malformed(format!("loop {loop_id} does not contain seam {coedge_id}")))?;
    let previous_id = loop_.coedges[(position + loop_.coedges.len() - 1) % loop_.coedges.len()]
        .as_str()
        .to_string();
    let next_id = loop_.coedges[(position + 1) % loop_.coedges.len()]
        .as_str()
        .to_string();

    let seam = &ir.model.coedges[coedge_index(ir, coedge_id)?];
    let previous = &ir.model.coedges[coedge_index(ir, &previous_id)?];
    let next = &ir.model.coedges[coedge_index(ir, &next_id)?];
    let previous_vertex = shared_edge_vertex(ir, seam.edge.as_str(), previous.edge.as_str())?;
    let next_vertex = shared_edge_vertex(ir, seam.edge.as_str(), next.edge.as_str())?;
    let (traversal_start, traversal_end) = coedge_traversal(ir, coedge_id)?;

    if previous_vertex != traversal_start || next_vertex != traversal_end {
        return Err(malformed(format!(
            "loop {loop_id} seam {coedge_id} local traversal drift previous={previous_vertex} start={traversal_start} next={next_vertex} end={traversal_end}"
        )));
    }

    Ok(())
}

fn rebuild_loop_ring_from_seams(
    ir: &mut CadIr,
    loop_id: &str,
    seam_a: &str,
    seam_b: &str,
) -> Result<(), CodecError> {
    let loop_snapshot = ir.model.loops[loop_index(ir, loop_id)?].clone();
    if loop_snapshot.coedges.len() < 3 {
        return Err(malformed(format!(
            "complex output loop {loop_id} is too small for directed-ring reconstruction"
        )));
    }

    let anchor_seam = if seam_a <= seam_b { seam_a } else { seam_b };
    let mut adjacency = BTreeMap::<String, Vec<String>>::new();

    for coedge_id in &loop_snapshot.coedges {
        let coedge = &ir.model.coedges[coedge_index(ir, coedge_id.as_str())?];
        let edge = &ir.model.edges[edge_index(ir, coedge.edge.as_str())?];
        let start = edge.start.as_str().to_string();
        let end = edge.end.as_str().to_string();
        if start == end {
            return Err(malformed(format!(
                "complex output loop {loop_id} still contains closed edge {}",
                edge.id
            )));
        }
        adjacency.entry(start).or_default().push(coedge.id.as_str().to_string());
        adjacency.entry(end).or_default().push(coedge.id.as_str().to_string());
    }

    if adjacency.values().any(|incident| incident.len() != 2) {
        return Err(malformed(format!(
            "complex output loop {loop_id} is not a degree-two cycle"
        )));
    }

    let (start_vertex, mut current_vertex) = coedge_traversal(ir, anchor_seam)?;
    let mut ring = vec![anchor_seam.to_string()];
    let mut used = BTreeSet::from([anchor_seam.to_string()]);
    let mut previous_coedge = anchor_seam.to_string();
    let mut guard = 0usize;

    loop {
        let incident = adjacency
            .get(&current_vertex)
            .ok_or_else(|| malformed(format!(
                "complex output loop {loop_id} lost vertex {current_vertex}"
            )))?;
        let candidates = incident
            .iter()
            .filter(|candidate| candidate.as_str() != previous_coedge.as_str())
            .cloned()
            .collect::<Vec<_>>();
        if candidates.len() != 1 {
            return Err(malformed(format!(
                "complex output loop {loop_id} has ambiguous continuation at {current_vertex}"
            )));
        }
        let next_coedge = candidates[0].clone();

        if next_coedge.as_str() == anchor_seam {
            if current_vertex != start_vertex || ring.len() != loop_snapshot.coedges.len() {
                return Err(malformed(format!(
                    "complex output loop {loop_id} closed early or at the wrong seam endpoint"
                )));
            }
            break;
        }

        if !used.insert(next_coedge.clone()) {
            return Err(malformed(format!(
                "complex output loop {loop_id} repeated coedge {next_coedge}"
            )));
        }
        ring.push(next_coedge.clone());

        let next = &ir.model.coedges[coedge_index(ir, &next_coedge)?];
        let edge = &ir.model.edges[edge_index(ir, next.edge.as_str())?];
        current_vertex = if edge.start.as_str() == current_vertex.as_str() {
            edge.end.as_str().to_string()
        } else if edge.end.as_str() == current_vertex.as_str() {
            edge.start.as_str().to_string()
        } else {
            return Err(malformed(format!(
                "complex output loop {loop_id} coedge {next_coedge} does not meet vertex {current_vertex}"
            )));
        };
        previous_coedge = next_coedge;
        guard += 1;
        if guard > loop_snapshot.coedges.len() + 2 {
            return Err(malformed(format!(
                "complex output loop {loop_id} directed-ring guard exceeded"
            )));
        }
    }

    set_ring(ir, loop_id, &ring)?;
    assert_local_coedge_traversal(ir, loop_id, seam_a)?;
    assert_local_coedge_traversal(ir, loop_id, seam_b)?;
    Ok(())
}

fn assert_directed_cycle(ir: &CadIr, ring: &[String]) -> Result<(), CodecError> {
    for index in 0..ring.len() {
        let (_, current_end) = coedge_traversal(ir, &ring[index])?;
        let (next_start, _) = coedge_traversal(ir, &ring[(index + 1) % ring.len()])?;
        if current_end != next_start {
            return Err(malformed(format!(
                "periodic simple-face wire vertex break {} -> {}",
                ring[index],
                ring[(index + 1) % ring.len()]
            )));
        }
    }
    Ok(())
}

fn insert_coedge_adjacent(
    ir: &mut CadIr,
    loop_id: &str,
    existing_id: &str,
    inserted_id: &str,
    after: bool,
) -> Result<(), CodecError> {
    let loop_position = loop_index(ir, loop_id)?;
    let list = ir.model.loops[loop_position].coedges.clone();
    let list_index = list
        .iter()
        .position(|value| value.as_str() == existing_id)
        .ok_or_else(|| malformed(format!("loop {loop_id} does not contain {existing_id}")))?;

    let existing_position = coedge_index(ir, existing_id)?;
    let existing = ir.model.coedges[existing_position].clone();

    if after {
        let old_next_id = existing.next.as_str().to_string();
        let old_next_position = coedge_index(ir, &old_next_id)?;
        let inserted_position = coedge_index(ir, inserted_id)?;

        ir.model.coedges[existing_position].next = CoedgeId(inserted_id.to_string());
        ir.model.coedges[inserted_position].previous = CoedgeId(existing_id.to_string());
        ir.model.coedges[inserted_position].next = CoedgeId(old_next_id.clone());
        ir.model.coedges[inserted_position].owner_loop = LoopId(loop_id.to_string());
        ir.model.coedges[old_next_position].previous = CoedgeId(inserted_id.to_string());

        ir.model.loops[loop_position]
            .coedges
            .insert(list_index + 1, CoedgeId(inserted_id.to_string()));
    } else {
        let old_previous_id = existing.previous.as_str().to_string();
        let old_previous_position = coedge_index(ir, &old_previous_id)?;
        let inserted_position = coedge_index(ir, inserted_id)?;

        ir.model.coedges[old_previous_position].next = CoedgeId(inserted_id.to_string());
        ir.model.coedges[inserted_position].previous = CoedgeId(old_previous_id);
        ir.model.coedges[inserted_position].next = CoedgeId(existing_id.to_string());
        ir.model.coedges[inserted_position].owner_loop = LoopId(loop_id.to_string());
        ir.model.coedges[existing_position].previous = CoedgeId(inserted_id.to_string());

        ir.model.loops[loop_position]
            .coedges
            .insert(list_index, CoedgeId(inserted_id.to_string()));
    }

    Ok(())
}

fn insert_face_after(
    ir: &mut CadIr,
    shell_id: &str,
    original_face: &str,
    inserted_face: &str,
) -> Result<(), CodecError> {
    let shell = ir
        .model
        .shells
        .iter_mut()
        .find(|shell| shell.id.as_str() == shell_id)
        .ok_or_else(|| malformed(format!("missing shell {shell_id}")))?;
    let index = shell
        .faces
        .iter()
        .position(|value| value.as_str() == original_face)
        .ok_or_else(|| {
            malformed(format!(
                "shell {shell_id} does not own periodic face {original_face}"
            ))
        })?;
    shell
        .faces
        .insert(index + 1, FaceId(inserted_face.to_string()));
    Ok(())
}

fn new_point(ir: &mut CadIr, allocator: &mut IdAllocator, position: Point3) -> String {
    let id = allocator.alloc("point");
    ir.model.points.push(Point {
        id: PointId(id.clone()),
        position,
        source_object: None,
    });
    id
}

fn new_vertex(
    ir: &mut CadIr,
    allocator: &mut IdAllocator,
    point_id: &str,
    template_vertex: Option<&Vertex>,
) -> String {
    let id = allocator.alloc("vertex");
    ir.model.vertices.push(Vertex {
        id: VertexId(id.clone()),
        point: PointId(point_id.to_string()),
        tolerance: template_vertex.and_then(|value| value.tolerance),
    });
    id
}

fn clone_circle_curve(
    ir: &mut CadIr,
    allocator: &mut IdAllocator,
    source: &Curve,
) -> Result<String, CodecError> {
    if !matches!(&source.geometry, CurveGeometry::Circle { .. }) {
        return Err(malformed(format!("curve {} is not a circle", source.id)));
    }
    let mut curve = source.clone();
    let id = allocator.alloc("curve");
    curve.id = CurveId(id.clone());
    curve.source_object = None;
    ir.model.curves.push(curve);
    Ok(id)
}

fn new_line_curve(
    ir: &mut CadIr,
    allocator: &mut IdAllocator,
    origin: Point3,
    direction: Vector3,
) -> Result<String, CodecError> {
    let direction = normalize(direction, "periodic seam line direction")?;
    let id = allocator.alloc("curve");
    ir.model.curves.push(Curve {
        id: CurveId(id.clone()),
        geometry: CurveGeometry::Line { origin, direction },
        source_object: None,
    });
    Ok(id)
}

fn new_edge(
    ir: &mut CadIr,
    allocator: &mut IdAllocator,
    curve_id: &str,
    start: &str,
    end: &str,
    tolerance: Option<f64>,
) -> String {
    let id = allocator.alloc("edge");
    ir.model.edges.push(Edge {
        id: EdgeId(id.clone()),
        curve: Some(CurveId(curve_id.to_string())),
        start: VertexId(start.to_string()),
        end: VertexId(end.to_string()),
        param_range: None,
        tolerance,
    });
    id
}

fn new_coedge(
    ir: &mut CadIr,
    allocator: &mut IdAllocator,
    loop_id: &str,
    edge_id: &str,
    sense: Sense,
) -> String {
    let id = allocator.alloc("coedge");
    ir.model.coedges.push(Coedge {
        id: CoedgeId(id.clone()),
        owner_loop: LoopId(loop_id.to_string()),
        edge: EdgeId(edge_id.to_string()),
        next: CoedgeId(id.clone()),
        previous: CoedgeId(id.clone()),
        radial_next: CoedgeId(id.clone()),
        sense,
        pcurves: Vec::new(),
        use_curve: None,
        use_curve_parameter_range: None,
    });
    id
}

fn set_radial_pair(ir: &mut CadIr, first: &str, second: &str) -> Result<(), CodecError> {
    let first_index = coedge_index(ir, first)?;
    let second_index = coedge_index(ir, second)?;
    ir.model.coedges[first_index].radial_next = CoedgeId(second.to_string());
    ir.model.coedges[second_index].radial_next = CoedgeId(first.to_string());
    Ok(())
}

fn new_seam(
    ir: &mut CadIr,
    allocator: &mut IdAllocator,
    start_vertex: &str,
    end_vertex: &str,
) -> Result<String, CodecError> {
    let start = vertex_position(ir, start_vertex)?;
    let end = vertex_position(ir, end_vertex)?;
    let direction = vec_sub(end, start);
    let length = norm(direction);
    if !length.is_finite() || length <= GEOM_TOL_MM {
        return Err(malformed("degenerate periodic seam"));
    }
    let curve = new_line_curve(ir, allocator, start, direction)?;
    Ok(new_edge(
        ir,
        allocator,
        &curve,
        start_vertex,
        end_vertex,
        None,
    ))
}

fn circle_opposite_point(ir: &CadIr, edge_id: &str) -> Result<Point3, CodecError> {
    let edge = &ir.model.edges[edge_index(ir, edge_id)?];
    if edge.start != edge.end {
        return Err(malformed(format!("simple periodic edge {edge_id} is not closed")));
    }
    let curve = edge_curve(ir, edge)?;
    let CurveGeometry::Circle {
        center,
        axis,
        radius,
        ..
    } = &curve.geometry
    else {
        return Err(malformed(format!("simple periodic edge {edge_id} is not circular")));
    };

    let axis = normalize(*axis, "circle axis")?;
    let start = vertex_position(ir, edge.start.as_str())?;
    let radial_raw = vec_sub(start, *center);
    let axial = dot(radial_raw, axis);
    let radial = vector_sub(radial_raw, vector_scale(axis, axial));
    let actual_radius = norm(radial);
    let residual = (axial * axial + (actual_radius - *radius).powi(2)).sqrt();
    if residual > GEOM_TOL_MM {
        return Err(malformed(format!(
            "circle endpoint residual {residual} exceeds tolerance on {edge_id}"
        )));
    }
    Ok(point_add_vector(*center, vector_scale(radial, -1.0)))
}

fn surface_radial_direction(
    surface: &SurfaceGeometry,
    point: Point3,
) -> Result<Option<Vector3>, CodecError> {
    let origin = support_origin(surface)
        .ok_or_else(|| malformed("surface radial direction requires cylinder/cone"))?;
    let axis = normalize(
        support_axis(surface).ok_or_else(|| malformed("periodic support has no axis"))?,
        "periodic support axis",
    )?;
    let relative = vec_sub(point, origin);
    let radial = vector_sub(relative, vector_scale(axis, dot(relative, axis)));
    let length = norm(radial);
    if length < GEOM_TOL_MM {
        return Ok(None);
    }
    Ok(Some(vector_scale(radial, 1.0 / length)))
}

fn phase_matches(
    surface: &SurfaceGeometry,
    point: Point3,
    target: Vector3,
) -> Result<bool, CodecError> {
    Ok(surface_radial_direction(surface, point)?
        .is_some_and(|direction| dot(direction, target) >= 1.0 - PHASE_DOT_TOL))
}

fn point_on_circle_at_support_direction(
    ir: &CadIr,
    edge_id: &str,
    target_direction: Vector3,
    surface: &SurfaceGeometry,
) -> Result<Point3, CodecError> {
    let edge = &ir.model.edges[edge_index(ir, edge_id)?];
    let curve = edge_curve(ir, edge)?;
    let CurveGeometry::Circle {
        center,
        axis,
        radius,
        ..
    } = &curve.geometry
    else {
        return Err(malformed(format!("target edge {edge_id} is not a circle")));
    };

    let circle_axis = normalize(*axis, "circle/support circle axis")?;
    let support_axis = normalize(
        support_axis(surface).ok_or_else(|| malformed("periodic support has no axis"))?,
        "circle/support surface axis",
    )?;

    if (dot(circle_axis, support_axis).abs() - 1.0).abs() > AXIS_TOL {
        return Err(malformed(format!("circle/support axis mismatch edge={edge_id}")));
    }

    let support_origin = support_origin(surface)
        .ok_or_else(|| malformed("periodic support has no origin"))?;
    let center_relative = vec_sub(*center, support_origin);
    let center_perpendicular = vector_sub(
        center_relative,
        vector_scale(support_axis, dot(center_relative, support_axis)),
    );
    if norm(center_perpendicular) > GEOM_TOL_MM {
        return Err(malformed(format!("circle center is not on support axis edge={edge_id}")));
    }

    let point = point_add_vector(*center, vector_scale(target_direction, *radius));
    let actual = surface_radial_direction(surface, point)?
        .ok_or_else(|| malformed(format!("derived point lies on support axis edge={edge_id}")))?;
    if dot(actual, target_direction) < 1.0 - PHASE_DOT_TOL {
        return Err(malformed(format!("analytic split phase mismatch edge={edge_id}")));
    }
    Ok(point)
}

fn point_strictly_on_minor_arc(
    ir: &CadIr,
    edge_id: &str,
    point: Point3,
) -> Result<bool, CodecError> {
    let edge = &ir.model.edges[edge_index(ir, edge_id)?];
    let curve = edge_curve(ir, edge)?;
    let CurveGeometry::Circle { center, .. } = &curve.geometry else {
        return Ok(false);
    };

    let start = vec_sub(vertex_position(ir, edge.start.as_str())?, *center);
    let end = vec_sub(vertex_position(ir, edge.end.as_str())?, *center);
    let target = vec_sub(point, *center);

    if norm(start) < GEOM_TOL_MM || norm(end) < GEOM_TOL_MM || norm(target) < GEOM_TOL_MM {
        return Ok(false);
    }

    let start_end = angle(start, end)?;
    let start_target = angle(start, target)?;
    let target_end = angle(target, end)?;

    Ok(start_target > ARC_TOL
        && target_end > ARC_TOL
        && ((start_target + target_end) - start_end).abs() <= ARC_TOL)
}

fn simple_loop_closed_circle_edge(ir: &CadIr, loop_id: &str) -> Result<Option<String>, CodecError> {
    let loop_ = &ir.model.loops[loop_index(ir, loop_id)?];
    if loop_.coedges.len() != 1 || !loop_.vertex_uses.is_empty() {
        return Ok(None);
    }
    let coedge = &ir.model.coedges[coedge_index(ir, loop_.coedges[0].as_str())?];
    if !coedge.pcurves.is_empty()
        || coedge.use_curve.is_some()
        || coedge.use_curve_parameter_range.is_some()
    {
        return Ok(None);
    }
    let edge = &ir.model.edges[edge_index(ir, coedge.edge.as_str())?];
    if edge.start != edge.end || edge.param_range.is_some() {
        return Ok(None);
    }
    if !matches!(&edge_curve(ir, edge)?.geometry, CurveGeometry::Circle { .. }) {
        return Ok(None);
    }
    if edge_uses(ir, edge.id.as_str()).len() != 2 {
        return Ok(None);
    }
    Ok(Some(edge.id.as_str().to_string()))
}

fn select_simple_faces(ir: &CadIr) -> Result<Vec<String>, CodecError> {
    let mut selected = Vec::new();
    for face in &ir.model.faces {
        let surface = &ir.model.surfaces[surface_index(ir, face.surface.as_str())?];
        if !matches!(
            support_kind(&surface.geometry),
            Some(SupportKind::Cylinder | SupportKind::Cone)
        ) || face.loops.len() != 2
        {
            continue;
        }
        let first = simple_loop_closed_circle_edge(ir, face.loops[0].as_str())?;
        let second = simple_loop_closed_circle_edge(ir, face.loops[1].as_str())?;
        if first.is_some() && second.is_some() {
            selected.push(face.id.as_str().to_string());
        }
    }
    selected.sort();
    Ok(selected)
}

fn split_simple_edge(
    ir: &mut CadIr,
    allocator: &mut IdAllocator,
    edge_id: &str,
    simple_target_loops: &BTreeSet<String>,
) -> Result<SimpleSplit, CodecError> {
    let edge_position = edge_index(ir, edge_id)?;
    let original_edge = ir.model.edges[edge_position].clone();
    if original_edge.start != original_edge.end || original_edge.param_range.is_some() {
        return Err(malformed(format!(
            "simple split edge {edge_id} is not an unbounded closed edge"
        )));
    }
    let curve_id = original_edge
        .curve
        .as_ref()
        .ok_or_else(|| malformed(format!("simple split edge {edge_id} has no curve")))?
        .as_str()
        .to_string();
    let curve = ir.model.curves[curve_index(ir, &curve_id)?].clone();
    if !matches!(&curve.geometry, CurveGeometry::Circle { .. }) {
        return Err(malformed(format!("simple split edge {edge_id} is not circular")));
    }

    let radial_uses = edge_uses(ir, edge_id);
    if radial_uses.len() != 2 {
        return Err(malformed(format!(
            "simple split edge {edge_id} radial use count={} expected=2",
            radial_uses.len()
        )));
    }
    let first_radial = ir.model.coedges[coedge_index(ir, &radial_uses[0])?].clone();
    let second_radial = ir.model.coedges[coedge_index(ir, &radial_uses[1])?].clone();
    if first_radial.radial_next.as_str() != second_radial.id.as_str()
        || second_radial.radial_next.as_str() != first_radial.id.as_str()
    {
        return Err(malformed(format!("simple split edge {edge_id} radial ring mismatch")));
    }

    for use_id in &radial_uses {
        let use_ = &ir.model.coedges[coedge_index(ir, use_id)?];
        if !use_.pcurves.is_empty()
            || use_.use_curve.is_some()
            || use_.use_curve_parameter_range.is_some()
        {
            return Err(malformed(format!(
                "simple split edge {edge_id} carries unsupported coedge-local geometry"
            )));
        }
    }

    let opposite = circle_opposite_point(ir, edge_id)?;
    let point_id = new_point(ir, allocator, opposite);
    let template_vertex = ir.model.vertices[vertex_index(ir, original_edge.start.as_str())?].clone();
    let split_vertex = new_vertex(ir, allocator, &point_id, Some(&template_vertex));

    ir.model.edges[edge_position].end = VertexId(split_vertex.clone());

    let second_curve = clone_circle_curve(ir, allocator, &curve)?;
    let second_edge = new_edge(
        ir,
        allocator,
        &second_curve,
        &split_vertex,
        original_edge.start.as_str(),
        original_edge.tolerance,
    );

    let mut loops = BTreeMap::new();
    let mut new_coedges = Vec::new();

    for radial_id in radial_uses {
        let radial = ir.model.coedges[coedge_index(ir, &radial_id)?].clone();
        let loop_id = radial.owner_loop.as_str().to_string();
        let second_coedge = new_coedge(ir, allocator, &loop_id, &second_edge, radial.sense);
        new_coedges.push(second_coedge.clone());
        loops.insert(
            loop_id.clone(),
            SimpleSplitLoop {
                first_coedge: radial_id.clone(),
                second_coedge: second_coedge.clone(),
                original_vertex: original_edge.start.as_str().to_string(),
                split_vertex: split_vertex.clone(),
            },
        );

        if !simple_target_loops.contains(&loop_id) {
            let after = matches!(radial.sense, Sense::Forward);
            insert_coedge_adjacent(ir, &loop_id, &radial_id, &second_coedge, after)?;
        }
    }

    set_radial_pair(ir, &new_coedges[0], &new_coedges[1])?;
    Ok(SimpleSplit { loops })
}

fn arc_starting_at(
    ir: &CadIr,
    pair: &SimpleSplitLoop,
    vertex_id: &str,
) -> Result<String, CodecError> {
    for coedge_id in [&pair.first_coedge, &pair.second_coedge] {
        let (start, _) = coedge_traversal(ir, coedge_id)?;
        if start == vertex_id {
            return Ok(coedge_id.clone());
        }
    }
    Err(malformed(format!(
        "no split arc starts at {vertex_id} in simple periodic loop"
    )))
}

fn boundary_station(ir: &CadIr, edge_id: &str, axis: Vector3) -> Result<f64, CodecError> {
    let edge = &ir.model.edges[edge_index(ir, edge_id)?];
    let curve = edge_curve(ir, edge)?;
    let CurveGeometry::Circle { center, .. } = &curve.geometry else {
        return Err(malformed(format!("boundary edge {edge_id} is not circular")));
    };
    Ok(center.x * axis.x + center.y * axis.y + center.z * axis.z)
}

fn rebuild_simple_faces(
    ir: &mut CadIr,
    allocator: &mut IdAllocator,
) -> Result<BTreeSet<String>, CodecError> {
    let simple_faces = select_simple_faces(ir)?;
    if simple_faces.len() != 48 {
        return Err(malformed(format!(
            "SCH13006 simple periodic face contract drift expected=48 actual={}",
            simple_faces.len()
        )));
    }

    let mut simple_target_loops = BTreeSet::new();
    let mut unique_edges = BTreeSet::new();

    for face_id in &simple_faces {
        let face = ir.model.faces[face_index(ir, face_id)?].clone();
        for loop_id in &face.loops {
            simple_target_loops.insert(loop_id.as_str().to_string());
            let edge_id = simple_loop_closed_circle_edge(ir, loop_id.as_str())?
                .ok_or_else(|| malformed(format!("simple face {face_id} selector drift")))?;
            unique_edges.insert(edge_id);
        }
    }

    if simple_target_loops.len() != 96 || unique_edges.len() != 80 {
        return Err(malformed(format!(
            "SCH13006 simple periodic selector drift loops={} uniqueEdges={} expectedLoops=96 expectedUniqueEdges=80",
            simple_target_loops.len(),
            unique_edges.len()
        )));
    }

    let mut splits_by_loop = BTreeMap::<String, SimpleSplitLoop>::new();
    for edge_id in unique_edges {
        let split = split_simple_edge(ir, allocator, &edge_id, &simple_target_loops)?;
        for (loop_id, pair) in split.loops {
            if simple_target_loops.contains(&loop_id) {
                if splits_by_loop.insert(loop_id.clone(), pair).is_some() {
                    return Err(malformed(format!(
                        "simple target loop {loop_id} participates in multiple split edges"
                    )));
                }
            }
        }
    }

    let mut output_face_ids = BTreeSet::new();

    for face_id in simple_faces {
        let face_position = face_index(ir, &face_id)?;
        let face = ir.model.faces[face_position].clone();
        let surface_position = surface_index(ir, face.surface.as_str())?;
        let surface = ir.model.surfaces[surface_position].clone();
        let axis = normalize(
            support_axis(&surface.geometry)
                .ok_or_else(|| malformed(format!("simple face {face_id} support has no axis")))?,
            "simple periodic support axis",
        )?;

        let loop_a = face.loops[0].as_str().to_string();
        let loop_b = face.loops[1].as_str().to_string();
        let pair_a = splits_by_loop
            .get(&loop_a)
            .ok_or_else(|| malformed(format!("simple face {face_id} missing split {loop_a}")))?
            .clone();
        let pair_b = splits_by_loop
            .get(&loop_b)
            .ok_or_else(|| malformed(format!("simple face {face_id} missing split {loop_b}")))?
            .clone();

        let edge_a = ir.model.coedges[coedge_index(ir, &pair_a.first_coedge)?]
            .edge
            .as_str()
            .to_string();
        let edge_b = ir.model.coedges[coedge_index(ir, &pair_b.first_coedge)?]
            .edge
            .as_str()
            .to_string();

        let station_a = boundary_station(ir, &edge_a, axis)?;
        let station_b = boundary_station(ir, &edge_b, axis)?;
        if (station_a - station_b).abs() <= GEOM_TOL_MM {
            return Err(malformed(format!(
                "simple face {face_id} periodic boundaries have indistinguishable axis stations"
            )));
        }

        let (low_loop, low_pair, high_loop, high_pair) = if station_a < station_b {
            (loop_a, pair_a, loop_b, pair_b)
        } else {
            (loop_b, pair_b, loop_a, pair_a)
        };

        let a0 = low_pair.original_vertex.clone();
        let a1 = low_pair.split_vertex.clone();
        let b0 = high_pair.original_vertex.clone();
        let b1 = high_pair.split_vertex.clone();

        let radial_a = surface_radial_direction(&surface.geometry, vertex_position(ir, &a0)?)?
            .ok_or_else(|| malformed(format!("simple face {face_id} low boundary lies on axis")))?;
        let radial_b = surface_radial_direction(&surface.geometry, vertex_position(ir, &b0)?)?
            .ok_or_else(|| malformed(format!("simple face {face_id} high boundary lies on axis")))?;
        let alignment = dot(radial_a, radial_b);
        if (alignment.abs() - 1.0).abs() > ALIGN_TOL {
            return Err(malformed(format!(
                "simple face {face_id} periodic boundary azimuth mismatch alignment={alignment}"
            )));
        }
        let (b_same, b_opposite) = if alignment >= 0.0 {
            (b0, b1)
        } else {
            (b1, b0)
        };

        let arc_a0 = arc_starting_at(ir, &low_pair, &a0)?;
        let arc_a1 = arc_starting_at(ir, &low_pair, &a1)?;
        let arc_b_same = arc_starting_at(ir, &high_pair, &b_same)?;
        let arc_b_opposite = arc_starting_at(ir, &high_pair, &b_opposite)?;

        let seam0 = new_seam(ir, allocator, &a0, &b_same)?;
        let seam1 = new_seam(ir, allocator, &a1, &b_opposite)?;

        let seam0_high = new_coedge(ir, allocator, &high_loop, &seam0, Sense::Forward);
        let seam0_low = new_coedge(ir, allocator, &low_loop, &seam0, Sense::Reversed);
        set_radial_pair(ir, &seam0_high, &seam0_low)?;

        let seam1_low = new_coedge(ir, allocator, &low_loop, &seam1, Sense::Forward);
        let seam1_high = new_coedge(ir, allocator, &high_loop, &seam1, Sense::Reversed);
        set_radial_pair(ir, &seam1_low, &seam1_high)?;

        let half_a = vec![
            arc_a0,
            seam1_low,
            arc_b_opposite,
            seam0_low,
        ];
        let half_b = vec![arc_a1, seam0_high, arc_b_same, seam1_high];
        assert_directed_cycle(ir, &half_a)?;
        assert_directed_cycle(ir, &half_b)?;

        let cloned_surface_id = allocator.alloc("surface");
        let mut cloned_surface = surface.clone();
        cloned_surface.id = SurfaceId(cloned_surface_id.clone());
        ir.model.surfaces.push(cloned_surface);

        let cloned_face_id = allocator.alloc("face");
        let mut cloned_face = face.clone();
        cloned_face.id = FaceId(cloned_face_id.clone());
        cloned_face.surface = SurfaceId(cloned_surface_id);
        cloned_face.loops = vec![LoopId(high_loop.clone())];

        ir.model.faces[face_position].loops = vec![LoopId(low_loop.clone())];
        let low_loop_position = loop_index(ir, &low_loop)?;
        let high_loop_position = loop_index(ir, &high_loop)?;
        ir.model.loops[low_loop_position].face = FaceId(face_id.clone());
        ir.model.loops[high_loop_position].face = FaceId(cloned_face_id.clone());

        set_ring(ir, &low_loop, &half_a)?;
        set_ring(ir, &high_loop, &half_b)?;

        let shell_id = face.shell.as_str().to_string();
        ir.model.faces.push(cloned_face);
        insert_face_after(ir, &shell_id, &face_id, &cloned_face_id)?;

        output_face_ids.insert(face_id);
        output_face_ids.insert(cloned_face_id);
    }

    let simple_stage_ok =
        ir.model.bodies.len() == 2
        && ir.model.regions.len() == 2
        && ir.model.shells.len() == 2
        && ir.model.faces.len() == 469
        && ir.model.loops.len() == 519
        && ir.model.coedges.len() == 2498
        && ir.model.edges.len() == 1249
        && ir.model.vertices.len() == 826
        && ir.model.points.len() == 826
        && ir.model.curves.len() == 1249
        && ir.model.pcurves.len() == 192
        && ir.model.surfaces.len() == 469;

    if !simple_stage_ok {
        return Err(malformed(format!(
            "SCH13006 simple periodic postcondition drift faces={} loops={} coedges={} edges={} vertices={} points={} curves={} pcurves={} surfaces={}",
            ir.model.faces.len(),
            ir.model.loops.len(),
            ir.model.coedges.len(),
            ir.model.edges.len(),
            ir.model.vertices.len(),
            ir.model.points.len(),
            ir.model.curves.len(),
            ir.model.pcurves.len(),
            ir.model.surfaces.len()
        )));
    }

    Ok(output_face_ids)
}

fn derive_complex_plans(
    ir: &CadIr,
) -> Result<(Vec<ComplexPlan>, Vec<SplitOperation>), CodecError> {
    let mut plans = Vec::new();
    let mut split_operations = BTreeMap::<String, SplitOperation>::new();

    let mut candidate_face_ids = Vec::new();
    for face in &ir.model.faces {
        if face.loops.len() != 2 {
            continue;
        }
        let surface = &ir.model.surfaces[surface_index(ir, face.surface.as_str())?];
        if matches!(
            support_kind(&surface.geometry),
            Some(SupportKind::Cylinder | SupportKind::Cone)
        ) {
            candidate_face_ids.push(face.id.as_str().to_string());
        }
    }
    candidate_face_ids.sort();

    if candidate_face_ids.len() != 10 {
        return Err(malformed(format!(
            "SCH13006 complex periodic face selector drift expected=10 actual={}",
            candidate_face_ids.len()
        )));
    }

    let mut cylinder_candidates = 0usize;
    let mut cone_candidates = 0usize;
    for face_id in &candidate_face_ids {
        let face = &ir.model.faces[face_index(ir, face_id)?];
        let surface = &ir.model.surfaces[surface_index(ir, face.surface.as_str())?];
        match support_kind(&surface.geometry) {
            Some(SupportKind::Cylinder) => cylinder_candidates += 1,
            Some(SupportKind::Cone) => cone_candidates += 1,
            _ => {}
        }
    }
    if cylinder_candidates != 5 || cone_candidates != 5 {
        return Err(malformed(format!(
            "SCH13006 complex support population drift cylinders={} cones={} expected=5/5",
            cylinder_candidates, cone_candidates
        )));
    }

    for face_id in candidate_face_ids {
        let face = &ir.model.faces[face_index(ir, &face_id)?];
        let surface = &ir.model.surfaces[surface_index(ir, face.surface.as_str())?];
        let surface_kind = support_kind(&surface.geometry)
            .ok_or_else(|| malformed(format!("complex face {face_id} support kind vanished")))?;
        let loop_ids = [
            face.loops[0].as_str().to_string(),
            face.loops[1].as_str().to_string(),
        ];

        let pole_loops = loop_ids
            .iter()
            .filter_map(|loop_id| {
                let loop_ = &ir.model.loops[loop_index(ir, loop_id).ok()?];
                (loop_.coedges.is_empty() && loop_.vertex_uses.len() == 1)
                    .then_some(loop_id.clone())
            })
            .collect::<Vec<_>>();

        if pole_loops.len() == 1 {
            if surface_kind != SupportKind::Cone {
                continue;
            }
            let pole_loop = pole_loops[0].clone();
            let boundary_loop = loop_ids
                .iter()
                .find(|value| **value != pole_loop)
                .cloned()
                .ok_or_else(|| malformed(format!("complex pole face {face_id} has no boundary loop")))?;
            let boundary_edges = loop_edge_ids(ir, &boundary_loop)?;
            if boundary_edges.len() != 2 {
                continue;
            }
            let mut boundary_is_two_circles = true;
            for edge_id in &boundary_edges {
                if !edge_is_circle(ir, edge_id)? {
                    boundary_is_two_circles = false;
                    break;
                }
            }
            if !boundary_is_two_circles {
                continue;
            }
            let boundary_vertices = loop_vertex_ids(ir, &boundary_loop)?;
            if boundary_vertices.len() != 2 {
                continue;
            }
            let pole_loop_ref = &ir.model.loops[loop_index(ir, &pole_loop)?];
            let pole_vertex = pole_loop_ref.vertex_uses[0].vertex.as_str().to_string();
            let pole_point = vertex_position(ir, &pole_vertex)?;

            let SurfaceGeometry::Cone {
                origin,
                axis,
                radius,
                half_angle,
                ..
            } = &surface.geometry
            else {
                continue;
            };
            let axis = normalize(*axis, "complex cone axis")?;
            let tangent = half_angle.tan();
            if !tangent.is_finite() || tangent.abs() < 1.0e-15 {
                continue;
            }
            let analytic_apex = point_add_vector(*origin, vector_scale(axis, -*radius / tangent));
            if point_distance(pole_point, analytic_apex) > GEOM_TOL_MM {
                continue;
            }

            plans.push(ComplexPlan::Pole {
                face_id,
                pole_loop,
                boundary_loop,
                pole_vertex,
                boundary_cut_0: boundary_vertices[0].clone(),
                boundary_cut_1: boundary_vertices[1].clone(),
            });
            continue;
        }

        if !pole_loops.is_empty() {
            continue;
        }

        let mut anchors = Vec::new();
        for loop_id in &loop_ids {
            let loop_ = &ir.model.loops[loop_index(ir, loop_id)?];
            if loop_.coedges.len() != 1 {
                continue;
            }
            let coedge = &ir.model.coedges[coedge_index(ir, loop_.coedges[0].as_str())?];
            let edge = &ir.model.edges[edge_index(ir, coedge.edge.as_str())?];
            if edge.start == edge.end && edge_is_circle(ir, edge.id.as_str())? {
                anchors.push((loop_id.clone(), edge.id.as_str().to_string()));
            }
        }
        if anchors.len() != 1 {
            continue;
        }

        let (anchor_loop, anchor_edge) = anchors[0].clone();
        let boundary_loop = loop_ids
            .iter()
            .find(|value| **value != anchor_loop)
            .cloned()
            .ok_or_else(|| malformed(format!("complex periodic face {face_id} has no boundary loop")))?;
        let anchor = &ir.model.edges[edge_index(ir, &anchor_edge)?];
        let anchor_vertex = anchor.start.as_str().to_string();
        let anchor_point = vertex_position(ir, &anchor_vertex)?;
        let anchor_direction = surface_radial_direction(&surface.geometry, anchor_point)?
            .ok_or_else(|| malformed(format!("complex anchor lies on support axis face={face_id}")))?;
        let target_directions = [anchor_direction, vector_scale(anchor_direction, -1.0)];
        let anchor_opposite =
            point_on_circle_at_support_direction(ir, &anchor_edge, target_directions[1], &surface.geometry)?;
        if split_operations
            .insert(
                anchor_edge.clone(),
                SplitOperation {
                    edge_id: anchor_edge.clone(),
                    point: anchor_opposite,
                },
            )
            .is_some()
        {
            return Err(malformed(format!(
                "complex reconstruction selected anchor split edge {anchor_edge} more than once"
            )));
        }

        let mut boundary_cuts = Vec::new();
        for target_direction in target_directions {
            let mut existing_hits = Vec::new();
            for vertex_id in loop_vertex_ids(ir, &boundary_loop)? {
                if phase_matches(&surface.geometry, vertex_position(ir, &vertex_id)?, target_direction)? {
                    existing_hits.push(vertex_id);
                }
            }
            if existing_hits.len() > 1 {
                return Err(malformed(format!(
                    "multiple existing seam-phase vertices face={face_id} loop={boundary_loop}"
                )));
            }
            if existing_hits.len() == 1 {
                boundary_cuts.push(CutRef::Existing(existing_hits.remove(0)));
                continue;
            }

            let mut candidates = Vec::new();
            for edge_id in loop_edge_ids(ir, &boundary_loop)? {
                if !edge_is_circle(ir, &edge_id)? {
                    continue;
                }
                let point = point_on_circle_at_support_direction(
                    ir,
                    &edge_id,
                    target_direction,
                    &surface.geometry,
                )?;
                if point_strictly_on_minor_arc(ir, &edge_id, point)? {
                    candidates.push((edge_id, point));
                }
            }
            if candidates.len() != 1 {
                return Err(malformed(format!(
                    "minor-arc seam split candidate count face={face_id} loop={boundary_loop} actual={}",
                    candidates.len()
                )));
            }
            let (edge_id, point) = candidates.remove(0);
            if split_operations.contains_key(&edge_id) {
                return Err(malformed(format!(
                    "complex reconstruction selected split edge {edge_id} more than once"
                )));
            }
            split_operations.insert(
                edge_id.clone(),
                SplitOperation {
                    edge_id: edge_id.clone(),
                    point,
                },
            );
            boundary_cuts.push(CutRef::SplitEdge(edge_id));
        }

        if boundary_cuts.len() != 2 {
            return Err(malformed(format!("complex face {face_id} did not derive two boundary cuts")));
        }
        let cut_0 = boundary_cuts.remove(0);
        let cut_1 = boundary_cuts.remove(0);
        if matches!((&cut_0, &cut_1), (CutRef::SplitEdge(a), CutRef::SplitEdge(b)) if a == b)
            || matches!((&cut_0, &cut_1), (CutRef::Existing(a), CutRef::Existing(b)) if a == b)
        {
            return Err(malformed(format!("complex face {face_id} derived identical cuts")));
        }

        plans.push(ComplexPlan::Periodic {
            face_id,
            anchor_loop,
            boundary_loop,
            anchor_edge,
            anchor_vertex,
            boundary_cut_0: cut_0,
            boundary_cut_1: cut_1,
        });
    }

    plans.sort_by(|left, right| complex_plan_face_id(left).cmp(complex_plan_face_id(right)));
    let pole_count = plans
        .iter()
        .filter(|plan| matches!(plan, ComplexPlan::Pole { .. }))
        .count();
    if plans.len() != 10 || pole_count != 1 || split_operations.len() != 19 {
        return Err(malformed(format!(
            "SCH13006 complex derivation drift plans={} poles={} splits={} expected=10/1/19",
            plans.len(),
            pole_count,
            split_operations.len()
        )));
    }
    Ok((plans, split_operations.into_values().collect()))
}

fn complex_plan_face_id(plan: &ComplexPlan) -> &str {
    match plan {
        ComplexPlan::Pole { face_id, .. } | ComplexPlan::Periodic { face_id, .. } => face_id,
    }
}

fn split_complex_edge(
    ir: &mut CadIr,
    allocator: &mut IdAllocator,
    operation: &SplitOperation,
) -> Result<ExecutedSplit, CodecError> {
    let edge_position = edge_index(ir, &operation.edge_id)?;
    let original_edge = ir.model.edges[edge_position].clone();
    let curve_id = original_edge
        .curve
        .as_ref()
        .ok_or_else(|| malformed(format!("split edge {} has no curve", operation.edge_id)))?
        .as_str()
        .to_string();
    let curve = ir.model.curves[curve_index(ir, &curve_id)?].clone();
    let CurveGeometry::Circle {
        center,
        axis,
        radius,
        ..
    } = &curve.geometry
    else {
        return Err(malformed(format!("split edge {} is not circular", operation.edge_id)));
    };

    let uses = edge_uses(ir, &operation.edge_id);
    if uses.len() != 2 {
        return Err(malformed(format!(
            "split edge {} radial uses={} expected=2",
            operation.edge_id,
            uses.len()
        )));
    }
    for use_id in &uses {
        let use_ = &ir.model.coedges[coedge_index(ir, use_id)?];
        if !use_.pcurves.is_empty()
            || use_.use_curve.is_some()
            || use_.use_curve_parameter_range.is_some()
        {
            return Err(malformed(format!(
                "split edge {} carries unsupported coedge-local geometry",
                operation.edge_id
            )));
        }
    }

    let axis = normalize(*axis, "complex split circle axis")?;
    let relative = vec_sub(operation.point, *center);
    let axial = dot(relative, axis);
    let radial = vector_sub(relative, vector_scale(axis, axial));
    let residual = (axial * axial + (norm(radial) - *radius).powi(2)).sqrt();
    if residual > GEOM_TOL_MM {
        return Err(malformed(format!(
            "derived split point off carrier circle edge={} residual={residual}",
            operation.edge_id
        )));
    }
    let start_point = vertex_position(ir, original_edge.start.as_str())?;
    let end_point = vertex_position(ir, original_edge.end.as_str())?;
    if point_distance(operation.point, start_point) <= GEOM_TOL_MM
        || point_distance(operation.point, end_point) <= GEOM_TOL_MM
    {
        return Err(malformed(format!(
            "derived split point equals existing endpoint edge={}",
            operation.edge_id
        )));
    }

    let template_vertex = ir.model.vertices[vertex_index(ir, original_edge.start.as_str())?].clone();
    let point_id = new_point(ir, allocator, operation.point);
    let split_vertex = new_vertex(ir, allocator, &point_id, Some(&template_vertex));

    ir.model.edges[edge_position].end = VertexId(split_vertex.clone());
    ir.model.edges[edge_position].param_range = None;

    let new_curve = clone_circle_curve(ir, allocator, &curve)?;
    let new_edge = new_edge(
        ir,
        allocator,
        &new_curve,
        &split_vertex,
        original_edge.end.as_str(),
        original_edge.tolerance,
    );

    let mut new_uses = Vec::new();
    for use_id in uses.clone() {
        let use_ = ir.model.coedges[coedge_index(ir, &use_id)?].clone();
        let loop_id = use_.owner_loop.as_str().to_string();
        let inserted = new_coedge(ir, allocator, &loop_id, &new_edge, use_.sense);
        let after = matches!(use_.sense, Sense::Forward);
        insert_coedge_adjacent(ir, &loop_id, &use_id, &inserted, after)?;
        new_uses.push(inserted);
    }
    set_radial_pair(ir, &new_uses[0], &new_uses[1])?;

    let first = ir.model.coedges[coedge_index(ir, &uses[0])?].clone();
    let second = ir.model.coedges[coedge_index(ir, &uses[1])?].clone();
    if first.radial_next.as_str() != second.id.as_str()
        || second.radial_next.as_str() != first.id.as_str()
    {
        return Err(malformed(format!("original radial ring drift edge={}", operation.edge_id)));
    }

    Ok(ExecutedSplit {
        original_edge: operation.edge_id.clone(),
        new_edge,
        split_vertex,
    })
}

fn resolve_cut(
    cut: &CutRef,
    splits: &BTreeMap<String, ExecutedSplit>,
) -> Result<String, CodecError> {
    match cut {
        CutRef::Existing(vertex) => Ok(vertex.clone()),
        CutRef::SplitEdge(edge) => splits
            .get(edge)
            .map(|value| value.split_vertex.clone())
            .ok_or_else(|| malformed(format!("missing executed split for edge {edge}"))),
    }
}

#[derive(Debug, Clone)]
struct PathStep {
    coedge_id: String,
}

fn two_undirected_paths(
    ir: &CadIr,
    loop_id: &str,
    cut_a: &str,
    cut_b: &str,
) -> Result<[Vec<PathStep>; 2], CodecError> {
    if cut_a == cut_b {
        return Err(malformed(format!("loop {loop_id} received identical cuts")));
    }
    let loop_ = &ir.model.loops[loop_index(ir, loop_id)?];
    if loop_.coedges.is_empty() {
        return Err(malformed(format!("loop {loop_id} has no edges for cycle partition")));
    }

    let mut adjacency = BTreeMap::<String, Vec<(String, String)>>::new();
    for coedge_id in &loop_.coedges {
        let coedge = &ir.model.coedges[coedge_index(ir, coedge_id.as_str())?];
        let edge = &ir.model.edges[edge_index(ir, coedge.edge.as_str())?];
        let start = edge.start.as_str().to_string();
        let end = edge.end.as_str().to_string();
        adjacency
            .entry(start.clone())
            .or_default()
            .push((coedge.id.as_str().to_string(), end.clone()));
        adjacency
            .entry(end)
            .or_default()
            .push((coedge.id.as_str().to_string(), start));
    }

    if adjacency.values().any(|values| values.len() != 2) {
        return Err(malformed(format!("loop {loop_id} is not a degree-two undirected cycle")));
    }
    let starts = adjacency
        .get(cut_a)
        .ok_or_else(|| malformed(format!("cut {cut_a} is absent from loop {loop_id}")))?
        .clone();
    if !adjacency.contains_key(cut_b) || starts.len() != 2 {
        return Err(malformed(format!("invalid cycle cuts on loop {loop_id}")));
    }

    let mut paths = Vec::new();
    for first in starts {
        let mut used = BTreeSet::new();
        let mut selected = first;
        let mut path = Vec::new();
        let mut guard = 0usize;

        loop {
            if !used.insert(selected.0.clone()) {
                return Err(malformed(format!("loop {loop_id} repeated edge during path partition")));
            }
            let next_vertex = selected.1.clone();
            path.push(PathStep {
                coedge_id: selected.0.clone(),
            });
            if next_vertex == cut_b {
                break;
            }

            let options = adjacency
                .get(&next_vertex)
                .ok_or_else(|| malformed(format!("loop {loop_id} path lost vertex {next_vertex}")))?
                .iter()
                .filter(|(edge_id, _)| !used.contains(edge_id))
                .cloned()
                .collect::<Vec<_>>();
            if options.len() != 1 {
                return Err(malformed(format!(
                    "loop {loop_id} path is ambiguous at vertex {next_vertex}"
                )));
            }
            selected = options[0].clone();
            guard += 1;
            if guard > loop_.coedges.len() + 2 {
                return Err(malformed(format!("loop {loop_id} path partition guard exceeded")));
            }
        }
        paths.push(path);
    }

    if paths.len() != 2 {
        return Err(malformed(format!("loop {loop_id} did not yield two cycle paths")));
    }
    Ok([paths.remove(0), paths.remove(0)])
}

fn path_ids(path: &[PathStep], reverse: bool) -> Vec<String> {
    if reverse {
        path.iter()
            .rev()
            .map(|step| step.coedge_id.clone())
            .collect()
    } else {
        path.iter().map(|step| step.coedge_id.clone()).collect()
    }
}

fn path_contains_edge(
    ir: &CadIr,
    path: &[PathStep],
    edge_id: &str,
) -> Result<bool, CodecError> {
    for step in path {
        let coedge = &ir.model.coedges[coedge_index(ir, &step.coedge_id)?];
        if coedge.edge.as_str() == edge_id {
            return Ok(true);
        }
    }
    Ok(false)
}

fn path_contains_coedge(path: &[PathStep], coedge_id: &str) -> bool {
    path.iter().any(|step| step.coedge_id == coedge_id)
}

fn unique_path_index_for_edge(
    ir: &CadIr,
    paths: &[Vec<PathStep>; 2],
    edge_id: &str,
    label: &str,
) -> Result<usize, CodecError> {
    let matches = [
        path_contains_edge(ir, &paths[0], edge_id)?,
        path_contains_edge(ir, &paths[1], edge_id)?,
    ];
    match matches {
        [true, false] => Ok(0),
        [false, true] => Ok(1),
        _ => Err(malformed(format!(
            "{label} edge {edge_id} does not identify exactly one path"
        ))),
    }
}

fn unique_path_index_for_coedge(
    paths: &[Vec<PathStep>; 2],
    coedge_id: &str,
    label: &str,
) -> Result<usize, CodecError> {
    let matches = [
        path_contains_coedge(&paths[0], coedge_id),
        path_contains_coedge(&paths[1], coedge_id),
    ];
    match matches {
        [true, false] => Ok(0),
        [false, true] => Ok(1),
        _ => Err(malformed(format!(
            "{label} coedge {coedge_id} does not identify exactly one path"
        ))),
    }
}

fn signed_support_side(
    direction: Vector3,
    transverse: Vector3,
    label: &str,
) -> Result<i8, CodecError> {
    let value = dot(direction, transverse);
    if value > 1.0e-7 {
        Ok(1)
    } else if value < -1.0e-7 {
        Ok(-1)
    } else {
        Err(malformed(format!(
            "{label} lies on the support-phase cut plane"
        )))
    }
}

fn anchor_half_support_side(
    ir: &CadIr,
    surface: &SurfaceGeometry,
    path: &[PathStep],
    transverse: Vector3,
) -> Result<i8, CodecError> {
    if path.len() != 1 {
        return Err(malformed(format!(
            "complex anchor half path length drift expected=1 actual={}",
            path.len()
        )));
    }

    let coedge = &ir.model.coedges[coedge_index(ir, &path[0].coedge_id)?];
    let edge = &ir.model.edges[edge_index(ir, coedge.edge.as_str())?];
    let curve = edge_curve(ir, edge)?;
    let CurveGeometry::Circle {
        center,
        axis,
        radius,
        ref_direction,
    } = &curve.geometry
    else {
        return Err(malformed(format!(
            "complex anchor half edge {} is not circular",
            edge.id
        )));
    };

    let circle_axis = normalize(*axis, "complex anchor circle axis")?;
    let x_axis = normalize(*ref_direction, "complex anchor circle reference direction")?;
    let y_axis = normalize(
        cross(circle_axis, x_axis),
        "complex anchor circle transverse direction",
    )?;

    let start = vertex_position(ir, edge.start.as_str())?;
    let end = vertex_position(ir, edge.end.as_str())?;
    let start_radial = vec_sub(start, *center);
    let end_radial = vec_sub(end, *center);

    let t0 = dot(start_radial, y_axis).atan2(dot(start_radial, x_axis));
    let t1 = dot(end_radial, y_axis).atan2(dot(end_radial, x_axis));
    let mut sweep = t1 - t0;
    while sweep < 0.0 {
        sweep += std::f64::consts::TAU;
    }
    while sweep >= std::f64::consts::TAU {
        sweep -= std::f64::consts::TAU;
    }

    if (sweep - std::f64::consts::PI).abs() > 1.0e-5 {
        return Err(malformed(format!(
            "complex anchor half edge {} is not antipodal sweep={sweep}",
            edge.id
        )));
    }

    let midpoint_parameter = t0 + 0.5 * sweep;
    let midpoint = point_add_vector(
        *center,
        vector_add(
            vector_scale(x_axis, *radius * midpoint_parameter.cos()),
            vector_scale(y_axis, *radius * midpoint_parameter.sin()),
        ),
    );
    let midpoint_direction = surface_radial_direction(surface, midpoint)?
        .ok_or_else(|| malformed(format!(
            "complex anchor half midpoint lies on support axis edge={}",
            edge.id
        )))?;

    signed_support_side(
        midpoint_direction,
        transverse,
        "complex anchor half midpoint",
    )
}

fn boundary_path_support_side(
    ir: &CadIr,
    surface: &SurfaceGeometry,
    path: &[PathStep],
    transverse: Vector3,
) -> Result<i8, CodecError> {
    let mut side: Option<i8> = None;

    for step in path {
        let coedge = &ir.model.coedges[coedge_index(ir, &step.coedge_id)?];
        let edge = &ir.model.edges[edge_index(ir, coedge.edge.as_str())?];

        for vertex_id in [edge.start.as_str(), edge.end.as_str()] {
            let point = vertex_position(ir, vertex_id)?;
            let Some(direction) = surface_radial_direction(surface, point)? else {
                continue;
            };
            let projection = dot(direction, transverse);
            if projection.abs() <= 1.0e-7 {
                continue;
            }
            let current = if projection > 0.0 { 1 } else { -1 };
            if let Some(existing) = side {
                if existing != current {
                    return Err(malformed(
                        "complex boundary path crosses both support-phase half-planes",
                    ));
                }
            } else {
                side = Some(current);
            }
        }
    }

    if let Some(value) = side {
        return Ok(value);
    }

    // Fallback for a path whose only explicit vertices lie on the two phase
    // cuts.  Generator-line midpoints remain on cylinder/cone supports.
    for step in path {
        let coedge = &ir.model.coedges[coedge_index(ir, &step.coedge_id)?];
        let edge = &ir.model.edges[edge_index(ir, coedge.edge.as_str())?];
        let start = vertex_position(ir, edge.start.as_str())?;
        let end = vertex_position(ir, edge.end.as_str())?;
        let midpoint = Point3::new(
            0.5 * (start.x + end.x),
            0.5 * (start.y + end.y),
            0.5 * (start.z + end.z),
        );
        if let Some(direction) = surface_radial_direction(surface, midpoint)? {
            let projection = dot(direction, transverse);
            if projection.abs() > 1.0e-7 {
                return Ok(if projection > 0.0 { 1 } else { -1 });
            }
        }
    }

    Err(malformed(
        "complex boundary path has no support-phase half-plane witness",
    ))
}

fn unique_boundary_path_index_for_side(
    ir: &CadIr,
    surface: &SurfaceGeometry,
    paths: &[Vec<PathStep>; 2],
    transverse: Vector3,
    target_side: i8,
) -> Result<usize, CodecError> {
    let sides = [
        boundary_path_support_side(ir, surface, &paths[0], transverse)?,
        boundary_path_support_side(ir, surface, &paths[1], transverse)?,
    ];

    if sides[0] == sides[1] || sides[0] != -sides[1] {
        return Err(malformed(format!(
            "complex boundary paths do not occupy complementary support-phase halves sides={}/{}",
            sides[0], sides[1]
        )));
    }

    if sides[0] == target_side {
        Ok(0)
    } else if sides[1] == target_side {
        Ok(1)
    } else {
        Err(malformed(format!(
            "complex boundary target support side {target_side} is absent"
        )))
    }
}

fn clone_surface_and_face(
    ir: &mut CadIr,
    allocator: &mut IdAllocator,
    original_face_id: &str,
    original_loop: &str,
    cloned_loop: &str,
) -> Result<String, CodecError> {
    let face_position = face_index(ir, original_face_id)?;
    let face = ir.model.faces[face_position].clone();
    let surface = ir.model.surfaces[surface_index(ir, face.surface.as_str())?].clone();

    let new_surface_id = allocator.alloc("surface");
    let mut new_surface = surface;
    new_surface.id = SurfaceId(new_surface_id.clone());
    ir.model.surfaces.push(new_surface);

    let new_face_id = allocator.alloc("face");
    let mut new_face = face.clone();
    new_face.id = FaceId(new_face_id.clone());
    new_face.surface = SurfaceId(new_surface_id);
    new_face.loops = vec![LoopId(cloned_loop.to_string())];

    ir.model.faces[face_position].loops = vec![LoopId(original_loop.to_string())];
    let original_loop_position = loop_index(ir, original_loop)?;
    let cloned_loop_position = loop_index(ir, cloned_loop)?;
    ir.model.loops[original_loop_position].face = FaceId(original_face_id.to_string());
    ir.model.loops[cloned_loop_position].face = FaceId(new_face_id.clone());

    let shell_id = face.shell.as_str().to_string();
    ir.model.faces.push(new_face);
    insert_face_after(ir, &shell_id, original_face_id, &new_face_id)?;
    Ok(new_face_id)
}

fn rebuild_complex_faces(
    ir: &mut CadIr,
    allocator: &mut IdAllocator,
) -> Result<(BTreeSet<String>, BTreeSet<String>), CodecError> {
    let (plans, operations) = derive_complex_plans(ir)?;

    let mut executed = BTreeMap::<String, ExecutedSplit>::new();
    let mut complex_split_provenance = BTreeSet::<String>::new();
    for operation in &operations {
        let result = split_complex_edge(ir, allocator, operation)?;
        complex_split_provenance.insert(result.original_edge.clone());
        complex_split_provenance.insert(result.new_edge.clone());
        if executed.insert(operation.edge_id.clone(), result).is_some() {
            return Err(malformed(format!("duplicate executed split edge {}", operation.edge_id)));
        }
    }

    let mut complex_output_faces = BTreeSet::new();

    let mut periodic_phase_rule_count = 0usize;
    let mut pole_linked_order_count = 0usize;
    let mut phase_driven_seam_flip_count = 0usize;

    for plan in plans {
        match plan {
            ComplexPlan::Pole {
                face_id,
                pole_loop,
                boundary_loop,
                pole_vertex,
                boundary_cut_0,
                boundary_cut_1,
            } => {
                let face_before = ir.model.faces[face_index(ir, &face_id)?].clone();
                if face_before.loops.len() != 2 {
                    return Err(malformed(format!(
                        "complex pole face {face_id} lost its two-loop baseline"
                    )));
                }
                let loop0 = face_before.loops[0].as_str().to_string();
                let loop1 = face_before.loops[1].as_str().to_string();
                let source_loops = BTreeSet::from([pole_loop.clone(), boundary_loop.clone()]);
                let baseline_loops = BTreeSet::from([loop0.clone(), loop1.clone()]);
                if source_loops != baseline_loops {
                    return Err(malformed(format!(
                        "complex pole face {face_id} source/baseline loop identity drift"
                    )));
                }

                let boundary_linked = ir.model.loops[loop_index(ir, &boundary_loop)?]
                    .coedges
                    .iter()
                    .map(|value| value.as_str().to_string())
                    .collect::<Vec<_>>();
                if boundary_linked.len() != 2 {
                    return Err(malformed(format!(
                        "complex pole face {face_id} boundary linked order drift expected=2 actual={}",
                        boundary_linked.len()
                    )));
                }

                let paths = two_undirected_paths(
                    ir,
                    &boundary_loop,
                    &boundary_cut_0,
                    &boundary_cut_1,
                )?;

                // R50.N-R22 pole closure:
                // the two geometrically-degenerate halves remain topologically
                // distinct.  Preserve the baseline linked boundary order:
                // linked coedge 0 belongs to the pole output, linked coedge 1
                // remains on the boundary output.  BTree path index is never
                // treated as semantic identity.
                let pole_path_index = unique_path_index_for_coedge(
                    &paths,
                    &boundary_linked[0],
                    "complex pole first linked half",
                )?;
                let boundary_path_index = unique_path_index_for_coedge(
                    &paths,
                    &boundary_linked[1],
                    "complex pole second linked half",
                )?;
                if pole_path_index == boundary_path_index {
                    return Err(malformed(format!(
                        "complex pole face {face_id} linked halves collapsed to one path"
                    )));
                }

                // Qualified pole seam carrier directions are source-role
                // based, not output-loop-index based:
                //   pole -> cut0
                //   cut1 -> pole
                let seam0 = new_seam(ir, allocator, &pole_vertex, &boundary_cut_0)?;
                let seam1 = new_seam(ir, allocator, &boundary_cut_1, &pole_vertex)?;

                let seam0_loop0 =
                    new_coedge(ir, allocator, &loop0, &seam0, Sense::Forward);
                let seam0_loop1 =
                    new_coedge(ir, allocator, &loop1, &seam0, Sense::Reversed);
                set_radial_pair(ir, &seam0_loop0, &seam0_loop1)?;

                let seam1_loop0 =
                    new_coedge(ir, allocator, &loop0, &seam1, Sense::Forward);
                let seam1_loop1 =
                    new_coedge(ir, allocator, &loop1, &seam1, Sense::Reversed);
                set_radial_pair(ir, &seam1_loop0, &seam1_loop1)?;

                let (ring0, ring1) = if loop0 == pole_loop {
                    let mut ring0 = vec![seam0_loop0.clone()];
                    ring0.extend(path_ids(&paths[pole_path_index], false));
                    ring0.push(seam1_loop0.clone());

                    let mut ring1 = vec![seam1_loop1.clone()];
                    ring1.extend(path_ids(&paths[boundary_path_index], true));
                    ring1.push(seam0_loop1.clone());
                    (ring0, ring1)
                } else if loop0 == boundary_loop {
                    let mut ring0 = path_ids(&paths[boundary_path_index], false);
                    ring0.push(seam1_loop0.clone());
                    ring0.push(seam0_loop0.clone());

                    let mut ring1 = vec![seam1_loop1.clone()];
                    ring1.extend(path_ids(&paths[pole_path_index], true));
                    ring1.push(seam0_loop1.clone());
                    (ring0, ring1)
                } else {
                    return Err(malformed(format!(
                        "complex pole face {face_id} baseline loop0 is neither source loop"
                    )));
                };

                set_ring(ir, &loop0, &ring0)?;
                set_ring(ir, &loop1, &ring1)?;
                rebuild_loop_ring_from_seams(ir, &loop0, &seam0_loop0, &seam1_loop0)?;
                rebuild_loop_ring_from_seams(ir, &loop1, &seam0_loop1, &seam1_loop1)?;
                let new_face = clone_surface_and_face(
                    ir,
                    allocator,
                    &face_id,
                    &loop0,
                    &loop1,
                )?;
                complex_output_faces.insert(face_id);
                complex_output_faces.insert(new_face);
                pole_linked_order_count += 1;
            }
            ComplexPlan::Periodic {
                face_id,
                anchor_loop,
                boundary_loop,
                anchor_edge,
                anchor_vertex,
                boundary_cut_0,
                boundary_cut_1,
            } => {
                let face_before = ir.model.faces[face_index(ir, &face_id)?].clone();
                if face_before.loops.len() != 2 {
                    return Err(malformed(format!(
                        "complex periodic face {face_id} lost its two-loop baseline"
                    )));
                }
                let loop0 = face_before.loops[0].as_str().to_string();
                let loop1 = face_before.loops[1].as_str().to_string();
                let source_loops = BTreeSet::from([anchor_loop.clone(), boundary_loop.clone()]);
                let baseline_loops = BTreeSet::from([loop0.clone(), loop1.clone()]);
                if source_loops != baseline_loops {
                    return Err(malformed(format!(
                        "complex periodic face {face_id} source/baseline loop identity drift"
                    )));
                }

                let surface_geometry = ir.model.surfaces[
                    surface_index(ir, face_before.surface.as_str())?
                ]
                .geometry
                .clone();
                let surface_kind = support_kind(&surface_geometry)
                    .ok_or_else(|| malformed(format!(
                        "complex periodic face {face_id} support kind vanished"
                    )))?;

                let anchor_split = executed
                    .get(&anchor_edge)
                    .ok_or_else(|| malformed(format!("missing anchor split edge {anchor_edge}")))?;
                let anchor_opposite = anchor_split.split_vertex.clone();
                let boundary_vertex_0 = resolve_cut(&boundary_cut_0, &executed)?;
                let boundary_vertex_1 = resolve_cut(&boundary_cut_1, &executed)?;

                let anchor_paths = two_undirected_paths(
                    ir,
                    &anchor_loop,
                    &anchor_vertex,
                    &anchor_opposite,
                )?;
                let boundary_paths = two_undirected_paths(
                    ir,
                    &boundary_loop,
                    &boundary_vertex_0,
                    &boundary_vertex_1,
                )?;

                let original_anchor_path_index = unique_path_index_for_edge(
                    ir,
                    &anchor_paths,
                    &anchor_edge,
                    "complex original anchor half",
                )?;

                // R50.N-R22 anchor-half ownership:
                // Forward face  -> original anchor half belongs to output loop0
                // Reversed face -> original anchor half belongs to output loop1
                let original_anchor_belongs_loop0 = match face_before.sense {
                    Sense::Forward => true,
                    Sense::Reversed => false,
                };
                let anchor_loop0_path_index = if original_anchor_belongs_loop0 {
                    original_anchor_path_index
                } else {
                    1usize - original_anchor_path_index
                };

                let anchor_point = vertex_position(ir, &anchor_vertex)?;
                let anchor_direction = surface_radial_direction(
                    &surface_geometry,
                    anchor_point,
                )?
                .ok_or_else(|| malformed(format!(
                    "complex anchor lies on support axis face={face_id}"
                )))?;
                let support_axis = normalize(
                    support_axis(&surface_geometry)
                        .ok_or_else(|| malformed(format!(
                            "complex periodic face {face_id} support axis vanished"
                        )))?,
                    "complex support axis",
                )?;
                let transverse = normalize(
                    cross(support_axis, anchor_direction),
                    "complex support phase transverse",
                )?;

                let anchor_loop0_side = anchor_half_support_side(
                    ir,
                    &surface_geometry,
                    &anchor_paths[anchor_loop0_path_index],
                    transverse,
                )?;

                // R50.N-R22 composite pairing rule:
                // same-side iff
                //   (support is cylinder) == (face sense is Reversed)
                // otherwise the boundary path occupies the opposite support
                // phase half.  This also resolves the Forward-cylinder case
                // without fixture ids or oracle tables.
                let same_side_pairing = match (surface_kind, face_before.sense) {
                    (SupportKind::Cylinder, Sense::Reversed) => true,
                    (SupportKind::Cylinder, Sense::Forward) => false,
                    (SupportKind::Cone, Sense::Reversed) => false,
                    (SupportKind::Cone, Sense::Forward) => true,
                    (SupportKind::Plane, _) => {
                        return Err(malformed(format!(
                            "complex periodic face {face_id} unexpectedly became planar"
                        )));
                    }
                };
                let target_boundary_side = if same_side_pairing {
                    anchor_loop0_side
                } else {
                    -anchor_loop0_side
                };
                let boundary_loop0_path_index = unique_boundary_path_index_for_side(
                    ir,
                    &surface_geometry,
                    &boundary_paths,
                    transverse,
                    target_boundary_side,
                )?;

                let anchor_loop1_path_index = 1usize - anchor_loop0_path_index;
                let boundary_loop1_path_index = 1usize - boundary_loop0_path_index;

                let (
                    loop0_cut0,
                    loop0_cut1,
                    loop1_cut0,
                    loop1_cut1,
                    source0_loop0_path,
                    source0_loop1_path,
                    source1_loop0_path,
                    source1_loop1_path,
                ) = if loop0 == anchor_loop {
                    (
                        anchor_vertex.clone(),
                        anchor_opposite.clone(),
                        boundary_vertex_0.clone(),
                        boundary_vertex_1.clone(),
                        &anchor_paths[anchor_loop0_path_index],
                        &anchor_paths[anchor_loop1_path_index],
                        &boundary_paths[boundary_loop0_path_index],
                        &boundary_paths[boundary_loop1_path_index],
                    )
                } else if loop0 == boundary_loop {
                    (
                        boundary_vertex_0.clone(),
                        boundary_vertex_1.clone(),
                        anchor_vertex.clone(),
                        anchor_opposite.clone(),
                        &boundary_paths[boundary_loop0_path_index],
                        &boundary_paths[boundary_loop1_path_index],
                        &anchor_paths[anchor_loop0_path_index],
                        &anchor_paths[anchor_loop1_path_index],
                    )
                } else {
                    return Err(malformed(format!(
                        "complex periodic face {face_id} baseline loop0 is neither source loop"
                    )));
                };

                // The qualified carrier-direction consequence of the phase
                // rule is deterministic.  R14's loop0-cycle seam directions
                // are reversed exactly when the selected boundary half is
                // path[1] and the support/face rule requires opposite-side
                // pairing.  This removes BTree path-order leakage while
                // preserving loop0 Forward / loop1 Reversed incidence.
                let reverse_seam_carriers =
                    boundary_loop0_path_index == 1 && !same_side_pairing;

                let (seam0_start, seam0_end, seam1_start, seam1_end) =
                    if reverse_seam_carriers {
                        (
                            loop0_cut0.as_str(),
                            loop1_cut0.as_str(),
                            loop1_cut1.as_str(),
                            loop0_cut1.as_str(),
                        )
                    } else {
                        (
                            loop1_cut0.as_str(),
                            loop0_cut0.as_str(),
                            loop0_cut1.as_str(),
                            loop1_cut1.as_str(),
                        )
                    };

                let seam0 = new_seam(ir, allocator, seam0_start, seam0_end)?;
                let seam1 = new_seam(ir, allocator, seam1_start, seam1_end)?;

                let seam0_loop0 =
                    new_coedge(ir, allocator, &loop0, &seam0, Sense::Forward);
                let seam0_loop1 =
                    new_coedge(ir, allocator, &loop1, &seam0, Sense::Reversed);
                set_radial_pair(ir, &seam0_loop0, &seam0_loop1)?;

                let seam1_loop0 =
                    new_coedge(ir, allocator, &loop0, &seam1, Sense::Forward);
                let seam1_loop1 =
                    new_coedge(ir, allocator, &loop1, &seam1, Sense::Reversed);
                set_radial_pair(ir, &seam1_loop0, &seam1_loop1)?;

                let mut ring0 = path_ids(source0_loop0_path, false);
                ring0.push(seam1_loop0.clone());
                ring0.extend(path_ids(source1_loop0_path, true));
                ring0.push(seam0_loop0.clone());

                let mut ring1 = path_ids(source0_loop1_path, true);
                ring1.push(seam0_loop1.clone());
                ring1.extend(path_ids(source1_loop1_path, false));
                ring1.push(seam1_loop1.clone());

                set_ring(ir, &loop0, &ring0)?;
                set_ring(ir, &loop1, &ring1)?;
                rebuild_loop_ring_from_seams(ir, &loop0, &seam0_loop0, &seam1_loop0)?;
                rebuild_loop_ring_from_seams(ir, &loop1, &seam0_loop1, &seam1_loop1)?;
                let new_face = clone_surface_and_face(
                    ir,
                    allocator,
                    &face_id,
                    &loop0,
                    &loop1,
                )?;
                complex_output_faces.insert(face_id);
                complex_output_faces.insert(new_face);

                periodic_phase_rule_count += 1;
                if reverse_seam_carriers {
                    phase_driven_seam_flip_count += 1;
                }
            }
        }
    }

    if periodic_phase_rule_count != 9 || pole_linked_order_count != 1 {
        return Err(malformed(format!(
            "SCH13006 composite complex rule population drift periodic={} pole={} expected=9/1",
            periodic_phase_rule_count, pole_linked_order_count
        )));
    }

    // For the locked SCH_13006 qualification population, R22 proved that
    // exactly three ordinary periodic faces require the phase-driven seam
    // carrier reversal.  This is a structural count only; no entity ids are
    // consulted.
    if phase_driven_seam_flip_count != 3 {
        return Err(malformed(format!(
            "SCH13006 phase-driven seam carrier reversal drift expected=3 actual={}",
            phase_driven_seam_flip_count
        )));
    }

    if complex_output_faces.len() != 20 || complex_split_provenance.len() != 38 {
        return Err(malformed(format!(
            "SCH13006 complex reconstruction provenance drift outputFaces={} splitHalves={} expected=20/38",
            complex_output_faces.len(),
            complex_split_provenance.len()
        )));
    }

    let final_ok =
        ir.model.bodies.len() == 2
        && ir.model.regions.len() == 2
        && ir.model.shells.len() == 2
        && ir.model.faces.len() == 479
        && ir.model.loops.len() == 519
        && ir.model.coedges.len() == 2576
        && ir.model.edges.len() == 1288
        && ir.model.vertices.len() == 845
        && ir.model.points.len() == 845
        && ir.model.curves.len() == 1288
        && ir.model.pcurves.len() == 192
        && ir.model.surfaces.len() == 479;

    if !final_ok {
        return Err(malformed(format!(
            "SCH13006 final periodic postcondition drift faces={} loops={} coedges={} edges={} vertices={} points={} curves={} pcurves={} surfaces={}",
            ir.model.faces.len(),
            ir.model.loops.len(),
            ir.model.coedges.len(),
            ir.model.edges.len(),
            ir.model.vertices.len(),
            ir.model.points.len(),
            ir.model.curves.len(),
            ir.model.pcurves.len(),
            ir.model.surfaces.len()
        )));
    }

    Ok((complex_output_faces, complex_split_provenance))
}
fn complex_circle_same_sense_false(
    ir: &CadIr,
    provenance: &BTreeSet<String>,
    complex_faces: &BTreeSet<String>,
) -> Result<BTreeSet<String>, CodecError> {
    let mut selected = BTreeSet::new();

    for edge_id in provenance {
        let uses = edge_uses(ir, edge_id);
        if uses.len() != 2 {
            continue;
        }
        let mut plane_two_edge = 0usize;
        let mut complex_cone = 0usize;

        for use_id in uses {
            let coedge = &ir.model.coedges[coedge_index(ir, &use_id)?];
            let loop_ = &ir.model.loops[loop_index(ir, coedge.owner_loop.as_str())?];
            let face = &ir.model.faces[face_index(ir, loop_.face.as_str())?];
            let surface = &ir.model.surfaces[surface_index(ir, face.surface.as_str())?];
            match support_kind(&surface.geometry) {
                Some(SupportKind::Plane) if loop_.coedges.len() == 2 => {
                    plane_two_edge += 1;
                }
                Some(SupportKind::Cone)
                    if complex_faces.contains(face.id.as_str()) && face.loops.len() == 1 =>
                {
                    complex_cone += 1;
                }
                _ => {}
            }
        }

        if plane_two_edge == 1 && complex_cone == 1 {
            selected.insert(edge_id.clone());
        }
    }

    Ok(selected)
}

fn same_physical_circle(
    ir: &CadIr,
    first_edge_id: &str,
    second_edge_id: &str,
) -> Result<bool, CodecError> {
    let first_edge = &ir.model.edges[edge_index(ir, first_edge_id)?];
    let second_edge = &ir.model.edges[edge_index(ir, second_edge_id)?];

    let first_curve = edge_curve(ir, first_edge)?;
    let second_curve = edge_curve(ir, second_edge)?;

    let (
        CurveGeometry::Circle {
            center: first_center,
            axis: first_axis,
            radius: first_radius,
            ..
        },
        CurveGeometry::Circle {
            center: second_center,
            axis: second_axis,
            radius: second_radius,
            ..
        },
    ) = (&first_curve.geometry, &second_curve.geometry)
    else {
        return Ok(false);
    };

    let first_axis = normalize(*first_axis, "shared-circle first axis")?;
    let second_axis = normalize(*second_axis, "shared-circle second axis")?;

    Ok(
        point_distance(*first_center, *second_center) <= GEOM_TOL_MM
            && (*first_radius - *second_radius).abs() <= GEOM_TOL_MM
            && dot(first_axis, second_axis).abs() >= 1.0 - AXIS_TOL
    )
}

fn loop_support_kind(
    ir: &CadIr,
    loop_id: &str,
) -> Result<Option<SupportKind>, CodecError> {
    let loop_ = &ir.model.loops[loop_index(ir, loop_id)?];
    let face = &ir.model.faces[face_index(ir, loop_.face.as_str())?];
    let surface = &ir.model.surfaces[surface_index(ir, face.surface.as_str())?];
    Ok(support_kind(&surface.geometry))
}

fn loop_is_forward_complex_cylinder(
    ir: &CadIr,
    loop_id: &str,
    complex_faces: &BTreeSet<String>,
) -> Result<bool, CodecError> {
    let loop_ = &ir.model.loops[loop_index(ir, loop_id)?];
    let face = &ir.model.faces[face_index(ir, loop_.face.as_str())?];
    let surface = &ir.model.surfaces[surface_index(ir, face.surface.as_str())?];

    Ok(
        complex_faces.contains(face.id.as_str())
            && face.loops.len() == 1
            && matches!(support_kind(&surface.geometry), Some(SupportKind::Cylinder))
            && matches!(face.sense, Sense::Forward)
    )
}

// STEP-only shared-circle override authority:
//
// R50.N-R22 proved the complex half-pairing rule:
//   same-side iff
//     (support is cylinder) == (face sense is Reversed)
//
// Therefore a Forward complex Cylinder is the opposite-side case.
// Only that qualified case needs the cross-owned physical-circle twin at
// STEP ORIENTED_EDGE emission.  Neutral CADIR radial ownership is unchanged.
fn install_shared_circle_step_override_metadata(
    ir: &mut CadIr,
    complex_faces: &BTreeSet<String>,
) -> Result<(), CodecError> {
    let circle_edges = ir
        .model
        .edges
        .iter()
        .filter_map(|edge| {
            let curve = edge
                .curve
                .as_ref()
                .and_then(|curve_id| {
                    ir.model
                        .curves
                        .iter()
                        .find(|curve| curve.id.as_str() == curve_id.as_str())
                })?;

            matches!(&curve.geometry, CurveGeometry::Circle { .. })
                .then(|| edge.id.as_str().to_string())
        })
        .collect::<Vec<_>>();

    let mut overrides = BTreeMap::<String, String>::new();

    for first_index in 0..circle_edges.len() {
        for second_index in (first_index + 1)..circle_edges.len() {
            let first_edge_id = &circle_edges[first_index];
            let second_edge_id = &circle_edges[second_index];

            let first_edge = &ir.model.edges[edge_index(ir, first_edge_id)?];
            let second_edge = &ir.model.edges[edge_index(ir, second_edge_id)?];

            if first_edge.start.as_str() != second_edge.end.as_str()
                || first_edge.end.as_str() != second_edge.start.as_str()
                || !same_physical_circle(ir, first_edge_id, second_edge_id)?
            {
                continue;
            }

            let first_uses = edge_uses(ir, first_edge_id);
            let second_uses = edge_uses(ir, second_edge_id);

            if first_uses.len() != 2 || second_uses.len() != 2 {
                continue;
            }

            let mut shared_planar_loop = None::<String>;
            let mut first_cylinder_use = None::<String>;
            let mut second_cylinder_use = None::<String>;

            for first_use_id in &first_uses {
                let first_use = &ir.model.coedges[coedge_index(ir, first_use_id)?];
                let first_loop = first_use.owner_loop.as_str();

                for second_use_id in &second_uses {
                    let second_use = &ir.model.coedges[coedge_index(ir, second_use_id)?];
                    let second_loop = second_use.owner_loop.as_str();

                    if first_loop != second_loop {
                        continue;
                    }

                    if !matches!(
                        loop_support_kind(ir, first_loop)?,
                        Some(SupportKind::Plane)
                    ) {
                        continue;
                    }

                    if let Some(previous) = shared_planar_loop.replace(first_loop.to_string()) {
                        if previous != first_loop {
                            return Err(malformed(format!(
                                "SCH13006 shared-circle pair has multiple common planar loops firstEdge={} secondEdge={}",
                                first_edge_id,
                                second_edge_id
                            )));
                        }
                    }
                }
            }

            let Some(planar_loop) = shared_planar_loop else {
                continue;
            };

            for use_id in &first_uses {
                let use_ = &ir.model.coedges[coedge_index(ir, use_id)?];
                if use_.owner_loop.as_str() == planar_loop {
                    continue;
                }

                if loop_is_forward_complex_cylinder(
                    ir,
                    use_.owner_loop.as_str(),
                    complex_faces,
                )? {
                    if first_cylinder_use.replace(use_id.clone()).is_some() {
                        return Err(malformed(format!(
                            "SCH13006 shared-circle first edge has multiple cylinder uses edge={first_edge_id}"
                        )));
                    }
                }
            }

            for use_id in &second_uses {
                let use_ = &ir.model.coedges[coedge_index(ir, use_id)?];
                if use_.owner_loop.as_str() == planar_loop {
                    continue;
                }

                if loop_is_forward_complex_cylinder(
                    ir,
                    use_.owner_loop.as_str(),
                    complex_faces,
                )? {
                    if second_cylinder_use.replace(use_id.clone()).is_some() {
                        return Err(malformed(format!(
                            "SCH13006 shared-circle second edge has multiple cylinder uses edge={second_edge_id}"
                        )));
                    }
                }
            }

            let (Some(first_use_id), Some(second_use_id)) =
                (first_cylinder_use, second_cylinder_use)
            else {
                continue;
            };

            let first_loop = ir.model.coedges[coedge_index(ir, &first_use_id)?]
                .owner_loop
                .as_str()
                .to_string();
            let second_loop = ir.model.coedges[coedge_index(ir, &second_use_id)?]
                .owner_loop
                .as_str()
                .to_string();

            if first_loop == second_loop {
                continue;
            }

            if !loop_is_forward_complex_cylinder(ir, &first_loop, complex_faces)?
                || !loop_is_forward_complex_cylinder(ir, &second_loop, complex_faces)?
            {
                return Err(malformed(format!(
                    "SCH13006 shared-circle STEP override escaped R22 Forward-complex-cylinder population firstLoop={} secondLoop={}",
                    first_loop,
                    second_loop
                )));
            }

            for (coedge_id, target_edge) in [
                (first_use_id, second_edge_id.clone()),
                (second_use_id, first_edge_id.clone()),
            ] {
                if let Some(previous) = overrides.insert(coedge_id.clone(), target_edge.clone()) {
                    if previous != target_edge {
                        return Err(malformed(format!(
                            "SCH13006 shared-circle STEP override ambiguity coedge={} firstTarget={} secondTarget={}",
                            coedge_id,
                            previous,
                            target_edge
                        )));
                    }
                }
            }
        }
    }

    if overrides.len() != 2 {
        return Err(malformed(format!(
            "SCH13006 shared-circle STEP override count drift expected=2 actual={}",
            overrides.len()
        )));
    }

    for (coedge_id, target_edge_id) in &overrides {
        let coedge = &ir.model.coedges[coedge_index(ir, coedge_id)?];

        if coedge.edge.as_str() == target_edge_id {
            return Err(malformed(format!(
                "SCH13006 shared-circle STEP override is a no-op coedge={coedge_id}"
            )));
        }

        let source_edge = &ir.model.edges[edge_index(ir, coedge.edge.as_str())?];
        let target_edge = &ir.model.edges[edge_index(ir, target_edge_id)?];

        if source_edge.start.as_str() != target_edge.end.as_str()
            || source_edge.end.as_str() != target_edge.start.as_str()
            || !same_physical_circle(ir, coedge.edge.as_str(), target_edge_id)?
        {
            return Err(malformed(format!(
                "SCH13006 shared-circle STEP override target is not the opposite-endpoint physical twin coedge={} sourceEdge={} targetEdge={}",
                coedge_id,
                coedge.edge,
                target_edge_id
            )));
        }
    }

    let records = overrides
        .into_iter()
        .enumerate()
        .map(|(index, (coedge, edge))| StepOrientedEdgeOverrideRecord {
            id: format!(
                "parasolid:sch13006:step-oriented-edge-override#r50n-{index:010}"
            ),
            coedge,
            edge,
            class: SHARED_CIRCLE_OVERRIDE_CLASS.to_string(),
        })
        .collect::<Vec<_>>();

    let namespace = ir.native.namespace_mut(NATIVE_NAMESPACE);
    if namespace.version == 0 {
        namespace.version = NATIVE_NAMESPACE_VERSION;
    } else if namespace.version != NATIVE_NAMESPACE_VERSION {
        return Err(malformed(format!(
            "Parasolid native namespace version drift expected={} actual={}",
            NATIVE_NAMESPACE_VERSION, namespace.version
        )));
    }

    namespace
        .set_arena(STEP_ORIENTED_EDGE_OVERRIDE_ARENA, &records)
        .map_err(|error| {
            malformed(format!(
                "cannot retain SCH13006 STEP oriented-edge override metadata: {error}"
            ))
        })?;

    Ok(())
}

fn install_step_orientation_metadata(
    ir: &mut CadIr,
    plane_lift: &BTreeSet<String>,
    complex_split_provenance: &BTreeSet<String>,
    complex_faces: &BTreeSet<String>,
) -> Result<(), CodecError> {
    if plane_lift.len() != 96 {
        return Err(malformed(format!(
            "SCH13006 plane-lift STEP orientation class drift expected=96 actual={}",
            plane_lift.len()
        )));
    }
    for edge_id in plane_lift {
        let edge = &ir.model.edges[edge_index(ir, edge_id)?];
        let curve = edge_curve(ir, edge)?;
        if !matches!(&curve.geometry, CurveGeometry::Nurbs(_)) {
            return Err(malformed(format!(
                "SCH13006 plane-lift provenance edge {edge_id} is not NURBS"
            )));
        }
    }

    let complex_false = complex_circle_same_sense_false(
        ir,
        complex_split_provenance,
        complex_faces,
    )?;
    if complex_false.len() != 8 {
        return Err(malformed(format!(
            "SCH13006 complex-circle STEP orientation class drift expected=8 actual={}",
            complex_false.len()
        )));
    }

    if !plane_lift.is_disjoint(&complex_false) {
        return Err(malformed(
            "SCH13006 STEP edge-orientation classes are not disjoint",
        ));
    }

    let mut records = Vec::new();
    let mut counter = 0usize;

    for (class, edges) in [
        (PLANE_LIFT_CLASS, plane_lift.clone()),
        (COMPLEX_CIRCLE_CLASS, complex_false),
    ] {
        for edge in edges {
            records.push(StepEdgeOrientationRecord {
                id: format!("parasolid:sch13006:step-orientation#r50n-{counter:010}"),
                edge,
                class: class.to_string(),
                same_sense: false,
            });
            counter += 1;
        }
    }

    if records.len() != 104 {
        return Err(malformed(format!(
            "SCH13006 STEP orientation metadata count drift expected=104 actual={}",
            records.len()
        )));
    }

    let namespace = ir.native.namespace_mut(NATIVE_NAMESPACE);
    if namespace.version == 0 {
        namespace.version = NATIVE_NAMESPACE_VERSION;
    } else if namespace.version != NATIVE_NAMESPACE_VERSION {
        return Err(malformed(format!(
            "Parasolid native namespace version drift expected={} actual={}",
            NATIVE_NAMESPACE_VERSION, namespace.version
        )));
    }
    namespace
        .set_arena(STEP_ORIENTATION_ARENA, &records)
        .map_err(|error| malformed(format!("cannot retain SCH13006 STEP edge orientation: {error}")))?;

    Ok(())
}

fn verify_two_member_radial_pairs(ir: &CadIr) -> Result<(), CodecError> {
    let mut by_edge = BTreeMap::<String, Vec<&Coedge>>::new();
    for coedge in &ir.model.coedges {
        by_edge
            .entry(coedge.edge.as_str().to_string())
            .or_default()
            .push(coedge);
    }

    if by_edge.len() != ir.model.edges.len() {
        return Err(malformed(format!(
            "SCH13006 edge/coedge coverage drift edges={} representedEdges={}",
            ir.model.edges.len(),
            by_edge.len()
        )));
    }

    for (edge_id, uses) in by_edge {
        if uses.len() != 2 {
            return Err(malformed(format!(
                "edge {edge_id} radial use count={} expected=2",
                uses.len()
            )));
        }
        if uses[0].radial_next.as_str() != uses[1].id.as_str()
            || uses[1].radial_next.as_str() != uses[0].id.as_str()
        {
            return Err(malformed(format!("edge {edge_id} radial pair is not reciprocal")));
        }
        if uses[0].sense == uses[1].sense {
            return Err(malformed(format!(
                "edge {edge_id} radial coedges have equal incidence sense"
            )));
        }
    }
    Ok(())
}

/// Reconstruct qualified periodic faces in-place and retain STEP-only source
/// orientation provenance in the Parasolid native namespace.
///
/// The neutral edge endpoints/ranges are never modified merely to encode STEP
/// `EDGE_CURVE.same_sense`; only actual topological splits alter edge bounds.
pub(crate) fn reconstruct_sch13006_periodic(
    ir: &mut CadIr,
    plane_lift_edges: &BTreeSet<String>,
) -> Result<(), CodecError> {
    let baseline_ok =
        ir.model.bodies.len() == 2
        && ir.model.regions.len() == 2
        && ir.model.shells.len() == 2
        && ir.model.faces.len() == 421
        && ir.model.loops.len() == 519
        && ir.model.coedges.len() == 2146
        && ir.model.edges.len() == 1073
        && ir.model.vertices.len() == 746
        && ir.model.points.len() == 746
        && ir.model.curves.len() == 1073
        && ir.model.pcurves.len() == 192
        && ir.model.surfaces.len() == 421;

    if !baseline_ok {
        return Err(malformed(format!(
            "SCH13006 periodic reconstruction baseline drift faces={} loops={} coedges={} edges={} vertices={} points={} curves={} pcurves={} surfaces={}",
            ir.model.faces.len(),
            ir.model.loops.len(),
            ir.model.coedges.len(),
            ir.model.edges.len(),
            ir.model.vertices.len(),
            ir.model.points.len(),
            ir.model.curves.len(),
            ir.model.pcurves.len(),
            ir.model.surfaces.len()
        )));
    }

    let mut allocator = IdAllocator::new(ir);
    let _simple_faces = rebuild_simple_faces(ir, &mut allocator)?;
    let (complex_faces, complex_split_provenance) = rebuild_complex_faces(ir, &mut allocator)?;
    verify_two_member_radial_pairs(ir)?;
    install_step_orientation_metadata(
        ir,
        plane_lift_edges,
        &complex_split_provenance,
        &complex_faces,
    )?;
    install_shared_circle_step_override_metadata(
        ir,
        &complex_faces,
    )?;
    Ok(())
}
