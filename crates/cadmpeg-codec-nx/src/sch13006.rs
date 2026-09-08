// SPDX-License-Identifier: Apache-2.0
//! Fail-closed Parasolid SCH_13006 entity scanner and topology census.
//!
//! This module intentionally implements only the SCH_13006 node types that are
//! physically present in the qualified TwinCore fixture. Unknown node types are
//! rejected rather than skipped. Geometry payloads are consumed according to
//! their published field types but are not converted to CADIR here.
#![deny(clippy::disallowed_methods)]

use std::collections::{BTreeMap, BTreeSet};

use cadmpeg_core::decode::View;
use cadmpeg_core::CodecError;

use crate::text_transmit::Sch13006Preamble;

const MAX_ENTITIES: usize = 2_000_000;
const MAX_VARIABLE_VALUES: u32 = 16_000_000;
const QUALIFIED_SCHEMA_TYPE_COUNT: usize = 31;

const POINTER_LIST: u32 = 74;
const BODY: u32 = 12;
const SHELL: u32 = 13;
const FACE: u32 = 14;
const LOOP: u32 = 15;
const EDGE: u32 = 16;
const FIN: u32 = 17;
const VERTEX: u32 = 18;
const REGION: u32 = 19;
const POINT: u32 = 29;

const LINE: u32 = 30;
const CIRCLE: u32 = 31;
const ELLIPSE: u32 = 32;

const PLANE: u32 = 50;
const CYLINDER: u32 = 51;
const CONE: u32 = 52;

const TRIMMED_CURVE: u32 = 133;
const B_CURVE: u32 = 134;
const SP_CURVE: u32 = 137;

const BSPLINE_VERTICES: u32 = 45;
const KNOT_MULT: u32 = 127;
const KNOT_SET: u32 = 128;
const CURVE_DATA: u32 = 135;
const NURBS_CURVE: u32 = 136;

const GEOMETRY_VECTOR_TOLERANCE: f64 = 1.0e-6;
const METRES_TO_MILLIMETRES: f64 = 1000.0;
const CADIR_COINCIDENCE_TOLERANCE_M: f64 =
    cadmpeg_ir::units::COINCIDENCE_TOLERANCE /
    METRES_TO_MILLIMETRES;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FieldKind {
    Signed,
    Unsigned,
    Short,
    Real,
    Character,
    Logical,
    Pointer,
    Vector3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VariableKind {
    Signed,
    Short,
    Real,
    Pointer,
    RawChar,
}

#[derive(Debug, Clone, Copy)]
struct SchemaSpec {
    fields: &'static [FieldKind],
    variable: Option<VariableKind>,
}

#[derive(Debug, Clone, Copy)]
enum FieldValue {
    Signed(i64),
    Unsigned(u64),
    Pointer(u32),
    Character(u8),
    Real(Option<f64>),
    Vector3(Option<[f64; 3]>),
}

#[derive(Debug, Clone, Copy)]
struct BodyNode {
    next: u32,
    previous: u32,
    body_type: u32,
    shell: u32,
    region: u32,
    edge: u32,
    vertex: u32,
}

#[derive(Debug, Clone, Copy)]
struct ShellNode {
    body: u32,
    next: u32,
    face: u32,
    edge: u32,
    vertex: u32,
    region: u32,
    front_face: u32,
}

#[derive(Debug, Clone, Copy)]
struct FaceNode {
    tolerance: Option<f64>,
    native_sense: u8,
    next: u32,
    previous: u32,
    loop_head: u32,
    shell: u32,
    surface: u32,
    next_on_surface: u32,
    previous_on_surface: u32,
    next_front: u32,
    previous_front: u32,
    front_shell: u32,
}

#[derive(Debug, Clone, Copy)]
struct LoopNode {
    fin: u32,
    face: u32,
    next: u32,
}

#[derive(Debug, Clone, Copy)]
struct EdgeNode {
    tolerance: Option<f64>,
    fin: u32,
    previous: u32,
    next: u32,
    curve: u32,
    next_on_curve: u32,
    previous_on_curve: u32,
    owner: u32,
}

#[derive(Debug, Clone, Copy)]
struct FinNode {
    native_sense: u8,
    loop_owner: u32,
    forward: u32,
    backward: u32,
    vertex: u32,
    other: u32,
    edge: u32,
    curve: u32,
    next_at_vertex: u32,
}

#[derive(Debug, Clone, Copy)]
struct VertexNode {
    tolerance: Option<f64>,
    fin: u32,
    previous: u32,
    next: u32,
    point: u32,
    owner: u32,
}

#[derive(Debug, Clone, Copy)]
struct RegionNode {
    body: u32,
    next: u32,
    previous: u32,
    shell: u32,
    kind: u8,
}

#[derive(Debug, Clone, Copy)]
struct PointGeometry {
    pvec: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
struct LineGeometry {
    pvec: [f64; 3],
    direction: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
struct CircleGeometry {
    centre: [f64; 3],
    normal: [f64; 3],
    x_axis: [f64; 3],
    radius: f64,
}

#[derive(Debug, Clone, Copy)]
struct EllipseGeometry {
    centre: [f64; 3],
    normal: [f64; 3],
    x_axis: [f64; 3],
    major_radius: f64,
    minor_radius: f64,
}

#[derive(Debug, Clone, Copy)]
struct PlaneGeometry {
    native_sense: u8,
    pvec: [f64; 3],
    normal: [f64; 3],
    x_axis: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
struct CylinderGeometry {
    native_sense: u8,
    pvec: [f64; 3],
    axis: [f64; 3],
    radius: f64,
    x_axis: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
struct ConeGeometry {
    native_sense: u8,
    pvec: [f64; 3],
    axis: [f64; 3],
    radius: f64,
    sin_half_angle: f64,
    cos_half_angle: f64,
    x_axis: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
struct TrimmedCurveGeometry {
    basis_curve: u32,
    point_1: [f64; 3],
    point_2: [f64; 3],
    parm_1: f64,
    parm_2: f64,
}

#[derive(Debug, Clone, Copy)]
struct BCurveGeometry {
    nurbs: u32,
    data: u32,
}

#[derive(Debug, Clone, Copy)]
struct NurbsCurveGeometry {
    degree: i64,
    n_vertices: i64,
    vertex_dim: i64,
    n_knots: i64,
    knot_type: u32,
    periodic: bool,
    closed: bool,
    rational: bool,
    curve_form: u32,
    bspline_vertices: u32,
    knot_mult: u32,
    knots: u32,
}

#[derive(Debug, Clone, Copy)]
struct SpCurveGeometry {
    surface: u32,
    b_curve: u32,
    original: u32,
    tolerance_to_original: Option<f64>,
}
#[derive(Debug, Clone)]
pub(crate) struct Sch13006GeometryBindings {
    pub(crate) face_surface_count: usize,
    pub(crate) unique_face_surface_count: usize,
    pub(crate) face_surface_owner_match_count: usize,
    pub(crate) face_plane_count: usize,
    pub(crate) face_cylinder_count: usize,
    pub(crate) face_cone_count: usize,

    pub(crate) direct_edge_curve_count: usize,
    pub(crate) null_edge_curve_count: usize,
    pub(crate) unique_direct_edge_curve_count: usize,
    pub(crate) direct_edge_curve_owner_match_count: usize,
    pub(crate) edge_line_count: usize,
    pub(crate) edge_circle_count: usize,
    pub(crate) edge_ellipse_count: usize,
    pub(crate) edge_trimmed_curve_count: usize,

    pub(crate) fin_curve_count: usize,
    pub(crate) unique_fin_curve_count: usize,
    pub(crate) fin_curve_owner_match_count: usize,
    pub(crate) fin_trimmed_curve_count: usize,

    pub(crate) vertex_point_count: usize,
    pub(crate) unique_vertex_point_count: usize,
    pub(crate) vertex_point_owner_match_count: usize,

    pub(crate) null_edge_two_fin_trim_count: usize,
    pub(crate) direct_edge_zero_fin_curve_count: usize,

    pub(crate) geometry_owner_domain_validation_count: usize,
    pub(crate) direct_owner_match_count: usize,

    pub(crate) trimmed_curve_count: usize,
    pub(crate) trimmed_basis_valid_count: usize,
    pub(crate) trimmed_parameter_direction_pass_count: usize,
    pub(crate) trimmed_basis_line_count: usize,
    pub(crate) trimmed_basis_circle_count: usize,
    pub(crate) trimmed_basis_ellipse_count: usize,
    pub(crate) trimmed_basis_b_curve_count: usize,
    pub(crate) trimmed_basis_sp_curve_count: usize,
    pub(crate) trimmed_basis_trimmed_curve_count: usize,

    pub(crate) sp_curve_count: usize,
    pub(crate) sp_surface_link_count: usize,
    pub(crate) sp_b_curve_link_count: usize,
    pub(crate) unique_sp_b_curve_count: usize,

    pub(crate) b_curve_count: usize,
    pub(crate) b_curve_nurbs_link_count: usize,
    pub(crate) unique_b_curve_nurbs_count: usize,

    pub(crate) nurbs_curve_count: usize,
    pub(crate) nurbs_payload_contract_count: usize,
    pub(crate) nurbs_knot_relation_pass_count: usize,
    pub(crate) nurbs_knot_order_pass_count: usize,
    pub(crate) rational_nurbs_count: usize,
    pub(crate) periodic_nurbs_count: usize,
    pub(crate) closed_nurbs_count: usize,
    pub(crate) nurbs_degree_min: i64,
    pub(crate) nurbs_degree_max: i64,
    pub(crate) nurbs_vertex_dim_min: i64,
    pub(crate) nurbs_vertex_dim_max: i64,

    pub(crate) referenced_bspline_vertex_node_count: usize,
    pub(crate) referenced_knot_mult_node_count: usize,
    pub(crate) referenced_knot_set_node_count: usize,
    pub(crate) bspline_scalar_count: usize,
    pub(crate) knot_value_count: usize,
    pub(crate) knot_multiplicity_count: usize,

    pub(crate) max_unit_vector_error: f64,
    pub(crate) max_orthogonality_error: f64,
    pub(crate) max_cone_trig_identity_error: f64,

    pub(crate) vertex_bounds_min_m: Option<[f64; 3]>,
    pub(crate) vertex_bounds_max_m: Option<[f64; 3]>,
    pub(crate) vertex_bounds_min_mm: Option<[f64; 3]>,
    pub(crate) vertex_bounds_max_mm: Option<[f64; 3]>,
    pub(crate) metres_to_millimetres: f64,

    pub(crate) sp_uv_contract_count: usize,
    pub(crate) sp_original_null_count: usize,
    pub(crate) sp_original_nonnull_count: usize,

    pub(crate) sp_support_plane_count: usize,
    pub(crate) sp_support_cylinder_count: usize,
    pub(crate) sp_support_cone_count: usize,

    pub(crate) sp_nonrational_uv_count: usize,
    pub(crate) sp_rational_uv_count: usize,

    pub(crate) trimmed_sp_curve_count: usize,
    pub(crate) unique_trimmed_sp_basis_count: usize,
    pub(crate) unwrapped_sp_curve_count: usize,
    pub(crate) sp_trim_reference_min: usize,
    pub(crate) sp_trim_reference_max: usize,

    pub(crate) trimmed_sp_direct_endpoint_order_count: usize,
    pub(crate) trimmed_sp_reversed_endpoint_order_count: usize,
    pub(crate) trimmed_sp_ambiguous_endpoint_order_count: usize,
    pub(crate) trimmed_sp_endpoint_witness_count: usize,

    pub(crate) direct_edge_trimmed_sp_count: usize,
    pub(crate) fin_trimmed_sp_count: usize,

    pub(crate) max_sp_embedding_error_m: f64,
    pub(crate) max_sp_embedding_error_mm: f64,
    pub(crate) cadir_coincidence_tolerance_mm: f64,
}
#[derive(Debug, Clone)]
pub(crate) struct Sch13006TopologyQualification {
    pub(crate) face_tolerance_count: usize,
    pub(crate) edge_tolerance_count: usize,
    pub(crate) vertex_tolerance_count: usize,

    pub(crate) max_face_tolerance_m: f64,
    pub(crate) max_edge_tolerance_m: f64,
    pub(crate) max_vertex_tolerance_m: f64,

    pub(crate) face_positive_sense_count: usize,
    pub(crate) face_negative_sense_count: usize,

    pub(crate) fin_positive_sense_count: usize,
    pub(crate) fin_negative_sense_count: usize,

    pub(crate) surface_positive_sense_count: usize,
    pub(crate) surface_negative_sense_count: usize,

    pub(crate) solid_shell_face_use_count: usize,
    pub(crate) solid_shell_opposite_void_count: usize,

    pub(crate) loop_with_fin_count: usize,
    pub(crate) fin_loop_owner_match_count: usize,
    pub(crate) fin_forward_backward_reciprocity_count: usize,
    pub(crate) radial_fin_count: usize,
    pub(crate) fin_endpoint_sense_match_count: usize,

    pub(crate) edge_radial_ring_count: usize,
    pub(crate) edge_anchor_sense_count: usize,

    pub(crate) singular_fin_count: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct Sch13006CadirQualification {
    pub(crate) validation_ok: bool,
    pub(crate) validation_error_count: usize,
    pub(crate) validation_warning_count: usize,

    pub(crate) body_count: usize,
    pub(crate) region_count: usize,
    pub(crate) shell_count: usize,
    pub(crate) face_count: usize,
    pub(crate) loop_count: usize,
    pub(crate) coedge_count: usize,
    pub(crate) singular_vertex_loop_count: usize,
    pub(crate) edge_count: usize,
    pub(crate) vertex_count: usize,
    pub(crate) point_count: usize,
    pub(crate) surface_count: usize,
    pub(crate) curve_count: usize,
    pub(crate) pcurve_count: usize,
    pub(crate) pcurve_use_count: usize,
    pub(crate) edge_parameter_range_count: usize,

    pub(crate) face_forward_sense_count: usize,
    pub(crate) face_reversed_sense_count: usize,
    pub(crate) coedge_forward_sense_count: usize,
    pub(crate) coedge_reversed_sense_count: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct Sch13006Census {
    pub(crate) topology_qualification:
        Sch13006TopologyQualification,
    pub(crate) cadir_qualification:
        Option<Sch13006CadirQualification>,
    pub(crate) cadir:
        Option<cadmpeg_ir::CadIr>,
    pub(crate) catalog_type_count: usize,
    pub(crate) total_entities: usize,
    pub(crate) terminator_index: u32,

    pub(crate) root_variable_length: u32,
    pub(crate) root_next_block: u32,
    pub(crate) root_entry_indices: Vec<u32>,

    pub(crate) body_count: usize,
    pub(crate) solid_body_count: usize,

    pub(crate) region_count: usize,
    pub(crate) solid_region_count: usize,
    pub(crate) void_region_count: usize,

    pub(crate) shell_count: usize,
    pub(crate) solid_region_shell_count: usize,

    pub(crate) face_count: usize,
    pub(crate) loop_count: usize,
    pub(crate) fin_count: usize,
    pub(crate) edge_count: usize,
    pub(crate) vertex_count: usize,
    pub(crate) point_count: usize,

    pub(crate) reachable_face_count: usize,
    pub(crate) reachable_loop_count: usize,
    pub(crate) reachable_fin_count: usize,
    pub(crate) reachable_edge_count: usize,
    pub(crate) reachable_vertex_count: usize,
    pub(crate) reachable_point_count: usize,

    pub(crate) validated_topology_pointer_count: usize,

    pub(crate) geometry_bindings: Sch13006GeometryBindings,

    pub(crate) line_count: usize,
    pub(crate) circle_count: usize,
    pub(crate) ellipse_count: usize,
    pub(crate) plane_count: usize,
    pub(crate) cylinder_count: usize,
    pub(crate) cone_count: usize,

    pub(crate) trimmed_curve_count: usize,
    pub(crate) sp_curve_count: usize,
    pub(crate) b_curve_count: usize,
    pub(crate) nurbs_curve_count: usize,
}

fn cadir_entity_id(
    kind: &str,
    index: u32,
) -> String {
    format!(
        "parasolid:sch13006:{kind}#{index}"
    )
}

fn cadir_point3(
    value: [f64; 3],
) -> cadmpeg_ir::math::Point3 {
    cadmpeg_ir::math::Point3::new(
        value[0] * METRES_TO_MILLIMETRES,
        value[1] * METRES_TO_MILLIMETRES,
        value[2] * METRES_TO_MILLIMETRES,
    )
}

fn cadir_vector3(
    value: [f64; 3],
) -> cadmpeg_ir::math::Vector3 {
    cadmpeg_ir::math::Vector3::new(
        value[0],
        value[1],
        value[2],
    )
}

fn cadir_native_sense(
    value: u8,
    label: &str,
) -> Result<cadmpeg_ir::topology::Sense, CodecError> {
    match value {
        b'+' => Ok(
            cadmpeg_ir::topology::Sense::Forward
        ),
        b'-' => Ok(
            cadmpeg_ir::topology::Sense::Reversed
        ),
        other => Err(
            CodecError::Malformed(
                format!(
                    "{label} has unsupported native sense 0x{other:02x}"
                ),
            ),
        ),
    }
}

fn cadir_region_chain_indices(
    start: u32,
    regions: &BTreeMap<u32, RegionNode>,
) -> Result<Vec<u32>, CodecError> {
    let mut output =
        Vec::<u32>::new();

    let mut seen =
        BTreeSet::<u32>::new();

    let mut current =
        start;

    while current != 0 {
        if !seen.insert(current) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "CADIR region chain cycles at REGION {current}"
                    ),
                ),
            );
        }

        let region =
            regions.get(
                &current
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "CADIR region chain references missing REGION {current}"
                    ),
                )
            })?;

        output.push(
            current
        );

        current =
            region.next;
    }

    Ok(output)
}

fn cadir_shell_chain_indices(
    start: u32,
    shells: &BTreeMap<u32, ShellNode>,
) -> Result<Vec<u32>, CodecError> {
    let mut output =
        Vec::<u32>::new();

    let mut seen =
        BTreeSet::<u32>::new();

    let mut current =
        start;

    while current != 0 {
        if !seen.insert(current) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "CADIR shell chain cycles at SHELL {current}"
                    ),
                ),
            );
        }

        let shell =
            shells.get(
                &current
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "CADIR shell chain references missing SHELL {current}"
                    ),
                )
            })?;

        output.push(
            current
        );

        current =
            shell.next;
    }

    Ok(output)
}

fn cadir_face_chain_indices(
    start: u32,
    front: bool,
    faces: &BTreeMap<u32, FaceNode>,
) -> Result<Vec<u32>, CodecError> {
    let mut output =
        Vec::<u32>::new();

    let mut seen =
        BTreeSet::<u32>::new();

    let mut current =
        start;

    while current != 0 {
        if !seen.insert(current) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "CADIR face chain cycles at FACE {current}"
                    ),
                ),
            );
        }

        let face =
            faces.get(
                &current
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "CADIR face chain references missing FACE {current}"
                    ),
                )
            })?;

        output.push(
            current
        );

        current =
            if front {
                face.next_front
            }
            else {
                face.next
            };
    }

    Ok(output)
}

fn cadir_loop_chain_indices(
    start: u32,
    loops: &BTreeMap<u32, LoopNode>,
) -> Result<Vec<u32>, CodecError> {
    let mut output =
        Vec::<u32>::new();

    let mut seen =
        BTreeSet::<u32>::new();

    let mut current =
        start;

    while current != 0 {
        if !seen.insert(current) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "CADIR loop chain cycles at LOOP {current}"
                    ),
                ),
            );
        }

        let loop_node =
            loops.get(
                &current
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "CADIR loop chain references missing LOOP {current}"
                    ),
                )
            })?;

        output.push(
            current
        );

        current =
            loop_node.next;
    }

    Ok(output)
}

fn cadir_fin_ring_indices(
    start: u32,
    fins: &BTreeMap<u32, FinNode>,
) -> Result<Vec<u32>, CodecError> {
    if start == 0 {
        return Err(
            CodecError::Malformed(
                "CADIR LOOP has a null FIN head"
                    .to_string(),
            ),
        );
    }

    let mut output =
        Vec::<u32>::new();

    let mut seen =
        BTreeSet::<u32>::new();

    let mut current =
        start;

    loop {
        if !seen.insert(current) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "CADIR FIN forward ring repeats before closure at FIN {current}"
                    ),
                ),
            );
        }

        let fin =
            fins.get(
                &current
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "CADIR FIN ring references missing FIN {current}"
                    ),
                )
            })?;

        output.push(
            current
        );

        current =
            fin.forward;

        if current == start {
            break;
        }

        if current == 0 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "CADIR FIN ring from {start} terminates at null"
                    ),
                ),
            );
        }

        if output.len() > fins.len() {
            return Err(
                CodecError::Malformed(
                    "CADIR FIN ring exceeds FIN population"
                        .to_string(),
                ),
            );
        }
    }

    Ok(output)
}

fn cadir_surface_native_sense(
    index: u32,
    planes: &BTreeMap<u32, PlaneGeometry>,
    cylinders: &BTreeMap<u32, CylinderGeometry>,
    cones: &BTreeMap<u32, ConeGeometry>,
) -> Result<u8, CodecError> {
    if let Some(value) =
        planes.get(
            &index
        )
    {
        return Ok(
            value.native_sense
        );
    }

    if let Some(value) =
        cylinders.get(
            &index
        )
    {
        return Ok(
            value.native_sense
        );
    }

    if let Some(value) =
        cones.get(
            &index
        )
    {
        return Ok(
            value.native_sense
        );
    }

    Err(
        CodecError::Malformed(
            format!(
                "CADIR face surface {index} has no qualified analytic carrier"
            ),
        ),
    )
}

fn cadir_surface_geometry(
    index: u32,
    planes: &BTreeMap<u32, PlaneGeometry>,
    cylinders: &BTreeMap<u32, CylinderGeometry>,
    cones: &BTreeMap<u32, ConeGeometry>,
) -> Result<cadmpeg_ir::geometry::SurfaceGeometry, CodecError> {
    if let Some(value) =
        planes.get(
            &index
        )
    {
        return Ok(
            cadmpeg_ir::geometry::SurfaceGeometry::Plane {
                origin:
                    cadir_point3(
                        value.pvec
                    ),

                normal:
                    cadir_vector3(
                        value.normal
                    ),

                u_axis:
                    cadir_vector3(
                        value.x_axis
                    ),
            },
        );
    }

    if let Some(value) =
        cylinders.get(
            &index
        )
    {
        return Ok(
            cadmpeg_ir::geometry::SurfaceGeometry::Cylinder {
                origin:
                    cadir_point3(
                        value.pvec
                    ),

                axis:
                    cadir_vector3(
                        value.axis
                    ),

                ref_direction:
                    cadir_vector3(
                        value.x_axis
                    ),

                radius:
                    value.radius *
                    METRES_TO_MILLIMETRES,
            },
        );
    }

    if let Some(value) =
        cones.get(
            &index
        )
    {
        let half_angle =
            value
                .sin_half_angle
                .atan2(
                    value.cos_half_angle
                );

        if !half_angle.is_finite() {
            return Err(
                CodecError::Malformed(
                    format!(
                        "CONE {index} produced a non-finite half angle"
                    ),
                ),
            );
        }

        return Ok(
            cadmpeg_ir::geometry::SurfaceGeometry::Cone {
                origin:
                    cadir_point3(
                        value.pvec
                    ),

                axis:
                    cadir_vector3(
                        value.axis
                    ),

                ref_direction:
                    cadir_vector3(
                        value.x_axis
                    ),

                radius:
                    value.radius *
                    METRES_TO_MILLIMETRES,

                ratio:
                    1.0,

                half_angle,
            },
        );
    }

    Err(
        CodecError::NotImplemented(
            format!(
                "surface {index} has no qualified SCH13006 -> CADIR analytic mapping"
            ),
        ),
    )
}

fn cadir_curve_base_index(
    index: u32,
    type_by_index: &BTreeMap<u32, u32>,
    trimmed_curves: &BTreeMap<u32, TrimmedCurveGeometry>,
    depth: usize,
) -> Result<u32, CodecError> {
    if depth > 32 {
        return Err(
            CodecError::Malformed(
                "trimmed-curve nesting exceeds CADIR qualification limit"
                    .to_string(),
            ),
        );
    }

    match type_by_index
        .get(
            &index
        )
        .copied()
    {
        Some(
            LINE |
            CIRCLE |
            ELLIPSE
        ) => Ok(index),

        Some(
            TRIMMED_CURVE
        ) => {
            let trimmed =
                trimmed_curves.get(
                    &index
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "missing TRIMMED_CURVE {index}"
                        ),
                    )
                })?;

            cadir_curve_base_index(
                trimmed.basis_curve,
                type_by_index,
                trimmed_curves,
                depth + 1,
            )
        }

        Some(other) => Err(
            CodecError::NotImplemented(
                format!(
                    "direct CADIR edge curve {index} resolves to unsupported type {other}"
                ),
            ),
        ),

        None => Err(
            CodecError::Malformed(
                format!(
                    "direct CADIR edge curve {index} is missing"
                ),
            ),
        ),
    }
}

fn cadir_curve_geometry(
    index: u32,
    type_by_index: &BTreeMap<u32, u32>,
    trimmed_curves: &BTreeMap<u32, TrimmedCurveGeometry>,
    lines: &BTreeMap<u32, LineGeometry>,
    circles: &BTreeMap<u32, CircleGeometry>,
    ellipses: &BTreeMap<u32, EllipseGeometry>,
) -> Result<cadmpeg_ir::geometry::CurveGeometry, CodecError> {
    let base =
        cadir_curve_base_index(
            index,
            type_by_index,
            trimmed_curves,
            0,
        )?;

    match type_by_index
        .get(
            &base
        )
        .copied()
    {
        Some(LINE) => {
            let value =
                lines.get(
                    &base
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "missing LINE {base}"
                        ),
                    )
                })?;

            Ok(
                cadmpeg_ir::geometry::CurveGeometry::Line {
                    origin:
                        cadir_point3(
                            value.pvec
                        ),

                    direction:
                        cadir_vector3(
                            value.direction
                        ),
                },
            )
        }

        Some(CIRCLE) => {
            let value =
                circles.get(
                    &base
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "missing CIRCLE {base}"
                        ),
                    )
                })?;

            Ok(
                cadmpeg_ir::geometry::CurveGeometry::Circle {
                    center:
                        cadir_point3(
                            value.centre
                        ),

                    axis:
                        cadir_vector3(
                            value.normal
                        ),

                    ref_direction:
                        cadir_vector3(
                            value.x_axis
                        ),

                    radius:
                        value.radius *
                        METRES_TO_MILLIMETRES,
                },
            )
        }

        Some(ELLIPSE) => {
            let value =
                ellipses.get(
                    &base
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "missing ELLIPSE {base}"
                        ),
                    )
                })?;

            Ok(
                cadmpeg_ir::geometry::CurveGeometry::Ellipse {
                    center:
                        cadir_point3(
                            value.centre
                        ),

                    axis:
                        cadir_vector3(
                            value.normal
                        ),

                    major_direction:
                        cadir_vector3(
                            value.x_axis
                        ),

                    major_radius:
                        value.major_radius *
                        METRES_TO_MILLIMETRES,

                    minor_radius:
                        value.minor_radius *
                        METRES_TO_MILLIMETRES,
                },
            )
        }

        _ => Err(
            CodecError::Malformed(
                format!(
                    "qualified direct curve base {base} is inconsistent"
                ),
            ),
        ),
    }
}

fn cadir_effective_curve_sense(
    index: u32,
    type_by_index: &BTreeMap<u32, u32>,
    trimmed_curves: &BTreeMap<u32, TrimmedCurveGeometry>,
    curve_sense_by_index: &BTreeMap<u32, u8>,
) -> Result<u8, CodecError> {
    let base =
        cadir_curve_base_index(
            index,
            type_by_index,
            trimmed_curves,
            0,
        )?;

    curve_sense_by_index
        .get(
            &base
        )
        .copied()
        .ok_or_else(|| {
            CodecError::Malformed(
                format!(
                    "qualified CADIR curve base {base} has no native sense"
                ),
            )
        })
}

fn cadir_periodic_parameter_from_point(
    geometry: &cadmpeg_ir::geometry::CurveGeometry,
    point: cadmpeg_ir::math::Point3,
    label: &str,
) -> Result<f64, CodecError> {
    let (
        center,
        axis,
        reference,
        x_radius,
        y_radius,
    ) =
        match geometry {
            cadmpeg_ir::geometry::CurveGeometry::Circle {
                center,
                axis,
                ref_direction,
                radius,
            } => (
                *center,
                *axis,
                *ref_direction,
                *radius,
                *radius,
            ),

            cadmpeg_ir::geometry::CurveGeometry::Ellipse {
                center,
                axis,
                major_direction,
                major_radius,
                minor_radius,
            } => (
                *center,
                *axis,
                *major_direction,
                *major_radius,
                *minor_radius,
            ),

            _ => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "{label} is not a qualified periodic analytic curve"
                        ),
                    ),
                )
            }
        };

    if !x_radius.is_finite() ||
        !y_radius.is_finite() ||
        x_radius <= 0.0 ||
        y_radius <= 0.0
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "{label} has invalid periodic radii x={x_radius} y={y_radius}"
                ),
            ),
        );
    }

    let delta_x =
        point.x -
        center.x;

    let delta_y =
        point.y -
        center.y;

    let delta_z =
        point.z -
        center.z;

    let transverse_x =
        axis.y *
        reference.z -
        axis.z *
        reference.y;

    let transverse_y =
        axis.z *
        reference.x -
        axis.x *
        reference.z;

    let transverse_z =
        axis.x *
        reference.y -
        axis.y *
        reference.x;

    let x =
        (
            delta_x *
            reference.x +
            delta_y *
            reference.y +
            delta_z *
            reference.z
        ) /
        x_radius;

    let y =
        (
            delta_x *
            transverse_x +
            delta_y *
            transverse_y +
            delta_z *
            transverse_z
        ) /
        y_radius;

    if !x.is_finite() ||
        !y.is_finite()
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "{label} produced non-finite periodic endpoint coordinates"
                ),
            ),
        );
    }

    let parameter =
        y.atan2(
            x
        );

    if !parameter.is_finite() {
        return Err(
            CodecError::Malformed(
                format!(
                    "{label} produced a non-finite periodic endpoint parameter"
                ),
            ),
        );
    }

    let evaluated =
        cadmpeg_ir::eval::curve_point(
            geometry,
            parameter,
        )
        .ok_or_else(|| {
            CodecError::Malformed(
                format!(
                    "{label} cannot evaluate recovered periodic parameter={parameter}"
                ),
            )
        })?;

    let error =
        cadir_distance3(
            evaluated,
            point,
        );

    let tolerance =
        cadmpeg_ir::units::COINCIDENCE_TOLERANCE;

    if !error.is_finite() ||
        error >
        tolerance
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "{label} periodic endpoint parameter witness failed: parameter={parameter} errorMm={error} toleranceMm={tolerance}"
                ),
            ),
        );
    }

    Ok(parameter)
}

fn cadir_expand_nurbs_knots(
    nurbs_index: u32,
    nurbs: &NurbsCurveGeometry,
    knot_multiplicities: &BTreeMap<u32, Vec<i16>>,
    knot_sets: &BTreeMap<u32, Vec<f64>>,
) -> Result<Vec<f64>, CodecError> {
    let multiplicities =
        knot_multiplicities.get(
            &nurbs.knot_mult
        )
        .ok_or_else(|| {
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {nurbs_index} missing KNOT_MULT {}",
                    nurbs.knot_mult
                ),
            )
        })?;

    let values =
        knot_sets.get(
            &nurbs.knots
        )
        .ok_or_else(|| {
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {nurbs_index} missing KNOT_SET {}",
                    nurbs.knots
                ),
            )
        })?;

    if multiplicities.len() !=
        values.len()
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {nurbs_index} knot/multiplicity length mismatch"
                ),
            ),
        );
    }

    let expected_distinct =
        usize::try_from(
            nurbs.n_knots
        )
        .map_err(|_| {
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {nurbs_index} n_knots is invalid"
                ),
            )
        })?;

    if values.len() !=
        expected_distinct
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {nurbs_index} distinct knot count mismatch"
                ),
            ),
        );
    }

    let mut expanded =
        Vec::<f64>::new();

    for (
        knot,
        multiplicity,
    ) in values
        .iter()
        .copied()
        .zip(
            multiplicities
                .iter()
                .copied()
        )
    {
        if multiplicity <= 0 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {nurbs_index} has non-positive knot multiplicity {multiplicity}"
                    ),
                ),
            );
        }

        let count =
            usize::try_from(
                multiplicity
            )
            .map_err(|_| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {nurbs_index} knot multiplicity does not fit usize"
                    ),
                )
            })?;

        expanded.extend(
            std::iter::repeat(
                knot
            )
            .take(
                count
            ),
        );
    }

    if !expanded
        .windows(2)
        .all(
            |pair|
                pair[0] <= pair[1]
        )
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {nurbs_index} expanded knots are not nondecreasing"
                ),
            ),
        );
    }

    Ok(expanded)
}

fn cadir_sp_pcurve(
    fin_index: u32,
    face_surface: u32,
    fin: &FinNode,
    type_by_index: &BTreeMap<u32, u32>,
    trimmed_curves: &BTreeMap<u32, TrimmedCurveGeometry>,
    sp_curves: &BTreeMap<u32, SpCurveGeometry>,
    b_curves: &BTreeMap<u32, BCurveGeometry>,
    nurbs_curves: &BTreeMap<u32, NurbsCurveGeometry>,
    bspline_vertices: &BTreeMap<u32, Vec<f64>>,
    knot_multiplicities: &BTreeMap<u32, Vec<i16>>,
    knot_sets: &BTreeMap<u32, Vec<f64>>,
) -> Result<
    (
        cadmpeg_ir::geometry::Pcurve,
        cadmpeg_ir::topology::PcurveUse,
    ),
    CodecError,
> {
    let trimmed =
        trimmed_curves.get(
            &fin.curve
        )
        .ok_or_else(|| {
            CodecError::NotImplemented(
                format!(
                    "curveless EDGE FIN {fin_index} does not carry a qualified TRIMMED_CURVE"
                ),
            )
        })?;

    if type_by_index
        .get(
            &trimmed.basis_curve
        )
        .copied() !=
        Some(SP_CURVE)
    {
        return Err(
            CodecError::NotImplemented(
                format!(
                    "curveless EDGE FIN {fin_index} TRIMMED_CURVE {} is not SP_CURVE based",
                    fin.curve
                ),
            ),
        );
    }

    let sp =
        sp_curves.get(
            &trimmed.basis_curve
        )
        .ok_or_else(|| {
            CodecError::Malformed(
                format!(
                    "missing SP_CURVE {}",
                    trimmed.basis_curve
                ),
            )
        })?;

    if sp.surface !=
        face_surface
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "FIN {fin_index} SP_CURVE {} surface {} does not match owning FACE surface {face_surface}",
                    trimmed.basis_curve,
                    sp.surface
                ),
            ),
        );
    }

    if sp.original != 0 {
        return Err(
            CodecError::NotImplemented(
                format!(
                    "SP_CURVE {} original pointer {} is outside the qualified CADIR profile",
                    trimmed.basis_curve,
                    sp.original
                ),
            ),
        );
    }

    let b_curve =
        b_curves.get(
            &sp.b_curve
        )
        .ok_or_else(|| {
            CodecError::Malformed(
                format!(
                    "SP_CURVE {} references missing B_CURVE {}",
                    trimmed.basis_curve,
                    sp.b_curve
                ),
            )
        })?;

    let nurbs =
        nurbs_curves.get(
            &b_curve.nurbs
        )
        .ok_or_else(|| {
            CodecError::Malformed(
                format!(
                    "B_CURVE {} references missing NURBS_CURVE {}",
                    sp.b_curve,
                    b_curve.nurbs
                ),
            )
        })?;

    if nurbs.vertex_dim != 2 {
        return Err(
            CodecError::NotImplemented(
                format!(
                    "SP_CURVE {} NURBS vertex_dim={} is outside the qualified 2D UV profile",
                    trimmed.basis_curve,
                    nurbs.vertex_dim
                ),
            ),
        );
    }

    if nurbs.rational ||
        nurbs.periodic ||
        nurbs.closed
    {
        return Err(
            CodecError::NotImplemented(
                format!(
                    "SP_CURVE {} requires an unqualified rational/periodic/closed UV NURBS mapping",
                    trimmed.basis_curve
                ),
            ),
        );
    }

    let degree =
        u32::try_from(
            nurbs.degree
        )
        .map_err(|_| {
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {} has invalid degree {}",
                    b_curve.nurbs,
                    nurbs.degree
                ),
            )
        })?;

    let control_count =
        usize::try_from(
            nurbs.n_vertices
        )
        .map_err(|_| {
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {} has invalid control count {}",
                    b_curve.nurbs,
                    nurbs.n_vertices
                ),
            )
        })?;

    let raw =
        bspline_vertices.get(
            &nurbs.bspline_vertices
        )
        .ok_or_else(|| {
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {} references missing BSPLINE_VERTICES {}",
                    b_curve.nurbs,
                    nurbs.bspline_vertices
                ),
            )
        })?;

    let expected_scalars =
        control_count
            .checked_mul(2)
            .ok_or_else(|| {
                CodecError::Malformed(
                    "SP UV control scalar count overflow"
                        .to_string(),
                )
            })?;

    if raw.len() !=
        expected_scalars
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {} UV scalar count mismatch expected={} actual={}",
                    b_curve.nurbs,
                    expected_scalars,
                    raw.len()
                ),
            ),
        );
    }

    let surface_type =
        type_by_index
            .get(
                &face_surface
            )
            .copied()
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "missing FACE support surface {face_surface}"
                    ),
                )
            })?;

    let mut control_points =
        Vec::<cadmpeg_ir::math::Point2>::with_capacity(
            control_count
        );

    for pair in
        raw.chunks_exact(2)
    {
        let (
            u,
            v,
        ) =
            match surface_type {
                PLANE => (
                    pair[0] *
                        METRES_TO_MILLIMETRES,

                    pair[1] *
                        METRES_TO_MILLIMETRES,
                ),

                CYLINDER |
                CONE => (
                    pair[0],

                    pair[1] *
                        METRES_TO_MILLIMETRES,
                ),

                other => {
                    return Err(
                        CodecError::NotImplemented(
                            format!(
                                "SP pcurve surface type {other} is outside the qualified analytic UV mapping"
                            ),
                        ),
                    )
                }
            };

        if !u.is_finite() ||
            !v.is_finite()
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "SP_CURVE {} produced non-finite CADIR UV control point",
                        trimmed.basis_curve
                    ),
                ),
            );
        }

        control_points.push(
            cadmpeg_ir::math::Point2::new(
                u,
                v,
            ),
        );
    }

    let knots =
        cadir_expand_nurbs_knots(
            b_curve.nurbs,
            nurbs,
            knot_multiplicities,
            knot_sets,
        )?;

    let required_knot_count =
        control_count
            .checked_add(
                usize::try_from(
                    degree
                )
                .map_err(|_| {
                    CodecError::Malformed(
                        "NURBS degree does not fit usize"
                            .to_string(),
                    )
                })?
            )
            .and_then(
                |value|
                    value.checked_add(1)
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    "NURBS CADIR knot relation overflow"
                        .to_string(),
                )
            })?;

    if knots.len() !=
        required_knot_count
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {} CADIR expanded knot relation failed expected={} actual={}",
                    b_curve.nurbs,
                    required_knot_count,
                    knots.len()
                ),
            ),
        );
    }

    let range =
        [
            trimmed.parm_1,
            trimmed.parm_2,
        ];

    if !range[0].is_finite() ||
        !range[1].is_finite() ||
        range[0] == range[1]
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "TRIMMED_CURVE {} has invalid pcurve range {:?}",
                    fin.curve,
                    range
                ),
            ),
        );
    }

    let id =
        cadmpeg_ir::ids::PcurveId(
            cadir_entity_id(
                "pcurve",
                fin_index,
            )
        );

    let pcurve =
        cadmpeg_ir::geometry::Pcurve {
            id:
                id.clone(),

            geometry:
                cadmpeg_ir::geometry::PcurveGeometry::Nurbs {
                    degree,
                    knots,
                    control_points,
                    weights:
                        None,
                    periodic:
                        false,
                },

            wrapper_reversed:
                None,

            native_tail_flags:
                None,

            parameter_range:
                Some(
                    range
                ),

            fit_tolerance:
                None,
        };

    let use_ =
        cadmpeg_ir::topology::PcurveUse {
            pcurve:
                id,

            isoparametric:
                None,

            parameter_range:
                Some(
                    range
                ),
        };

    Ok(
        (
            pcurve,
            use_,
        )
    )
}

fn cadir_distance3(
    left: cadmpeg_ir::math::Point3,
    right: cadmpeg_ir::math::Point3,
) -> f64 {
    let dx =
        left.x -
        right.x;

    let dy =
        left.y -
        right.y;

    let dz =
        left.z -
        right.z;

    (
        dx * dx +
        dy * dy +
        dz * dz
    )
        .sqrt()
}

fn cadir_pcurve_nurbs_point(
    pcurve: &cadmpeg_ir::geometry::Pcurve,
    parameter: f64,
) -> Result<cadmpeg_ir::math::Point2, CodecError> {
    if !parameter.is_finite() {
        return Err(
            CodecError::Malformed(
                "CADIR pcurve evaluation received a non-finite parameter"
                    .to_string(),
            ),
        );
    }

    let (
        degree,
        knots,
        control_points,
        weights,
        periodic,
    ) =
        match &pcurve.geometry {
            cadmpeg_ir::geometry::PcurveGeometry::Nurbs {
                degree,
                knots,
                control_points,
                weights,
                periodic,
            } => (
                *degree,
                knots,
                control_points,
                weights,
                *periodic,
            ),

            _ => {
                return Err(
                    CodecError::NotImplemented(
                        "plane-lift recovery requires a NURBS pcurve"
                            .to_string(),
                    ),
                )
            }
        };

    let chart =
        cadmpeg_ir::geometry::NurbsCurve {
            degree,

            knots:
                knots.clone(),

            control_points:
                control_points
                    .iter()
                    .map(
                        |point|
                            cadmpeg_ir::math::Point3::new(
                                point.u,
                                point.v,
                                0.0,
                            )
                    )
                    .collect(),

            weights:
                weights.clone(),

            periodic,
        };

    let point =
        cadmpeg_ir::eval::nurbs_curve_point(
            chart.degree,
            &chart.knots,
            &chart.control_points,
            chart.weights.as_deref(),
            parameter,
        )
        .ok_or_else(|| {
            CodecError::Malformed(
                format!(
                    "CADIR pcurve {} could not be evaluated at parameter {parameter}",
                    pcurve.id.0
                ),
            )
        })?;

    if !point.x.is_finite() ||
        !point.y.is_finite()
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "CADIR pcurve {} produced a non-finite UV point",
                    pcurve.id.0
                ),
            ),
        );
    }

    Ok(
        cadmpeg_ir::math::Point2::new(
            point.x,
            point.y,
        )
    )
}

fn cadir_surface_pcurve_point(
    surface:
        &cadmpeg_ir::geometry::SurfaceGeometry,
    pcurve:
        &cadmpeg_ir::geometry::Pcurve,
    parameter:
        f64,
) -> Result<cadmpeg_ir::math::Point3, CodecError> {
    let uv =
        cadir_pcurve_nurbs_point(
            pcurve,
            parameter,
        )?;

    cadmpeg_ir::eval::surface_point(
        surface,
        uv.u,
        uv.v,
    )
    .ok_or_else(|| {
        CodecError::Malformed(
            format!(
                "CADIR surface could not evaluate pcurve {} at parameter {parameter}",
                pcurve.id.0
            ),
        )
    })
}

fn cadir_pcurve_endpoint_sense(
    label:
        &str,
    surface:
        &cadmpeg_ir::geometry::SurfaceGeometry,
    pcurve:
        &cadmpeg_ir::geometry::Pcurve,
    range:
        [f64; 2],
    start:
        cadmpeg_ir::math::Point3,
    end:
        cadmpeg_ir::math::Point3,
) -> Result<u8, CodecError> {
    let first =
        cadir_surface_pcurve_point(
            surface,
            pcurve,
            range[0],
        )?;

    let last =
        cadir_surface_pcurve_point(
            surface,
            pcurve,
            range[1],
        )?;

    let direct_error =
        cadir_distance3(
            first,
            start,
        )
        .max(
            cadir_distance3(
                last,
                end,
            )
        );

    let reversed_error =
        cadir_distance3(
            first,
            end,
        )
        .max(
            cadir_distance3(
                last,
                start,
            )
        );

    let tolerance =
        cadmpeg_ir::units::COINCIDENCE_TOLERANCE;

    let direct_pass =
        direct_error <=
        tolerance;

    let reversed_pass =
        reversed_error <=
        tolerance;

    match (
        direct_pass,
        reversed_pass,
    ) {
        (
            true,
            false,
        ) => Ok(b'+'),

        (
            false,
            true,
        ) => Ok(b'-'),

        (
            true,
            true,
        ) => Err(
            CodecError::Malformed(
                format!(
                    "{label} pcurve endpoint orientation is ambiguous: directErrorMm={direct_error} reversedErrorMm={reversed_error} toleranceMm={tolerance}"
                ),
            ),
        ),

        (
            false,
            false,
        ) => Err(
            CodecError::Malformed(
                format!(
                    "{label} pcurve endpoints do not witness the EDGE endpoints: directErrorMm={direct_error} reversedErrorMm={reversed_error} toleranceMm={tolerance}"
                ),
            ),
        ),
    }
}

fn cadir_plane_cone_cross_support_witness(
    edge_index:
        u32,
    plane_surface:
        &cadmpeg_ir::geometry::SurfaceGeometry,
    plane_pcurve:
        &cadmpeg_ir::geometry::Pcurve,
    cone_surface:
        &cadmpeg_ir::geometry::SurfaceGeometry,
    cone_pcurve:
        &cadmpeg_ir::geometry::Pcurve,
    start:
        cadmpeg_ir::math::Point3,
    end:
        cadmpeg_ir::math::Point3,
) -> Result<
    (
        u8,
        [f64; 2],
        f64,
    ),
    CodecError,
> {
    const SAMPLE_COUNT:
        usize =
        129;

    let plane_range =
        plane_pcurve
            .parameter_range
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} plane pcurve has no physical parameter range"
                    ),
                )
            })?;

    let cone_range =
        cone_pcurve
            .parameter_range
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} cone pcurve has no physical parameter range"
                    ),
                )
            })?;

    if plane_range !=
        cone_range
    {
        return Err(
            CodecError::NotImplemented(
                format!(
                    "EDGE {edge_index} null-curve recovery requires identical plane/cone pcurve parameter ranges; plane={plane_range:?} cone={cone_range:?}"
                ),
            ),
        );
    }

    let plane_sense =
        cadir_pcurve_endpoint_sense(
            "plane",
            plane_surface,
            plane_pcurve,
            plane_range,
            start,
            end,
        )?;

    let cone_sense =
        cadir_pcurve_endpoint_sense(
            "cone",
            cone_surface,
            cone_pcurve,
            cone_range,
            start,
            end,
        )?;

    let mut max_error =
        0.0_f64;

    for sample in
        0 .. SAMPLE_COUNT
    {
        let alpha =
            sample as f64 /
            (
                SAMPLE_COUNT -
                1
            ) as f64;

        let plane_alpha =
            if plane_sense ==
                b'+'
            {
                alpha
            }
            else {
                1.0 -
                alpha
            };

        let cone_alpha =
            if cone_sense ==
                b'+'
            {
                alpha
            }
            else {
                1.0 -
                alpha
            };

        let plane_parameter =
            plane_range[0] +
            plane_alpha *
            (
                plane_range[1] -
                plane_range[0]
            );

        let cone_parameter =
            cone_range[0] +
            cone_alpha *
            (
                cone_range[1] -
                cone_range[0]
            );

        let plane_point =
            cadir_surface_pcurve_point(
                plane_surface,
                plane_pcurve,
                plane_parameter,
            )?;

        let cone_point =
            cadir_surface_pcurve_point(
                cone_surface,
                cone_pcurve,
                cone_parameter,
            )?;

        let error =
            cadir_distance3(
                plane_point,
                cone_point,
            );

        if !error.is_finite() {
            return Err(
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} plane/cone cross-support witness produced a non-finite error"
                    ),
                ),
            );
        }

        max_error =
            max_error.max(
                error
            );
    }

    let tolerance =
        cadmpeg_ir::units::COINCIDENCE_TOLERANCE;

    if max_error >
        tolerance
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "EDGE {edge_index} plane/cone pcurves disagree beyond CADIR coincidence tolerance: maxErrorMm={max_error} toleranceMm={tolerance}"
                ),
            ),
        );
    }

    Ok(
        (
            plane_sense,
            plane_range,
            max_error,
        )
    )
}

#[allow(clippy::too_many_arguments)]
fn build_qualified_cadir(
    root_entry_indices: &[u32],
    type_by_index: &BTreeMap<u32, u32>,
    bodies: &BTreeMap<u32, BodyNode>,
    regions: &BTreeMap<u32, RegionNode>,
    shells: &BTreeMap<u32, ShellNode>,
    faces: &BTreeMap<u32, FaceNode>,
    loops: &BTreeMap<u32, LoopNode>,
    fins: &BTreeMap<u32, FinNode>,
    edges: &BTreeMap<u32, EdgeNode>,
    vertices: &BTreeMap<u32, VertexNode>,
    points: &BTreeMap<u32, PointGeometry>,
    lines: &BTreeMap<u32, LineGeometry>,
    circles: &BTreeMap<u32, CircleGeometry>,
    ellipses: &BTreeMap<u32, EllipseGeometry>,
    planes: &BTreeMap<u32, PlaneGeometry>,
    cylinders: &BTreeMap<u32, CylinderGeometry>,
    cones: &BTreeMap<u32, ConeGeometry>,
    trimmed_curves: &BTreeMap<u32, TrimmedCurveGeometry>,
    sp_curves: &BTreeMap<u32, SpCurveGeometry>,
    b_curves: &BTreeMap<u32, BCurveGeometry>,
    nurbs_curves: &BTreeMap<u32, NurbsCurveGeometry>,
    bspline_vertices: &BTreeMap<u32, Vec<f64>>,
    knot_multiplicities: &BTreeMap<u32, Vec<i16>>,
    knot_sets: &BTreeMap<u32, Vec<f64>>,
    curve_sense_by_index: &BTreeMap<u32, u8>,
    solid_shells: &BTreeSet<u32>,
    reachable_faces: &BTreeSet<u32>,
    reachable_loops: &BTreeSet<u32>,
    reachable_fins: &BTreeSet<u32>,
    reachable_edges: &BTreeSet<u32>,
    reachable_vertices: &BTreeSet<u32>,
    reachable_points: &BTreeSet<u32>,
) -> Result<(cadmpeg_ir::CadIr, Sch13006CadirQualification), CodecError> {
    let mut ir =
        cadmpeg_ir::CadIr::empty(
            cadmpeg_ir::units::Units::default()
        );

    // --------------------------------------------------------
    // POINTS + VERTICES
    // --------------------------------------------------------

    let mut emitted_points =
        BTreeSet::<u32>::new();

    for vertex_index in
        reachable_vertices
    {
        let vertex =
            vertices.get(
                vertex_index
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "reachable VERTEX {vertex_index} disappeared during CADIR construction"
                    ),
                )
            })?;

        let point =
            points.get(
                &vertex.point
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "VERTEX {vertex_index} references missing POINT {}",
                        vertex.point
                    ),
                )
            })?;

        if emitted_points.insert(
            vertex.point
        ) {
            ir.model.points.push(
                cadmpeg_ir::topology::Point {
                    id:
                        cadmpeg_ir::ids::PointId(
                            cadir_entity_id(
                                "point",
                                vertex.point,
                            )
                        ),

                    position:
                        cadir_point3(
                            point.pvec
                        ),

                    source_object:
                        None,
                },
            );
        }

        ir.model.vertices.push(
            cadmpeg_ir::topology::Vertex {
                id:
                    cadmpeg_ir::ids::VertexId(
                        cadir_entity_id(
                            "vertex",
                            *vertex_index,
                        )
                    ),

                point:
                    cadmpeg_ir::ids::PointId(
                        cadir_entity_id(
                            "point",
                            vertex.point,
                        )
                    ),

                tolerance:
                    vertex
                        .tolerance
                        .map(
                            |value|
                                value *
                                METRES_TO_MILLIMETRES
                        ),
            },
        );
    }

    if &emitted_points !=
        reachable_points
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "CADIR point reachability mismatch emitted={} qualified={}",
                    emitted_points.len(),
                    reachable_points.len()
                ),
            ),
        );
    }

    // --------------------------------------------------------
    // EDGES + DIRECT 3D CURVE LOCI
    // --------------------------------------------------------

    let mut emitted_curves =
        BTreeSet::<u32>::new();

    let mut lifted_curve_sense_by_edge =
        BTreeMap::<u32, u8>::new();

    // CADIR requires carrier-backed parameter ranges to be ascending.
    // When canonical carrier direction is opposite the native EDGE anchor
    // direction, swap the CADIR edge endpoints and toggle all owning coedges.
    let mut reversed_direct_edge_orientation =
        BTreeSet::<u32>::new();

    let mut max_plane_cone_lift_error_mm =
        0.0_f64;

    for edge_index in
        reachable_edges
    {
        let edge =
            edges.get(
                edge_index
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "reachable EDGE {edge_index} disappeared during CADIR construction"
                    ),
                )
            })?;

        let anchor =
            fins.get(
                &edge.fin
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} has missing anchor FIN {}",
                        edge.fin
                    ),
                )
            })?;

        if anchor.edge !=
            *edge_index ||
            anchor.native_sense != b'+'
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} CADIR anchor contract failed"
                    ),
                ),
            );
        }

        let negative =
            fins.get(
                &anchor.other
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} positive anchor FIN {} has missing radial successor {}",
                        edge.fin,
                        anchor.other
                    ),
                )
            })?;

        if negative.edge !=
            *edge_index ||
            negative.native_sense != b'-'
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} negative endpoint FIN contract failed"
                    ),
                ),
            );
        }

        let (
            curve,
            edge_param_range,
        ) =
            if edge.curve == 0 {
                let support_for_fin =
                    |
                        fin_index:
                            u32,
                        fin:
                            &FinNode,
                    |
                    -> Result<
                        (
                            u32,
                            u32,
                        ),
                        CodecError,
                    >
                    {
                        let loop_node =
                            loops.get(
                                &fin.loop_owner
                            )
                            .ok_or_else(|| {
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} FIN {fin_index} references missing LOOP {}",
                                        fin.loop_owner
                                    ),
                                )
                            })?;

                        let face =
                            faces.get(
                                &loop_node.face
                            )
                            .ok_or_else(|| {
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} FIN {fin_index} LOOP {} references missing FACE {}",
                                        fin.loop_owner,
                                        loop_node.face
                                    ),
                                )
                            })?;

                        let surface_type =
                            type_by_index
                                .get(
                                    &face.surface
                                )
                                .copied()
                                .ok_or_else(|| {
                                    CodecError::Malformed(
                                        format!(
                                            "EDGE {edge_index} FIN {fin_index} FACE {} support surface {} has no schema type",
                                            loop_node.face,
                                            face.surface
                                        ),
                                    )
                                })?;

                        Ok(
                            (
                                face.surface,
                                surface_type,
                            )
                        )
                    };

                let (
                    anchor_surface,
                    anchor_surface_type,
                ) =
                    support_for_fin(
                        edge.fin,
                        anchor,
                    )?;

                let (
                    negative_surface,
                    negative_surface_type,
                ) =
                    support_for_fin(
                        anchor.other,
                        negative,
                    )?;

                let (
                    plane_fin_index,
                    plane_fin,
                    plane_surface_index,
                    cone_fin_index,
                    cone_fin,
                    cone_surface_index,
                ) =
                    match (
                        anchor_surface_type,
                        negative_surface_type,
                    ) {
                        (
                            PLANE,
                            CONE,
                        ) => (
                            edge.fin,
                            anchor,
                            anchor_surface,
                            anchor.other,
                            negative,
                            negative_surface,
                        ),

                        (
                            CONE,
                            PLANE,
                        ) => (
                            anchor.other,
                            negative,
                            negative_surface,
                            edge.fin,
                            anchor,
                            anchor_surface,
                        ),

                        (
                            first,
                            second,
                        ) => {
                            return Err(
                                CodecError::NotImplemented(
                                    format!(
                                        "EDGE {edge_index} has no native 3D curve and is outside the qualified plane/cone recovery profile: radial support types=({first},{second})"
                                    ),
                                ),
                            )
                        }
                    };

                let (
                    plane_pcurve,
                    _,
                ) =
                    cadir_sp_pcurve(
                        plane_fin_index,
                        plane_surface_index,
                        plane_fin,
                        type_by_index,
                        trimmed_curves,
                        sp_curves,
                        b_curves,
                        nurbs_curves,
                        bspline_vertices,
                        knot_multiplicities,
                        knot_sets,
                    )?;

                let (
                    cone_pcurve,
                    _,
                ) =
                    cadir_sp_pcurve(
                        cone_fin_index,
                        cone_surface_index,
                        cone_fin,
                        type_by_index,
                        trimmed_curves,
                        sp_curves,
                        b_curves,
                        nurbs_curves,
                        bspline_vertices,
                        knot_multiplicities,
                        knot_sets,
                    )?;

                let plane_surface =
                    cadir_surface_geometry(
                        plane_surface_index,
                        planes,
                        cylinders,
                        cones,
                    )?;

                let cone_surface =
                    cadir_surface_geometry(
                        cone_surface_index,
                        planes,
                        cylinders,
                        cones,
                    )?;

                let start_vertex =
                    vertices.get(
                        &anchor.vertex
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "EDGE {edge_index} start VERTEX {} is missing",
                                anchor.vertex
                            ),
                        )
                    })?;

                let end_vertex =
                    vertices.get(
                        &negative.vertex
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "EDGE {edge_index} end VERTEX {} is missing",
                                negative.vertex
                            ),
                        )
                    })?;

                let start_geometry =
                    points.get(
                        &start_vertex.point
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "EDGE {edge_index} start VERTEX {} POINT {} is missing",
                                anchor.vertex,
                                start_vertex.point
                            ),
                        )
                    })?;

                let end_geometry =
                    points.get(
                        &end_vertex.point
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "EDGE {edge_index} end VERTEX {} POINT {} is missing",
                                negative.vertex,
                                end_vertex.point
                            ),
                        )
                    })?;

                let start_point =
                    cadir_point3(
                        start_geometry.pvec
                    );

                let end_point =
                    cadir_point3(
                        end_geometry.pvec
                    );

                let (
                    lifted_curve_sense,
                    lifted_range,
                    cross_support_error,
                ) =
                    cadir_plane_cone_cross_support_witness(
                        *edge_index,
                        &plane_surface,
                        &plane_pcurve,
                        &cone_surface,
                        &cone_pcurve,
                        start_point,
                        end_point,
                    )?;

                max_plane_cone_lift_error_mm =
                    max_plane_cone_lift_error_mm
                        .max(
                            cross_support_error
                        );

                let (
                    degree,
                    knots,
                    control_points,
                    weights,
                    periodic,
                ) =
                    match &plane_pcurve.geometry {
                        cadmpeg_ir::geometry::PcurveGeometry::Nurbs {
                            degree,
                            knots,
                            control_points,
                            weights,
                            periodic,
                        } => (
                            *degree,
                            knots,
                            control_points,
                            weights,
                            *periodic,
                        ),

                        _ => {
                            return Err(
                                CodecError::NotImplemented(
                                    format!(
                                        "EDGE {edge_index} plane-lift recovery requires a NURBS pcurve"
                                    ),
                                ),
                            )
                        }
                    };

                let lifted_control_points =
                    control_points
                        .iter()
                        .map(
                            |uv| {
                                cadmpeg_ir::eval::surface_point(
                                    &plane_surface,
                                    uv.u,
                                    uv.v,
                                )
                                .ok_or_else(|| {
                                    CodecError::Malformed(
                                        format!(
                                            "EDGE {edge_index} plane support could not lift NURBS control point ({},{})",
                                            uv.u,
                                            uv.v
                                        ),
                                    )
                                })
                            }
                        )
                        .collect::<
                            Result<
                                Vec<_>,
                                CodecError,
                            >
                        >()?;

                let curve_id =
                    cadmpeg_ir::ids::CurveId(
                        cadir_entity_id(
                            "plane-lift-curve",
                            *edge_index,
                        )
                    );

                if periodic {
                    return Err(
                        CodecError::NotImplemented(
                            format!(
                                "EDGE {edge_index} plane-lift recovery requires the physically qualified non-periodic NURBS profile"
                            ),
                        ),
                    );
                }

                if lifted_range[0] >
                    lifted_range[1]
                {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "EDGE {edge_index} plane pcurve trim interval is not in canonical ascending order: {lifted_range:?}"
                            ),
                        ),
                    );
                }

                let mut lifted_nurbs =
                    cadmpeg_ir::geometry::NurbsCurve {
                        degree,

                        knots:
                            knots.clone(),

                        control_points:
                            lifted_control_points,

                        weights:
                            weights.clone(),

                        periodic,
                    };

                let [
                    carrier_lower,
                    carrier_upper,
                ] =
                    cadmpeg_ir::eval::nurbs_curve_parameter_domain(
                        &lifted_nurbs
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "EDGE {edge_index} lifted NURBS has no canonical parameter domain"
                            ),
                        )
                    })?;

                let canonical_edge_range =
                    if lifted_curve_sense ==
                        b'+'
                    {
                        lifted_range
                    }
                    else {
                        let reflection =
                            carrier_lower +
                            carrier_upper;

                        if !reflection.is_finite() {
                            return Err(
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} lifted NURBS parameter reflection is non-finite"
                                    ),
                                ),
                            );
                        }

                        lifted_nurbs
                            .control_points
                            .reverse();

                        if let Some(
                            lifted_weights
                        ) =
                            lifted_nurbs
                                .weights
                                .as_mut()
                        {
                            lifted_weights
                                .reverse();
                        }

                        lifted_nurbs.knots =
                            lifted_nurbs
                                .knots
                                .iter()
                                .rev()
                                .map(
                                    |knot| {
                                        reflection -
                                        *knot
                                    }
                                )
                                .collect();

                        if lifted_nurbs
                            .knots
                            .iter()
                            .any(
                                |knot|
                                    !knot.is_finite()
                            )
                        {
                            return Err(
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} reversed lifted NURBS produced a non-finite knot"
                                    ),
                                ),
                            );
                        }

                        if lifted_nurbs
                            .knots
                            .windows(2)
                            .any(
                                |pair|
                                    pair[0] >
                                    pair[1]
                            )
                        {
                            return Err(
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} exact NURBS reversal did not preserve nondecreasing knots"
                                    ),
                                ),
                            );
                        }

                        let [
                            reflected_lower,
                            reflected_upper,
                        ] =
                            cadmpeg_ir::eval::nurbs_curve_parameter_domain(
                                &lifted_nurbs
                            )
                            .ok_or_else(|| {
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} reversed lifted NURBS has no canonical parameter domain"
                                    ),
                                )
                            })?;

                        let domain_scale =
                            carrier_lower
                                .abs()
                                .max(
                                    carrier_upper
                                        .abs()
                                )
                                .max(1.0);

                        let domain_tolerance =
                            1.0e-12 *
                            domain_scale;

                        if (
                            reflected_lower -
                            carrier_lower
                        )
                        .abs() >
                            domain_tolerance
                            ||
                            (
                                reflected_upper -
                                carrier_upper
                            )
                            .abs() >
                            domain_tolerance
                        {
                            return Err(
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} exact NURBS reversal changed its canonical carrier domain: before=[{carrier_lower},{carrier_upper}] after=[{reflected_lower},{reflected_upper}]"
                                    ),
                                ),
                            );
                        }

                        let reflected_range =
                            [
                                reflection -
                                    lifted_range[1],

                                reflection -
                                    lifted_range[0],
                            ];

                        if !reflected_range[0]
                            .is_finite()
                            ||
                            !reflected_range[1]
                                .is_finite()
                            ||
                            reflected_range[0] >
                                reflected_range[1]
                        {
                            return Err(
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} reversed lifted NURBS produced invalid canonical trim range {reflected_range:?}"
                                    ),
                                ),
                            );
                        }

                        if reflected_range[0] <
                            reflected_lower -
                                domain_tolerance
                            ||
                            reflected_range[1] >
                                reflected_upper +
                                domain_tolerance
                        {
                            return Err(
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} reflected trim range {reflected_range:?} lies outside canonical NURBS domain [{reflected_lower},{reflected_upper}]"
                                    ),
                                ),
                            );
                        }

                        reflected_range
                    };

                let curve_geometry =
                    cadmpeg_ir::geometry::CurveGeometry::Nurbs(
                        lifted_nurbs
                    );

                // The stored 3D carrier is now always parameterized
                // in EDGE start -> end direction.
                //
                // Shadow the original plane-pcurve sense so the
                // existing FIN-vs-curve coedge logic sees a canonical
                // forward carrier. This preserves the original
                // curveless-edge coedge senses exactly.
                let lifted_curve_sense =
                    b'+';

                let first =
                    cadmpeg_ir::eval::curve_point(
                        &curve_geometry,
                        canonical_edge_range[0],
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "EDGE {edge_index} lifted 3D NURBS cannot evaluate at trim start"
                            ),
                        )
                    })?;

                let last =
                    cadmpeg_ir::eval::curve_point(
                        &curve_geometry,
                        canonical_edge_range[1],
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "EDGE {edge_index} lifted 3D NURBS cannot evaluate at trim end"
                            ),
                        )
                    })?;

                let direct_error =
                    cadir_distance3(
                        first,
                        start_point,
                    )
                    .max(
                        cadir_distance3(
                            last,
                            end_point,
                        )
                    );

                let reversed_error =
                    cadir_distance3(
                        first,
                        end_point,
                    )
                    .max(
                        cadir_distance3(
                            last,
                            start_point,
                        )
                    );

                let tolerance =
                    cadmpeg_ir::units::COINCIDENCE_TOLERANCE;

                match lifted_curve_sense {
                    b'+' => {
                        if direct_error >
                            tolerance
                        {
                            return Err(
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} lifted curve direct endpoint witness failed: errorMm={direct_error} toleranceMm={tolerance}"
                                    ),
                                ),
                            );
                        }
                    }

                    b'-' => {
                        if reversed_error >
                            tolerance
                        {
                            return Err(
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} lifted curve reversed endpoint witness failed: errorMm={reversed_error} toleranceMm={tolerance}"
                                    ),
                                ),
                            );
                        }
                    }

                    _ => {
                        return Err(
                            CodecError::Malformed(
                                format!(
                                    "EDGE {edge_index} lifted curve produced invalid sense"
                                ),
                            )
                        )
                    }
                }

                if lifted_curve_sense_by_edge
                    .insert(
                        *edge_index,
                        lifted_curve_sense,
                    )
                    .is_some()
                {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "duplicate lifted 3D curve sense for EDGE {edge_index}"
                            ),
                        ),
                    );
                }

                ir.model.curves.push(
                    cadmpeg_ir::geometry::Curve {
                        id:
                            curve_id.clone(),

                        geometry:
                            curve_geometry,

                        source_object:
                            None,
                    },
                );

                (
                    Some(
                        curve_id
                    ),
                    Some(
                        canonical_edge_range
                    ),
                )
            }
            else {
                let curve_geometry =
                    cadir_curve_geometry(
                        edge.curve,
                        type_by_index,
                        trimmed_curves,
                        lines,
                        circles,
                        ellipses,
                    )?;

                let start_vertex =
                    vertices
                        .get(
                            &anchor.vertex
                        )
                        .ok_or_else(|| {
                            CodecError::Malformed(
                                format!(
                                    "EDGE {edge_index} start VERTEX {} is missing",
                                    anchor.vertex
                                ),
                            )
                        })?;

                let end_vertex =
                    vertices
                        .get(
                            &negative.vertex
                        )
                        .ok_or_else(|| {
                            CodecError::Malformed(
                                format!(
                                    "EDGE {edge_index} end VERTEX {} is missing",
                                    negative.vertex
                                ),
                            )
                        })?;

                let start_point =
                    cadir_point3(
                        points
                            .get(
                                &start_vertex.point
                            )
                            .ok_or_else(|| {
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} start VERTEX {} POINT {} is missing",
                                        anchor.vertex,
                                        start_vertex.point
                                    ),
                                )
                            })?
                            .pvec
                    );

                let end_point =
                    cadir_point3(
                        points
                            .get(
                                &end_vertex.point
                            )
                            .ok_or_else(|| {
                                CodecError::Malformed(
                                    format!(
                                        "EDGE {edge_index} end VERTEX {} POINT {} is missing",
                                        negative.vertex,
                                        end_vertex.point
                                    ),
                                )
                            })?
                            .pvec
                    );

                let tolerance =
                    cadmpeg_ir::units::COINCIDENCE_TOLERANCE;

                let edge_curve_type =
                    type_by_index
                        .get(
                            &edge.curve
                        )
                        .copied()
                        .ok_or_else(|| {
                            CodecError::Malformed(
                                format!(
                                    "EDGE {edge_index} references missing curve {}",
                                    edge.curve
                                ),
                            )
                        })?;

                let edge_param_range =
                    match edge_curve_type {
                        TRIMMED_CURVE => {
                            let trimmed =
                                trimmed_curves
                                    .get(
                                        &edge.curve
                                    )
                                    .ok_or_else(|| {
                                        CodecError::Malformed(
                                            format!(
                                                "EDGE {edge_index} references missing TRIMMED_CURVE {}",
                                                edge.curve
                                            ),
                                        )
                                    })?;

                            let base_curve =
                                cadir_curve_base_index(
                                    edge.curve,
                                    type_by_index,
                                    trimmed_curves,
                                    0,
                                )?;

                            let base_type =
                                type_by_index
                                    .get(
                                        &base_curve
                                    )
                                    .copied()
                                    .ok_or_else(|| {
                                        CodecError::Malformed(
                                            format!(
                                                "EDGE {edge_index} direct TRIMMED_CURVE {} resolves to missing base curve {base_curve}",
                                                edge.curve
                                            ),
                                        )
                                    })?;

                            let native_range =
                                match base_type {
                                    LINE => [
                                        trimmed.parm_1 *
                                            METRES_TO_MILLIMETRES,

                                        trimmed.parm_2 *
                                            METRES_TO_MILLIMETRES,
                                    ],

                                    CIRCLE |
                                    ELLIPSE => [
                                        trimmed.parm_1,
                                        trimmed.parm_2,
                                    ],

                                    other => {
                                        return Err(
                                            CodecError::NotImplemented(
                                                format!(
                                                    "EDGE {edge_index} direct TRIMMED_CURVE {} resolves to unsupported parameter carrier type {other}",
                                                    edge.curve
                                                ),
                                            ),
                                        )
                                    }
                                };

                            if !native_range[0].is_finite() ||
                                !native_range[1].is_finite() ||
                                native_range[0] ==
                                    native_range[1]
                            {
                                return Err(
                                    CodecError::Malformed(
                                        format!(
                                            "EDGE {edge_index} direct TRIMMED_CURVE {} has invalid native parameter range {native_range:?}",
                                            edge.curve
                                        ),
                                    ),
                                );
                            }

                            let trim_first =
                                cadir_point3(
                                    trimmed.point_1
                                );

                            let trim_last =
                                cadir_point3(
                                    trimmed.point_2
                                );

                            let native_first =
                                cadmpeg_ir::eval::curve_point(
                                    &curve_geometry,
                                    native_range[0],
                                )
                                .ok_or_else(|| {
                                    CodecError::Malformed(
                                        format!(
                                            "EDGE {edge_index} direct TRIMMED_CURVE {} cannot evaluate first native parameter={}",
                                            edge.curve,
                                            native_range[0]
                                        ),
                                    )
                                })?;

                            let native_last =
                                cadmpeg_ir::eval::curve_point(
                                    &curve_geometry,
                                    native_range[1],
                                )
                                .ok_or_else(|| {
                                    CodecError::Malformed(
                                        format!(
                                            "EDGE {edge_index} direct TRIMMED_CURVE {} cannot evaluate second native parameter={}",
                                            edge.curve,
                                            native_range[1]
                                        ),
                                    )
                                })?;

                            let native_witness_error =
                                cadir_distance3(
                                    native_first,
                                    trim_first,
                                )
                                .max(
                                    cadir_distance3(
                                        native_last,
                                        trim_last,
                                    )
                                );

                            if native_witness_error >
                                tolerance
                            {
                                return Err(
                                    CodecError::Malformed(
                                        format!(
                                            "EDGE {edge_index} direct TRIMMED_CURVE {} native parameter/point witness failed: errorMm={native_witness_error} toleranceMm={tolerance} nativeRange={native_range:?}",
                                            edge.curve
                                        ),
                                    ),
                                );
                            }

                            let (
                                canonical_range,
                                canonical_trim_start,
                                canonical_trim_end,
                            ) =
                                match base_type {
                                    LINE => {
                                        if native_range[0] <
                                            native_range[1]
                                        {
                                            (
                                                native_range,
                                                trim_first,
                                                trim_last,
                                            )
                                        }
                                        else {
                                            (
                                                [
                                                    native_range[1],
                                                    native_range[0],
                                                ],
                                                trim_last,
                                                trim_first,
                                            )
                                        }
                                    }

                                    CIRCLE |
                                    ELLIPSE => {
                                        let tau =
                                            std::f64::consts::TAU;

                                        let signed_delta =
                                            native_range[1] -
                                            native_range[0];

                                        let sweep =
                                            signed_delta.abs();

                                        let sweep_tolerance =
                                            1.0e-9_f64;

                                        if !sweep.is_finite() ||
                                            sweep <= 0.0 ||
                                            sweep >
                                                tau +
                                                sweep_tolerance
                                        {
                                            return Err(
                                                CodecError::Malformed(
                                                    format!(
                                                        "EDGE {edge_index} direct periodic TRIMMED_CURVE {} has invalid native sweep {sweep}; nativeRange={native_range:?}",
                                                        edge.curve
                                                    ),
                                                ),
                                            );
                                        }

                                        let (
                                            ordered_start,
                                            ordered_trim_start,
                                            ordered_trim_end,
                                        ) =
                                            if signed_delta >
                                                0.0
                                            {
                                                (
                                                    native_range[0],
                                                    trim_first,
                                                    trim_last,
                                                )
                                            }
                                            else {
                                                (
                                                    native_range[1],
                                                    trim_last,
                                                    trim_first,
                                                )
                                            };

                                        let start =
                                            ordered_start
                                                .rem_euclid(
                                                    tau
                                                );

                                        let end =
                                            start +
                                            sweep;

                                        (
                                            [
                                                start,
                                                end,
                                            ],
                                            ordered_trim_start,
                                            ordered_trim_end,
                                        )
                                    }

                                    _ =>
                                        unreachable!(),
                                };

                            let canonical_first =
                                cadmpeg_ir::eval::curve_point(
                                    &curve_geometry,
                                    canonical_range[0],
                                )
                                .ok_or_else(|| {
                                    CodecError::Malformed(
                                        format!(
                                            "EDGE {edge_index} direct TRIMMED_CURVE {} cannot evaluate canonical trim start={}",
                                            edge.curve,
                                            canonical_range[0]
                                        ),
                                    )
                                })?;

                            let canonical_last =
                                cadmpeg_ir::eval::curve_point(
                                    &curve_geometry,
                                    canonical_range[1],
                                )
                                .ok_or_else(|| {
                                    CodecError::Malformed(
                                        format!(
                                            "EDGE {edge_index} direct TRIMMED_CURVE {} cannot evaluate canonical trim end={}",
                                            edge.curve,
                                            canonical_range[1]
                                        ),
                                    )
                                })?;

                            let canonical_witness_error =
                                cadir_distance3(
                                    canonical_first,
                                    canonical_trim_start,
                                )
                                .max(
                                    cadir_distance3(
                                        canonical_last,
                                        canonical_trim_end,
                                    )
                                );

                            if canonical_witness_error >
                                tolerance
                            {
                                return Err(
                                    CodecError::Malformed(
                                        format!(
                                            "EDGE {edge_index} direct TRIMMED_CURVE {} canonical parameter/point witness failed: errorMm={canonical_witness_error} toleranceMm={tolerance} nativeRange={native_range:?} canonicalRange={canonical_range:?}",
                                            edge.curve
                                        ),
                                    ),
                                );
                            }

                            let direct_error =
                                cadir_distance3(
                                    canonical_trim_start,
                                    start_point,
                                )
                                .max(
                                    cadir_distance3(
                                        canonical_trim_end,
                                        end_point,
                                    )
                                );

                            let reversed_error =
                                cadir_distance3(
                                    canonical_trim_start,
                                    end_point,
                                )
                                .max(
                                    cadir_distance3(
                                        canonical_trim_end,
                                        start_point,
                                    )
                                );

                            match (
                                direct_error <=
                                    tolerance,
                                reversed_error <=
                                    tolerance,
                            ) {
                                (
                                    true,
                                    false,
                                ) => {}

                                (
                                    false,
                                    true,
                                ) => {
                                    if !reversed_direct_edge_orientation
                                        .insert(
                                            *edge_index
                                        )
                                    {
                                        return Err(
                                            CodecError::Malformed(
                                                format!(
                                                    "EDGE {edge_index} duplicate direct orientation reversal"
                                                ),
                                            ),
                                        );
                                    }
                                }

                                (
                                    true,
                                    true,
                                ) => {
                                    return Err(
                                        CodecError::Malformed(
                                            format!(
                                                "EDGE {edge_index} direct TRIMMED_CURVE {} canonical endpoint orientation is ambiguous: directErrorMm={direct_error} reversedErrorMm={reversed_error} toleranceMm={tolerance}",
                                                edge.curve
                                            ),
                                        ),
                                    )
                                }

                                (
                                    false,
                                    false,
                                ) => {
                                    return Err(
                                        CodecError::Malformed(
                                            format!(
                                                "EDGE {edge_index} direct TRIMMED_CURVE {} canonical endpoints do not witness native EDGE endpoints: directErrorMm={direct_error} reversedErrorMm={reversed_error} toleranceMm={tolerance}",
                                                edge.curve
                                            ),
                                        ),
                                    )
                                }
                            }

                            Some(
                                canonical_range
                            )
                        }

                        CIRCLE |
                        ELLIPSE => {
                            let endpoint_distance =
                                cadir_distance3(
                                    start_point,
                                    end_point,
                                );

                            if !endpoint_distance.is_finite() {
                                return Err(
                                    CodecError::Malformed(
                                        format!(
                                            "EDGE {edge_index} periodic endpoint distance is non-finite"
                                        ),
                                    ),
                                );
                            }

                            // Coincident-endpoint analytic carriers represent
                            // closed/full-period edges. Leave them unbounded:
                            // no sub-arc may be inferred from coincident points.
                            if endpoint_distance <=
                                tolerance
                            {
                                None
                            }
                            else {
                                let start_parameter =
                                    cadir_periodic_parameter_from_point(
                                        &curve_geometry,
                                        start_point,
                                        &format!(
                                            "EDGE {edge_index} start"
                                        ),
                                    )?;

                                let end_parameter =
                                    cadir_periodic_parameter_from_point(
                                        &curve_geometry,
                                        end_point,
                                        &format!(
                                            "EDGE {edge_index} end"
                                        ),
                                    )?;

                                let curve_sense =
                                    cadir_effective_curve_sense(
                                        edge.curve,
                                        type_by_index,
                                        trimmed_curves,
                                        curve_sense_by_index,
                                    )?;

                                let tau =
                                    std::f64::consts::TAU;

                                let (
                                    mut ordered_start,
                                    mut sweep,
                                    mut canonical_expected_start,
                                    mut canonical_expected_end,
                                ) =
                                    match curve_sense {
                                        b'+' => (
                                            start_parameter,
                                            (
                                                end_parameter -
                                                start_parameter
                                            )
                                            .rem_euclid(
                                                tau
                                            ),
                                            start_point,
                                            end_point,
                                        ),

                                        b'-' => (
                                            end_parameter,
                                            (
                                                start_parameter -
                                                end_parameter
                                            )
                                            .rem_euclid(
                                                tau
                                            ),
                                            end_point,
                                            start_point,
                                        ),

                                        other => {
                                            return Err(
                                                CodecError::Malformed(
                                                    format!(
                                                        "EDGE {edge_index} periodic curve {} has invalid native sense 0x{other:02x}",
                                                        edge.curve
                                                    ),
                                                ),
                                            )
                                        }
                                    };

                                let sweep_tolerance =
                                    1.0e-12_f64;

                                // R50.N production periodic canonicalization.
                                // Qualified R113-R15-R8 evidence proves the same major-arc
                                // defect for direct bounded CIRCLE and ELLIPSE carriers.
                                // Keep CADIR ranges ascending and let the existing endpoint/
                                // coedge canonicalization carry topological orientation.
                                // TRIMMED_CURVE wrapper semantics remain untouched.
                                if (edge_curve_type ==
                                        CIRCLE ||
                                    edge_curve_type ==
                                        ELLIPSE) &&
                                    sweep >
                                        tau *
                                        0.5 +
                                        sweep_tolerance
                                {
                                    ordered_start =
                                        match curve_sense {
                                            b'+' =>
                                                end_parameter,

                                            b'-' =>
                                                start_parameter,

                                            _ =>
                                                unreachable!(),
                                        };

                                    sweep =
                                        tau -
                                        sweep;

                                    std::mem::swap(
                                        &mut canonical_expected_start,
                                        &mut canonical_expected_end,
                                    );
                                }

                                if !sweep.is_finite() ||
                                    sweep <=
                                        sweep_tolerance ||
                                    sweep >
                                        tau +
                                        1.0e-9
                                {
                                    return Err(
                                        CodecError::Malformed(
                                            format!(
                                                "EDGE {edge_index} periodic endpoint-derived sweep is invalid: startParameter={start_parameter} endParameter={end_parameter} curveSense=0x{curve_sense:02x} sweep={sweep}"
                                            ),
                                        ),
                                    );
                                }

                                let canonical_start =
                                    ordered_start
                                        .rem_euclid(
                                            tau
                                        );

                                let canonical_range =
                                    [
                                        canonical_start,
                                        canonical_start +
                                            sweep,
                                    ];

                                let canonical_first =
                                    cadmpeg_ir::eval::curve_point(
                                        &curve_geometry,
                                        canonical_range[0],
                                    )
                                    .ok_or_else(|| {
                                        CodecError::Malformed(
                                            format!(
                                                "EDGE {edge_index} cannot evaluate recovered periodic range start={}",
                                                canonical_range[0]
                                            ),
                                        )
                                    })?;

                                let canonical_last =
                                    cadmpeg_ir::eval::curve_point(
                                        &curve_geometry,
                                        canonical_range[1],
                                    )
                                    .ok_or_else(|| {
                                        CodecError::Malformed(
                                            format!(
                                                "EDGE {edge_index} cannot evaluate recovered periodic range end={}",
                                                canonical_range[1]
                                            ),
                                        )
                                    })?;

                                let periodic_witness_error =
                                    cadir_distance3(
                                        canonical_first,
                                        canonical_expected_start,
                                    )
                                    .max(
                                        cadir_distance3(
                                            canonical_last,
                                            canonical_expected_end,
                                        )
                                    );

                                if periodic_witness_error >
                                    tolerance
                                {
                                    return Err(
                                        CodecError::Malformed(
                                            format!(
                                                "EDGE {edge_index} recovered periodic range witness failed: errorMm={periodic_witness_error} toleranceMm={tolerance} range={canonical_range:?} curveSense=0x{curve_sense:02x}"
                                            ),
                                        ),
                                    );
                                }

                                let direct_error =
                                    cadir_distance3(
                                        canonical_expected_start,
                                        start_point,
                                    )
                                    .max(
                                        cadir_distance3(
                                            canonical_expected_end,
                                            end_point,
                                        )
                                    );

                                let reversed_error =
                                    cadir_distance3(
                                        canonical_expected_start,
                                        end_point,
                                    )
                                    .max(
                                        cadir_distance3(
                                            canonical_expected_end,
                                            start_point,
                                        )
                                    );

                                match (
                                    direct_error <=
                                        tolerance,
                                    reversed_error <=
                                        tolerance,
                                ) {
                                    (
                                        true,
                                        false,
                                    ) => {}

                                    (
                                        false,
                                        true,
                                    ) => {
                                        if !reversed_direct_edge_orientation
                                            .insert(
                                                *edge_index
                                            )
                                        {
                                            return Err(
                                                CodecError::Malformed(
                                                    format!(
                                                        "EDGE {edge_index} duplicate recovered periodic orientation reversal"
                                                    ),
                                                ),
                                            );
                                        }
                                    }

                                    (
                                        true,
                                        true,
                                    ) => {
                                        return Err(
                                            CodecError::Malformed(
                                                format!(
                                                    "EDGE {edge_index} recovered periodic orientation is ambiguous"
                                                ),
                                            ),
                                        )
                                    }

                                    (
                                        false,
                                        false,
                                    ) => {
                                        return Err(
                                            CodecError::Malformed(
                                                format!(
                                                    "EDGE {edge_index} recovered periodic range does not witness native EDGE endpoints"
                                                ),
                                            ),
                                        )
                                    }
                                }

                                Some(
                                    canonical_range
                                )
                            }
                        }

                        _ =>
                            None,
                    };

                if emitted_curves.insert(
                    edge.curve
                ) {
                    ir.model.curves.push(
                        cadmpeg_ir::geometry::Curve {
                            id:
                                cadmpeg_ir::ids::CurveId(
                                    cadir_entity_id(
                                        "curve",
                                        edge.curve,
                                    )
                                ),

                            geometry:
                                curve_geometry,

                            source_object:
                                None,
                        },
                    );
                }

                (
                    Some(
                        cadmpeg_ir::ids::CurveId(
                            cadir_entity_id(
                                "curve",
                                edge.curve,
                            )
                        )
                    ),
                    edge_param_range,
                )
            };

        let (
            cadir_start_vertex,
            cadir_end_vertex,
        ) =
            if reversed_direct_edge_orientation
                .contains(
                    edge_index
                )
            {
                (
                    negative.vertex,
                    anchor.vertex,
                )
            }
            else {
                (
                    anchor.vertex,
                    negative.vertex,
                )
            };

        ir.model.edges.push(
            cadmpeg_ir::topology::Edge {
                id:
                    cadmpeg_ir::ids::EdgeId(
                        cadir_entity_id(
                            "edge",
                            *edge_index,
                        )
                    ),

                curve,

                start:
                    cadmpeg_ir::ids::VertexId(
                        cadir_entity_id(
                            "vertex",
                            cadir_start_vertex,
                        )
                    ),

                end:
                    cadmpeg_ir::ids::VertexId(
                        cadir_entity_id(
                            "vertex",
                            cadir_end_vertex,
                        )
                    ),

                param_range:
                    edge_param_range,

                tolerance:
                    edge
                        .tolerance
                        .map(
                            |value|
                                value *
                                METRES_TO_MILLIMETRES
                        ),
            },
        );
    }

    // --------------------------------------------------------
    // MATERIAL BODY / REGION / SHELL OWNERSHIP
    // --------------------------------------------------------

    let mut emitted_material_regions =
        BTreeSet::<u32>::new();

    let mut emitted_solid_shells =
        BTreeSet::<u32>::new();

    let mut face_shell_owner =
        BTreeMap::<u32, u32>::new();

    let root_set =
        root_entry_indices
            .iter()
            .copied()
            .collect::<BTreeSet<_>>();

    if root_set.len() !=
        root_entry_indices.len()
    {
        return Err(
            CodecError::Malformed(
                "CADIR root body list contains duplicates"
                    .to_string(),
            ),
        );
    }

    for body_index in
        root_entry_indices
    {
        let body =
            bodies.get(
                body_index
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "root entry {body_index} is not a BODY"
                    ),
                )
            })?;

        if body.body_type != 1 {
            return Err(
                CodecError::NotImplemented(
                    format!(
                        "BODY {body_index} type {} is outside the qualified solid CADIR profile",
                        body.body_type
                    ),
                ),
            );
        }

        let region_chain =
            cadir_region_chain_indices(
                body.region,
                regions,
            )?;

        let owned_regions =
            regions
                .iter()
                .filter_map(
                    |(
                        index,
                        region,
                    )| {
                        (
                            region.body ==
                            *body_index
                        )
                            .then_some(
                                *index
                            )
                    },
                )
                .collect::<BTreeSet<_>>();

        let chain_set =
            region_chain
                .iter()
                .copied()
                .collect::<BTreeSet<_>>();

        if chain_set !=
            owned_regions
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "BODY {body_index} region chain/owner population mismatch"
                    ),
                ),
            );
        }

        let mut body_regions =
            Vec::<cadmpeg_ir::ids::RegionId>::new();

        for region_index in
            region_chain
        {
            let region =
                regions.get(
                    &region_index
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "missing REGION {region_index}"
                        ),
                    )
                })?;

            if region.kind != b'S' {
                continue;
            }

            emitted_material_regions.insert(
                region_index
            );

            let shell_chain =
                cadir_shell_chain_indices(
                    region.shell,
                    shells,
                )?;

            if shell_chain.len() != 1 {
                return Err(
                    CodecError::NotImplemented(
                        format!(
                            "solid REGION {region_index} has {} material-shell entries; multi-shell CADIR mapping remains fail-closed",
                            shell_chain.len()
                        ),
                    ),
                );
            }

            let shell_index =
                shell_chain[0];

            if !solid_shells.contains(
                &shell_index
            ) {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "REGION {region_index} shell {shell_index} is not in the qualified solid-shell set"
                        ),
                    ),
                );
            }

            let shell =
                shells.get(
                    &shell_index
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "missing SHELL {shell_index}"
                        ),
                    )
                })?;

            if shell.region !=
                region_index ||
                shell.body !=
                *body_index
            {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "SHELL {shell_index} material ownership does not match BODY {body_index} REGION {region_index}"
                        ),
                    ),
                );
            }

            emitted_solid_shells.insert(
                shell_index
            );

            let mut shell_faces =
                BTreeSet::<u32>::new();

            for face_index in
                cadir_face_chain_indices(
                    shell.face,
                    false,
                    faces,
                )?
                .into_iter()
                .chain(
                    cadir_face_chain_indices(
                        shell.front_face,
                        true,
                        faces,
                    )?
                )
            {
                shell_faces.insert(
                    face_index
                );

                if !reachable_faces.contains(
                    &face_index
                ) {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "material SHELL {shell_index} references FACE {face_index} outside qualified reachability"
                            ),
                        ),
                    );
                }

                if let Some(previous) =
                    face_shell_owner.insert(
                        face_index,
                        shell_index,
                    )
                {
                    if previous !=
                        shell_index
                    {
                        return Err(
                            CodecError::Malformed(
                                format!(
                                    "FACE {face_index} belongs to multiple material shells {previous}/{shell_index}"
                                ),
                            ),
                        );
                    }
                }
            }

            let shell_id =
                cadmpeg_ir::ids::ShellId(
                    cadir_entity_id(
                        "shell",
                        shell_index,
                    )
                );

            let region_id =
                cadmpeg_ir::ids::RegionId(
                    cadir_entity_id(
                        "region",
                        region_index,
                    )
                );

            ir.model.shells.push(
                cadmpeg_ir::topology::Shell {
                    id:
                        shell_id.clone(),

                    region:
                        region_id.clone(),

                    faces:
                        shell_faces
                            .into_iter()
                            .map(
                                |index|
                                    cadmpeg_ir::ids::FaceId(
                                        cadir_entity_id(
                                            "face",
                                            index,
                                        )
                                    )
                            )
                            .collect(),

                    wire_edges:
                        Vec::new(),

                    free_vertices:
                        Vec::new(),
                },
            );

            ir.model.regions.push(
                cadmpeg_ir::topology::Region {
                    id:
                        region_id.clone(),

                    body:
                        cadmpeg_ir::ids::BodyId(
                            cadir_entity_id(
                                "body",
                                *body_index,
                            )
                        ),

                    shells:
                        vec![
                            shell_id
                        ],
                },
            );

            body_regions.push(
                region_id
            );
        }

        if body_regions.is_empty() {
            return Err(
                CodecError::Malformed(
                    format!(
                        "solid BODY {body_index} exposes no material region"
                    ),
                ),
            );
        }

        ir.model.bodies.push(
            cadmpeg_ir::topology::Body {
                id:
                    cadmpeg_ir::ids::BodyId(
                        cadir_entity_id(
                            "body",
                            *body_index,
                        )
                    ),

                kind:
                    cadmpeg_ir::topology::BodyKind::Solid,

                regions:
                    body_regions,

                transform:
                    None,

                name:
                    None,

                color:
                    None,

                visible:
                    None,
            },
        );
    }

    let qualified_material_regions =
        regions
            .iter()
            .filter_map(
                |(
                    index,
                    region,
                )| {
                    (
                        region.kind == b'S'
                    )
                        .then_some(
                            *index
                        )
                },
            )
            .collect::<BTreeSet<_>>();

    if emitted_material_regions !=
        qualified_material_regions
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "CADIR material region coverage mismatch emitted={} qualified={}",
                    emitted_material_regions.len(),
                    qualified_material_regions.len()
                ),
            ),
        );
    }

    if &emitted_solid_shells !=
        solid_shells
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "CADIR material shell coverage mismatch emitted={} qualified={}",
                    emitted_solid_shells.len(),
                    solid_shells.len()
                ),
            ),
        );
    }

    let mapped_faces =
        face_shell_owner
            .keys()
            .copied()
            .collect::<BTreeSet<_>>();

    if &mapped_faces !=
        reachable_faces
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "CADIR material face coverage mismatch emitted={} qualified={}",
                    mapped_faces.len(),
                    reachable_faces.len()
                ),
            ),
        );
    }

    // --------------------------------------------------------
    // SURFACES / FACES / LOOPS / COEDGES / PCURVES
    // --------------------------------------------------------

    let mut emitted_surfaces =
        BTreeSet::<u32>::new();

    let mut emitted_loops =
        BTreeSet::<u32>::new();

    let mut emitted_fins =
        BTreeSet::<u32>::new();

    let mut emitted_pcurves =
        BTreeSet::<u32>::new();

    let mut singular_vertex_loop_count =
        0usize;

    for face_index in
        reachable_faces
    {
        let face =
            faces.get(
                face_index
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "reachable FACE {face_index} disappeared during CADIR construction"
                    ),
                )
            })?;

        let shell_index =
            face_shell_owner
                .get(
                    face_index
                )
                .copied()
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "FACE {face_index} has no qualified material shell"
                        ),
                    )
                })?;

        if emitted_surfaces.insert(
            face.surface
        ) {
            ir.model.surfaces.push(
                cadmpeg_ir::geometry::Surface {
                    id:
                        cadmpeg_ir::ids::SurfaceId(
                            cadir_entity_id(
                                "surface",
                                face.surface,
                            )
                        ),

                    geometry:
                        cadir_surface_geometry(
                            face.surface,
                            planes,
                            cylinders,
                            cones,
                        )?,

                    source_object:
                        None,
                },
            );
        }

        let surface_native_sense =
            cadir_surface_native_sense(
                face.surface,
                planes,
                cylinders,
                cones,
            )?;

        let face_sense =
            if face.native_sense ==
                surface_native_sense
            {
                cadmpeg_ir::topology::Sense::Forward
            }
            else {
                let _ =
                    cadir_native_sense(
                        face.native_sense,
                        "FACE"
                    )?;

                let _ =
                    cadir_native_sense(
                        surface_native_sense,
                        "surface"
                    )?;

                cadmpeg_ir::topology::Sense::Reversed
            };

        let loop_indices =
            cadir_loop_chain_indices(
                face.loop_head,
                loops,
            )?;

        let mut face_loop_ids =
            Vec::<cadmpeg_ir::ids::LoopId>::new();

        for loop_index in
            loop_indices
        {
            if !emitted_loops.insert(
                loop_index
            ) {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "LOOP {loop_index} is emitted by multiple FACE chains"
                        ),
                    ),
                );
            }

            let loop_node =
                loops.get(
                    &loop_index
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "missing LOOP {loop_index}"
                        ),
                    )
                })?;

            if loop_node.face !=
                *face_index
            {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "LOOP {loop_index} owner FACE {} != {face_index}",
                            loop_node.face
                        ),
                    ),
                );
            }

            let ring =
                cadir_fin_ring_indices(
                    loop_node.fin,
                    fins,
                )?;

            let loop_id =
                cadmpeg_ir::ids::LoopId(
                    cadir_entity_id(
                        "loop",
                        loop_index,
                    )
                );

            let mut coedge_ids =
                Vec::<cadmpeg_ir::ids::CoedgeId>::new();

            let mut vertex_uses =
                Vec::<cadmpeg_ir::topology::VertexUse>::new();

            if ring.len() == 1 {
                let fin_index =
                    ring[0];

                let fin =
                    fins.get(
                        &fin_index
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "missing singular FIN {fin_index}"
                            ),
                        )
                    })?;

                if fin.edge == 0 &&
                    fin.curve == 0 &&
                    fin.other == 0 &&
                    fin.forward == fin_index &&
                    fin.backward == fin_index
                {
                    if !reachable_vertices.contains(
                        &fin.vertex
                    ) {
                        return Err(
                            CodecError::Malformed(
                                format!(
                                    "singular FIN {fin_index} references unreachable VERTEX {}",
                                    fin.vertex
                                ),
                            ),
                        );
                    }

                    singular_vertex_loop_count +=
                        1;

                    emitted_fins.insert(
                        fin_index
                    );

                    vertex_uses.push(
                        cadmpeg_ir::topology::VertexUse {
                            vertex:
                                cadmpeg_ir::ids::VertexId(
                                    cadir_entity_id(
                                        "vertex",
                                        fin.vertex,
                                    )
                                ),

                            after:
                                None,

                            pcurves:
                                Vec::new(),
                        },
                    );
                }
            }

            if vertex_uses.is_empty() {
                for fin_index in
                    ring
                {
                    let fin =
                        fins.get(
                            &fin_index
                        )
                        .ok_or_else(|| {
                            CodecError::Malformed(
                                format!(
                                    "missing FIN {fin_index}"
                                ),
                            )
                        })?;

                    if fin.loop_owner !=
                        loop_index
                    {
                        return Err(
                            CodecError::Malformed(
                                format!(
                                    "FIN {fin_index} LOOP owner {} != {loop_index}",
                                    fin.loop_owner
                                ),
                            ),
                        );
                    }

                    if fin.edge == 0 {
                        return Err(
                            CodecError::Malformed(
                                format!(
                                    "non-singular FIN {fin_index} has null EDGE"
                                ),
                            ),
                        );
                    }

                    let edge =
                        edges.get(
                            &fin.edge
                        )
                        .ok_or_else(|| {
                            CodecError::Malformed(
                                format!(
                                    "FIN {fin_index} references missing EDGE {}",
                                    fin.edge
                                ),
                            )
                        })?;

                    if fin.other == 0 {
                        return Err(
                            CodecError::Malformed(
                                format!(
                                    "radial FIN {fin_index} has null FIN.other"
                                ),
                            ),
                        );
                    }

                    let radial =
                        fins.get(
                            &fin.other
                        )
                        .ok_or_else(|| {
                            CodecError::Malformed(
                                format!(
                                    "FIN {fin_index} radial successor {} is missing",
                                    fin.other
                                ),
                            )
                        })?;

                    if radial.edge !=
                        fin.edge
                    {
                        return Err(
                            CodecError::Malformed(
                                format!(
                                    "FIN {fin_index} radial successor crosses EDGE ownership"
                                ),
                            ),
                        );
                    }

                    let mut sense =
                        if edge.curve == 0 {
                            let curve_sense =
                                lifted_curve_sense_by_edge
                                    .get(
                                        &fin.edge
                                    )
                                    .copied()
                                    .ok_or_else(|| {
                                        CodecError::Malformed(
                                            format!(
                                                "curveless EDGE {} has no qualified lifted 3D curve sense",
                                                fin.edge
                                            ),
                                        )
                                    })?;

                            if fin.native_sense ==
                                curve_sense
                            {
                                cadmpeg_ir::topology::Sense::Forward
                            }
                            else {
                                let _ =
                                    cadir_native_sense(
                                        fin.native_sense,
                                        "curveless-edge FIN",
                                    )?;

                                let _ =
                                    cadir_native_sense(
                                        curve_sense,
                                        "lifted edge curve",
                                    )?;

                                cadmpeg_ir::topology::Sense::Reversed
                            }
                        }
                        else {
                            let curve_sense =
                                cadir_effective_curve_sense(
                                    edge.curve,
                                    type_by_index,
                                    trimmed_curves,
                                    curve_sense_by_index,
                                )?;

                            if fin.native_sense ==
                                curve_sense
                            {
                                cadmpeg_ir::topology::Sense::Forward
                            }
                            else {
                                let _ =
                                    cadir_native_sense(
                                        fin.native_sense,
                                        "FIN",
                                    )?;

                                let _ =
                                    cadir_native_sense(
                                        curve_sense,
                                        "edge curve",
                                    )?;

                                cadmpeg_ir::topology::Sense::Reversed
                            }
                        };

                    if reversed_direct_edge_orientation
                        .contains(
                            &fin.edge
                        )
                    {
                        sense =
                            match sense {
                                cadmpeg_ir::topology::Sense::Forward =>
                                    cadmpeg_ir::topology::Sense::Reversed,

                                cadmpeg_ir::topology::Sense::Reversed =>
                                    cadmpeg_ir::topology::Sense::Forward,
                            };
                    }

                    let mut pcurve_uses =
                        Vec::<cadmpeg_ir::topology::PcurveUse>::new();

                    if edge.curve == 0 {
                        let (
                            pcurve,
                            use_,
                        ) =
                            cadir_sp_pcurve(
                                fin_index,
                                face.surface,
                                fin,
                                type_by_index,
                                trimmed_curves,
                                sp_curves,
                                b_curves,
                                nurbs_curves,
                                bspline_vertices,
                                knot_multiplicities,
                                knot_sets,
                            )?;

                        if !emitted_pcurves.insert(
                            fin_index
                        ) {
                            return Err(
                                CodecError::Malformed(
                                    format!(
                                        "duplicate CADIR pcurve identity for FIN {fin_index}"
                                    ),
                                ),
                            );
                        }

                        ir.model.pcurves.push(
                            pcurve
                        );

                        pcurve_uses.push(
                            use_
                        );
                    }
                    else if fin.curve != 0 {
                        return Err(
                            CodecError::NotImplemented(
                                format!(
                                    "direct EDGE {} FIN {fin_index} carries local curve {}; A6I refuses to guess a use-curve mapping",
                                    fin.edge,
                                    fin.curve
                                ),
                            ),
                        );
                    }

                    emitted_fins.insert(
                        fin_index
                    );

                    let coedge_id =
                        cadmpeg_ir::ids::CoedgeId(
                            cadir_entity_id(
                                "coedge",
                                fin_index,
                            )
                        );

                    coedge_ids.push(
                        coedge_id.clone()
                    );

                    ir.model.coedges.push(
                        cadmpeg_ir::topology::Coedge {
                            id:
                                coedge_id,

                            owner_loop:
                                loop_id.clone(),

                            edge:
                                cadmpeg_ir::ids::EdgeId(
                                    cadir_entity_id(
                                        "edge",
                                        fin.edge,
                                    )
                                ),

                            next:
                                cadmpeg_ir::ids::CoedgeId(
                                    cadir_entity_id(
                                        "coedge",
                                        fin.forward,
                                    )
                                ),

                            previous:
                                cadmpeg_ir::ids::CoedgeId(
                                    cadir_entity_id(
                                        "coedge",
                                        fin.backward,
                                    )
                                ),

                            radial_next:
                                cadmpeg_ir::ids::CoedgeId(
                                    cadir_entity_id(
                                        "coedge",
                                        fin.other,
                                    )
                                ),

                            sense,

                            pcurves:
                                pcurve_uses,

                            use_curve:
                                None,

                            use_curve_parameter_range:
                                None,
                        },
                    );
                }
            }

            ir.model.loops.push(
                cadmpeg_ir::topology::Loop {
                    id:
                        loop_id.clone(),

                    face:
                        cadmpeg_ir::ids::FaceId(
                            cadir_entity_id(
                                "face",
                                *face_index,
                            )
                        ),

                    boundary_role:
                        cadmpeg_ir::topology::LoopBoundaryRole::Unspecified,

                    coedges:
                        coedge_ids,

                    vertex_uses,
                },
            );

            face_loop_ids.push(
                loop_id
            );
        }

        ir.model.faces.push(
            cadmpeg_ir::topology::Face {
                id:
                    cadmpeg_ir::ids::FaceId(
                        cadir_entity_id(
                            "face",
                            *face_index,
                        )
                    ),

                shell:
                    cadmpeg_ir::ids::ShellId(
                        cadir_entity_id(
                            "shell",
                            shell_index,
                        )
                    ),

                surface:
                    cadmpeg_ir::ids::SurfaceId(
                        cadir_entity_id(
                            "surface",
                            face.surface,
                        )
                    ),

                sense:
                    face_sense,

                loops:
                    face_loop_ids,

                name:
                    None,

                color:
                    None,

                tolerance:
                    face
                        .tolerance
                        .map(
                            |value|
                                value *
                                METRES_TO_MILLIMETRES
                        ),
            },
        );
    }

    if &emitted_loops !=
        reachable_loops
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "CADIR LOOP coverage mismatch emitted={} qualified={}",
                    emitted_loops.len(),
                    reachable_loops.len()
                ),
            ),
        );
    }

    if &emitted_fins !=
        reachable_fins
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "CADIR FIN/coedge coverage mismatch emitted={} qualified={}",
                    emitted_fins.len(),
                    reachable_fins.len()
                ),
            ),
        );
    }

    // --------------------------------------------------------
    // PRODUCTION PERIODIC RECONSTRUCTION + STEP PROVENANCE
    // --------------------------------------------------------

    let plane_lift_step_false_edges =
        lifted_curve_sense_by_edge
            .keys()
            .copied()
            .map(|edge_index| cadir_entity_id("edge", edge_index))
            .collect::<BTreeSet<_>>();

    crate::sch13006_periodic::reconstruct_sch13006_periodic(
        &mut ir,
        &plane_lift_step_false_edges,
    )?;

    // --------------------------------------------------------
    // CANONICALIZE + VALIDATE
    // --------------------------------------------------------

    ir.finalize();

    let report =
        cadmpeg_ir::validate_neutral(
            &ir,
            Vec::new(),
        );

    let validation_error_count =
        report.error_count();

    let validation_warning_count =
        report.warning_count();

    if !report.is_ok() {
        let details =
            report
                .findings
                .iter()
                .take(8)
                .map(
                    |finding| {
                        format!(
                            "{}:{}:{}",
                            finding.check,
                            finding
                                .entity
                                .as_deref()
                                .unwrap_or("-"),
                            finding.message
                        )
                    },
                )
                .collect::<Vec<_>>()
                .join(" | ");

        return Err(
            CodecError::Malformed(
                format!(
                    "transient SCH13006 CADIR validation failed errors={validation_error_count} warnings={validation_warning_count}: {details}"
                ),
            ),
        );
    }

    let pcurve_use_count =
        ir.model
            .coedges
            .iter()
            .map(
                |coedge|
                    coedge.pcurves.len()
            )
            .sum::<usize>() +
        ir.model
            .loops
            .iter()
            .flat_map(
                |loop_|
                    loop_.vertex_uses.iter()
            )
            .map(
                |use_|
                    use_.pcurves.len()
            )
            .sum::<usize>();

    let edge_parameter_range_count =
        ir.model
            .edges
            .iter()
            .filter(
                |edge|
                    edge.param_range.is_some()
            )
            .count();

    let face_forward_sense_count =
        ir.model
            .faces
            .iter()
            .filter(
                |face| {
                    face.sense ==
                    cadmpeg_ir::topology::Sense::Forward
                },
            )
            .count();

    let face_reversed_sense_count =
        ir.model
            .faces
            .len()
            .saturating_sub(
                face_forward_sense_count
            );

    let coedge_forward_sense_count =
        ir.model
            .coedges
            .iter()
            .filter(
                |coedge| {
                    coedge.sense ==
                    cadmpeg_ir::topology::Sense::Forward
                },
            )
            .count();

    let coedge_reversed_sense_count =
        ir.model
            .coedges
            .len()
            .saturating_sub(
                coedge_forward_sense_count
            );

    let qualification =
        Sch13006CadirQualification {
            validation_ok:
                true,

            validation_error_count,
            validation_warning_count,

            body_count:
                ir.model.bodies.len(),

            region_count:
                ir.model.regions.len(),

            shell_count:
                ir.model.shells.len(),

            face_count:
                ir.model.faces.len(),

            loop_count:
                ir.model.loops.len(),

            coedge_count:
                ir.model.coedges.len(),

            singular_vertex_loop_count,

            edge_count:
                ir.model.edges.len(),

            vertex_count:
                ir.model.vertices.len(),

            point_count:
                ir.model.points.len(),

            surface_count:
                ir.model.surfaces.len(),

            curve_count:
                ir.model.curves.len(),

            pcurve_count:
                ir.model.pcurves.len(),

            pcurve_use_count,

            edge_parameter_range_count,

            face_forward_sense_count,
            face_reversed_sense_count,

            coedge_forward_sense_count,
            coedge_reversed_sense_count,
        };

    Ok(
        (
            ir,
            qualification,
        )
    )
}

#[cfg(test)]
pub(crate) fn parse_census(
    logical: View<'_>,
    preamble: &Sch13006Preamble,
) -> Result<Sch13006Census, CodecError> {
    parse_census_impl(
        logical,
        preamble,
        false,
    )
}

pub(crate) fn parse_census_with_cadir_qualification(
    logical: View<'_>,
    preamble: &Sch13006Preamble,
) -> Result<Sch13006Census, CodecError> {
    parse_census_impl(
        logical,
        preamble,
        true,
    )
}

fn parse_census_impl(
    logical: View<'_>,
    preamble: &Sch13006Preamble,
    qualify_cadir: bool,
) -> Result<Sch13006Census, CodecError> {
    if preamble.user_field_size != 0 {
        return Err(
            CodecError::NotImplemented(
                format!(
                    "SCH_13006 user fields are not qualified: {}",
                    preamble.user_field_size
                ),
            ),
        );
    }

    if preamble.first_node_type != POINTER_LIST {
        return Err(
            CodecError::NotImplemented(
                format!(
                    "qualified SCH_13006 part-array root requires node type {POINTER_LIST}, found {}",
                    preamble.first_node_type
                ),
            ),
        );
    }

    let mut cursor =
        Cursor::at(
            logical.window(),
            preamble.after_first_node_type_offset,
        )?;

    let mut first_type =
        Some(
            preamble.first_node_type
        );

    let mut type_by_index =
        BTreeMap::<u32, u32>::new();

    let mut type_counts =
        BTreeMap::<u32, usize>::new();

    let mut bodies =
        BTreeMap::<u32, BodyNode>::new();

    let mut shells =
        BTreeMap::<u32, ShellNode>::new();

    let mut faces =
        BTreeMap::<u32, FaceNode>::new();

    let mut loops =
        BTreeMap::<u32, LoopNode>::new();

    let mut edges =
        BTreeMap::<u32, EdgeNode>::new();

    let mut fins =
        BTreeMap::<u32, FinNode>::new();

    let mut vertices =
        BTreeMap::<u32, VertexNode>::new();

    let mut regions =
        BTreeMap::<u32, RegionNode>::new();

    let mut geometry_owner_by_index =
        BTreeMap::<u32, u32>::new();

    let mut curve_sense_by_index =
        BTreeMap::<u32, u8>::new();

    let mut points =
        BTreeMap::<u32, PointGeometry>::new();

    let mut lines =
        BTreeMap::<u32, LineGeometry>::new();

    let mut circles =
        BTreeMap::<u32, CircleGeometry>::new();

    let mut ellipses =
        BTreeMap::<u32, EllipseGeometry>::new();

    let mut planes =
        BTreeMap::<u32, PlaneGeometry>::new();

    let mut cylinders =
        BTreeMap::<u32, CylinderGeometry>::new();

    let mut cones =
        BTreeMap::<u32, ConeGeometry>::new();

    let mut trimmed_curves =
        BTreeMap::<u32, TrimmedCurveGeometry>::new();

    let mut b_curves =
        BTreeMap::<u32, BCurveGeometry>::new();

    let mut nurbs_curves =
        BTreeMap::<u32, NurbsCurveGeometry>::new();

    let mut sp_curves =
        BTreeMap::<u32, SpCurveGeometry>::new();

    let mut bspline_vertices =
        BTreeMap::<u32, Vec<f64>>::new();

    let mut knot_multiplicities =
        BTreeMap::<u32, Vec<i16>>::new();

    let mut knot_sets =
        BTreeMap::<u32, Vec<f64>>::new();

    let mut root_variable_length =
        None;

    let mut root_next_block =
        None;

    let mut root_entries =
        None;

    let mut total_entities =
        0usize;

    let terminator_index =
        loop {
            if total_entities >= MAX_ENTITIES {
                return Err(
                    cadmpeg_core::decode::refuse_local_limit(
                        "parasolid-sch13006-entity-count",
                        MAX_ENTITIES as u64,
                        total_entities as u64 + 1,
                        None,
                    ),
                );
            }

            let type_id =
                match first_type.take() {
                    Some(value) => value,
                    None => cursor.read_u32(
                        "entity node type",
                    )?,
                };

            if type_id == 1 {
                let index =
                    cursor.read_u32(
                        "transmit terminator index",
                    )?;

                break index;
            }

            let spec =
                schema_spec(
                    type_id
                )
                .ok_or_else(|| {
                    CodecError::NotImplemented(
                        format!(
                            "SCH_13006 node type {type_id} is outside the physically qualified catalog"
                        ),
                    )
                })?;

            let variable_length =
                match spec.variable {
                    Some(_) => {
                        let value =
                            cursor.read_u32(
                                "variable node length",
                            )?;

                        if value > MAX_VARIABLE_VALUES {
                            return Err(
                                CodecError::Malformed(
                                    format!(
                                        "SCH_13006 variable node type {type_id} length {value} exceeds the qualified envelope"
                                    ),
                                ),
                            );
                        }

                        Some(value)
                    }

                    None => None,
                };

            let index =
                cursor.read_u32(
                    "entity index",
                )?;

            if index == 0 {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "SCH_13006 node type {type_id} has illegal index 0"
                        ),
                    ),
                );
            }

            if type_by_index
                .insert(
                    index,
                    type_id,
                )
                .is_some()
            {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "duplicate SCH_13006 entity index {index}"
                        ),
                    ),
                );
            }

            let mut fields =
                Vec::with_capacity(
                    spec.fields.len()
                );

            for field in
                spec.fields
            {
                fields.push(
                    cursor.read_field(
                        *field
                    )?
                );
            }

            let mut variable_pointers =
                Vec::<u32>::new();

            let mut variable_reals =
                Vec::<f64>::new();

            let mut variable_shorts =
                Vec::<i16>::new();

            if let Some(variable_kind) =
                spec.variable
            {
                let count =
                    variable_length
                        .ok_or_else(|| {
                            CodecError::Malformed(
                                "internal SCH_13006 variable-node state is inconsistent"
                                    .to_string(),
                            )
                        })?;

                match variable_kind {
                    VariableKind::RawChar => {
                        cursor.skip_raw_chars(
                            usize::try_from(count)
                                .map_err(|_| {
                                    CodecError::Malformed(
                                        "raw-char variable length does not fit host usize"
                                            .to_string(),
                                    )
                                })?,
                        )?;
                    }

                    VariableKind::Pointer => {
                        for _ in 0..count {
                            let value =
                                cursor.read_pointer(
                                    "variable pointer",
                                )?;

                            if type_id == POINTER_LIST {
                                variable_pointers.push(
                                    value
                                );
                            }
                        }
                    }

                    VariableKind::Signed => {
                        for _ in 0..count {
                            let _ =
                                cursor.read_signed(
                                    "variable signed integer",
                                )?;
                        }
                    }

                    VariableKind::Short => {
                        for _ in 0..count {
                            let value =
                                cursor.read_signed(
                                    "variable short integer",
                                )?;

                            variable_shorts.push(
                                i16::try_from(
                                    value
                                )
                                .map_err(|_| {
                                    CodecError::Malformed(
                                        format!(
                                            "variable short value {value} does not fit i16"
                                        ),
                                    )
                                })?,
                            );
                        }
                    }

                    VariableKind::Real => {
                        for _ in 0..count {
                            let value =
                                cursor
                                    .read_real(
                                        "variable real",
                                    )?
                                    .ok_or_else(|| {
                                        CodecError::Malformed(
                                            "qualified numeric payload contains a null real"
                                                .to_string(),
                                        )
                                    })?;

                            variable_reals.push(
                                value
                            );
                        }
                    }
                }
            }

            if matches!(
                type_id,
                POINT |
                LINE |
                CIRCLE |
                ELLIPSE |
                PLANE |
                CYLINDER |
                CONE |
                TRIMMED_CURVE |
                B_CURVE |
                SP_CURVE
            ) {
                let owner =
                    value_pointer(
                        &fields,
                        2,
                        "geometry owner",
                    )?;

                geometry_owner_by_index.insert(
                    index,
                    owner,
                );
            }

            if matches!(
                type_id,
                LINE |
                CIRCLE |
                ELLIPSE |
                TRIMMED_CURVE |
                B_CURVE |
                SP_CURVE
            ) {
                let sense =
                    value_character(
                        &fields,
                        6,
                        "curve sense",
                    )?;

                if !matches!(
                    sense,
                    b'+' | b'-'
                ) {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "curve {index} has invalid sense byte 0x{sense:02x}"
                            ),
                        ),
                    );
                }

                if type_id == TRIMMED_CURVE &&
                    sense != b'+'
                {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "TRIMMED_CURVE {index} must have positive sense"
                            ),
                        ),
                    );
                }

                curve_sense_by_index.insert(
                    index,
                    sense,
                );
            }

            *type_counts
                .entry(type_id)
                .or_insert(0) += 1;

            match type_id {
                POINTER_LIST if index == 1 => {
                    if root_entries.is_some() {
                        return Err(
                            CodecError::Malformed(
                                "duplicate root POINTER_LIS_BLOCK index 1"
                                    .to_string(),
                            ),
                        );
                    }

                    let n_entries =
                        value_u32(
                            &fields,
                            0,
                            "root POINTER_LIS_BLOCK n_entries",
                        )?;

                    let next_block =
                        value_pointer(
                            &fields,
                            1,
                            "root POINTER_LIS_BLOCK next_block",
                        )?;

                    let variable_length =
                        variable_length
                            .ok_or_else(|| {
                                CodecError::Malformed(
                                    "root POINTER_LIS_BLOCK is missing its variable length"
                                        .to_string(),
                                )
                            })?;

                    if next_block != 0 {
                        return Err(
                            CodecError::NotImplemented(
                                "chained root POINTER_LIS_BLOCK is not yet qualified"
                                    .to_string(),
                            ),
                        );
                    }

                    if n_entries != variable_length {
                        return Err(
                            CodecError::Malformed(
                                format!(
                                    "root POINTER_LIS_BLOCK n_entries={n_entries} differs from variable length={variable_length}"
                                ),
                            ),
                        );
                    }

                    if variable_pointers.len() !=
                        usize::try_from(
                            variable_length
                        )
                        .map_err(|_| {
                            CodecError::Malformed(
                                "root variable length does not fit host usize"
                                    .to_string(),
                            )
                        })?
                    {
                        return Err(
                            CodecError::Malformed(
                                "root POINTER_LIS_BLOCK payload length is inconsistent"
                                    .to_string(),
                            ),
                        );
                    }

                    root_variable_length =
                        Some(
                            variable_length
                        );

                    root_next_block =
                        Some(
                            next_block
                        );

                    root_entries =
                        Some(
                            variable_pointers
                        );
                }

                BODY => {
                    bodies.insert(
                        index,
                        BodyNode {
                            next:
                                value_pointer(
                                    &fields,
                                    10,
                                    "BODY next",
                                )?,

                            previous:
                                value_pointer(
                                    &fields,
                                    11,
                                    "BODY previous",
                                )?,

                            body_type:
                                value_u32(
                                    &fields,
                                    14,
                                    "BODY body_type",
                                )?,

                            shell:
                                value_pointer(
                                    &fields,
                                    16,
                                    "BODY shell",
                                )?,

                            region:
                                value_pointer(
                                    &fields,
                                    20,
                                    "BODY region",
                                )?,

                            edge:
                                value_pointer(
                                    &fields,
                                    21,
                                    "BODY edge",
                                )?,

                            vertex:
                                value_pointer(
                                    &fields,
                                    22,
                                    "BODY vertex",
                                )?,
                        },
                    );
                }

                SHELL => {
                    shells.insert(
                        index,
                        ShellNode {
                            body:
                                value_pointer(
                                    &fields,
                                    2,
                                    "SHELL body",
                                )?,

                            next:
                                value_pointer(
                                    &fields,
                                    3,
                                    "SHELL next",
                                )?,

                            face:
                                value_pointer(
                                    &fields,
                                    4,
                                    "SHELL face",
                                )?,

                            edge:
                                value_pointer(
                                    &fields,
                                    5,
                                    "SHELL edge",
                                )?,

                            vertex:
                                value_pointer(
                                    &fields,
                                    6,
                                    "SHELL vertex",
                                )?,

                            region:
                                value_pointer(
                                    &fields,
                                    7,
                                    "SHELL region",
                                )?,

                            front_face:
                                value_pointer(
                                    &fields,
                                    8,
                                    "SHELL front_face",
                                )?,
                        },
                    );
                }

                FACE => {
                    faces.insert(
                        index,
                        FaceNode {
                            tolerance:
                                value_optional_real(
                                    &fields,
                                    2,
                                    "FACE tolerance",
                                )?,

                            native_sense:
                                value_character(
                                    &fields,
                                    8,
                                    "FACE native sense",
                                )?,

                            next:
                                value_pointer(
                                    &fields,
                                    3,
                                    "FACE next",
                                )?,

                            previous:
                                value_pointer(
                                    &fields,
                                    4,
                                    "FACE previous",
                                )?,

                            loop_head:
                                value_pointer(
                                    &fields,
                                    5,
                                    "FACE loop",
                                )?,

                            shell:
                                value_pointer(
                                    &fields,
                                    6,
                                    "FACE shell",
                                )?,

                            surface:
                                value_pointer(
                                    &fields,
                                    7,
                                    "FACE surface",
                                )?,

                            next_on_surface:
                                value_pointer(
                                    &fields,
                                    9,
                                    "FACE next_on_surface",
                                )?,

                            previous_on_surface:
                                value_pointer(
                                    &fields,
                                    10,
                                    "FACE previous_on_surface",
                                )?,

                            next_front:
                                value_pointer(
                                    &fields,
                                    11,
                                    "FACE next_front",
                                )?,

                            previous_front:
                                value_pointer(
                                    &fields,
                                    12,
                                    "FACE previous_front",
                                )?,

                            front_shell:
                                value_pointer(
                                    &fields,
                                    13,
                                    "FACE front_shell",
                                )?,
                        },
                    );
                }

                LOOP => {
                    loops.insert(
                        index,
                        LoopNode {
                            fin:
                                value_pointer(
                                    &fields,
                                    2,
                                    "LOOP fin",
                                )?,

                            face:
                                value_pointer(
                                    &fields,
                                    3,
                                    "LOOP face",
                                )?,

                            next:
                                value_pointer(
                                    &fields,
                                    4,
                                    "LOOP next",
                                )?,
                        },
                    );
                }

                EDGE => {
                    edges.insert(
                        index,
                        EdgeNode {
                            tolerance:
                                value_optional_real(
                                    &fields,
                                    2,
                                    "EDGE tolerance",
                                )?,

                            fin:
                                value_pointer(
                                    &fields,
                                    3,
                                    "EDGE fin",
                                )?,

                            previous:
                                value_pointer(
                                    &fields,
                                    4,
                                    "EDGE previous",
                                )?,

                            next:
                                value_pointer(
                                    &fields,
                                    5,
                                    "EDGE next",
                                )?,

                            curve:
                                value_pointer(
                                    &fields,
                                    6,
                                    "EDGE curve",
                                )?,

                            next_on_curve:
                                value_pointer(
                                    &fields,
                                    7,
                                    "EDGE next_on_curve",
                                )?,

                            previous_on_curve:
                                value_pointer(
                                    &fields,
                                    8,
                                    "EDGE previous_on_curve",
                                )?,

                            owner:
                                value_pointer(
                                    &fields,
                                    9,
                                    "EDGE owner",
                                )?,
                        },
                    );
                }

                FIN => {
                    fins.insert(
                        index,
                        FinNode {
                            native_sense:
                                value_character(
                                    &fields,
                                    9,
                                    "FIN native sense",
                                )?,

                            loop_owner:
                                value_pointer(
                                    &fields,
                                    1,
                                    "FIN loop",
                                )?,

                            forward:
                                value_pointer(
                                    &fields,
                                    2,
                                    "FIN forward",
                                )?,

                            backward:
                                value_pointer(
                                    &fields,
                                    3,
                                    "FIN backward",
                                )?,

                            vertex:
                                value_pointer(
                                    &fields,
                                    4,
                                    "FIN vertex",
                                )?,

                            other:
                                value_pointer(
                                    &fields,
                                    5,
                                    "FIN other",
                                )?,

                            edge:
                                value_pointer(
                                    &fields,
                                    6,
                                    "FIN edge",
                                )?,

                            curve:
                                value_pointer(
                                    &fields,
                                    7,
                                    "FIN curve",
                                )?,

                            next_at_vertex:
                                value_pointer(
                                    &fields,
                                    8,
                                    "FIN next_at_vx",
                                )?,
                        },
                    );
                }

                VERTEX => {
                    vertices.insert(
                        index,
                        VertexNode {
                            tolerance:
                                value_optional_real(
                                    &fields,
                                    6,
                                    "VERTEX tolerance",
                                )?,

                            fin:
                                value_pointer(
                                    &fields,
                                    2,
                                    "VERTEX fin",
                                )?,

                            previous:
                                value_pointer(
                                    &fields,
                                    3,
                                    "VERTEX previous",
                                )?,

                            next:
                                value_pointer(
                                    &fields,
                                    4,
                                    "VERTEX next",
                                )?,

                            point:
                                value_pointer(
                                    &fields,
                                    5,
                                    "VERTEX point",
                                )?,

                            owner:
                                value_pointer(
                                    &fields,
                                    7,
                                    "VERTEX owner",
                                )?,
                        },
                    );
                }

                REGION => {
                    regions.insert(
                        index,
                        RegionNode {
                            body:
                                value_pointer(
                                    &fields,
                                    2,
                                    "REGION body",
                                )?,

                            next:
                                value_pointer(
                                    &fields,
                                    3,
                                    "REGION next",
                                )?,

                            previous:
                                value_pointer(
                                    &fields,
                                    4,
                                    "REGION previous",
                                )?,

                            shell:
                                value_pointer(
                                    &fields,
                                    5,
                                    "REGION shell",
                                )?,

                            kind:
                                value_character(
                                    &fields,
                                    6,
                                    "REGION type",
                                )?,
                        },
                    );
                }

                POINT => {
                    points.insert(
                        index,
                        PointGeometry {
                            pvec:
                                value_vector3(
                                    &fields,
                                    5,
                                    "POINT pvec",
                                )?,
                        },
                    );
                }

                LINE => {
                    lines.insert(
                        index,
                        LineGeometry {
                            pvec:
                                value_vector3(
                                    &fields,
                                    7,
                                    "LINE pvec",
                                )?,

                            direction:
                                value_vector3(
                                    &fields,
                                    8,
                                    "LINE direction",
                                )?,
                        },
                    );
                }

                CIRCLE => {
                    circles.insert(
                        index,
                        CircleGeometry {
                            centre:
                                value_vector3(
                                    &fields,
                                    7,
                                    "CIRCLE centre",
                                )?,

                            normal:
                                value_vector3(
                                    &fields,
                                    8,
                                    "CIRCLE normal",
                                )?,

                            x_axis:
                                value_vector3(
                                    &fields,
                                    9,
                                    "CIRCLE x_axis",
                                )?,

                            radius:
                                value_required_real(
                                    &fields,
                                    10,
                                    "CIRCLE radius",
                                )?,
                        },
                    );
                }

                ELLIPSE => {
                    ellipses.insert(
                        index,
                        EllipseGeometry {
                            centre:
                                value_vector3(
                                    &fields,
                                    7,
                                    "ELLIPSE centre",
                                )?,

                            normal:
                                value_vector3(
                                    &fields,
                                    8,
                                    "ELLIPSE normal",
                                )?,

                            x_axis:
                                value_vector3(
                                    &fields,
                                    9,
                                    "ELLIPSE x_axis",
                                )?,

                            major_radius:
                                value_required_real(
                                    &fields,
                                    10,
                                    "ELLIPSE major radius",
                                )?,

                            minor_radius:
                                value_required_real(
                                    &fields,
                                    11,
                                    "ELLIPSE minor radius",
                                )?,
                        },
                    );
                }

                PLANE => {
                    planes.insert(
                        index,
                        PlaneGeometry {
                            native_sense:
                                value_character(
                                    &fields,
                                    6,
                                    "PLANE native sense",
                                )?,

                            pvec:
                                value_vector3(
                                    &fields,
                                    7,
                                    "PLANE pvec",
                                )?,

                            normal:
                                value_vector3(
                                    &fields,
                                    8,
                                    "PLANE normal",
                                )?,

                            x_axis:
                                value_vector3(
                                    &fields,
                                    9,
                                    "PLANE x_axis",
                                )?,
                        },
                    );
                }

                CYLINDER => {
                    cylinders.insert(
                        index,
                        CylinderGeometry {
                            native_sense:
                                value_character(
                                    &fields,
                                    6,
                                    "CYLINDER native sense",
                                )?,

                            pvec:
                                value_vector3(
                                    &fields,
                                    7,
                                    "CYLINDER pvec",
                                )?,

                            axis:
                                value_vector3(
                                    &fields,
                                    8,
                                    "CYLINDER axis",
                                )?,

                            radius:
                                value_required_real(
                                    &fields,
                                    9,
                                    "CYLINDER radius",
                                )?,

                            x_axis:
                                value_vector3(
                                    &fields,
                                    10,
                                    "CYLINDER x_axis",
                                )?,
                        },
                    );
                }

                CONE => {
                    cones.insert(
                        index,
                        ConeGeometry {
                            native_sense:
                                value_character(
                                    &fields,
                                    6,
                                    "CONE native sense",
                                )?,

                            pvec:
                                value_vector3(
                                    &fields,
                                    7,
                                    "CONE pvec",
                                )?,

                            axis:
                                value_vector3(
                                    &fields,
                                    8,
                                    "CONE axis",
                                )?,

                            radius:
                                value_required_real(
                                    &fields,
                                    9,
                                    "CONE radius",
                                )?,

                            sin_half_angle:
                                value_required_real(
                                    &fields,
                                    10,
                                    "CONE sin_half_angle",
                                )?,

                            cos_half_angle:
                                value_required_real(
                                    &fields,
                                    11,
                                    "CONE cos_half_angle",
                                )?,

                            x_axis:
                                value_vector3(
                                    &fields,
                                    12,
                                    "CONE x_axis",
                                )?,
                        },
                    );
                }

                TRIMMED_CURVE => {
                    trimmed_curves.insert(
                        index,
                        TrimmedCurveGeometry {
                            basis_curve:
                                value_pointer(
                                    &fields,
                                    7,
                                    "TRIMMED_CURVE basis_curve",
                                )?,

                            point_1:
                                value_vector3(
                                    &fields,
                                    8,
                                    "TRIMMED_CURVE point_1",
                                )?,

                            point_2:
                                value_vector3(
                                    &fields,
                                    9,
                                    "TRIMMED_CURVE point_2",
                                )?,

                            parm_1:
                                value_required_real(
                                    &fields,
                                    10,
                                    "TRIMMED_CURVE parm_1",
                                )?,

                            parm_2:
                                value_required_real(
                                    &fields,
                                    11,
                                    "TRIMMED_CURVE parm_2",
                                )?,
                        },
                    );
                }

                B_CURVE => {
                    b_curves.insert(
                        index,
                        BCurveGeometry {
                            nurbs:
                                value_pointer(
                                    &fields,
                                    7,
                                    "B_CURVE nurbs",
                                )?,

                            data:
                                value_pointer(
                                    &fields,
                                    8,
                                    "B_CURVE data",
                                )?,
                        },
                    );
                }

                NURBS_CURVE => {
                    nurbs_curves.insert(
                        index,
                        NurbsCurveGeometry {
                            degree:
                                value_i64(
                                    &fields,
                                    0,
                                    "NURBS_CURVE degree",
                                )?,

                            n_vertices:
                                value_i64(
                                    &fields,
                                    1,
                                    "NURBS_CURVE n_vertices",
                                )?,

                            vertex_dim:
                                value_i64(
                                    &fields,
                                    2,
                                    "NURBS_CURVE vertex_dim",
                                )?,

                            n_knots:
                                value_i64(
                                    &fields,
                                    3,
                                    "NURBS_CURVE n_knots",
                                )?,

                            knot_type:
                                value_u32(
                                    &fields,
                                    4,
                                    "NURBS_CURVE knot_type",
                                )?,

                            periodic:
                                value_logical_bool(
                                    &fields,
                                    5,
                                    "NURBS_CURVE periodic",
                                )?,

                            closed:
                                value_logical_bool(
                                    &fields,
                                    6,
                                    "NURBS_CURVE closed",
                                )?,

                            rational:
                                value_logical_bool(
                                    &fields,
                                    7,
                                    "NURBS_CURVE rational",
                                )?,

                            curve_form:
                                value_u32(
                                    &fields,
                                    8,
                                    "NURBS_CURVE curve_form",
                                )?,

                            bspline_vertices:
                                value_pointer(
                                    &fields,
                                    9,
                                    "NURBS_CURVE bspline_vertices",
                                )?,

                            knot_mult:
                                value_pointer(
                                    &fields,
                                    10,
                                    "NURBS_CURVE knot_mult",
                                )?,

                            knots:
                                value_pointer(
                                    &fields,
                                    11,
                                    "NURBS_CURVE knots",
                                )?,
                        },
                    );
                }

                SP_CURVE => {
                    sp_curves.insert(
                        index,
                        SpCurveGeometry {
                            surface:
                                value_pointer(
                                    &fields,
                                    7,
                                    "SP_CURVE surface",
                                )?,

                            b_curve:
                                value_pointer(
                                    &fields,
                                    8,
                                    "SP_CURVE b_curve",
                                )?,

                            original:
                                value_pointer(
                                    &fields,
                                    9,
                                    "SP_CURVE original",
                                )?,

                            tolerance_to_original:
                                value_optional_real(
                                    &fields,
                                    10,
                                    "SP_CURVE tolerance_to_original",
                                )?,
                        },
                    );
                }

                BSPLINE_VERTICES => {
                    bspline_vertices.insert(
                        index,
                        variable_reals,
                    );
                }

                KNOT_MULT => {
                    knot_multiplicities.insert(
                        index,
                        variable_shorts,
                    );
                }

                KNOT_SET => {
                    knot_sets.insert(
                        index,
                        variable_reals,
                    );
                }
                _ => {}
            }

            total_entities += 1;
        };

    if terminator_index != 0 {
        return Err(
            CodecError::Malformed(
                format!(
                    "SCH_13006 terminator index must be 0, found {terminator_index}"
                ),
            ),
        );
    }

    cursor.require_end()?;

    let root_variable_length =
        root_variable_length
            .ok_or_else(|| {
                CodecError::Malformed(
                    "SCH_13006 root POINTER_LIS_BLOCK was not found"
                        .to_string(),
                )
            })?;

    let root_next_block =
        root_next_block
            .ok_or_else(|| {
                CodecError::Malformed(
                    "SCH_13006 root next-block state is missing"
                        .to_string(),
                )
            })?;

    let root_entry_indices =
        root_entries
            .ok_or_else(|| {
                CodecError::Malformed(
                    "SCH_13006 root entries were not found"
                        .to_string(),
                )
            })?;

    let mut validated_pointer_count =
        0usize;

    for entry in
        &root_entry_indices
    {
        validate_target(
            &type_by_index,
            1,
            "root part",
            *entry,
            &[BODY],
            &mut validated_pointer_count,
        )?;
    }

    for (index, body) in
        &bodies
    {
        validate_target(
            &type_by_index,
            *index,
            "BODY next",
            body.next,
            &[BODY],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "BODY previous",
            body.previous,
            &[BODY],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "BODY shell",
            body.shell,
            &[SHELL],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "BODY region",
            body.region,
            &[REGION],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "BODY edge",
            body.edge,
            &[EDGE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "BODY vertex",
            body.vertex,
            &[VERTEX],
            &mut validated_pointer_count,
        )?;
    }

    for (index, shell) in
        &shells
    {
        validate_target(
            &type_by_index,
            *index,
            "SHELL body",
            shell.body,
            &[BODY],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "SHELL next",
            shell.next,
            &[SHELL],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "SHELL face",
            shell.face,
            &[FACE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "SHELL edge",
            shell.edge,
            &[EDGE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "SHELL vertex",
            shell.vertex,
            &[VERTEX],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "SHELL region",
            shell.region,
            &[REGION],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "SHELL front_face",
            shell.front_face,
            &[FACE],
            &mut validated_pointer_count,
        )?;
    }

    for (index, face) in
        &faces
    {
        validate_target(
            &type_by_index,
            *index,
            "FACE next",
            face.next,
            &[FACE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FACE previous",
            face.previous,
            &[FACE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FACE loop",
            face.loop_head,
            &[LOOP],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FACE shell",
            face.shell,
            &[SHELL],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FACE surface",
            face.surface,
            &[PLANE, CYLINDER, CONE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FACE next_on_surface",
            face.next_on_surface,
            &[FACE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FACE previous_on_surface",
            face.previous_on_surface,
            &[FACE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FACE next_front",
            face.next_front,
            &[FACE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FACE previous_front",
            face.previous_front,
            &[FACE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FACE front_shell",
            face.front_shell,
            &[SHELL],
            &mut validated_pointer_count,
        )?;
    }

    for (index, loop_node) in
        &loops
    {
        validate_target(
            &type_by_index,
            *index,
            "LOOP fin",
            loop_node.fin,
            &[FIN],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "LOOP face",
            loop_node.face,
            &[FACE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "LOOP next",
            loop_node.next,
            &[LOOP],
            &mut validated_pointer_count,
        )?;
    }

    for (index, edge) in
        &edges
    {
        validate_target(
            &type_by_index,
            *index,
            "EDGE fin",
            edge.fin,
            &[FIN],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "EDGE previous",
            edge.previous,
            &[EDGE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "EDGE next",
            edge.next,
            &[EDGE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "EDGE curve",
            edge.curve,
            &[
                LINE,
                CIRCLE,
                ELLIPSE,
                TRIMMED_CURVE,
                B_CURVE,
                SP_CURVE,
            ],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "EDGE next_on_curve",
            edge.next_on_curve,
            &[EDGE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "EDGE previous_on_curve",
            edge.previous_on_curve,
            &[EDGE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "EDGE owner",
            edge.owner,
            &[BODY, SHELL],
            &mut validated_pointer_count,
        )?;
    }

    for (index, fin) in
        &fins
    {
        validate_target(
            &type_by_index,
            *index,
            "FIN loop",
            fin.loop_owner,
            &[LOOP],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FIN forward",
            fin.forward,
            &[FIN],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FIN backward",
            fin.backward,
            &[FIN],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FIN vertex",
            fin.vertex,
            &[VERTEX],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FIN other",
            fin.other,
            &[FIN],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FIN edge",
            fin.edge,
            &[EDGE],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FIN curve",
            fin.curve,
            &[
                LINE,
                CIRCLE,
                ELLIPSE,
                TRIMMED_CURVE,
                B_CURVE,
                SP_CURVE,
            ],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "FIN next_at_vx",
            fin.next_at_vertex,
            &[FIN],
            &mut validated_pointer_count,
        )?;
    }

    for (index, vertex) in
        &vertices
    {
        validate_target(
            &type_by_index,
            *index,
            "VERTEX fin",
            vertex.fin,
            &[FIN],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "VERTEX previous",
            vertex.previous,
            &[VERTEX],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "VERTEX next",
            vertex.next,
            &[VERTEX],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "VERTEX point",
            vertex.point,
            &[POINT],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "VERTEX owner",
            vertex.owner,
            &[BODY, SHELL],
            &mut validated_pointer_count,
        )?;
    }

    for (index, region) in
        &regions
    {
        validate_target(
            &type_by_index,
            *index,
            "REGION body",
            region.body,
            &[BODY],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "REGION next",
            region.next,
            &[REGION],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "REGION previous",
            region.previous,
            &[REGION],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "REGION shell",
            region.shell,
            &[SHELL],
            &mut validated_pointer_count,
        )?;

        if !matches!(
            region.kind,
            b'S' | b'V'
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "REGION {index} has unsupported type byte 0x{:02x}",
                        region.kind
                    ),
                ),
            );
        }
    }

    let mut geometry_owner_domain_validation_count =
        0usize;

    for (geometry_index, owner) in
        &geometry_owner_by_index
    {
        if *owner == 0 {
            continue;
        }

        let geometry_type =
            type_by_index
                .get(
                    geometry_index
                )
                .copied()
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "geometry entity {geometry_index} has no registered type"
                        ),
                    )
                })?;

        let allowed_owner_types: &[u32] =
            match geometry_type {
                POINT =>
                    &[VERTEX, BODY],

                LINE |
                CIRCLE |
                ELLIPSE |
                TRIMMED_CURVE |
                B_CURVE |
                SP_CURVE =>
                    &[EDGE, FIN, BODY],

                PLANE |
                CYLINDER |
                CONE =>
                    &[FACE, BODY],

                _ => {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "unexpected qualified geometry type {geometry_type}"
                            ),
                        ),
                    );
                }
            };

        validate_target(
            &type_by_index,
            *geometry_index,
            "geometry owner",
            *owner,
            allowed_owner_types,
            &mut geometry_owner_domain_validation_count,
        )?;
    }

    let mut face_surfaces =
        BTreeSet::<u32>::new();

    let mut face_surface_owner_match_count =
        0usize;

    let mut face_plane_count =
        0usize;

    let mut face_cylinder_count =
        0usize;

    let mut face_cone_count =
        0usize;

    for (face_index, face) in
        &faces
    {
        if face.surface == 0 {
            continue;
        }

        face_surfaces.insert(
            face.surface
        );

        if geometry_owner_by_index
            .get(
                &face.surface
            ) ==
            Some(
                face_index
            )
        {
            face_surface_owner_match_count += 1;
        }

        match type_by_index.get(
            &face.surface
        ) {
            Some(&PLANE) =>
                face_plane_count += 1,

            Some(&CYLINDER) =>
                face_cylinder_count += 1,

            Some(&CONE) =>
                face_cone_count += 1,

            Some(other) => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "FACE {face_index} references unsupported qualified surface type {other}"
                        ),
                    ),
                );
            }

            None => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "FACE {face_index} references missing surface {}",
                            face.surface
                        ),
                    ),
                );
            }
        }
    }

    let face_surface_count =
        faces
            .values()
            .filter(
                |face|
                    face.surface != 0
            )
            .count();

    let mut direct_edge_curves =
        BTreeSet::<u32>::new();

    let mut direct_edge_curve_owner_match_count =
        0usize;

    let mut null_edge_curve_count =
        0usize;

    let mut edge_line_count =
        0usize;

    let mut edge_circle_count =
        0usize;

    let mut edge_ellipse_count =
        0usize;

    let mut edge_trimmed_curve_count =
        0usize;

    for (edge_index, edge) in
        &edges
    {
        if edge.curve == 0 {
            null_edge_curve_count += 1;
            continue;
        }

        direct_edge_curves.insert(
            edge.curve
        );

        if geometry_owner_by_index
            .get(
                &edge.curve
            ) ==
            Some(
                edge_index
            )
        {
            direct_edge_curve_owner_match_count += 1;
        }

        match type_by_index.get(
            &edge.curve
        ) {
            Some(&LINE) =>
                edge_line_count += 1,

            Some(&CIRCLE) =>
                edge_circle_count += 1,

            Some(&ELLIPSE) =>
                edge_ellipse_count += 1,

            Some(&TRIMMED_CURVE) =>
                edge_trimmed_curve_count += 1,

            Some(&B_CURVE) |
            Some(&SP_CURVE) => {}

            Some(other) => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "EDGE {edge_index} references unsupported qualified curve type {other}"
                        ),
                    ),
                );
            }

            None => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "EDGE {edge_index} references missing curve {}",
                            edge.curve
                        ),
                    ),
                );
            }
        }
    }

    let direct_edge_curve_count =
        edges.len() -
        null_edge_curve_count;

    let mut fin_curves =
        BTreeSet::<u32>::new();

    let mut fin_curve_owner_match_count =
        0usize;

    let mut fin_trimmed_curve_count =
        0usize;

    let mut fins_by_edge =
        BTreeMap::<u32, Vec<u32>>::new();

    for (fin_index, fin) in
        &fins
    {
        if fin.edge != 0 {
            fins_by_edge
                .entry(
                    fin.edge
                )
                .or_default()
                .push(
                    *fin_index
                );
        }

        if fin.curve == 0 {
            continue;
        }

        fin_curves.insert(
            fin.curve
        );

        if geometry_owner_by_index
            .get(
                &fin.curve
            ) ==
            Some(
                fin_index
            )
        {
            fin_curve_owner_match_count += 1;
        }

        match type_by_index.get(
            &fin.curve
        ) {
            Some(&TRIMMED_CURVE) =>
                fin_trimmed_curve_count += 1,

            Some(&LINE) |
            Some(&CIRCLE) |
            Some(&ELLIPSE) |
            Some(&B_CURVE) |
            Some(&SP_CURVE) => {}

            Some(other) => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "FIN {fin_index} references unsupported qualified curve type {other}"
                        ),
                    ),
                );
            }

            None => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "FIN {fin_index} references missing curve {}",
                            fin.curve
                        ),
                    ),
                );
            }
        }
    }

    let fin_curve_count =
        fins
            .values()
            .filter(
                |fin|
                    fin.curve != 0
            )
            .count();

    let mut vertex_points =
        BTreeSet::<u32>::new();

    let mut vertex_point_owner_match_count =
        0usize;

    for (vertex_index, vertex) in
        &vertices
    {
        if vertex.point == 0 {
            continue;
        }

        vertex_points.insert(
            vertex.point
        );

        if geometry_owner_by_index
            .get(
                &vertex.point
            ) ==
            Some(
                vertex_index
            )
        {
            vertex_point_owner_match_count += 1;
        }
    }

    let vertex_point_count =
        vertices
            .values()
            .filter(
                |vertex|
                    vertex.point != 0
            )
            .count();

    let mut null_edge_two_fin_trim_count =
        0usize;

    let mut direct_edge_zero_fin_curve_count =
        0usize;

    for (edge_index, edge) in
        &edges
    {
        let edge_fins =
            fins_by_edge
                .get(
                    edge_index
                )
                .map(
                    Vec::as_slice
                )
                .unwrap_or(
                    &[]
                );

        let curved_fin_count =
            edge_fins
                .iter()
                .filter(
                    |fin_index| {
                        fins.get(
                            fin_index
                        )
                        .is_some_and(
                            |fin|
                                fin.curve != 0
                        )
                    },
                )
                .count();

        if edge.curve == 0 &&
            edge_fins.len() == 2 &&
            curved_fin_count == 2
        {
            let every_trimmed =
                edge_fins
                    .iter()
                    .all(
                        |fin_index| {
                            fins.get(
                                fin_index
                            )
                            .and_then(
                                |fin| {
                                    type_by_index.get(
                                        &fin.curve
                                    )
                                },
                            ) ==
                            Some(
                                &TRIMMED_CURVE
                            )
                        },
                    );

            if every_trimmed {
                null_edge_two_fin_trim_count += 1;
            }
        }

        if edge.curve != 0 &&
            curved_fin_count == 0
        {
            direct_edge_zero_fin_curve_count += 1;
        }
    }

    let direct_owner_match_count =
        face_surface_owner_match_count +
        direct_edge_curve_owner_match_count +
        fin_curve_owner_match_count +
        vertex_point_owner_match_count;
    let mut max_unit_vector_error =
        0.0_f64;

    let mut max_orthogonality_error =
        0.0_f64;

    let mut max_cone_trig_identity_error =
        0.0_f64;

    for (index, point) in
        &points
    {
        validate_position(
            point.pvec,
            &format!(
                "POINT {index}"
            ),
        )?;
    }

    for (index, line) in
        &lines
    {
        validate_position(
            line.pvec,
            &format!(
                "LINE {index} pvec"
            ),
        )?;

        validate_unit_vector(
            line.direction,
            &format!(
                "LINE {index} direction"
            ),
            &mut max_unit_vector_error,
        )?;
    }

    for (index, circle) in
        &circles
    {
        validate_position(
            circle.centre,
            &format!(
                "CIRCLE {index} centre"
            ),
        )?;

        validate_unit_vector(
            circle.normal,
            &format!(
                "CIRCLE {index} normal"
            ),
            &mut max_unit_vector_error,
        )?;

        validate_unit_vector(
            circle.x_axis,
            &format!(
                "CIRCLE {index} x_axis"
            ),
            &mut max_unit_vector_error,
        )?;

        validate_orthogonality(
            circle.normal,
            circle.x_axis,
            &format!(
                "CIRCLE {index} normal/x_axis"
            ),
            &mut max_orthogonality_error,
        )?;

        if circle.radius <= 0.0 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "CIRCLE {index} has non-positive radius {}",
                        circle.radius
                    ),
                ),
            );
        }
    }

    for (index, ellipse) in
        &ellipses
    {
        validate_position(
            ellipse.centre,
            &format!(
                "ELLIPSE {index} centre"
            ),
        )?;

        validate_unit_vector(
            ellipse.normal,
            &format!(
                "ELLIPSE {index} normal"
            ),
            &mut max_unit_vector_error,
        )?;

        validate_unit_vector(
            ellipse.x_axis,
            &format!(
                "ELLIPSE {index} x_axis"
            ),
            &mut max_unit_vector_error,
        )?;

        validate_orthogonality(
            ellipse.normal,
            ellipse.x_axis,
            &format!(
                "ELLIPSE {index} normal/x_axis"
            ),
            &mut max_orthogonality_error,
        )?;

        if ellipse.major_radius <= 0.0 ||
            ellipse.minor_radius <= 0.0 ||
            ellipse.major_radius <
                ellipse.minor_radius
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "ELLIPSE {index} has invalid radii major={} minor={}",
                        ellipse.major_radius,
                        ellipse.minor_radius
                    ),
                ),
            );
        }
    }

    for (index, plane) in
        &planes
    {
        validate_position(
            plane.pvec,
            &format!(
                "PLANE {index} pvec"
            ),
        )?;

        validate_unit_vector(
            plane.normal,
            &format!(
                "PLANE {index} normal"
            ),
            &mut max_unit_vector_error,
        )?;

        validate_unit_vector(
            plane.x_axis,
            &format!(
                "PLANE {index} x_axis"
            ),
            &mut max_unit_vector_error,
        )?;

        validate_orthogonality(
            plane.normal,
            plane.x_axis,
            &format!(
                "PLANE {index} normal/x_axis"
            ),
            &mut max_orthogonality_error,
        )?;
    }

    for (index, cylinder) in
        &cylinders
    {
        validate_position(
            cylinder.pvec,
            &format!(
                "CYLINDER {index} pvec"
            ),
        )?;

        validate_unit_vector(
            cylinder.axis,
            &format!(
                "CYLINDER {index} axis"
            ),
            &mut max_unit_vector_error,
        )?;

        validate_unit_vector(
            cylinder.x_axis,
            &format!(
                "CYLINDER {index} x_axis"
            ),
            &mut max_unit_vector_error,
        )?;

        validate_orthogonality(
            cylinder.axis,
            cylinder.x_axis,
            &format!(
                "CYLINDER {index} axis/x_axis"
            ),
            &mut max_orthogonality_error,
        )?;

        if cylinder.radius <= 0.0 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "CYLINDER {index} has non-positive radius {}",
                        cylinder.radius
                    ),
                ),
            );
        }
    }

    for (index, cone) in
        &cones
    {
        validate_position(
            cone.pvec,
            &format!(
                "CONE {index} pvec"
            ),
        )?;

        validate_unit_vector(
            cone.axis,
            &format!(
                "CONE {index} axis"
            ),
            &mut max_unit_vector_error,
        )?;

        validate_unit_vector(
            cone.x_axis,
            &format!(
                "CONE {index} x_axis"
            ),
            &mut max_unit_vector_error,
        )?;

        validate_orthogonality(
            cone.axis,
            cone.x_axis,
            &format!(
                "CONE {index} axis/x_axis"
            ),
            &mut max_orthogonality_error,
        )?;

        if cone.radius < 0.0 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "CONE {index} has negative reference radius {}",
                        cone.radius
                    ),
                ),
            );
        }

        let trig_error =
            (
                cone.sin_half_angle *
                cone.sin_half_angle +
                cone.cos_half_angle *
                cone.cos_half_angle -
                1.0
            )
            .abs();

        max_cone_trig_identity_error =
            max_cone_trig_identity_error.max(
                trig_error
            );

        if trig_error >
            GEOMETRY_VECTOR_TOLERANCE
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "CONE {index} sin/cos half-angle identity error={trig_error}"
                    ),
                ),
            );
        }
    }

    let (
        vertex_bounds_min_m,
        vertex_bounds_max_m,
        vertex_bounds_min_mm,
        vertex_bounds_max_mm,
    ) =
        if let Some(first_point) =
            points.values().next()
        {
            let mut minimum =
                first_point.pvec;

            let mut maximum =
                first_point.pvec;

            for point in
                points.values()
            {
                for axis in
                    0..3
                {
                    minimum[axis] =
                        minimum[axis].min(
                            point.pvec[axis]
                        );

                    maximum[axis] =
                        maximum[axis].max(
                            point.pvec[axis]
                        );
                }
            }

            let minimum_mm =
                [
                    minimum[0] *
                        METRES_TO_MILLIMETRES,

                    minimum[1] *
                        METRES_TO_MILLIMETRES,

                    minimum[2] *
                        METRES_TO_MILLIMETRES,
                ];

            let maximum_mm =
                [
                    maximum[0] *
                        METRES_TO_MILLIMETRES,

                    maximum[1] *
                        METRES_TO_MILLIMETRES,

                    maximum[2] *
                        METRES_TO_MILLIMETRES,
                ];

            (
                Some(minimum),
                Some(maximum),
                Some(minimum_mm),
                Some(maximum_mm),
            )
        }
        else {
            (
                None,
                None,
                None,
                None,
            )
        };

    let mut trimmed_basis_valid_count =
        0usize;

    let mut trimmed_parameter_direction_pass_count =
        0usize;

    let mut trimmed_basis_line_count =
        0usize;

    let mut trimmed_basis_circle_count =
        0usize;

    let mut trimmed_basis_ellipse_count =
        0usize;

    let mut trimmed_basis_b_curve_count =
        0usize;

    let mut trimmed_basis_sp_curve_count =
        0usize;

    let mut trimmed_basis_trimmed_curve_count =
        0usize;

    for (index, trimmed) in
        &trimmed_curves
    {
        validate_position(
            trimmed.point_1,
            &format!(
                "TRIMMED_CURVE {index} point_1"
            ),
        )?;

        validate_position(
            trimmed.point_2,
            &format!(
                "TRIMMED_CURVE {index} point_2"
            ),
        )?;

        let basis_type =
            type_by_index
                .get(
                    &trimmed.basis_curve
                )
                .copied()
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "TRIMMED_CURVE {index} references missing basis curve {}",
                            trimmed.basis_curve
                        ),
                    )
                })?;

        match basis_type {
            LINE =>
                trimmed_basis_line_count += 1,

            CIRCLE =>
                trimmed_basis_circle_count += 1,

            ELLIPSE =>
                trimmed_basis_ellipse_count += 1,

            B_CURVE =>
                trimmed_basis_b_curve_count += 1,

            SP_CURVE =>
                trimmed_basis_sp_curve_count += 1,

            TRIMMED_CURVE =>
                trimmed_basis_trimmed_curve_count += 1,

            other => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "TRIMMED_CURVE {index} has unsupported basis type {other}"
                        ),
                    ),
                );
            }
        }

        trimmed_basis_valid_count += 1;

        let basis_sense =
            curve_sense_by_index
                .get(
                    &trimmed.basis_curve
                )
                .copied()
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "TRIMMED_CURVE {index} basis {} has no curve sense",
                            trimmed.basis_curve
                        ),
                    )
                })?;

        let direction_valid =
            match basis_sense {
                b'+' =>
                    trimmed.parm_2 >
                    trimmed.parm_1,

                b'-' =>
                    trimmed.parm_2 <
                    trimmed.parm_1,

                _ => false,
            };

        if !direction_valid {
            return Err(
                CodecError::Malformed(
                    format!(
                        "TRIMMED_CURVE {index} parameter direction disagrees with basis curve sense"
                    ),
                ),
            );
        }

        trimmed_parameter_direction_pass_count += 1;
    }

    let mut sp_surface_link_count =
        0usize;

    let mut sp_b_curve_link_count =
        0usize;

    let mut unique_sp_b_curves =
        BTreeSet::<u32>::new();

    for (index, sp_curve) in
        &sp_curves
    {
        validate_target(
            &type_by_index,
            *index,
            "SP_CURVE surface",
            sp_curve.surface,
            &[PLANE, CYLINDER, CONE],
            &mut validated_pointer_count,
        )?;

        sp_surface_link_count += 1;

        validate_target(
            &type_by_index,
            *index,
            "SP_CURVE b_curve",
            sp_curve.b_curve,
            &[B_CURVE],
            &mut validated_pointer_count,
        )?;

        sp_b_curve_link_count += 1;

        unique_sp_b_curves.insert(
            sp_curve.b_curve
        );

        validate_target(
            &type_by_index,
            *index,
            "SP_CURVE original",
            sp_curve.original,
            &[
                LINE,
                CIRCLE,
                ELLIPSE,
                TRIMMED_CURVE,
                B_CURVE,
                SP_CURVE,
            ],
            &mut validated_pointer_count,
        )?;

        if let Some(tolerance) =
            sp_curve.tolerance_to_original
        {
            if tolerance < 0.0 {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "SP_CURVE {index} has negative tolerance_to_original={tolerance}"
                        ),
                    ),
                );
            }
        }
    }

    let mut b_curve_nurbs_link_count =
        0usize;

    let mut unique_b_curve_nurbs =
        BTreeSet::<u32>::new();

    for (index, b_curve) in
        &b_curves
    {
        validate_target(
            &type_by_index,
            *index,
            "B_CURVE nurbs",
            b_curve.nurbs,
            &[NURBS_CURVE],
            &mut validated_pointer_count,
        )?;

        b_curve_nurbs_link_count += 1;

        unique_b_curve_nurbs.insert(
            b_curve.nurbs
        );

        validate_target(
            &type_by_index,
            *index,
            "B_CURVE data",
            b_curve.data,
            &[CURVE_DATA],
            &mut validated_pointer_count,
        )?;
    }

    let mut referenced_bspline_vertices =
        BTreeSet::<u32>::new();

    let mut referenced_knot_mult =
        BTreeSet::<u32>::new();

    let mut referenced_knot_set =
        BTreeSet::<u32>::new();

    let mut nurbs_payload_contract_count =
        0usize;

    let mut nurbs_knot_relation_pass_count =
        0usize;

    let mut nurbs_knot_order_pass_count =
        0usize;

    let mut rational_nurbs_count =
        0usize;

    let mut periodic_nurbs_count =
        0usize;

    let mut closed_nurbs_count =
        0usize;

    let mut nurbs_degree_min =
        i64::MAX;

    let mut nurbs_degree_max =
        i64::MIN;

    let mut nurbs_vertex_dim_min =
        i64::MAX;

    let mut nurbs_vertex_dim_max =
        i64::MIN;

    let mut bspline_scalar_count =
        0usize;

    let mut knot_value_count =
        0usize;

    let mut knot_multiplicity_count =
        0usize;

    for (index, nurbs) in
        &nurbs_curves
    {
        if nurbs.degree < 1 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} has invalid degree {}",
                        nurbs.degree
                    ),
                ),
            );
        }

        if nurbs.n_vertices <=
            nurbs.degree
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} n_vertices={} must exceed degree={}",
                        nurbs.n_vertices,
                        nurbs.degree
                    ),
                ),
            );
        }

        if !matches!(
            nurbs.vertex_dim,
            2 | 3 | 4
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} has unsupported vertex_dim={}",
                        nurbs.vertex_dim
                    ),
                ),
            );
        }

        if nurbs.n_knots < 2 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} has invalid distinct knot count={}",
                        nurbs.n_knots
                    ),
                ),
            );
        }

        if !matches!(
            nurbs.knot_type,
            1..=6
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} has invalid knot_type={}",
                        nurbs.knot_type
                    ),
                ),
            );
        }

        if !matches!(
            nurbs.curve_form,
            1..=7
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} has invalid curve_form={}",
                        nurbs.curve_form
                    ),
                ),
            );
        }

        if nurbs.rational {
            rational_nurbs_count += 1;

            if !matches!(
                nurbs.vertex_dim,
                3 | 4
            ) {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "rational NURBS_CURVE {index} has incompatible vertex_dim={}",
                            nurbs.vertex_dim
                        ),
                    ),
                );
            }
        }
        else if !matches!(
            nurbs.vertex_dim,
            2 | 3
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "non-rational NURBS_CURVE {index} has incompatible vertex_dim={}",
                        nurbs.vertex_dim
                    ),
                ),
            );
        }

        if nurbs.periodic {
            periodic_nurbs_count += 1;
        }

        if nurbs.closed {
            closed_nurbs_count += 1;
        }

        nurbs_degree_min =
            nurbs_degree_min.min(
                nurbs.degree
            );

        nurbs_degree_max =
            nurbs_degree_max.max(
                nurbs.degree
            );

        nurbs_vertex_dim_min =
            nurbs_vertex_dim_min.min(
                nurbs.vertex_dim
            );

        nurbs_vertex_dim_max =
            nurbs_vertex_dim_max.max(
                nurbs.vertex_dim
            );

        validate_target(
            &type_by_index,
            *index,
            "NURBS_CURVE bspline_vertices",
            nurbs.bspline_vertices,
            &[BSPLINE_VERTICES],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "NURBS_CURVE knot_mult",
            nurbs.knot_mult,
            &[KNOT_MULT],
            &mut validated_pointer_count,
        )?;

        validate_target(
            &type_by_index,
            *index,
            "NURBS_CURVE knots",
            nurbs.knots,
            &[KNOT_SET],
            &mut validated_pointer_count,
        )?;

        referenced_bspline_vertices.insert(
            nurbs.bspline_vertices
        );

        referenced_knot_mult.insert(
            nurbs.knot_mult
        );

        referenced_knot_set.insert(
            nurbs.knots
        );

        let vertices =
            bspline_vertices
                .get(
                    &nurbs.bspline_vertices
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {index} references missing BSPLINE_VERTICES {}",
                            nurbs.bspline_vertices
                        ),
                    )
                })?;

        let multiplicities =
            knot_multiplicities
                .get(
                    &nurbs.knot_mult
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {index} references missing KNOT_MULT {}",
                            nurbs.knot_mult
                        ),
                    )
                })?;

        let knots =
            knot_sets
                .get(
                    &nurbs.knots
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {index} references missing KNOT_SET {}",
                            nurbs.knots
                        ),
                    )
                })?;

        let n_vertices =
            usize::try_from(
                nurbs.n_vertices
            )
            .map_err(|_| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} n_vertices does not fit usize"
                    ),
                )
            })?;

        let vertex_dim =
            usize::try_from(
                nurbs.vertex_dim
            )
            .map_err(|_| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} vertex_dim does not fit usize"
                    ),
                )
            })?;

        let n_knots =
            usize::try_from(
                nurbs.n_knots
            )
            .map_err(|_| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} n_knots does not fit usize"
                    ),
                )
            })?;

        let expected_vertex_scalars =
            n_vertices
                .checked_mul(
                    vertex_dim
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {index} control-vertex scalar count overflow"
                        ),
                    )
                })?;

        if vertices.len() !=
            expected_vertex_scalars
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} BSPLINE_VERTICES length mismatch expected={expected_vertex_scalars} actual={}",
                        vertices.len()
                    ),
                ),
            );
        }

        if multiplicities.len() !=
            n_knots
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} KNOT_MULT length mismatch expected={n_knots} actual={}",
                        multiplicities.len()
                    ),
                ),
            );
        }

        if knots.len() !=
            n_knots
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} KNOT_SET length mismatch expected={n_knots} actual={}",
                        knots.len()
                    ),
                ),
            );
        }

        nurbs_payload_contract_count += 1;

        let mut multiplicity_sum =
            0usize;

        for multiplicity in
            multiplicities
        {
            if *multiplicity <= 0 {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {index} contains non-positive knot multiplicity {multiplicity}"
                        ),
                    ),
                );
            }

            let value =
                usize::try_from(
                    *multiplicity
                )
                .map_err(|_| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {index} knot multiplicity does not fit usize"
                        ),
                    )
                })?;

            multiplicity_sum =
                multiplicity_sum
                    .checked_add(
                        value
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "NURBS_CURVE {index} knot multiplicity sum overflow"
                            ),
                        )
                    })?;
        }

        let expected_expanded_knots =
            n_vertices
                .checked_add(
                    usize::try_from(
                        nurbs.degree
                    )
                    .map_err(|_| {
                        CodecError::Malformed(
                            format!(
                                "NURBS_CURVE {index} degree does not fit usize"
                            ),
                        )
                    })?,
                )
                .and_then(
                    |value|
                        value.checked_add(1)
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {index} expanded knot-count overflow"
                        ),
                    )
                })?;

        if multiplicity_sum !=
            expected_expanded_knots
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} expanded knot relation failed expected={expected_expanded_knots} actual={multiplicity_sum}"
                    ),
                ),
            );
        }

        nurbs_knot_relation_pass_count += 1;

        if !knots
            .windows(2)
            .all(
                |window|
                    window[0] <
                    window[1]
            )
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {index} distinct knot values are not strictly increasing"
                    ),
                ),
            );
        }

        nurbs_knot_order_pass_count += 1;

        bspline_scalar_count +=
            vertices.len();

        knot_value_count +=
            knots.len();

        knot_multiplicity_count +=
            multiplicities.len();
    }

    if nurbs_curves.is_empty() {
        nurbs_degree_min = 0;
        nurbs_degree_max = 0;
        nurbs_vertex_dim_min = 0;
        nurbs_vertex_dim_max = 0;
    }

    let evaluate_sp_point_m =
        |
            sp_index: u32,
            parameter: f64,
        |
         -> Result<[f64; 3], CodecError> {
            let sp_curve =
                sp_curves.get(
                    &sp_index
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "missing SP_CURVE {sp_index}"
                        ),
                    )
                })?;

            let b_curve =
                b_curves.get(
                    &sp_curve.b_curve
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "SP_CURVE {sp_index} references missing B_CURVE {}",
                            sp_curve.b_curve
                        ),
                    )
                })?;

            let nurbs =
                nurbs_curves.get(
                    &b_curve.nurbs
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "B_CURVE {} references missing NURBS_CURVE {}",
                            sp_curve.b_curve,
                            b_curve.nurbs
                        ),
                    )
                })?;

            let vertices_payload =
                bspline_vertices.get(
                    &nurbs.bspline_vertices
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {} references missing BSPLINE_VERTICES {}",
                            b_curve.nurbs,
                            nurbs.bspline_vertices
                        ),
                    )
                })?;

            let multiplicities =
                knot_multiplicities.get(
                    &nurbs.knot_mult
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {} references missing KNOT_MULT {}",
                            b_curve.nurbs,
                            nurbs.knot_mult
                        ),
                    )
                })?;

            let distinct_knots =
                knot_sets.get(
                    &nurbs.knots
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {} references missing KNOT_SET {}",
                            b_curve.nurbs,
                            nurbs.knots
                        ),
                    )
                })?;

            let uv =
                evaluate_nurbs_2d(
                    b_curve.nurbs,
                    nurbs,
                    vertices_payload,
                    multiplicities,
                    distinct_knots,
                    parameter,
                )?;

            embed_surface_point_m(
                sp_curve.surface,
                uv,
                &type_by_index,
                &planes,
                &cylinders,
                &cones,
            )
        };

    let mut sp_uv_contract_count =
        0usize;

    let mut sp_original_null_count =
        0usize;

    let mut sp_original_nonnull_count =
        0usize;

    let mut sp_support_plane_count =
        0usize;

    let mut sp_support_cylinder_count =
        0usize;

    let mut sp_support_cone_count =
        0usize;

    let mut sp_nonrational_uv_count =
        0usize;

    let mut sp_rational_uv_count =
        0usize;

    for (
        sp_index,
        sp_curve,
    ) in
        &sp_curves
    {
        if sp_curve.original == 0 {
            sp_original_null_count +=
                1;
        }
        else {
            sp_original_nonnull_count +=
                1;
        }

        match type_by_index
            .get(
                &sp_curve.surface
            )
            .copied()
        {
            Some(
                PLANE
            ) => {
                sp_support_plane_count +=
                    1;
            }

            Some(
                CYLINDER
            ) => {
                sp_support_cylinder_count +=
                    1;
            }

            Some(
                CONE
            ) => {
                sp_support_cone_count +=
                    1;
            }

            Some(
                other
            ) => {
                return Err(
                    CodecError::NotImplemented(
                        format!(
                            "SP_CURVE {sp_index} support type {other} is outside the qualified analytic set"
                        ),
                    ),
                );
            }

            None => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "SP_CURVE {sp_index} references missing surface {}",
                            sp_curve.surface
                        ),
                    ),
                );
            }
        }

        let b_curve =
            b_curves.get(
                &sp_curve.b_curve
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "SP_CURVE {sp_index} references missing B_CURVE {}",
                        sp_curve.b_curve
                    ),
                )
            })?;

        let nurbs =
            nurbs_curves.get(
                &b_curve.nurbs
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "B_CURVE {} references missing NURBS_CURVE {}",
                        sp_curve.b_curve,
                        b_curve.nurbs
                    ),
                )
            })?;

        match (
            nurbs.rational,
            nurbs.vertex_dim,
        ) {
            (
                false,
                2,
            ) => {
                sp_nonrational_uv_count +=
                    1;
            }

            (
                true,
                3,
            ) => {
                sp_rational_uv_count +=
                    1;
            }

            _ => {
                return Err(
                    CodecError::NotImplemented(
                        format!(
                            "SP_CURVE {sp_index} B_CURVE {} is not a qualified 2D NURBS: rational={} vertex_dim={}",
                            sp_curve.b_curve,
                            nurbs.rational,
                            nurbs.vertex_dim
                        ),
                    ),
                );
            }
        }

        let multiplicities =
            knot_multiplicities.get(
                &nurbs.knot_mult
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {} references missing KNOT_MULT {}",
                        b_curve.nurbs,
                        nurbs.knot_mult
                    ),
                )
            })?;

        let distinct_knots =
            knot_sets.get(
                &nurbs.knots
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {} references missing KNOT_SET {}",
                        b_curve.nurbs,
                        nurbs.knots
                    ),
                )
            })?;

        let expanded_knots =
            expand_nurbs_knots(
                b_curve.nurbs,
                multiplicities,
                distinct_knots,
            )?;

        let degree =
            usize::try_from(
                nurbs.degree
            )
            .map_err(|_| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {} degree does not fit usize",
                        b_curve.nurbs
                    ),
                )
            })?;

        let n_vertices =
            usize::try_from(
                nurbs.n_vertices
            )
            .map_err(|_| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {} n_vertices does not fit usize",
                        b_curve.nurbs
                    ),
                )
            })?;

        let domain_start =
            *expanded_knots
                .get(
                    degree
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {} missing lower domain knot",
                            b_curve.nurbs
                        ),
                    )
                })?;

        let domain_end =
            *expanded_knots
                .get(
                    n_vertices
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {} missing upper domain knot",
                            b_curve.nurbs
                        ),
                    )
                })?;

        let _ =
            evaluate_sp_point_m(
                *sp_index,
                domain_start,
            )?;

        let _ =
            evaluate_sp_point_m(
                *sp_index,
                domain_end,
            )?;

        sp_uv_contract_count +=
            1;
    }

    let mut trimmed_sp_curve_count =
        0usize;

    let mut trimmed_sp_basis_references =
        BTreeMap::<u32, usize>::new();

    let mut trimmed_sp_direct_endpoint_order_count =
        0usize;

    let mut trimmed_sp_reversed_endpoint_order_count =
        0usize;

    let mut trimmed_sp_ambiguous_endpoint_order_count =
        0usize;

    let mut trimmed_sp_endpoint_witness_count =
        0usize;

    let mut max_sp_embedding_error_m =
        0.0f64;

    for (
        trimmed_index,
        trimmed,
    ) in
        &trimmed_curves
    {
        if type_by_index
            .get(
                &trimmed.basis_curve
            )
            .copied() !=
            Some(
                SP_CURVE
            )
        {
            continue;
        }

        trimmed_sp_curve_count +=
            1;

        *trimmed_sp_basis_references
            .entry(
                trimmed.basis_curve
            )
            .or_insert(0) +=
            1;

        let evaluated_1 =
            evaluate_sp_point_m(
                trimmed.basis_curve,
                trimmed.parm_1,
            )?;

        let evaluated_2 =
            evaluate_sp_point_m(
                trimmed.basis_curve,
                trimmed.parm_2,
            )?;

        let direct_error =
            point_distance(
                evaluated_1,
                trimmed.point_1,
            )
            .max(
                point_distance(
                    evaluated_2,
                    trimmed.point_2,
                )
            );

        let reversed_error =
            point_distance(
                evaluated_1,
                trimmed.point_2,
            )
            .max(
                point_distance(
                    evaluated_2,
                    trimmed.point_1,
                )
            );

        let direct_pass =
            direct_error <=
            CADIR_COINCIDENCE_TOLERANCE_M;

        let reversed_pass =
            reversed_error <=
            CADIR_COINCIDENCE_TOLERANCE_M;

        let accepted_error =
            match (
                direct_pass,
                reversed_pass,
            ) {
                (
                    true,
                    false,
                ) => {
                    trimmed_sp_direct_endpoint_order_count +=
                        1;

                    direct_error
                }

                (
                    false,
                    true,
                ) => {
                    trimmed_sp_reversed_endpoint_order_count +=
                        1;

                    reversed_error
                }

                (
                    true,
                    true,
                ) => {
                    trimmed_sp_ambiguous_endpoint_order_count +=
                        1;

                    direct_error.min(
                        reversed_error
                    )
                }

                (
                    false,
                    false,
                ) => {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "TRIMMED_CURVE {trimmed_index} SP_CURVE {} embedding misses endpoint witnesses: direct_error_m={direct_error} reversed_error_m={reversed_error} tolerance_m={CADIR_COINCIDENCE_TOLERANCE_M}",
                                trimmed.basis_curve
                            ),
                        ),
                    );
                }
            };

        max_sp_embedding_error_m =
            max_sp_embedding_error_m.max(
                accepted_error
            );

        trimmed_sp_endpoint_witness_count +=
            2;
    }

    let unique_trimmed_sp_basis_count =
        trimmed_sp_basis_references.len();

    let unwrapped_sp_curve_count =
        sp_curves
            .len()
            .saturating_sub(
                unique_trimmed_sp_basis_count
            );

    let sp_trim_reference_min =
        trimmed_sp_basis_references
            .values()
            .copied()
            .min()
            .unwrap_or(0);

    let sp_trim_reference_max =
        trimmed_sp_basis_references
            .values()
            .copied()
            .max()
            .unwrap_or(0);

    let mut direct_edge_trimmed_sp_count =
        0usize;

    for edge in
        edges.values()
    {
        if let Some(
            trimmed
        ) =
            trimmed_curves.get(
                &edge.curve
            )
        {
            if type_by_index
                .get(
                    &trimmed.basis_curve
                )
                .copied() ==
                Some(
                    SP_CURVE
                )
            {
                direct_edge_trimmed_sp_count +=
                    1;
            }
        }
    }

    let mut fin_trimmed_sp_count =
        0usize;

    for fin in
        fins.values()
    {
        if let Some(
            trimmed
        ) =
            trimmed_curves.get(
                &fin.curve
            )
        {
            if type_by_index
                .get(
                    &trimmed.basis_curve
                )
                .copied() ==
                Some(
                    SP_CURVE
                )
            {
                fin_trimmed_sp_count +=
                    1;
            }
        }
    }

    let max_sp_embedding_error_mm =
        max_sp_embedding_error_m *
        METRES_TO_MILLIMETRES;

    let geometry_bindings =
        Sch13006GeometryBindings {
            face_surface_count,

            unique_face_surface_count:
                face_surfaces.len(),

            face_surface_owner_match_count,

            face_plane_count,

            face_cylinder_count,

            face_cone_count,

            direct_edge_curve_count,

            null_edge_curve_count,

            unique_direct_edge_curve_count:
                direct_edge_curves.len(),

            direct_edge_curve_owner_match_count,

            edge_line_count,

            edge_circle_count,

            edge_ellipse_count,

            edge_trimmed_curve_count,

            fin_curve_count,

            unique_fin_curve_count:
                fin_curves.len(),

            fin_curve_owner_match_count,

            fin_trimmed_curve_count,

            vertex_point_count,

            unique_vertex_point_count:
                vertex_points.len(),

            vertex_point_owner_match_count,

            null_edge_two_fin_trim_count,

            direct_edge_zero_fin_curve_count,

            geometry_owner_domain_validation_count,

            direct_owner_match_count,

            trimmed_curve_count:
                trimmed_curves.len(),

            trimmed_basis_valid_count,

            trimmed_parameter_direction_pass_count,

            trimmed_basis_line_count,

            trimmed_basis_circle_count,

            trimmed_basis_ellipse_count,

            trimmed_basis_b_curve_count,

            trimmed_basis_sp_curve_count,

            trimmed_basis_trimmed_curve_count,

            sp_curve_count:
                sp_curves.len(),

            sp_surface_link_count,

            sp_b_curve_link_count,

            unique_sp_b_curve_count:
                unique_sp_b_curves.len(),

            b_curve_count:
                b_curves.len(),

            b_curve_nurbs_link_count,

            unique_b_curve_nurbs_count:
                unique_b_curve_nurbs.len(),

            nurbs_curve_count:
                nurbs_curves.len(),

            nurbs_payload_contract_count,

            nurbs_knot_relation_pass_count,

            nurbs_knot_order_pass_count,

            rational_nurbs_count,

            periodic_nurbs_count,

            closed_nurbs_count,

            nurbs_degree_min,

            nurbs_degree_max,

            nurbs_vertex_dim_min,

            nurbs_vertex_dim_max,

            referenced_bspline_vertex_node_count:
                referenced_bspline_vertices.len(),

            referenced_knot_mult_node_count:
                referenced_knot_mult.len(),

            referenced_knot_set_node_count:
                referenced_knot_set.len(),

            bspline_scalar_count,

            knot_value_count,

            knot_multiplicity_count,

            max_unit_vector_error,

            max_orthogonality_error,

            max_cone_trig_identity_error,

            vertex_bounds_min_m,

            vertex_bounds_max_m,

            vertex_bounds_min_mm,

            vertex_bounds_max_mm,

            metres_to_millimetres:
                METRES_TO_MILLIMETRES,

            sp_uv_contract_count,

            sp_original_null_count,

            sp_original_nonnull_count,

            sp_support_plane_count,

            sp_support_cylinder_count,

            sp_support_cone_count,

            sp_nonrational_uv_count,

            sp_rational_uv_count,

            trimmed_sp_curve_count,

            unique_trimmed_sp_basis_count,

            unwrapped_sp_curve_count,

            sp_trim_reference_min,

            sp_trim_reference_max,

            trimmed_sp_direct_endpoint_order_count,

            trimmed_sp_reversed_endpoint_order_count,

            trimmed_sp_ambiguous_endpoint_order_count,

            trimmed_sp_endpoint_witness_count,

            direct_edge_trimmed_sp_count,

            fin_trimmed_sp_count,

            max_sp_embedding_error_m,

            max_sp_embedding_error_mm,

            cadir_coincidence_tolerance_mm:
                CADIR_COINCIDENCE_TOLERANCE_M *
                METRES_TO_MILLIMETRES,
        };
    let solid_body_count =
        bodies
            .values()
            .filter(
                |body|
                    body.body_type == 1
            )
            .count();

    let solid_region_count =
        regions
            .values()
            .filter(
                |region|
                    region.kind == b'S'
            )
            .count();

    let void_region_count =
        regions
            .values()
            .filter(
                |region|
                    region.kind == b'V'
            )
            .count();

    let mut solid_shells =
        BTreeSet::<u32>::new();

    for region in
        regions.values()
    {
        if region.kind == b'S' {
            collect_shell_chain(
                region.shell,
                &shells,
                &mut solid_shells,
            )?;
        }
    }

    let mut reachable_faces =
        BTreeSet::<u32>::new();

    for shell_index in
        &solid_shells
    {
        let shell =
            shells.get(
                shell_index
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "solid shell {shell_index} disappeared during reachability validation"
                    ),
                )
            })?;

        collect_face_chain(
            shell.face,
            false,
            &faces,
            &mut reachable_faces,
        )?;

        collect_face_chain(
            shell.front_face,
            true,
            &faces,
            &mut reachable_faces,
        )?;
    }

    let mut reachable_loops =
        BTreeSet::<u32>::new();

    for face_index in
        &reachable_faces
    {
        let face =
            faces.get(
                face_index
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "reachable FACE {face_index} is missing"
                    ),
                )
            })?;

        collect_loop_chain(
            face.loop_head,
            &loops,
            &mut reachable_loops,
        )?;
    }

    let mut reachable_fins =
        BTreeSet::<u32>::new();

    for loop_index in
        &reachable_loops
    {
        let loop_node =
            loops.get(
                loop_index
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "reachable LOOP {loop_index} is missing"
                    ),
                )
            })?;

        collect_fin_ring(
            loop_node.fin,
            &fins,
            &mut reachable_fins,
        )?;
    }

    let mut reachable_edges =
        BTreeSet::<u32>::new();

    for body in
        bodies.values()
    {
        collect_edge_chain(
            body.edge,
            &edges,
            &mut reachable_edges,
        )?;
    }

    for shell in
        shells.values()
    {
        collect_edge_chain(
            shell.edge,
            &edges,
            &mut reachable_edges,
        )?;
    }

    let mut reachable_vertices =
        BTreeSet::<u32>::new();

    for body in
        bodies.values()
    {
        collect_vertex_chain(
            body.vertex,
            &vertices,
            &mut reachable_vertices,
        )?;
    }

    for shell in
        shells.values()
    {
        if shell.vertex != 0 {
            reachable_vertices.insert(
                shell.vertex
            );
        }
    }

    let mut reachable_points =
        BTreeSet::<u32>::new();

    for vertex_index in
        &reachable_vertices
    {
        let vertex =
            vertices.get(
                vertex_index
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "reachable VERTEX {vertex_index} is missing"
                    ),
                )
            })?;

        if vertex.point != 0 {
            reachable_points.insert(
                vertex.point
            );
        }
    }

    // ===========================================================
    // P10.A.6H
    //
    // Qualify source-owned topology/orientation facts only.
    // No CADIR is constructed here.
    // No missing topology, sense, tolerance, or endpoint is inferred.
    // ===========================================================

    let mut face_tolerance_count =
        0usize;

    let mut edge_tolerance_count =
        0usize;

    let mut vertex_tolerance_count =
        0usize;

    let mut max_face_tolerance_m =
        0.0_f64;

    let mut max_edge_tolerance_m =
        0.0_f64;

    let mut max_vertex_tolerance_m =
        0.0_f64;

    let mut face_positive_sense_count =
        0usize;

    let mut face_negative_sense_count =
        0usize;

    let mut fin_positive_sense_count =
        0usize;

    let mut fin_negative_sense_count =
        0usize;

    let mut surface_positive_sense_count =
        0usize;

    let mut surface_negative_sense_count =
        0usize;

    for (index, face) in &faces {
        if let Some(tolerance) = face.tolerance {
            if !tolerance.is_finite() || tolerance < 0.0 {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "FACE {index} has invalid tolerance {tolerance}"
                        ),
                    ),
                );
            }

            face_tolerance_count += 1;
            max_face_tolerance_m =
                max_face_tolerance_m.max(tolerance);
        }

        match face.native_sense {
            b'+' => face_positive_sense_count += 1,
            b'-' => face_negative_sense_count += 1,
            other => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "FACE {index} has invalid native sense 0x{other:02x}"
                        ),
                    ),
                );
            }
        }
    }

    for (index, edge) in &edges {
        if let Some(tolerance) = edge.tolerance {
            if !tolerance.is_finite() || tolerance < 0.0 {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "EDGE {index} has invalid tolerance {tolerance}"
                        ),
                    ),
                );
            }

            edge_tolerance_count += 1;
            max_edge_tolerance_m =
                max_edge_tolerance_m.max(tolerance);
        }
    }

    for (index, vertex) in &vertices {
        if let Some(tolerance) = vertex.tolerance {
            if !tolerance.is_finite() || tolerance < 0.0 {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "VERTEX {index} has invalid tolerance {tolerance}"
                        ),
                    ),
                );
            }

            vertex_tolerance_count += 1;
            max_vertex_tolerance_m =
                max_vertex_tolerance_m.max(tolerance);
        }
    }

    for (index, plane) in &planes {
        match plane.native_sense {
            b'+' => surface_positive_sense_count += 1,
            b'-' => surface_negative_sense_count += 1,
            other => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "PLANE {index} has invalid native sense 0x{other:02x}"
                        ),
                    ),
                );
            }
        }
    }

    for (index, cylinder) in &cylinders {
        match cylinder.native_sense {
            b'+' => surface_positive_sense_count += 1,
            b'-' => surface_negative_sense_count += 1,
            other => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "CYLINDER {index} has invalid native sense 0x{other:02x}"
                        ),
                    ),
                );
            }
        }
    }

    for (index, cone) in &cones {
        match cone.native_sense {
            b'+' => surface_positive_sense_count += 1,
            b'-' => surface_negative_sense_count += 1,
            other => {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "CONE {index} has invalid native sense 0x{other:02x}"
                        ),
                    ),
                );
            }
        }
    }

    // -----------------------------------------------------------
    // H4 — material-side shell ownership.
    // -----------------------------------------------------------

    let mut solid_shell_face_use_count =
        0usize;

    let mut solid_shell_opposite_void_count =
        0usize;

    let mut material_face_uses =
        BTreeMap::<u32, usize>::new();

    for shell_index in &solid_shells {
        let shell =
            shells.get(shell_index).ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "solid shell {shell_index} is missing"
                    ),
                )
            })?;

        let solid_region =
            regions.get(&shell.region).ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "solid shell {shell_index} references missing REGION {}",
                        shell.region
                    ),
                )
            })?;

        if solid_region.kind != b'S' {
            return Err(
                CodecError::Malformed(
                    format!(
                        "solid shell {shell_index} resolves to non-solid REGION {}",
                        shell.region
                    ),
                ),
            );
        }

        for (head, front_side) in [
            (shell.face, false),
            (shell.front_face, true),
        ] {
            let mut current = head;
            let mut local = BTreeSet::<u32>::new();

            while current != 0 {
                if !local.insert(current) {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "solid shell {shell_index} FACE chain cycles at {current}"
                            ),
                        ),
                    );
                }

                let face =
                    faces.get(&current).ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "solid shell {shell_index} references missing FACE {current}"
                            ),
                        )
                    })?;

                let selected_shell =
                    if front_side {
                        face.front_shell
                    } else {
                        face.shell
                    };

                if selected_shell != *shell_index {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "FACE {current} material-side shell mismatch expected={shell_index} actual={selected_shell}"
                            ),
                        ),
                    );
                }

                *material_face_uses
                    .entry(current)
                    .or_insert(0) += 1;

                solid_shell_face_use_count += 1;

                let opposite_shell_index =
                    if front_side {
                        face.shell
                    } else {
                        face.front_shell
                    };

                if opposite_shell_index == 0 {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "FACE {current} has no opposite shell"
                            ),
                        ),
                    );
                }

                let opposite_shell =
                    shells
                        .get(&opposite_shell_index)
                        .ok_or_else(|| {
                            CodecError::Malformed(
                                format!(
                                    "FACE {current} references missing opposite SHELL {opposite_shell_index}"
                                ),
                            )
                        })?;

                let opposite_region =
                    regions
                        .get(&opposite_shell.region)
                        .ok_or_else(|| {
                            CodecError::Malformed(
                                format!(
                                    "opposite SHELL {opposite_shell_index} references missing REGION {}",
                                    opposite_shell.region
                                ),
                            )
                        })?;

                if opposite_region.kind != b'V' {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "FACE {current} opposite SHELL {opposite_shell_index} is not void-side"
                            ),
                        ),
                    );
                }

                if opposite_region.body != solid_region.body {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "FACE {current} opposite shell crosses BODY ownership"
                            ),
                        ),
                    );
                }

                solid_shell_opposite_void_count += 1;

                current =
                    if front_side {
                        face.next_front
                    } else {
                        face.next
                    };
            }
        }
    }

    if material_face_uses.len() != reachable_faces.len() {
        return Err(
            CodecError::Malformed(
                format!(
                    "material FACE coverage mismatch material={} reachable={}",
                    material_face_uses.len(),
                    reachable_faces.len()
                ),
            ),
        );
    }

    for face_index in &reachable_faces {
        let count =
            material_face_uses
                .get(face_index)
                .copied()
                .unwrap_or(0);

        if count != 1 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "reachable FACE {face_index} has material-side use count {count}; expected 1"
                    ),
                ),
            );
        }
    }

    // -----------------------------------------------------------
    // H5 — FIN loop rings and radial EDGE rings.
    //
    // forward/backward:
    //     loop traversal.
    //
    // other:
    //     next FIN in the radial ring around the same EDGE.
    //
    // No reciprocal two-FIN assumption is made.
    // -----------------------------------------------------------

    let mut loop_with_fin_count =
        0usize;

    let mut fin_loop_owner_match_count =
        0usize;

    let mut fin_forward_backward_reciprocity_count =
        0usize;

    let mut singular_fin_count =
        0usize;

    let mut loop_fins =
        BTreeSet::<u32>::new();

    let mut isolated_fins =
        BTreeSet::<u32>::new();

    for loop_index in &reachable_loops {
        let loop_node =
            loops.get(loop_index).ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "reachable LOOP {loop_index} disappeared"
                    ),
                )
            })?;

        if loop_node.fin == 0 {
            continue;
        }

        loop_with_fin_count += 1;

        let start =
            loop_node.fin;

        let mut current =
            start;

        let mut local =
            BTreeSet::<u32>::new();

        loop {
            if !local.insert(current) {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "FIN loop repeats {current} before returning to {start}"
                        ),
                    ),
                );
            }

            if !loop_fins.insert(current) {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "FIN {current} occurs in multiple LOOP rings"
                        ),
                    ),
                );
            }

            let fin =
                fins.get(&current).ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "reachable FIN {current} disappeared"
                        ),
                    )
                })?;

            if fin.loop_owner != *loop_index {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "FIN {current} LOOP owner mismatch expected={loop_index} actual={}",
                            fin.loop_owner
                        ),
                    ),
                );
            }

            fin_loop_owner_match_count += 1;

            if fin.forward == 0 ||
                fin.backward == 0
            {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "loop-owned FIN {current} has null neighbor"
                        ),
                    ),
                );
            }

            let forward =
                fins.get(&fin.forward).ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "FIN {current} forward {} is missing",
                            fin.forward
                        ),
                    )
                })?;

            let backward =
                fins.get(&fin.backward).ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "FIN {current} backward {} is missing",
                            fin.backward
                        ),
                    )
                })?;

            if forward.backward != current ||
                backward.forward != current
            {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "FIN {current} forward/backward reciprocity failed"
                        ),
                    ),
                );
            }

            fin_forward_backward_reciprocity_count += 1;

            if fin.edge == 0 {
                if fin.other != 0 ||
                    fin.curve != 0 ||
                    fin.forward != current ||
                    fin.backward != current
                {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "edge-less FIN {current} violates isolated-FIN contract"
                            ),
                        ),
                    );
                }

                isolated_fins.insert(current);
                singular_fin_count += 1;
            }

            if fin.forward == start {
                break;
            }

            current =
                fin.forward;

            if local.len() > fins.len() {
                return Err(
                    CodecError::Malformed(
                        "FIN loop traversal exceeded global FIN count"
                            .to_string(),
                    ),
                );
            }
        }
    }

    if loop_fins != reachable_fins {
        return Err(
            CodecError::Malformed(
                format!(
                    "reachable FIN coverage mismatch loop={} reachable={}",
                    loop_fins.len(),
                    reachable_fins.len()
                ),
            ),
        );
    }

    let mut radial_fin_count =
        0usize;

    let mut fin_endpoint_sense_match_count =
        0usize;

    let mut edge_radial_ring_count =
        0usize;

    let mut edge_anchor_sense_count =
        0usize;

    let mut radial_fins =
        BTreeSet::<u32>::new();

    for edge_index in &reachable_edges {
        let edge =
            edges.get(edge_index).ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "reachable EDGE {edge_index} disappeared"
                    ),
                )
            })?;

        let indexed_fins =
            fins_by_edge
                .get(edge_index)
                .map(Vec::as_slice)
                .unwrap_or(&[]);

        if indexed_fins.len() < 2 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} has {} radial FINs; expected at least 2",
                        indexed_fins.len()
                    ),
                ),
            );
        }

        if edge.fin == 0 ||
            !indexed_fins.contains(&edge.fin)
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} anchor FIN {} is absent from its indexed radial population",
                        edge.fin
                    ),
                ),
            );
        }

        let anchor =
            fins.get(&edge.fin).ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} anchor FIN {} is missing",
                        edge.fin
                    ),
                )
            })?;

        if anchor.edge != *edge_index {
            return Err(
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} anchor FIN {} references EDGE {}",
                        edge.fin,
                        anchor.edge
                    ),
                ),
            );
        }

        if anchor.native_sense != b'+' {
            return Err(
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} anchor FIN {} is not positive",
                        edge.fin
                    ),
                ),
            );
        }

        if anchor.other == 0 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} anchor FIN {} has null radial successor",
                        edge.fin
                    ),
                ),
            );
        }

        let negative_anchor =
            fins.get(&anchor.other).ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} anchor successor FIN {} is missing",
                        anchor.other
                    ),
                )
            })?;

        if negative_anchor.edge != *edge_index {
            return Err(
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} anchor successor FIN {} references EDGE {}",
                        anchor.other,
                        negative_anchor.edge
                    ),
                ),
            );
        }

        if negative_anchor.native_sense != b'-' {
            return Err(
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} anchor successor FIN {} is not negative",
                        anchor.other
                    ),
                ),
            );
        }

        edge_anchor_sense_count += 1;

        let positive_vertex =
            anchor.vertex;

        let negative_vertex =
            negative_anchor.vertex;

        let expected =
            indexed_fins
                .iter()
                .copied()
                .collect::<BTreeSet<_>>();

        let mut radial =
            BTreeSet::<u32>::new();

        let mut current =
            edge.fin;

        loop {
            if !radial.insert(current) {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "EDGE {edge_index} radial ring repeats FIN {current}"
                        ),
                    ),
                );
            }

            let fin =
                fins.get(&current).ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "EDGE {edge_index} radial FIN {current} is missing"
                        ),
                    )
                })?;

            if fin.edge != *edge_index {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "radial FIN {current} left EDGE {edge_index}"
                        ),
                    ),
                );
            }

            match fin.native_sense {
                b'+' => {
                    fin_positive_sense_count += 1;

                    if fin.vertex != positive_vertex {
                        return Err(
                            CodecError::Malformed(
                                format!(
                                    "positive FIN {current} EDGE {edge_index} vertex {} != positive endpoint {positive_vertex}",
                                    fin.vertex
                                ),
                            ),
                        );
                    }
                }

                b'-' => {
                    fin_negative_sense_count += 1;

                    if fin.vertex != negative_vertex {
                        return Err(
                            CodecError::Malformed(
                                format!(
                                    "negative FIN {current} EDGE {edge_index} vertex {} != negative endpoint {negative_vertex}",
                                    fin.vertex
                                ),
                            ),
                        );
                    }
                }

                other => {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "EDGE FIN {current} has unsupported sense 0x{other:02x}"
                            ),
                        ),
                    );
                }
            }

            fin_endpoint_sense_match_count += 1;
            radial_fin_count += 1;

            if fin.other == 0 {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "EDGE {edge_index} radial FIN {current} has null other"
                        ),
                    ),
                );
            }

            if fin.other == edge.fin {
                break;
            }

            current =
                fin.other;

            if radial.len() > indexed_fins.len() {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "EDGE {edge_index} radial traversal exceeded indexed FIN population"
                        ),
                    ),
                );
            }
        }

        if radial != expected {
            return Err(
                CodecError::Malformed(
                    format!(
                        "EDGE {edge_index} radial ring/index mismatch radial={} indexed={}",
                        radial.len(),
                        expected.len()
                    ),
                ),
            );
        }

        for fin_index in radial {
            if !radial_fins.insert(fin_index) {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "FIN {fin_index} belongs to multiple EDGE radial rings"
                        ),
                    ),
                );
            }
        }

        edge_radial_ring_count += 1;
    }

    let mut all_qualified_fins =
        radial_fins.clone();

    for fin_index in &isolated_fins {
        if !all_qualified_fins.insert(*fin_index) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "FIN {fin_index} classified as both radial and isolated"
                    ),
                ),
            );
        }
    }

    if all_qualified_fins.len() != fins.len() {
        return Err(
            CodecError::Malformed(
                format!(
                    "global FIN coverage mismatch qualified={} total={}",
                    all_qualified_fins.len(),
                    fins.len()
                ),
            ),
        );
    }
    let (cadir, cadir_qualification) =
        if qualify_cadir {
            let (
                ir,
                qualification,
            ) =
                build_qualified_cadir(
            &root_entry_indices,
            &type_by_index,
            &bodies,
            &regions,
            &shells,
            &faces,
            &loops,
            &fins,
            &edges,
            &vertices,
            &points,
            &lines,
            &circles,
            &ellipses,
            &planes,
            &cylinders,
            &cones,
            &trimmed_curves,
            &sp_curves,
            &b_curves,
            &nurbs_curves,
            &bspline_vertices,
            &knot_multiplicities,
            &knot_sets,
            &curve_sense_by_index,
            &solid_shells,
            &reachable_faces,
            &reachable_loops,
            &reachable_fins,
            &reachable_edges,
            &reachable_vertices,
            &reachable_points,
                )?;

            (
                Some(ir),
                Some(qualification),
            )
        } else {
            (
                None,
                None,
            )
        };

    let topology_qualification =
        Sch13006TopologyQualification {
            face_tolerance_count,
            edge_tolerance_count,
            vertex_tolerance_count,

            max_face_tolerance_m,
            max_edge_tolerance_m,
            max_vertex_tolerance_m,

            face_positive_sense_count,
            face_negative_sense_count,

            fin_positive_sense_count,
            fin_negative_sense_count,

            surface_positive_sense_count,
            surface_negative_sense_count,

            solid_shell_face_use_count,
            solid_shell_opposite_void_count,

            loop_with_fin_count,
            fin_loop_owner_match_count,
            fin_forward_backward_reciprocity_count,
            radial_fin_count,
            fin_endpoint_sense_match_count,

            edge_radial_ring_count,
            edge_anchor_sense_count,

            singular_fin_count,
        };
    Ok(
        Sch13006Census {
            topology_qualification,
            cadir_qualification,
            cadir,

            catalog_type_count:
                QUALIFIED_SCHEMA_TYPE_COUNT,

            total_entities,

            terminator_index,

            root_variable_length,

            root_next_block,

            root_entry_indices,

            body_count:
                bodies.len(),

            solid_body_count,

            region_count:
                regions.len(),

            solid_region_count,

            void_region_count,

            shell_count:
                shells.len(),

            solid_region_shell_count:
                solid_shells.len(),

            face_count:
                faces.len(),

            loop_count:
                loops.len(),

            fin_count:
                fins.len(),

            edge_count:
                edges.len(),

            vertex_count:
                vertices.len(),

            point_count:
                count_type(
                    &type_counts,
                    POINT,
                ),

            reachable_face_count:
                reachable_faces.len(),

            reachable_loop_count:
                reachable_loops.len(),

            reachable_fin_count:
                reachable_fins.len(),

            reachable_edge_count:
                reachable_edges.len(),

            reachable_vertex_count:
                reachable_vertices.len(),

            reachable_point_count:
                reachable_points.len(),

            validated_topology_pointer_count:
                validated_pointer_count,

            geometry_bindings,

            line_count:
                count_type(
                    &type_counts,
                    30,
                ),

            circle_count:
                count_type(
                    &type_counts,
                    31,
                ),

            ellipse_count:
                count_type(
                    &type_counts,
                    32,
                ),

            plane_count:
                count_type(
                    &type_counts,
                    50,
                ),

            cylinder_count:
                count_type(
                    &type_counts,
                    51,
                ),

            cone_count:
                count_type(
                    &type_counts,
                    52,
                ),

            trimmed_curve_count:
                count_type(
                    &type_counts,
                    133,
                ),

            sp_curve_count:
                count_type(
                    &type_counts,
                    137,
                ),

            b_curve_count:
                count_type(
                    &type_counts,
                    134,
                ),

            nurbs_curve_count:
                count_type(
                    &type_counts,
                    136,
                ),
        },
    )
}

fn count_type(
    counts: &BTreeMap<u32, usize>,
    type_id: u32,
) -> usize {
    counts
        .get(
            &type_id
        )
        .copied()
        .unwrap_or(0)
}

fn validate_target(
    types: &BTreeMap<u32, u32>,
    source: u32,
    label: &str,
    pointer: u32,
    allowed: &[u32],
    validated: &mut usize,
) -> Result<(), CodecError> {
    if pointer == 0 {
        return Ok(());
    }

    *validated =
        validated
            .checked_add(1)
            .ok_or_else(|| {
                CodecError::Malformed(
                    "topology pointer validation counter overflow"
                        .to_string(),
                )
            })?;

    let actual =
        types.get(
            &pointer
        )
        .copied()
        .ok_or_else(|| {
            CodecError::Malformed(
                format!(
                    "{label} from entity {source} references missing entity {pointer}"
                ),
            )
        })?;

    if !allowed.contains(
        &actual
    ) {
        return Err(
            CodecError::Malformed(
                format!(
                    "{label} from entity {source} references entity {pointer} type {actual}, expected one of {allowed:?}"
                ),
            ),
        );
    }

    Ok(())
}

fn collect_shell_chain(
    start: u32,
    shells: &BTreeMap<u32, ShellNode>,
    output: &mut BTreeSet<u32>,
) -> Result<(), CodecError> {
    let mut local =
        BTreeSet::<u32>::new();

    let mut current =
        start;

    while current != 0 {
        if !local.insert(
            current
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "cycle detected in SHELL next chain at {current}"
                    ),
                ),
            );
        }

        output.insert(
            current
        );

        let shell =
            shells.get(
                &current
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "SHELL chain references missing shell {current}"
                    ),
                )
            })?;

        current =
            shell.next;
    }

    Ok(())
}

fn collect_face_chain(
    start: u32,
    front: bool,
    faces: &BTreeMap<u32, FaceNode>,
    output: &mut BTreeSet<u32>,
) -> Result<(), CodecError> {
    let mut local =
        BTreeSet::<u32>::new();

    let mut current =
        start;

    while current != 0 {
        if !local.insert(
            current
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "cycle detected in FACE shell chain at {current}"
                    ),
                ),
            );
        }

        output.insert(
            current
        );

        let face =
            faces.get(
                &current
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "FACE chain references missing face {current}"
                    ),
                )
            })?;

        current =
            if front {
                face.next_front
            }
            else {
                face.next
            };
    }

    Ok(())
}

fn collect_loop_chain(
    start: u32,
    loops: &BTreeMap<u32, LoopNode>,
    output: &mut BTreeSet<u32>,
) -> Result<(), CodecError> {
    let mut local =
        BTreeSet::<u32>::new();

    let mut current =
        start;

    while current != 0 {
        if !local.insert(
            current
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "cycle detected in LOOP next chain at {current}"
                    ),
                ),
            );
        }

        output.insert(
            current
        );

        let loop_node =
            loops.get(
                &current
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "LOOP chain references missing loop {current}"
                    ),
                )
            })?;

        current =
            loop_node.next;
    }

    Ok(())
}

fn collect_fin_ring(
    start: u32,
    fins: &BTreeMap<u32, FinNode>,
    output: &mut BTreeSet<u32>,
) -> Result<(), CodecError> {
    if start == 0 {
        return Ok(());
    }

    let mut local =
        BTreeSet::<u32>::new();

    let mut current =
        start;

    loop {
        if !local.insert(
            current
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "FIN ring repeats entity {current} before returning cleanly to start {start}"
                    ),
                ),
            );
        }

        if !output.insert(
            current
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "FIN {current} is reachable from more than one LOOP ring"
                    ),
                ),
            );
        }

        let fin =
            fins.get(
                &current
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "FIN ring references missing fin {current}"
                    ),
                )
            })?;

        let next =
            fin.forward;

        if next == 0 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "FIN ring beginning at {start} terminates at null from FIN {current}"
                    ),
                ),
            );
        }

        if next == start {
            break;
        }

        current =
            next;

        if local.len() >
            fins.len()
        {
            return Err(
                CodecError::Malformed(
                    "FIN ring exceeded global FIN count"
                        .to_string(),
                ),
            );
        }
    }

    Ok(())
}

fn collect_edge_chain(
    start: u32,
    edges: &BTreeMap<u32, EdgeNode>,
    output: &mut BTreeSet<u32>,
) -> Result<(), CodecError> {
    let mut local =
        BTreeSet::<u32>::new();

    let mut current =
        start;

    while current != 0 {
        if !local.insert(
            current
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "cycle detected in EDGE next chain at {current}"
                    ),
                ),
            );
        }

        output.insert(
            current
        );

        let edge =
            edges.get(
                &current
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "EDGE chain references missing edge {current}"
                    ),
                )
            })?;

        current =
            edge.next;
    }

    Ok(())
}

fn collect_vertex_chain(
    start: u32,
    vertices: &BTreeMap<u32, VertexNode>,
    output: &mut BTreeSet<u32>,
) -> Result<(), CodecError> {
    let mut local =
        BTreeSet::<u32>::new();

    let mut current =
        start;

    while current != 0 {
        if !local.insert(
            current
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "cycle detected in VERTEX next chain at {current}"
                    ),
                ),
            );
        }

        output.insert(
            current
        );

        let vertex =
            vertices.get(
                &current
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "VERTEX chain references missing vertex {current}"
                    ),
                )
            })?;

        current =
            vertex.next;
    }

    Ok(())
}

fn value_pointer(
    fields: &[FieldValue],
    index: usize,
    label: &str,
) -> Result<u32, CodecError> {
    match fields.get(
        index
    ) {
        Some(
            FieldValue::Pointer(
                value
            ),
        ) => Ok(
            *value
        ),

        _ => Err(
            CodecError::Malformed(
                format!(
                    "internal SCH_13006 schema mismatch while reading pointer field {label}"
                ),
            ),
        ),
    }
}

fn value_u32(
    fields: &[FieldValue],
    index: usize,
    label: &str,
) -> Result<u32, CodecError> {
    match fields.get(
        index
    ) {
        Some(
            FieldValue::Unsigned(
                value
            ),
        ) => u32::try_from(
            *value
        )
        .map_err(|_| {
            CodecError::Malformed(
                format!(
                    "{label} does not fit u32"
                ),
            )
        }),

        Some(
            FieldValue::Signed(
                value
            ),
        ) if *value >= 0 => {
            u32::try_from(
                *value
            )
            .map_err(|_| {
                CodecError::Malformed(
                    format!(
                        "{label} does not fit u32"
                    ),
                )
            })
        }

        _ => Err(
            CodecError::Malformed(
                format!(
                    "internal SCH_13006 schema mismatch while reading integer field {label}"
                ),
            ),
        ),
    }
}

fn value_character(
    fields: &[FieldValue],
    index: usize,
    label: &str,
) -> Result<u8, CodecError> {
    match fields.get(
        index
    ) {
        Some(
            FieldValue::Character(
                value
            ),
        ) => Ok(
            *value
        ),

        _ => Err(
            CodecError::Malformed(
                format!(
                    "internal SCH_13006 schema mismatch while reading character field {label}"
                ),
            ),
        ),
    }
}

fn value_i64(
    fields: &[FieldValue],
    index: usize,
    label: &str,
) -> Result<i64, CodecError> {
    match fields.get(index) {
        Some(FieldValue::Signed(value)) =>
            Ok(*value),

        Some(FieldValue::Unsigned(value)) =>
            i64::try_from(*value)
                .map_err(|_| {
                    CodecError::Malformed(
                        format!(
                            "{label} does not fit i64"
                        ),
                    )
                }),

        _ => Err(
            CodecError::Malformed(
                format!(
                    "internal SCH_13006 schema mismatch while reading signed field {label}"
                ),
            ),
        ),
    }
}

fn value_required_real(
    fields: &[FieldValue],
    index: usize,
    label: &str,
) -> Result<f64, CodecError> {
    match fields.get(index) {
        Some(FieldValue::Real(Some(value))) =>
            Ok(*value),

        Some(FieldValue::Real(None)) =>
            Err(
                CodecError::Malformed(
                    format!(
                        "required SCH_13006 real {label} is null"
                    ),
                ),
            ),

        _ => Err(
            CodecError::Malformed(
                format!(
                    "internal SCH_13006 schema mismatch while reading real field {label}"
                ),
            ),
        ),
    }
}

fn value_optional_real(
    fields: &[FieldValue],
    index: usize,
    label: &str,
) -> Result<Option<f64>, CodecError> {
    match fields.get(index) {
        Some(FieldValue::Real(value)) =>
            Ok(*value),

        _ => Err(
            CodecError::Malformed(
                format!(
                    "internal SCH_13006 schema mismatch while reading optional real field {label}"
                ),
            ),
        ),
    }
}

fn value_vector3(
    fields: &[FieldValue],
    index: usize,
    label: &str,
) -> Result<[f64; 3], CodecError> {
    match fields.get(index) {
        Some(FieldValue::Vector3(Some(value))) =>
            Ok(*value),

        Some(FieldValue::Vector3(None)) =>
            Err(
                CodecError::Malformed(
                    format!(
                        "required SCH_13006 vector {label} is null"
                    ),
                ),
            ),

        _ => Err(
            CodecError::Malformed(
                format!(
                    "internal SCH_13006 schema mismatch while reading vector field {label}"
                ),
            ),
        ),
    }
}

fn value_logical_bool(
    fields: &[FieldValue],
    index: usize,
    label: &str,
) -> Result<bool, CodecError> {
    match fields.get(index) {
        Some(FieldValue::Character(b'T' | b'1')) =>
            Ok(true),

        Some(FieldValue::Character(b'F' | b'0')) =>
            Ok(false),

        Some(FieldValue::Character(value)) =>
            Err(
                CodecError::Malformed(
                    format!(
                        "invalid logical {label}: 0x{value:02x}"
                    ),
                ),
            ),

        _ => Err(
            CodecError::Malformed(
                format!(
                    "internal SCH_13006 schema mismatch while reading logical field {label}"
                ),
            ),
        ),
    }
}

fn vector_dot(
    left: [f64; 3],
    right: [f64; 3],
) -> f64 {
    left[0] * right[0] +
    left[1] * right[1] +
    left[2] * right[2]
}

fn vector_norm(
    value: [f64; 3],
) -> f64 {
    vector_dot(
        value,
        value,
    )
    .sqrt()
}

fn validate_position(
    value: [f64; 3],
    label: &str,
) -> Result<(), CodecError> {
    if value
        .iter()
        .any(
            |component|
                !component.is_finite()
        )
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "{label} contains a non-finite coordinate"
                ),
            ),
        );
    }

    Ok(())
}

fn validate_unit_vector(
    value: [f64; 3],
    label: &str,
    maximum_error: &mut f64,
) -> Result<(), CodecError> {
    validate_position(
        value,
        label,
    )?;

    let error =
        (
            vector_norm(value) -
            1.0
        )
        .abs();

    *maximum_error =
        maximum_error.max(
            error
        );

    if error >
        GEOMETRY_VECTOR_TOLERANCE
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "{label} is not unit length; error={error}"
                ),
            ),
        );
    }

    Ok(())
}

fn validate_orthogonality(
    first: [f64; 3],
    second: [f64; 3],
    label: &str,
    maximum_error: &mut f64,
) -> Result<(), CodecError> {
    let error =
        vector_dot(
            first,
            second,
        )
        .abs();

    *maximum_error =
        maximum_error.max(
            error
        );

    if error >
        GEOMETRY_VECTOR_TOLERANCE
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "{label} is not orthogonal; dot-error={error}"
                ),
            ),
        );
    }

    Ok(())
}
fn vector_cross(
    left: [f64; 3],
    right: [f64; 3],
) -> [f64; 3] {
    [
        left[1] * right[2] -
            left[2] * right[1],

        left[2] * right[0] -
            left[0] * right[2],

        left[0] * right[1] -
            left[1] * right[0],
    ]
}

fn point_distance(
    left: [f64; 3],
    right: [f64; 3],
) -> f64 {
    (
        (
            left[0] -
            right[0]
        )
        .powi(2) +
        (
            left[1] -
            right[1]
        )
        .powi(2) +
        (
            left[2] -
            right[2]
        )
        .powi(2)
    )
    .sqrt()
}

fn expand_nurbs_knots(
    curve_index: u32,
    multiplicities: &[i16],
    distinct_knots: &[f64],
) -> Result<Vec<f64>, CodecError> {
    if multiplicities.len() !=
        distinct_knots.len()
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} distinct-knot/multiplicity length mismatch"
                ),
            ),
        );
    }

    let mut expanded =
        Vec::<f64>::new();

    for (
        knot,
        multiplicity,
    ) in
        distinct_knots
            .iter()
            .zip(
                multiplicities
            )
    {
        if !knot.is_finite() {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {curve_index} contains a non-finite knot"
                    ),
                ),
            );
        }

        if *multiplicity <= 0 {
            return Err(
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {curve_index} has non-positive knot multiplicity {multiplicity}"
                    ),
                ),
            );
        }

        let count =
            usize::try_from(
                *multiplicity
            )
            .map_err(|_| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {curve_index} knot multiplicity does not fit usize"
                    ),
                )
            })?;

        expanded.extend(
            std::iter::repeat(
                *knot
            )
            .take(
                count
            ),
        );
    }

    if expanded
        .windows(2)
        .any(
            |window|
                window[0] >
                window[1]
        )
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} expanded knot vector is not nondecreasing"
                ),
            ),
        );
    }

    Ok(
        expanded
    )
}

fn evaluate_nurbs_2d(
    curve_index: u32,
    nurbs: &NurbsCurveGeometry,
    vertices: &[f64],
    multiplicities: &[i16],
    distinct_knots: &[f64],
    parameter: f64,
) -> Result<[f64; 2], CodecError> {
    if !parameter.is_finite() {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} received a non-finite evaluation parameter"
                ),
            ),
        );
    }

    if nurbs.periodic {
        return Err(
            CodecError::NotImplemented(
                format!(
                    "periodic 2D SP_CURVE NURBS is outside the current physical qualification: {curve_index}"
                ),
            ),
        );
    }

    let degree =
        usize::try_from(
            nurbs.degree
        )
        .map_err(|_| {
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} degree does not fit usize"
                ),
            )
        })?;

    let n_vertices =
        usize::try_from(
            nurbs.n_vertices
        )
        .map_err(|_| {
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} n_vertices does not fit usize"
                ),
            )
        })?;

    let vertex_dim =
        usize::try_from(
            nurbs.vertex_dim
        )
        .map_err(|_| {
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} vertex_dim does not fit usize"
                ),
            )
        })?;

    if n_vertices <=
        degree
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} control-point population does not exceed degree"
                ),
            ),
        );
    }

    let expected_scalars =
        n_vertices
            .checked_mul(
                vertex_dim
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {curve_index} control-point scalar-count overflow"
                    ),
                )
            })?;

    if vertices.len() !=
        expected_scalars
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} control-point scalar mismatch expected={expected_scalars} actual={}",
                    vertices.len()
                ),
            ),
        );
    }

    let expanded_knots =
        expand_nurbs_knots(
            curve_index,
            multiplicities,
            distinct_knots,
        )?;

    let expected_knots =
        n_vertices
            .checked_add(
                degree
            )
            .and_then(
                |value|
                    value.checked_add(1)
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {curve_index} expanded knot-count overflow"
                    ),
                )
            })?;

    if expanded_knots.len() !=
        expected_knots
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} expanded knot count expected={expected_knots} actual={}",
                    expanded_knots.len()
                ),
            ),
        );
    }

    let mut control_points =
        Vec::<[f64; 3]>::with_capacity(
            n_vertices
        );

    for vertex_index in
        0..n_vertices
    {
        let base =
            vertex_index
                .checked_mul(
                    vertex_dim
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {curve_index} control-point index overflow"
                        ),
                    )
                })?;

        match (
            nurbs.rational,
            vertex_dim,
        ) {
            (
                false,
                2,
            ) => {
                let u =
                    vertices[base];

                let v =
                    vertices[
                        base + 1
                    ];

                if !u.is_finite() ||
                    !v.is_finite()
                {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "NURBS_CURVE {curve_index} contains non-finite 2D control coordinates"
                            ),
                        ),
                    );
                }

                control_points.push(
                    [
                        u,
                        v,
                        1.0,
                    ],
                );
            }

            (
                true,
                3,
            ) => {
                let uw =
                    vertices[base];

                let vw =
                    vertices[
                        base + 1
                    ];

                let weight =
                    vertices[
                        base + 2
                    ];

                if !uw.is_finite() ||
                    !vw.is_finite() ||
                    !weight.is_finite() ||
                    weight <= 0.0
                {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "NURBS_CURVE {curve_index} contains invalid homogeneous 2D control data"
                            ),
                        ),
                    );
                }

                control_points.push(
                    [
                        uw,
                        vw,
                        weight,
                    ],
                );
            }

            _ => {
                return Err(
                    CodecError::NotImplemented(
                        format!(
                            "SP_CURVE requires non-rational vertex_dim=2 or rational vertex_dim=3; NURBS_CURVE {curve_index} has rational={} vertex_dim={}",
                            nurbs.rational,
                            nurbs.vertex_dim
                        ),
                    ),
                );
            }
        }
    }

    let domain_min =
        *expanded_knots
            .get(
                degree
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {curve_index} has no lower parameter-domain knot"
                    ),
                )
            })?;

    let domain_max =
        *expanded_knots
            .get(
                n_vertices
            )
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "NURBS_CURVE {curve_index} has no upper parameter-domain knot"
                    ),
                )
            })?;

    if !domain_min.is_finite() ||
        !domain_max.is_finite() ||
        domain_min >= domain_max
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} has invalid parameter domain [{domain_min},{domain_max}]"
                ),
            ),
        );
    }

    let parameter_epsilon =
        64.0 *
        f64::EPSILON *
        (
            1.0 +
            domain_min.abs() +
            domain_max.abs() +
            parameter.abs()
        );

    if parameter <
        domain_min -
        parameter_epsilon ||
        parameter >
        domain_max +
        parameter_epsilon
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} parameter {parameter} lies outside [{domain_min},{domain_max}]"
                ),
            ),
        );
    }

    let parameter =
        parameter.clamp(
            domain_min,
            domain_max,
        );

    let project =
        |value: [f64; 3]|
         -> Result<[f64; 2], CodecError> {
            let weight =
                value[2];

            if !weight.is_finite() ||
                weight <= 0.0
            {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {curve_index} evaluated to an invalid homogeneous weight {weight}"
                        ),
                    ),
                );
            }

            let u =
                value[0] /
                weight;

            let v =
                value[1] /
                weight;

            if !u.is_finite() ||
                !v.is_finite()
            {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {curve_index} evaluated to a non-finite UV point"
                        ),
                    ),
                );
            }

            Ok(
                [
                    u,
                    v,
                ],
            )
        };

    if (
        parameter -
        domain_min
    )
    .abs() <=
        parameter_epsilon
    {
        return project(
            control_points[0]
        );
    }

    if (
        parameter -
        domain_max
    )
    .abs() <=
        parameter_epsilon
    {
        return project(
            control_points[
                n_vertices - 1
            ]
        );
    }

    let mut span =
        None;

    for candidate in
        degree..
        n_vertices
    {
        if expanded_knots[
            candidate
        ] <= parameter &&
            parameter <
            expanded_knots[
                candidate + 1
            ]
        {
            span =
                Some(
                    candidate
                );

            break;
        }
    }

    let span =
        span.ok_or_else(|| {
            CodecError::Malformed(
                format!(
                    "NURBS_CURVE {curve_index} could not locate De Boor span for parameter {parameter}"
                ),
            )
        })?;

    let mut work =
        Vec::<[f64; 3]>::with_capacity(
            degree + 1
        );

    for local in
        0..=degree
    {
        let source_index =
            span -
            degree +
            local;

        work.push(
            control_points[
                source_index
            ]
        );
    }

    for recursion in
        1..=degree
    {
        for local in
            (
                recursion..
                degree + 1
            )
            .rev()
        {
            let knot_index =
                span -
                degree +
                local;

            let left =
                expanded_knots[
                    knot_index
                ];

            let right =
                expanded_knots[
                    knot_index +
                    degree -
                    recursion +
                    1
                ];

            let denominator =
                right -
                left;

            let denominator_epsilon =
                64.0 *
                f64::EPSILON *
                (
                    1.0 +
                    left.abs() +
                    right.abs()
                );

            if denominator.abs() <=
                denominator_epsilon
            {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {curve_index} encountered a zero De Boor denominator"
                        ),
                    ),
                );
            }

            let alpha =
                (
                    parameter -
                    left
                ) /
                denominator;

            if !alpha.is_finite() {
                return Err(
                    CodecError::Malformed(
                        format!(
                            "NURBS_CURVE {curve_index} produced non-finite De Boor alpha"
                        ),
                    ),
                );
            }

            for coordinate in
                0..3
            {
                work[local][coordinate] =
                    (
                        1.0 -
                        alpha
                    ) *
                    work[
                        local - 1
                    ][coordinate] +
                    alpha *
                    work[local][coordinate];
            }
        }
    }

    project(
        work[
            degree
        ]
    )
}

fn embed_surface_point_m(
    surface_index: u32,
    uv: [f64; 2],
    type_by_index: &BTreeMap<u32, u32>,
    planes: &BTreeMap<u32, PlaneGeometry>,
    cylinders: &BTreeMap<u32, CylinderGeometry>,
    cones: &BTreeMap<u32, ConeGeometry>,
) -> Result<[f64; 3], CodecError> {
    let u =
        uv[0];

    let v =
        uv[1];

    if !u.is_finite() ||
        !v.is_finite()
    {
        return Err(
            CodecError::Malformed(
                format!(
                    "surface {surface_index} received non-finite UV coordinates"
                ),
            ),
        );
    }

    let surface_type =
        type_by_index
            .get(
                &surface_index
            )
            .copied()
            .ok_or_else(|| {
                CodecError::Malformed(
                    format!(
                        "SP_CURVE references missing surface {surface_index}"
                    ),
                )
            })?;

    let point =
        match surface_type {
            PLANE => {
                let plane =
                    planes.get(
                        &surface_index
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "missing PLANE {surface_index}"
                            ),
                        )
                    })?;

                let y_axis =
                    vector_cross(
                        plane.normal,
                        plane.x_axis,
                    );

                [
                    plane.pvec[0] +
                        u *
                        plane.x_axis[0] +
                        v *
                        y_axis[0],

                    plane.pvec[1] +
                        u *
                        plane.x_axis[1] +
                        v *
                        y_axis[1],

                    plane.pvec[2] +
                        u *
                        plane.x_axis[2] +
                        v *
                        y_axis[2],
                ]
            }

            CYLINDER => {
                let cylinder =
                    cylinders.get(
                        &surface_index
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "missing CYLINDER {surface_index}"
                            ),
                        )
                    })?;

                let y_axis =
                    vector_cross(
                        cylinder.axis,
                        cylinder.x_axis,
                    );

                let cosine =
                    u.cos();

                let sine =
                    u.sin();

                [
                    cylinder.pvec[0] +
                        cylinder.radius *
                        (
                            cylinder.x_axis[0] *
                            cosine +
                            y_axis[0] *
                            sine
                        ) +
                        v *
                        cylinder.axis[0],

                    cylinder.pvec[1] +
                        cylinder.radius *
                        (
                            cylinder.x_axis[1] *
                            cosine +
                            y_axis[1] *
                            sine
                        ) +
                        v *
                        cylinder.axis[1],

                    cylinder.pvec[2] +
                        cylinder.radius *
                        (
                            cylinder.x_axis[2] *
                            cosine +
                            y_axis[2] *
                            sine
                        ) +
                        v *
                        cylinder.axis[2],
                ]
            }

            CONE => {
                let cone =
                    cones.get(
                        &surface_index
                    )
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "missing CONE {surface_index}"
                            ),
                        )
                    })?;

                if cone
                    .cos_half_angle
                    .abs() <=
                    f64::EPSILON
                {
                    return Err(
                        CodecError::Malformed(
                            format!(
                                "CONE {surface_index} has unusable cos_half_angle={}",
                                cone.cos_half_angle
                            ),
                        ),
                    );
                }

                let y_axis =
                    vector_cross(
                        cone.axis,
                        cone.x_axis,
                    );

                let tangent =
                    cone.sin_half_angle /
                    cone.cos_half_angle;

                let radial =
                    cone.radius +
                    v *
                    tangent;

                let cosine =
                    u.cos();

                let sine =
                    u.sin();

                [
                    cone.pvec[0] +
                        v *
                        cone.axis[0] +
                        radial *
                        (
                            cone.x_axis[0] *
                            cosine +
                            y_axis[0] *
                            sine
                        ),

                    cone.pvec[1] +
                        v *
                        cone.axis[1] +
                        radial *
                        (
                            cone.x_axis[1] *
                            cosine +
                            y_axis[1] *
                            sine
                        ),

                    cone.pvec[2] +
                        v *
                        cone.axis[2] +
                        radial *
                        (
                            cone.x_axis[2] *
                            cosine +
                            y_axis[2] *
                            sine
                        ),
                ]
            }

            other => {
                return Err(
                    CodecError::NotImplemented(
                        format!(
                            "SP_CURVE support surface type {other} is outside the qualified analytic embedding set"
                        ),
                    ),
                );
            }
        };

    validate_position(
        point,
        &format!(
            "SP_CURVE embedded surface {surface_index}"
        ),
    )?;

    Ok(
        point
    )
}

fn schema_spec(
    type_id: u32,
) -> Option<SchemaSpec> {
    use FieldKind::{
        Character as C,
        Logical as L,
        Pointer as P,
        Real as F,
        Short as N,
        Signed as D,
        Unsigned as U,
        Vector3 as V,
    };

    let spec =
        match type_id {
            12 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, P,
                    F, F,
                    P, P, P,
                    U, P, U, U,
                    P, P, P, P, P, P, P,
                ],
                variable: None,
            },

            13 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, P, P, P,
                ],
                variable: None,
            },

            14 => SchemaSpec {
                fields: &[
                    D, P, F, P, P, P, P, P,
                    C,
                    P, P, P, P, P,
                ],
                variable: None,
            },

            15 => SchemaSpec {
                fields: &[
                    D, P, P, P, P,
                ],
                variable: None,
            },

            16 => SchemaSpec {
                fields: &[
                    D, P, F, P, P, P, P, P, P, P,
                ],
                variable: None,
            },

            17 => SchemaSpec {
                fields: &[
                    P, P, P, P, P, P, P, P, P, C,
                ],
                variable: None,
            },

            18 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, F, P,
                ],
                variable: None,
            },

            19 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, C,
                ],
                variable: None,
            },

            29 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, V,
                ],
                variable: None,
            },

            30 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, C, V, V,
                ],
                variable: None,
            },

            31 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, C,
                    V, V, V, F,
                ],
                variable: None,
            },

            32 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, C,
                    V, V, V, F, F,
                ],
                variable: None,
            },

            45 => SchemaSpec {
                fields: &[],
                variable: Some(
                    VariableKind::Real
                ),
            },

            50 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, C, V, V, V,
                ],
                variable: None,
            },

            51 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, C,
                    V, V, F, V,
                ],
                variable: None,
            },

            52 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, C,
                    V, V, F, F, F, V,
                ],
                variable: None,
            },

            70 => SchemaSpec {
                fields: &[
                    D, P, P, P,
                    D, D, D, D,
                    P, P,
                    D, L,
                ],
                variable: None,
            },

            74 => SchemaSpec {
                fields: &[
                    D, P,
                ],
                variable: Some(
                    VariableKind::Pointer
                ),
            },

            79 => SchemaSpec {
                fields: &[],
                variable: Some(
                    VariableKind::RawChar
                ),
            },

            80 => SchemaSpec {
                fields: &[
                    P, P, D,
                    U, U, U, U, U, U, U, U,
                    P,
                    L, L, L, L, L, L, L,
                    L, L, L, L, L, L, L,
                ],
                variable: Some(
                    VariableKind::Short
                ),
            },

            81 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, P,
                ],
                variable: Some(
                    VariableKind::Pointer
                ),
            },

            82 => SchemaSpec {
                fields: &[],
                variable: Some(
                    VariableKind::Signed
                ),
            },

            83 => SchemaSpec {
                fields: &[],
                variable: Some(
                    VariableKind::Real
                ),
            },

            127 => SchemaSpec {
                fields: &[],
                variable: Some(
                    VariableKind::Short
                ),
            },

            128 => SchemaSpec {
                fields: &[],
                variable: Some(
                    VariableKind::Real
                ),
            },

            133 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, C,
                    P, V, V, F, F,
                ],
                variable: None,
            },

            134 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, C, P, P,
                ],
                variable: None,
            },

            135 => SchemaSpec {
                fields: &[
                    U, P,
                ],
                variable: None,
            },

            136 => SchemaSpec {
                fields: &[
                    N, D, N, D, U,
                    L, L, L,
                    U,
                    P, P, P,
                ],
                variable: None,
            },

            137 => SchemaSpec {
                fields: &[
                    D, P, P, P, P, P, C,
                    P, P, P, F,
                ],
                variable: None,
            },

            141 => SchemaSpec {
                fields: &[
                    P, P, P, P,
                ],
                variable: None,
            },

            _ => return None,
        };

    Some(
        spec
    )
}

#[derive(Debug, Clone, Copy)]
struct Cursor<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl<'a> Cursor<'a> {
    fn at(
        bytes: &'a [u8],
        at: usize,
    ) -> Result<Self, CodecError> {
        if at > bytes.len() {
            return Err(
                CodecError::Malformed(
                    format!(
                        "SCH_13006 cursor offset {at} escapes logical stream length {}",
                        bytes.len()
                    ),
                ),
            );
        }

        Ok(
            Self {
                bytes,
                at,
            },
        )
    }

    fn skip_spaces(
        &mut self,
    ) {
        while self.bytes
            .get(
                self.at
            ) ==
            Some(&b' ')
        {
            self.at += 1;
        }
    }

    fn consume_optional_separator(
        &mut self,
    ) {
        if self.bytes
            .get(
                self.at
            ) ==
            Some(&b' ')
        {
            self.at += 1;
        }
    }

    fn read_u32(
        &mut self,
        label: &str,
    ) -> Result<u32, CodecError> {
        let value =
            self.read_unsigned(
                label
            )?;

        u32::try_from(
            value
        )
        .map_err(|_| {
            CodecError::Malformed(
                format!(
                    "{label}={value} does not fit u32"
                ),
            )
        })
    }

    fn read_unsigned(
        &mut self,
        label: &str,
    ) -> Result<u64, CodecError> {
        self.skip_spaces();

        let start =
            self.at;

        while self.bytes
            .get(
                self.at
            )
            .is_some_and(
                |byte|
                    byte.is_ascii_digit()
            )
        {
            self.at += 1;
        }

        if self.at == start {
            return Err(
                CodecError::Malformed(
                    format!(
                        "expected unsigned {label} at logical offset {start}"
                    ),
                ),
            );
        }

        let token =
            std::str::from_utf8(
                &self.bytes[
                    start..
                    self.at
                ],
            )
            .map_err(|_| {
                CodecError::Malformed(
                    format!(
                        "{label} is not valid ASCII"
                    ),
                )
            })?;

        let value =
            token.parse::<u64>()
                .map_err(|_| {
                    CodecError::Malformed(
                        format!(
                            "unable to parse unsigned {label}: {token}"
                        ),
                    )
                })?;

        self.consume_optional_separator();

        Ok(
            value
        )
    }

    fn read_signed(
        &mut self,
        label: &str,
    ) -> Result<i64, CodecError> {
        self.skip_spaces();

        let start =
            self.at;

        if self.bytes
            .get(
                self.at
            )
            .is_some_and(
                |byte|
                    matches!(
                        *byte,
                        b'+' | b'-'
                    )
            )
        {
            self.at += 1;
        }

        let digit_start =
            self.at;

        while self.bytes
            .get(
                self.at
            )
            .is_some_and(
                |byte|
                    byte.is_ascii_digit()
            )
        {
            self.at += 1;
        }

        if self.at == digit_start {
            return Err(
                CodecError::Malformed(
                    format!(
                        "expected signed {label} at logical offset {start}"
                    ),
                ),
            );
        }

        let token =
            std::str::from_utf8(
                &self.bytes[
                    start..
                    self.at
                ],
            )
            .map_err(|_| {
                CodecError::Malformed(
                    format!(
                        "{label} is not valid ASCII"
                    ),
                )
            })?;

        let value =
            token.parse::<i64>()
                .map_err(|_| {
                    CodecError::Malformed(
                        format!(
                            "unable to parse signed {label}: {token}"
                        ),
                    )
                })?;

        self.consume_optional_separator();

        Ok(
            value
        )
    }

    fn read_pointer(
        &mut self,
        label: &str,
    ) -> Result<u32, CodecError> {
        self.skip_spaces();

        if self.bytes
            .get(
                self.at
            ) ==
            Some(&b'?')
        {
            self.at += 1;
        }

        self.read_u32(
            label
        )
    }

    fn read_real(
        &mut self,
        label: &str,
    ) -> Result<Option<f64>, CodecError> {
        self.skip_spaces();

        if self.bytes
            .get(
                self.at
            ) ==
            Some(&b'?')
        {
            self.at += 1;
            self.consume_optional_separator();

            return Ok(
                None
            );
        }

        let start =
            self.at;

        while self.at <
            self.bytes.len() &&
            self.bytes[
                self.at
            ] != b' '
        {
            self.at += 1;
        }

        if self.at == start {
            return Err(
                CodecError::Malformed(
                    format!(
                        "expected real {label} at logical offset {start}"
                    ),
                ),
            );
        }

        let token =
            std::str::from_utf8(
                &self.bytes[
                    start..
                    self.at
                ],
            )
            .map_err(|_| {
                CodecError::Malformed(
                    format!(
                        "{label} is not valid ASCII"
                    ),
                )
            })?;

        let value =
            token.parse::<f64>()
                .map_err(|_| {
                    CodecError::Malformed(
                        format!(
                            "unable to parse real {label}: {token}"
                        ),
                    )
                })?;

        if !value.is_finite() {
            return Err(
                CodecError::Malformed(
                    format!(
                        "non-finite SCH_13006 real {label}: {token}"
                    ),
                ),
            );
        }

        self.consume_optional_separator();

        Ok(
            Some(
                value
            )
        )
    }

    fn read_character(
        &mut self,
        label: &str,
    ) -> Result<u8, CodecError> {
        self.skip_spaces();

        let value =
            *self.bytes
                .get(
                    self.at
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "missing character {label} at end of logical stream"
                        ),
                    )
                })?;

        if !value.is_ascii() {
            return Err(
                CodecError::Malformed(
                    format!(
                        "non-ASCII character in {label}"
                    ),
                ),
            );
        }

        self.at += 1;

        self.consume_optional_separator();

        Ok(
            value
        )
    }

    fn read_logical(
        &mut self,
        label: &str,
    ) -> Result<u8, CodecError> {
        let value =
            self.read_character(
                label
            )?;

        if !matches!(
            value,
            b'T' | b'F' | b'0' | b'1'
        ) {
            return Err(
                CodecError::Malformed(
                    format!(
                        "invalid logical {label}: byte 0x{value:02x}"
                    ),
                ),
            );
        }

        Ok(
            value
        )
    }

    fn read_vector3(
        &mut self,
        label: &str,
    ) -> Result<Option<[f64; 3]>, CodecError> {
        self.skip_spaces();

        if self.bytes
            .get(
                self.at
            ) ==
            Some(&b'?')
        {
            self.at += 1;
            self.consume_optional_separator();

            return Ok(None);
        }

        let mut values =
            [0.0_f64; 3];

        for component in
            0..3
        {
            values[component] =
                self
                    .read_real(
                        &format!(
                            "{label}[{component}]"
                        ),
                    )?
                    .ok_or_else(|| {
                        CodecError::Malformed(
                            format!(
                                "{label}[{component}] is unexpectedly null"
                            ),
                        )
                    })?;
        }

        Ok(
            Some(values)
        )
    }

    fn skip_raw_chars(
        &mut self,
        count: usize,
    ) -> Result<(), CodecError> {
        let end =
            self.at
                .checked_add(
                    count
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        "raw character extent overflow"
                            .to_string(),
                    )
                })?;

        let bytes =
            self.bytes
                .get(
                    self.at..
                    end
                )
                .ok_or_else(|| {
                    CodecError::Malformed(
                        format!(
                            "raw character field length {count} exceeds remaining SCH_13006 stream"
                        ),
                    )
                })?;

        if bytes
            .iter()
            .any(
                |byte|
                    !byte.is_ascii()
            )
        {
            return Err(
                CodecError::Malformed(
                    "raw character field contains non-ASCII data"
                        .to_string(),
                ),
            );
        }

        self.at =
            end;

        self.consume_optional_separator();

        Ok(())
    }

    fn read_field(
        &mut self,
        kind: FieldKind,
    ) -> Result<FieldValue, CodecError> {
        match kind {
            FieldKind::Signed |
            FieldKind::Short => {
                Ok(
                    FieldValue::Signed(
                        self.read_signed(
                            "schema integer"
                        )?
                    ),
                )
            }

            FieldKind::Unsigned => {
                Ok(
                    FieldValue::Unsigned(
                        self.read_unsigned(
                            "schema unsigned"
                        )?
                    ),
                )
            }

            FieldKind::Pointer => {
                Ok(
                    FieldValue::Pointer(
                        self.read_pointer(
                            "schema pointer"
                        )?
                    ),
                )
            }

            FieldKind::Real => {
                Ok(
                    FieldValue::Real(
                        self.read_real(
                            "schema real"
                        )?
                    ),
                )
            }

            FieldKind::Character => {
                Ok(
                    FieldValue::Character(
                        self.read_character(
                            "schema character"
                        )?
                    ),
                )
            }

            FieldKind::Logical => {
                Ok(
                    FieldValue::Character(
                        self.read_logical(
                            "schema logical"
                        )?
                    ),
                )
            }

            FieldKind::Vector3 => {
                Ok(
                    FieldValue::Vector3(
                        self.read_vector3(
                            "schema vector"
                        )?
                    ),
                )
            }
        }
    }

    fn require_end(
        &mut self,
    ) -> Result<(), CodecError> {
        self.skip_spaces();

        if self.at !=
            self.bytes.len()
        {
            return Err(
                CodecError::Malformed(
                    format!(
                        "SCH_13006 parser reached terminator with {} unconsumed logical bytes",
                        self.bytes.len() - self.at
                    ),
                ),
            );
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use cadmpeg_core::decode::{
        DecodeArena,
        DecodeContext,
        DecodePolicy,
    };

    #[test]
    fn parses_minimal_sch13006_topology_and_validates_links() {
        let logical =
            b"74 1 1 1 0 2 12 2 10 0 0 0 0 0 0 1e3 1e-8 0 0 0 1 0 1 1 3 0 0 0 4 0 0 13 3 1 0 2 0 0 0 0 4 0 19 4 2 0 2 0 0 3 S 1 0";

        let arena =
            DecodeArena::new();

        let policy =
            DecodePolicy::default();

        let (_, view) =
            DecodeContext::from_root_bytes(
                logical,
                &arena,
                &policy,
            )
            .expect(
                "bounded synthetic SCH_13006 stream",
            );

        let preamble =
            Sch13006Preamble {
                user_field_size: 0,
                first_node_type: 74,
                after_first_node_type_offset: 3,
            };

        let census =
            parse_census(
                view,
                &preamble,
            )
            .expect(
                "minimal SCH_13006 topology",
            );

        assert_eq!(
            census.total_entities,
            4
        );

        assert_eq!(
            census.root_entry_indices,
            vec![2]
        );

        assert_eq!(
            census.body_count,
            1
        );

        assert_eq!(
            census.solid_body_count,
            1
        );

        assert_eq!(
            census.region_count,
            1
        );

        assert_eq!(
            census.solid_region_count,
            1
        );

        assert_eq!(
            census.shell_count,
            1
        );

        assert_eq!(
            census.solid_region_shell_count,
            1
        );

        assert_eq!(
            census.terminator_index,
            0
        );
    }

    #[test]
    fn qualified_catalog_is_intentionally_fail_closed() {
        assert_eq!(
            QUALIFIED_SCHEMA_TYPE_COUNT,
            31
        );

        assert!(
            schema_spec(
                12
            )
            .is_some()
        );

        assert!(
            schema_spec(
                141
            )
            .is_some()
        );

        assert!(
            schema_spec(
                9999
            )
            .is_none()
        );
    }
}
