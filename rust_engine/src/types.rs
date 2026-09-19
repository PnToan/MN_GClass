#![allow(dead_code)]
// src/types.rs
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Deserialize)]
pub struct JobInput {
    pub protocol_version: Option<u32>,
    pub plugin: Option<String>,
    pub plugin_version: Option<String>,
    pub source: Option<String>,
    pub max_workers: Option<usize>,
    pub optimization_attempts: Option<usize>,
    pub materials: Vec<MaterialStateInput>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct MaterialStateInput {
    pub key: Option<String>,
    pub material: Option<String>,
    pub thickness: Option<f64>,
    pub group_index: Option<usize>,
    pub configuration: Option<ConfigurationInput>,
    pub original_part_count: Option<usize>,
    pub parts: Vec<PartInput>,
    pub learning_key: Option<String>,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct ConfigurationInput {
    pub board_width: Option<f64>,
    pub board_height: Option<f64>,
    pub edge_margin: Option<f64>,
    pub cut_gap: Option<f64>,
    pub part_spacing: Option<f64>,
    pub rotation_divisions: Option<usize>,
    pub rotate_step: Option<f64>,
    pub compact_directions: Option<Vec<String>>,
    pub target_sheet_utilization: Option<f64>,
    pub sheet_in_sheet: Option<bool>,
    pub selective_repack: Option<bool>,
    pub small_part_threshold: Option<f64>,
    pub small_part_clearance: Option<f64>,
    pub small_part_edge_zone: Option<f64>,
    pub merge_cut_paths: Option<bool>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ClusterChild {
    pub entity_id: Option<String>,
    pub id: Option<String>,
    pub name: Option<String>,
    pub offset_x: Option<f64>,
    pub offset_y: Option<f64>,
    pub rotation_degrees: Option<f64>,
    pub width: Option<f64>,
    pub height: Option<f64>,
    pub area: Option<f64>,
    pub contour: Option<Vec<[f64; 2]>>,
    pub holes: Option<Vec<Vec<[f64; 2]>>>,
    pub render_contour: Option<Vec<[f64; 2]>>,
    pub render_holes: Option<Vec<Vec<[f64; 2]>>>,
    pub collision_contour: Option<Vec<[f64; 2]>>,
    pub collision_holes: Option<Vec<Vec<[f64; 2]>>>,
    pub color: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub draw_layers: Option<Vec<DrawLayerData>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub has_grain_label: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub grain_arrow_degrees: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub base_rotation_degrees: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub two_sided: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub manual_cluster_child: Option<bool>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct DrawPathData {
    pub points: Vec<[f64; 2]>,
    pub closed: Option<bool>,
    pub surface_face: Option<bool>,
    pub surface_face_id: Option<serde_json::Value>,
    pub surface_loop_type: Option<String>,
    pub geometry_kind: Option<String>,
    pub _nesting_packed_local: Option<bool>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct DrawLayerData {
    pub name: Option<String>,
    pub layer_name: Option<String>,
    pub color: Option<String>,
    pub side: Option<String>,
    pub r#type: Option<String>,
    pub paths: Option<Vec<DrawPathData>>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct PartInput {
    pub id: Option<String>,
    pub entity_id: Option<String>,
    pub name: Option<String>,
    pub width: Option<f64>,
    pub height: Option<f64>,
    pub area: Option<f64>,
    pub contour: Vec<[f64; 2]>,
    pub holes: Option<Vec<Vec<[f64; 2]>>>,
    pub render_contour: Option<Vec<[f64; 2]>>,
    pub render_holes: Option<Vec<Vec<[f64; 2]>>>,
    pub collision_contour: Option<Vec<[f64; 2]>>,
    pub collision_holes: Option<Vec<Vec<[f64; 2]>>>,
    pub draw_layers: Option<Vec<DrawLayerData>>,
    pub rotations: Option<Vec<f64>>,
    pub base_rotation_degrees: Option<f64>,
    pub rotation_degrees: Option<f64>,
    pub grain_locked: Option<bool>,
    pub has_grain_label: Option<bool>,
    pub grain_arrow_degrees: Option<f64>,
    pub free_rotation: Option<bool>,
    pub rotation_divisions: Option<usize>,
    pub color: Option<String>,
    pub logical_part_count: Option<usize>,
    pub manual_cluster_macro: Option<bool>,
    pub manual_cluster_children: Option<Vec<ClusterChild>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub two_sided: Option<bool>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ProgressOutput {
    pub running: bool,
    pub overall_percent: f64,
    pub optimization_iteration: usize,
    pub optimization_total: usize,
    pub message: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct PreviewOutput {
    pub ok: bool,
    pub partial: bool,
    pub layouts: Vec<LayoutOutput>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ResultOutput {
    pub ok: bool,
    pub layouts: Vec<LayoutOutput>,
    pub population_total: usize,
    pub message: String,
    pub cancelled: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayoutOutput {
    pub material: String,
    pub thickness: f64,
    pub board_width: f64,
    pub board_height: f64,
    pub edge_margin: f64,
    pub part_spacing: f64,
    #[serde(default)]
    pub cut_gap: Option<f64>,
    #[serde(default)]
    pub small_part_threshold: f64,
    #[serde(default)]
    pub small_part_clearance: f64,
    #[serde(default)]
    pub small_part_edge_zone: f64,
    #[serde(default)]
    pub merge_cut_paths: bool,
    pub sheets: Vec<SheetOutput>,
    pub compact_directions: Vec<String>,
    pub target_sheet_utilization: f64,
    pub waste_area: f64,
    pub compact_area: f64,
    pub last_sheet_compact_area: f64,
    pub front_load_score: f64,
    pub alignment_score: f64,
    pub compact_direction_score: f64,
    pub target_deficit_score: f64,
    pub comb_pair_placements: usize,
    pub staircase_pattern_placements: usize,
    pub priority_zone_fill_placements: usize,
    pub pocket_evacuated_sheets: usize,
    pub priority_zone_fill_area: f64,
    pub safety_certified: bool,
    pub safety_version: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SheetOutput {
    pub index: usize,
    pub utilization: f64,
    pub used_percent: f64,
    pub placements: Vec<PlacementOutput>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlacementOutput {
    pub entity_id: String,
    pub id: String,
    pub name: Option<String>,
    pub x: f64,
    pub y: f64,
    pub rotation_degrees: f64,
    pub base_rotation_degrees: Option<f64>,
    pub width: f64,
    pub height: f64,
    pub packed_width: f64,
    pub packed_height: f64,
    pub area: f64,
    pub contour: Vec<[f64; 2]>,
    pub holes: Option<Vec<Vec<[f64; 2]>>>,
    pub draw_layers: Option<Vec<DrawLayerData>>,
    pub color: Option<String>,
    pub manual_cluster_macro: Option<bool>,
    pub manual_cluster_children: Option<Vec<ClusterChild>>,
    pub logical_part_count: Option<usize>,
    pub has_grain_label: Option<bool>,
    pub grain_arrow_degrees: Option<f64>,
    pub grain_locked: Option<bool>,
    pub small_part: Option<bool>,
    pub small_part_clearance: Option<f64>,
    pub small_part_edge_protected: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub two_sided: Option<bool>,
}
