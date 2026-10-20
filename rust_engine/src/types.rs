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
    pub optimization_seed: Option<u64>,
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
    pub optimization_seed: Option<u64>,
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
    pub optimization_seed: Option<u64>,
    pub small_part_threshold: Option<f64>,
    pub small_part_clearance: Option<f64>,
    pub small_part_edge_zone: Option<f64>,
    pub merge_cut_paths: Option<bool>,
}

#[derive(Debug, Clone, Serialize)]
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

static ROOM_REGISTRY: std::sync::OnceLock<std::sync::RwLock<std::collections::BTreeMap<String, String>>> = std::sync::OnceLock::new();

fn room_registry() -> &'static std::sync::RwLock<std::collections::BTreeMap<String, String>> {
    ROOM_REGISTRY.get_or_init(|| std::sync::RwLock::new(std::collections::BTreeMap::new()))
}

pub fn clear_room_registry() {
    if let Ok(mut guard) = room_registry().write() {
        guard.clear();
    }
}

pub fn normalize_room_name(raw: Option<&str>) -> Option<String> {
    let trimmed = raw?.trim();
    if trimmed.is_empty() || trimmed == "---" {
        None
    } else {
        Some(trimmed.to_lowercase())
    }
}

pub fn register_part_room(id: Option<&str>, entity_id: Option<&str>, room: Option<&str>) {
    let Some(norm_room) = normalize_room_name(room) else { return; };
    if let Ok(mut guard) = room_registry().write() {
        if let Some(eid) = entity_id.map(str::trim).filter(|s| !s.is_empty()) {
            guard.insert(format!("e:{}", eid), norm_room.clone());
        }
        if let Some(pid) = id.map(str::trim).filter(|s| !s.is_empty()) {
            guard.insert(format!("i:{}", pid), norm_room);
        }
    }
}

pub fn lookup_room_by_ids(id: Option<&str>, entity_id: Option<&str>) -> Option<String> {
    let Ok(guard) = room_registry().read() else { return None; };
    if let Some(eid) = entity_id.map(str::trim).filter(|s| !s.is_empty()) {
        if let Some(r) = guard.get(&format!("e:{}", eid)) {
            return Some(r.clone());
        }
    }
    if let Some(pid) = id.map(str::trim).filter(|s| !s.is_empty()) {
        if let Some(r) = guard.get(&format!("i:{}", pid)) {
            return Some(r.clone());
        }
    }
    None
}

pub fn get_part_room_key(part: &PartInput) -> String {
    if let Some(r) = lookup_room_by_ids(part.id.as_deref(), part.entity_id.as_deref()) {
        return r;
    }
    if let Some(children) = &part.manual_cluster_children {
        let mut common: Option<String> = None;
        for c in children {
            if let Some(cr) = lookup_room_by_ids(c.id.as_deref(), c.entity_id.as_deref()) {
                match &common {
                    None => common = Some(cr),
                    Some(prev) if prev == &cr => {}
                    Some(_) => return String::new(),
                }
            }
        }
        if let Some(r) = common {
            return r;
        }
    }
    String::new()
}

pub fn get_placement_room_key(placement: &PlacementOutput) -> String {
    if let Some(r) = lookup_room_by_ids(Some(placement.id.as_str()), Some(placement.entity_id.as_str())) {
        return r;
    }
    if let Some(children) = &placement.manual_cluster_children {
        let mut common: Option<String> = None;
        for c in children {
            if let Some(cr) = lookup_room_by_ids(c.id.as_deref(), c.entity_id.as_deref()) {
                match &common {
                    None => common = Some(cr),
                    Some(prev) if prev == &cr => {}
                    Some(_) => return String::new(),
                }
            }
        }
        if let Some(r) = common {
            return r;
        }
    }
    String::new()
}

impl<'de> Deserialize<'de> for ClusterChild {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct ClusterChildWire {
            entity_id: Option<String>,
            id: Option<String>,
            name: Option<String>,
            offset_x: Option<f64>,
            offset_y: Option<f64>,
            rotation_degrees: Option<f64>,
            width: Option<f64>,
            height: Option<f64>,
            area: Option<f64>,
            contour: Option<Vec<[f64; 2]>>,
            holes: Option<Vec<Vec<[f64; 2]>>>,
            render_contour: Option<Vec<[f64; 2]>>,
            render_holes: Option<Vec<Vec<[f64; 2]>>>,
            collision_contour: Option<Vec<[f64; 2]>>,
            collision_holes: Option<Vec<Vec<[f64; 2]>>>,
            color: Option<String>,
            #[serde(default)]
            draw_layers: Option<Vec<DrawLayerData>>,
            #[serde(default)]
            has_grain_label: Option<bool>,
            #[serde(default)]
            grain_arrow_degrees: Option<f64>,
            #[serde(default)]
            base_rotation_degrees: Option<f64>,
            #[serde(default)]
            two_sided: Option<bool>,
            #[serde(default)]
            manual_cluster_child: Option<bool>,
            #[serde(default)]
            room_name: Option<String>,
        }
        let w = ClusterChildWire::deserialize(deserializer)?;
        register_part_room(w.id.as_deref(), w.entity_id.as_deref(), w.room_name.as_deref());
        Ok(ClusterChild {
            entity_id: w.entity_id,
            id: w.id,
            name: w.name,
            offset_x: w.offset_x,
            offset_y: w.offset_y,
            rotation_degrees: w.rotation_degrees,
            width: w.width,
            height: w.height,
            area: w.area,
            contour: w.contour,
            holes: w.holes,
            render_contour: w.render_contour,
            render_holes: w.render_holes,
            collision_contour: w.collision_contour,
            collision_holes: w.collision_holes,
            color: w.color,
            draw_layers: w.draw_layers,
            has_grain_label: w.has_grain_label,
            grain_arrow_degrees: w.grain_arrow_degrees,
            base_rotation_degrees: w.base_rotation_degrees,
            two_sided: w.two_sided,
            manual_cluster_child: w.manual_cluster_child,
        })
    }
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

#[derive(Debug, Clone, Serialize)]
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub has_circular_arc: Option<bool>,
}

impl<'de> Deserialize<'de> for PartInput {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct PartInputWire {
            id: Option<String>,
            entity_id: Option<String>,
            name: Option<String>,
            width: Option<f64>,
            height: Option<f64>,
            area: Option<f64>,
            contour: Vec<[f64; 2]>,
            holes: Option<Vec<Vec<[f64; 2]>>>,
            render_contour: Option<Vec<[f64; 2]>>,
            render_holes: Option<Vec<Vec<[f64; 2]>>>,
            collision_contour: Option<Vec<[f64; 2]>>,
            collision_holes: Option<Vec<Vec<[f64; 2]>>>,
            draw_layers: Option<Vec<DrawLayerData>>,
            rotations: Option<Vec<f64>>,
            base_rotation_degrees: Option<f64>,
            rotation_degrees: Option<f64>,
            grain_locked: Option<bool>,
            has_grain_label: Option<bool>,
            grain_arrow_degrees: Option<f64>,
            free_rotation: Option<bool>,
            rotation_divisions: Option<usize>,
            color: Option<String>,
            logical_part_count: Option<usize>,
            manual_cluster_macro: Option<bool>,
            manual_cluster_children: Option<Vec<ClusterChild>>,
            #[serde(default)]
            two_sided: Option<bool>,
            #[serde(default)]
            has_circular_arc: Option<bool>,
            #[serde(default)]
            room_name: Option<String>,
        }
        let w = PartInputWire::deserialize(deserializer)?;
        register_part_room(w.id.as_deref(), w.entity_id.as_deref(), w.room_name.as_deref());
        Ok(PartInput {
            id: w.id,
            entity_id: w.entity_id,
            name: w.name,
            width: w.width,
            height: w.height,
            area: w.area,
            contour: w.contour,
            holes: w.holes,
            render_contour: w.render_contour,
            render_holes: w.render_holes,
            collision_contour: w.collision_contour,
            collision_holes: w.collision_holes,
            draw_layers: w.draw_layers,
            rotations: w.rotations,
            base_rotation_degrees: w.base_rotation_degrees,
            rotation_degrees: w.rotation_degrees,
            grain_locked: w.grain_locked,
            has_grain_label: w.has_grain_label,
            grain_arrow_degrees: w.grain_arrow_degrees,
            free_rotation: w.free_rotation,
            rotation_divisions: w.rotation_divisions,
            color: w.color,
            logical_part_count: w.logical_part_count,
            manual_cluster_macro: w.manual_cluster_macro,
            manual_cluster_children: w.manual_cluster_children,
            two_sided: w.two_sided,
            has_circular_arc: w.has_circular_arc,
        })
    }
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
    pub free_rotation: Option<bool>,
    pub small_part: Option<bool>,
    pub small_part_clearance: Option<f64>,
    pub small_part_edge_protected: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub two_sided: Option<bool>,
}
