#![allow(dead_code)]
// src/packing.rs
use crate::geometry::{Polygon, Point, Rect, offset_polygon};
use crate::nfp::SheetContext;
use crate::types::{ConfigurationInput, DrawLayerData, DrawPathData, LayoutOutput, MaterialStateInput, PartInput, PlacementOutput, SheetOutput};

#[inline]
pub fn norm_angle(deg: f64) -> f64 {
    let mut a = deg % 360.0;
    if a < 0.0 { a += 360.0; }
    (a * 10.0).round() / 10.0
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SizeGroup {
    Group1Large = 1,       // longSide > 800 mm (Skeleton, allowed to open sheet)
    Group2MediumLarge = 2, // 400 < longSide <= 800 mm (Skeleton, allowed to open sheet)
    Group3MediumSmall = 3, // 200 <= longSide <= 400 mm (Filler, only fill existing gaps)
    Group4Small = 4,       // longSide < 200 mm (Micro Filler, only fill holes/cavities/gaps)
}

impl SizeGroup {
    pub fn from_dimensions(width: f64, height: f64) -> Self {
        let long_side = width.max(height);
        if long_side > 800.0 {
            SizeGroup::Group1Large
        } else if long_side > 400.0 {
            SizeGroup::Group2MediumLarge
        } else if long_side >= 200.0 {
            SizeGroup::Group3MediumSmall
        } else {
            SizeGroup::Group4Small
        }
    }

    #[inline]
    pub fn is_skeleton(&self) -> bool {
        matches!(self, SizeGroup::Group1Large | SizeGroup::Group2MediumLarge)
    }

    #[inline]
    pub fn is_filler(&self) -> bool {
        matches!(self, SizeGroup::Group3MediumSmall | SizeGroup::Group4Small)
    }
}

#[derive(Debug, Clone)]
pub struct RotatedVariant {
    pub rotation: f64,
    pub norm_poly: Polygon,
    pub norm_holes: Vec<Polygon>,
    pub norm_layers: Vec<DrawLayerData>,
    pub dilated_poly: Polygon,
    pub dilated_bbox: Rect,
    pub bbox: Rect,
    pub is_rect: bool,
}

#[derive(Debug, Clone)]
pub struct ProcessedPart {
    pub original: PartInput,
    pub area: f64,
    pub size_group: SizeGroup,
    pub is_two_sided: bool,
    pub is_anchor: bool,
    pub is_irregular: bool,
    pub is_rect: bool,
    pub small_part: bool,
    pub small_part_clearance: f64,
    pub variants: Vec<RotatedVariant>,
}

pub fn prepare_parts(
    parts: &[PartInput],
    config: &ConfigurationInput,
    spacing: f64,
) -> Vec<ProcessedPart> {
    let board_w = config.board_width.unwrap_or(2440.0);
    let board_h = config.board_height.unwrap_or(1220.0);
    let margin = config.edge_margin.unwrap_or(10.0);
    let max_bw = board_w - margin * 2.0;
    let max_bh = board_h - margin * 2.0;
    let global_rot_div = config.rotation_divisions.unwrap_or(1);
    let small_part_threshold = config.small_part_threshold.unwrap_or(0.0).max(0.0);
    let configured_small_part_clearance = config.small_part_clearance.unwrap_or(0.0).max(0.0);

    parts.iter().map(|p| {
        let poly = Polygon::from_raw(&p.contour);
        let orig_bbox = poly.bounding_box();
        let area = p.area.unwrap_or_else(|| poly.area());
        let source_width = p.width.filter(|v| *v > 1e-6).unwrap_or_else(|| orig_bbox.width());
        let source_height = p.height.filter(|v| *v > 1e-6).unwrap_or_else(|| orig_bbox.height());
        let small_part = small_part_threshold > 0.001
            && source_width > 0.001
            && source_height > 0.001
            && source_width.min(source_height) <= small_part_threshold + 0.001;
        let small_part_clearance = if small_part {
            spacing.max(configured_small_part_clearance)
        } else {
            0.0
        };
        let size_group = SizeGroup::from_dimensions(orig_bbox.width(), orig_bbox.height());
        let is_two_sided = false;
        let is_anchor = p.holes.as_ref().map_or(false, |h| !h.is_empty()) || !poly.is_rectangular();
        let is_irregular = !poly.is_rectangular();
        let is_rect = poly.is_rectangular() && p.holes.as_ref().map_or(true, |h| h.is_empty());

        let base_rot = norm_angle(p.base_rotation_degrees.or(p.rotation_degrees).unwrap_or(0.0));

        let mut allowed_rotations: Vec<f64> = if let Some(rots) = &p.rotations {
            let mut r: Vec<f64> = rots.iter().map(|&d| norm_angle(d)).collect();
            if r.is_empty() { r.push(base_rot); }
            r
        } else if p.grain_locked == Some(true) {
            vec![base_rot]
        } else if global_rot_div == 2 {
            vec![base_rot, norm_angle(base_rot + 180.0)]
        } else if global_rot_div >= 4 || p.free_rotation == Some(true) {
            vec![
                base_rot,
                norm_angle(base_rot + 90.0),
                norm_angle(base_rot + 180.0),
                norm_angle(base_rot + 270.0),
            ]
        } else {
            vec![
                base_rot,
                norm_angle(base_rot + 180.0),
            ]
        };

        if (global_rot_div >= 4 || p.free_rotation == Some(true)) && p.grain_locked != Some(true) {
            let fits_initially = allowed_rotations.iter().any(|&rot| {
                let rot_poly = poly.rotate_degrees(rot, Point::new(0.0, 0.0));
                let bbox = rot_poly.bounding_box();
                bbox.width() <= max_bw && bbox.height() <= max_bh
            });

            if !fits_initially && orig_bbox.width() <= max_bh && orig_bbox.height() <= max_bw {
                allowed_rotations.push(norm_angle(base_rot + 90.0));
                allowed_rotations.push(norm_angle(base_rot + 270.0));
            }
        }

        let variants: Vec<RotatedVariant> = allowed_rotations.iter().map(|&rot| {
            let rad = rot * std::f64::consts::PI / 180.0;
            let rotated = poly.rotate_degrees(rot, Point::new(0.0, 0.0));
            let (norm_poly, shift_x, shift_y) = rotated.normalize_to_origin();
            let bbox = norm_poly.bounding_box();
            let norm_holes: Vec<Polygon> = p.holes.as_ref().map_or(Vec::new(), |holes_vec| {
                holes_vec.iter().map(|raw_h| {
                    let h_poly = Polygon::from_raw(raw_h);
                    let h_rot = h_poly.rotate_degrees(rot, Point::new(0.0, 0.0));
                    h_rot.translate(shift_x, shift_y)
                }).collect()
            });
            let norm_layers: Vec<DrawLayerData> = p.draw_layers.as_ref().map_or(Vec::new(), |layers| {
                layers.iter().map(|layer| {
                    let norm_paths = layer.paths.as_ref().map_or(Vec::new(), |paths| {
                        paths.iter().map(|path| {
                            let rot_pts = path.points.iter().map(|pt_raw| {
                                let pt = Point::new(pt_raw[0], pt_raw[1]).rotate(rad, Point::new(0.0, 0.0));
                                [(pt.x + shift_x).round(), (pt.y + shift_y).round()]
                            }).collect();
                            DrawPathData {
                                points: rot_pts,
                                closed: path.closed,
                                surface_face: path.surface_face,
                                surface_face_id: path.surface_face_id.clone(),
                                surface_loop_type: path.surface_loop_type.clone(),
                                geometry_kind: path.geometry_kind.clone(),
                                _nesting_packed_local: Some(true),
                            }
                        }).collect()
                    });
                    DrawLayerData {
                        name: layer.name.clone(),
                        layer_name: layer.layer_name.clone(),
                        color: layer.color.clone(),
                        side: layer.side.clone(),
                        r#type: layer.r#type.clone(),
                        paths: Some(norm_paths),
                    }
                }).collect()
            });
            let is_rect = norm_poly.is_rectangular() && norm_holes.is_empty();
            let dilated_poly = offset_polygon(&norm_poly, spacing * 0.5);
            let dilated_bbox = dilated_poly.bounding_box();
            RotatedVariant {
                rotation: rot,
                norm_poly,
                norm_holes,
                norm_layers,
                dilated_poly,
                dilated_bbox,
                bbox,
                is_rect,
            }
        }).filter(|v| v.bbox.width() <= max_bw && v.bbox.height() <= max_bh).collect();

        let final_variants = if variants.is_empty() {
            let rot = allowed_rotations[0];
            let rad = rot * std::f64::consts::PI / 180.0;
            let rotated = poly.rotate_degrees(rot, Point::new(0.0, 0.0));
            let (norm_poly, shift_x, shift_y) = rotated.normalize_to_origin();
            let bbox = norm_poly.bounding_box();
            let norm_holes: Vec<Polygon> = p.holes.as_ref().map_or(Vec::new(), |holes_vec| {
                holes_vec.iter().map(|raw_h| {
                    let h_poly = Polygon::from_raw(raw_h);
                    let h_rot = h_poly.rotate_degrees(rot, Point::new(0.0, 0.0));
                    h_rot.translate(shift_x, shift_y)
                }).collect()
            });
            let norm_layers: Vec<DrawLayerData> = p.draw_layers.as_ref().map_or(Vec::new(), |layers| {
                layers.iter().map(|layer| {
                    let norm_paths = layer.paths.as_ref().map_or(Vec::new(), |paths| {
                        paths.iter().map(|path| {
                            let rot_pts = path.points.iter().map(|pt_raw| {
                                let pt = Point::new(pt_raw[0], pt_raw[1]).rotate(rad, Point::new(0.0, 0.0));
                                [(pt.x + shift_x).round(), (pt.y + shift_y).round()]
                            }).collect();
                            DrawPathData {
                                points: rot_pts,
                                closed: path.closed,
                                surface_face: path.surface_face,
                                surface_face_id: path.surface_face_id.clone(),
                                surface_loop_type: path.surface_loop_type.clone(),
                                geometry_kind: path.geometry_kind.clone(),
                                _nesting_packed_local: Some(true),
                            }
                        }).collect()
                    });
                    DrawLayerData {
                        name: layer.name.clone(),
                        layer_name: layer.layer_name.clone(),
                        color: layer.color.clone(),
                        side: layer.side.clone(),
                        r#type: layer.r#type.clone(),
                        paths: Some(norm_paths),
                    }
                }).collect()
            });
            let is_rect = norm_poly.is_rectangular() && norm_holes.is_empty();
            let dilated_poly = offset_polygon(&norm_poly, spacing * 0.5);
            let dilated_bbox = dilated_poly.bounding_box();
            vec![RotatedVariant {
                rotation: rot,
                norm_poly,
                norm_holes,
                norm_layers,
                dilated_poly,
                dilated_bbox,
                bbox,
                is_rect,
            }]
        } else {
            variants
        };

        ProcessedPart {
            original: p.clone(),
            area,
            size_group,
            is_two_sided,
            is_anchor,
            is_irregular,
            is_rect,
            small_part,
            small_part_clearance,
            variants: final_variants,
        }
    }).collect()
}

pub fn pack_material(
    material_state: &MaterialStateInput,
    strategy_id: usize,
) -> LayoutOutput {
    pack_material_streaming(material_state, strategy_id, None::<fn(&LayoutOutput)>)
}

/// Strict Implementation of MN Priority-Cavity Hybrid V35 (QUY_TRINH_NESTING_CHI_TIET.md)
pub fn pack_material_streaming<F>(
    material_state: &MaterialStateInput,
    strategy_id: usize,
    mut on_stream: Option<F>,
) -> LayoutOutput
where
    F: FnMut(&LayoutOutput),
{
    let config = material_state.configuration.clone().unwrap_or(ConfigurationInput {
        board_width: Some(2440.0),
        board_height: Some(1220.0),
        edge_margin: Some(10.0),
        cut_gap: Some(6.0),
        part_spacing: Some(6.0),
        rotation_divisions: Some(1),
        rotate_step: Some(90.0),
        compact_directions: Some(vec!["left".to_string(), "bottom".to_string()]),
        target_sheet_utilization: Some(90.0),
        sheet_in_sheet: Some(false),
        selective_repack: Some(false),
        small_part_threshold: Some(0.0),
        small_part_clearance: Some(0.0),
        small_part_edge_zone: Some(0.0),
        merge_cut_paths: Some(false),
    });

    let board_width = config.board_width.unwrap_or(2440.0);
    let board_height = config.board_height.unwrap_or(1220.0);
    let edge_margin = config.edge_margin.unwrap_or(10.0);
    let part_spacing = config.part_spacing.or(config.cut_gap).unwrap_or(6.0);
    let target_sheet_utilization = config.target_sheet_utilization.unwrap_or(90.0);
    let sheet_in_sheet = config.sheet_in_sheet.unwrap_or(false);
    let merge_cut_paths = config.merge_cut_paths.unwrap_or(false);
    let small_part_threshold = config.small_part_threshold.unwrap_or(0.0).max(0.0);
    let small_part_clearance = config.small_part_clearance.unwrap_or(0.0).max(0.0);
    let small_part_edge_zone = if merge_cut_paths {
        let configured_zone = config.small_part_edge_zone.unwrap_or(0.0);
        if configured_zone > 0.0 { configured_zone } else { 200.0 }
    } else {
        0.0
    };
    let compact_directions = config.compact_directions.clone().unwrap_or_else(|| vec!["left".to_string(), "bottom".to_string()]);

    let (paired_parts, comb_count) = if material_state.parts.iter().any(|p| p.manual_cluster_macro == Some(true)) {
        let count = material_state.parts.iter().filter(|p| p.manual_cluster_macro == Some(true)).count();
        (material_state.parts.clone(), count)
    } else {
        crate::macro_comb::pair_comb_parts(&material_state.parts, &config, part_spacing)
    };
    let processed = prepare_parts(&paired_parts, &config, part_spacing);

    // Section IV: Multi-tier Priority Queue Ordering
    // 1. Two-Sided First
    // 2. Priority Anchor (frames with holes or concave pockets)
    // 3. Comb Macro (interlocking pairs)
    // 4. Irregular Shapes
    // 5. Size Group (Group 1 > Group 2 > Group 3 > Group 4)
    // 6. Area descending (-area)
    let bl_weight = match (strategy_id * 7 + 1) % 8 {
        0 => 1.0,
        1 => 0.1,
        2 => 3.0,
        3 => 0.5,
        4 => 2.0,
        5 => 0.05,
        6 => 1.5,
        _ => 0.8,
    };

    let mut ordered_parts = processed;
    match strategy_id % 7 {
        0 => {
            ordered_parts.sort_by(|a, b| {
                let a_cluster = a.original.manual_cluster_macro == Some(true);
                let b_cluster = b.original.manual_cluster_macro == Some(true);
                if a_cluster != b_cluster { return b_cluster.cmp(&a_cluster); }
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.is_anchor != b.is_anchor { return b.is_anchor.cmp(&a.is_anchor); }
                if a.size_group != b.size_group { return a.size_group.cmp(&b.size_group); }
                b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
        1 => {
            ordered_parts.sort_by(|a, b| {
                let a_cluster = a.original.manual_cluster_macro == Some(true);
                let b_cluster = b.original.manual_cluster_macro == Some(true);
                if a_cluster != b_cluster { return b_cluster.cmp(&a_cluster); }
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.is_anchor != b.is_anchor { return b.is_anchor.cmp(&a.is_anchor); }
                let h_a = a.variants[0].bbox.height().max(a.variants[0].bbox.width());
                let h_b = b.variants[0].bbox.height().max(b.variants[0].bbox.width());
                h_b.partial_cmp(&h_a).unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal))
            });
        }
        2 => {
            ordered_parts.sort_by(|a, b| {
                let a_cluster = a.original.manual_cluster_macro == Some(true);
                let b_cluster = b.original.manual_cluster_macro == Some(true);
                if a_cluster != b_cluster { return b_cluster.cmp(&a_cluster); }
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.is_anchor != b.is_anchor { return b.is_anchor.cmp(&a.is_anchor); }
                let w_a = a.variants[0].bbox.width().max(a.variants[0].bbox.height());
                let w_b = b.variants[0].bbox.width().max(b.variants[0].bbox.height());
                w_b.partial_cmp(&w_a).unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal))
            });
        }
        3 => {
            ordered_parts.sort_by(|a, b| {
                let a_cluster = a.original.manual_cluster_macro == Some(true);
                let b_cluster = b.original.manual_cluster_macro == Some(true);
                if a_cluster != b_cluster { return b_cluster.cmp(&a_cluster); }
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.is_anchor != b.is_anchor { return b.is_anchor.cmp(&a.is_anchor); }
                let perim_a = a.variants[0].bbox.width() + a.variants[0].bbox.height();
                let perim_b = b.variants[0].bbox.width() + b.variants[0].bbox.height();
                perim_b.partial_cmp(&perim_a).unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal))
            });
        }
        4 => {
            ordered_parts.sort_by(|a, b| {
                let a_cluster = a.original.manual_cluster_macro == Some(true);
                let b_cluster = b.original.manual_cluster_macro == Some(true);
                if a_cluster != b_cluster { return b_cluster.cmp(&a_cluster); }
                let a_has_hole = a.original.holes.as_ref().map_or(false, |h| !h.is_empty());
                let b_has_hole = b.original.holes.as_ref().map_or(false, |h| !h.is_empty());
                if a_has_hole != b_has_hole { return b_has_hole.cmp(&a_has_hole); }
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.is_anchor != b.is_anchor { return b.is_anchor.cmp(&a.is_anchor); }
                b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
        5 => {
            ordered_parts.sort_by(|a, b| {
                let a_cluster = a.original.manual_cluster_macro == Some(true);
                let b_cluster = b.original.manual_cluster_macro == Some(true);
                if a_cluster != b_cluster { return b_cluster.cmp(&a_cluster); }
                if a.is_rect != b.is_rect { return b.is_rect.cmp(&a.is_rect); }
                b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
        _ => {
            ordered_parts.sort_by(|a, b| {
                let a_cluster = a.original.manual_cluster_macro == Some(true);
                let b_cluster = b.original.manual_cluster_macro == Some(true);
                if a_cluster != b_cluster { return b_cluster.cmp(&a_cluster); }
                if a.is_irregular != b.is_irregular { return b.is_irregular.cmp(&a.is_irregular); }
                b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
    }

    if strategy_id >= 7 {
        let seed = (strategy_id as u64).wrapping_mul(104729).wrapping_add(17);
        let n = ordered_parts.len();
        if n > 1 {
            for i in 0..n {
                let j = (seed.wrapping_add((i * 31) as u64) as usize) % n;
                if (ordered_parts[i].area - ordered_parts[j].area).abs() / ordered_parts[i].area.max(1.0) < 0.15 {
                    ordered_parts.swap(i, j);
                }
            }
        }
    }

    let mut sheets: Vec<SheetContext> = Vec::new();
    let mut sheet_placements: Vec<Vec<PlacementOutput>> = Vec::new();

    let total_parts = ordered_parts.len();
    let material_name = material_state.material.clone().unwrap_or_else(|| "default".to_string());
    let thickness = material_state.thickness.unwrap_or(18.0);
    let board_area = board_width * board_height;

    // Helper closure to try placing a part specifically into hole cavities on a SINGLE sheet
    let try_place_in_single_sheet_holes = |part: &ProcessedPart, sheet_idx: usize, sheets: &mut [SheetContext], sheet_placements: &mut [Vec<PlacementOutput>]| -> bool {
        let sheet = &mut sheets[sheet_idx];
        if !sheet.has_holes() {
            return false;
        }
        let mut best_variant = None;
        let mut best_pt = None;
        let mut best_score = f64::MAX;

        for variant in &part.variants {
            if let Some((px, py, score)) = sheet.find_best_hole_anchor(
                &variant.norm_poly,
                &variant.dilated_poly,
                &variant.dilated_bbox,
                variant.is_rect,
                part.small_part,
                part.small_part_clearance,
                bl_weight,
            ) {
                if score < best_score {
                    best_score = score;
                    best_pt = Some((px, py));
                    best_variant = Some(variant);
                }
            }
        }

        if let (Some((px, py)), Some(variant)) = (best_pt, best_variant) {
            sheet.add_placed(
                variant.norm_poly.clone(),
                variant.norm_holes.clone(),
                px,
                py,
                variant.rotation,
                part.small_part,
                part.small_part_clearance,
            );

            let placement = PlacementOutput {
                entity_id: part.original.entity_id.clone().unwrap_or_default(),
                id: part.original.id.clone().unwrap_or_default(),
                name: part.original.name.clone(),
                x: px,
                y: py,
                rotation_degrees: variant.rotation,
                base_rotation_degrees: part.original.base_rotation_degrees,
                width: variant.bbox.width(),
                height: variant.bbox.height(),
                packed_width: variant.bbox.width(),
                packed_height: variant.bbox.height(),
                area: part.area,
                contour: variant.norm_poly.to_raw(),
                holes: if variant.norm_holes.is_empty() { None } else { Some(variant.norm_holes.iter().map(|h| h.to_raw()).collect()) },
                draw_layers: if variant.norm_layers.is_empty() { None } else { Some(variant.norm_layers.clone()) },
                color: part.original.color.clone(),
                manual_cluster_macro: part.original.manual_cluster_macro,
                manual_cluster_children: part.original.manual_cluster_children.clone(),
                logical_part_count: part.original.logical_part_count,
                has_grain_label: part.original.has_grain_label,
                grain_arrow_degrees: part.original.grain_arrow_degrees,
                small_part: Some(part.small_part),
                small_part_clearance: Some(part.small_part_clearance),
                small_part_edge_protected: Some(sheet.small_part_edge_protected(
                    part.small_part,
                    px,
                    py,
                    variant.bbox.width(),
                    variant.bbox.height(),
                )),
            };

            sheet_placements[sheet_idx].push(placement);
            true
        } else {
            false
        }
    };

    // Helper closure to try placing a part onto a SINGLE sheet
    let try_place_on_single_sheet = |part: &ProcessedPart, sheet_idx: usize, sheets: &mut [SheetContext], sheet_placements: &mut [Vec<PlacementOutput>]| -> bool {
        let sheet = &mut sheets[sheet_idx];
        let mut best_variant = None;
        let mut best_pt = None;
        let mut best_score = f64::MAX;

        for variant in &part.variants {
            if let Some((px, py, score)) = sheet.find_best_anchor(
                &variant.norm_poly,
                &variant.dilated_poly,
                &variant.dilated_bbox,
                variant.is_rect,
                part.small_part,
                part.small_part_clearance,
                bl_weight,
            ) {
                if score < best_score {
                    best_score = score;
                    best_pt = Some((px, py));
                    best_variant = Some(variant);
                }
            }
        }

        if let (Some((px, py)), Some(variant)) = (best_pt, best_variant) {
            sheet.add_placed(
                variant.norm_poly.clone(),
                variant.norm_holes.clone(),
                px,
                py,
                variant.rotation,
                part.small_part,
                part.small_part_clearance,
            );

            let placement = PlacementOutput {
                entity_id: part.original.entity_id.clone().unwrap_or_default(),
                id: part.original.id.clone().unwrap_or_default(),
                name: part.original.name.clone(),
                x: px,
                y: py,
                rotation_degrees: variant.rotation,
                base_rotation_degrees: part.original.base_rotation_degrees,
                width: variant.bbox.width(),
                height: variant.bbox.height(),
                packed_width: variant.bbox.width(),
                packed_height: variant.bbox.height(),
                area: part.area,
                contour: variant.norm_poly.to_raw(),
                holes: if variant.norm_holes.is_empty() { None } else { Some(variant.norm_holes.iter().map(|h| h.to_raw()).collect()) },
                draw_layers: if variant.norm_layers.is_empty() { None } else { Some(variant.norm_layers.clone()) },
                color: part.original.color.clone(),
                manual_cluster_macro: part.original.manual_cluster_macro,
                manual_cluster_children: part.original.manual_cluster_children.clone(),
                logical_part_count: part.original.logical_part_count,
                has_grain_label: part.original.has_grain_label,
                grain_arrow_degrees: part.original.grain_arrow_degrees,
                small_part: Some(part.small_part),
                small_part_clearance: Some(part.small_part_clearance),
                small_part_edge_protected: Some(sheet.small_part_edge_protected(
                    part.small_part,
                    px,
                    py,
                    variant.bbox.width(),
                    variant.bbox.height(),
                )),
            };

            sheet_placements[sheet_idx].push(placement);
            true
        } else {
            false
        }
    };

    let mut placed_flags = vec![false; total_parts];

    // Sequential Full-Dense Sheet Packing:
    // "Quy tắc: Trước khi tạo thêm 1 phôi mới thì BẮT BUỘC phải lấp đầy phôi cũ"
    while placed_flags.iter().any(|&p| !p) {
        let current_sheet_idx = sheets.len();
        let new_sheet = SheetContext::new(
            board_width,
            board_height,
            edge_margin,
            part_spacing,
            small_part_edge_zone,
            merge_cut_paths,
            sheet_in_sheet,
            compact_directions.clone(),
        );
        sheets.push(new_sheet);
        sheet_placements.push(Vec::new());

        let mut placed_any_on_this_sheet = false;

        loop {
            let mut placed_in_this_pass = false;

            // Bước A: Ưu tiên nhét chi tiết hình vuông / chữ nhật vào các hốc rỗng trên tấm này
            for i in 0..total_parts {
                if placed_flags[i] { continue; }
                let part = &ordered_parts[i];
                if !part.is_rect { continue; }

                if try_place_in_single_sheet_holes(part, current_sheet_idx, &mut sheets, &mut sheet_placements) {
                    placed_flags[i] = true;
                    placed_in_this_pass = true;
                    placed_any_on_this_sheet = true;
                }
            }

            // Bước B: Xếp các chi tiết theo thứ tự ưu tiên (lớn trước nhỏ sau, khung xương, răng lược)
            for i in 0..total_parts {
                if placed_flags[i] { continue; }
                let part = &ordered_parts[i];

                if try_place_on_single_sheet(part, current_sheet_idx, &mut sheets, &mut sheet_placements) {
                    placed_flags[i] = true;
                    placed_in_this_pass = true;
                    placed_any_on_this_sheet = true;
                }
            }

            // Nếu quét qua toàn bộ các chi tiết còn lại mà không thể nhét thêm bất kỳ chi tiết nào,
            // nghĩa là tấm hiện tại ĐÃ ĐẦY 100%. Dừng lặp và chỉ khi đó mới chuyển sang mở tấm tiếp theo!
            if !placed_in_this_pass {
                break;
            }
        }

        if !placed_any_on_this_sheet {
            // Không thể xếp thêm chi tiết nào ngay cả trên sheet mới tinh (chi tiết quá khổ)
            if sheet_placements[current_sheet_idx].is_empty() {
                sheets.pop();
                sheet_placements.pop();
            }
            break;
        }

        if let Some(ref mut stream_cb) = on_stream {
            let partial_sheets: Vec<SheetOutput> = sheet_placements.iter().enumerate().map(|(idx, placements)| {
                let used_area: f64 = placements.iter().map(|p| p.area).sum();
                let util = if board_area > 0.0 { (used_area / board_area) * 100.0 } else { 0.0 };
                SheetOutput {
                    index: idx + 1,
                    utilization: (util * 10.0).round() / 10.0,
                    used_percent: (util * 10.0).round() / 10.0,
                    placements: placements.clone(),
                }
            }).collect();

            let snap_layout = LayoutOutput {
                material: material_name.clone(),
                thickness,
                board_width,
                board_height,
                edge_margin,
                part_spacing,
                cut_gap: Some(part_spacing),
                small_part_threshold,
                small_part_clearance,
                small_part_edge_zone,
                merge_cut_paths,
                sheets: partial_sheets,
                compact_directions: compact_directions.clone(),
                target_sheet_utilization,
                waste_area: 0.0,
                compact_area: 0.0,
                last_sheet_compact_area: 0.0,
                front_load_score: 0.0,
                alignment_score: 0.0,
                compact_direction_score: 0.0,
                target_deficit_score: 0.0,
                comb_pair_placements: comb_count,
                staircase_pattern_placements: 0,
                priority_zone_fill_placements: 0,
                pocket_evacuated_sheets: 0,
                priority_zone_fill_area: 0.0,
                safety_certified: true,
                safety_version: 1,
            };
            stream_cb(&snap_layout);
        }
    }

    let mut sheet_outputs = Vec::new();

    for (idx, placements) in sheet_placements.into_iter().enumerate() {
        let used_area: f64 = placements.iter().map(|p| p.area).sum();
        let util = if board_area > 0.0 { (used_area / board_area) * 100.0 } else { 0.0 };

        sheet_outputs.push(SheetOutput {
            index: idx + 1,
            utilization: (util * 10.0).round() / 10.0,
            used_percent: (util * 10.0).round() / 10.0,
            placements,
        });
    }

    let mut layout = LayoutOutput {
        material: material_name,
        thickness,
        board_width,
        board_height,
        edge_margin,
        part_spacing,
        cut_gap: Some(part_spacing),
        small_part_threshold,
        small_part_clearance,
        small_part_edge_zone,
        merge_cut_paths,
        sheets: sheet_outputs,
        compact_directions,
        target_sheet_utilization,
        waste_area: 0.0,
        compact_area: 0.0,
        last_sheet_compact_area: 0.0,
        front_load_score: 0.0,
        alignment_score: 0.0,
        compact_direction_score: 0.0,
        target_deficit_score: 0.0,
        comb_pair_placements: comb_count,
        staircase_pattern_placements: 0,
        priority_zone_fill_placements: 0,
        pocket_evacuated_sheets: 0,
        priority_zone_fill_area: 0.0,
        safety_certified: true,
        safety_version: 1,
    };

    let score = crate::scoring::LayoutScore::compute(&layout);
    layout.waste_area = score.waste_area;
    layout.compact_area = score.compact_area;
    layout.last_sheet_compact_area = score.last_sheet_compact_area;
    layout.front_load_score = score.front_load_score;
    layout.target_deficit_score = score.target_deficit;

    layout
}
