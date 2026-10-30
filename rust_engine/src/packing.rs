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
    pub is_grain_locked: bool,
    pub shape_key: u64,
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
    let mut geom_cache: std::collections::HashMap<u64, Vec<(f64, f64, f64, Polygon, Vec<Polygon>, Polygon, Rect, Rect, bool)>> = std::collections::HashMap::new();

    parts.iter().map(|p| {
        let poly = Polygon::from_raw(&p.contour);
        let orig_bbox = poly.bounding_box();
        let area = p.area.unwrap_or_else(|| poly.area());
        let source_width = p.width.filter(|v| *v > 1e-6).unwrap_or_else(|| orig_bbox.width());
        let source_height = p.height.filter(|v| *v > 1e-6).unwrap_or_else(|| orig_bbox.height());
        let is_macro = p.manual_cluster_macro == Some(true);
        let small_part = !is_macro
            && small_part_threshold > 0.001
            && source_width > 0.001
            && source_height > 0.001
            && source_width.min(source_height) <= small_part_threshold + 0.001;
        let small_part_clearance = if small_part && configured_small_part_clearance > 0.001 {
            spacing.max(configured_small_part_clearance)
        } else {
            0.0
        };
        let effective_holes: Option<Vec<Vec<[f64; 2]>>> = if p.holes.as_ref().map_or(false, |h| !h.is_empty()) {
            p.holes.clone()
        } else if p.manual_cluster_macro == Some(true) {
            p.manual_cluster_children.as_ref().and_then(|children| {
                crate::macro_comb::extract_macro_cavity_holes(children, orig_bbox.width(), orig_bbox.height())
            })
        } else {
            None
        };
        let size_group = SizeGroup::from_dimensions(orig_bbox.width(), orig_bbox.height());
        let is_two_sided = p.two_sided.unwrap_or(false)
            || (p.manual_cluster_macro == Some(true) && p.manual_cluster_children.as_ref().map_or(false, |children| {
                children.iter().any(|c| c.two_sided == Some(true))
            }));
        let is_anchor = effective_holes.as_ref().map_or(false, |h| !h.is_empty()) || !poly.is_rectangular();
        let is_irregular = !poly.is_rectangular();
        let is_rect = poly.is_rectangular() && effective_holes.as_ref().map_or(true, |h| h.is_empty());

        let has_child_grain = p.manual_cluster_macro == Some(true) && p.manual_cluster_children.as_ref().map_or(false, |children| {
            children.iter().any(|c| c.has_grain_label == Some(true) || c.grain_arrow_degrees.is_some() || c.base_rotation_degrees.is_some())
        });
        let has_direct_grain = p.has_grain_label == Some(true)
            || p.grain_locked == Some(true)
            || p.grain_arrow_degrees.is_some();
        let is_free_rot = p.free_rotation == Some(true) || global_rot_div >= 4;
        let is_part_grain_locked = if is_free_rot {
            false
        } else {
            (global_rot_div <= 2) || has_direct_grain || has_child_grain
        };
        let base_rot = norm_angle(p.base_rotation_degrees.or(p.rotation_degrees).unwrap_or(0.0));

        let mut allowed_rotations: Vec<f64> = if is_part_grain_locked {
            if let Some(rots) = &p.rotations {
                let r: Vec<f64> = rots.iter()
                    .map(|&d| norm_angle(d))
                    .filter(|&rot| {
                        let diff = (rot - base_rot).abs() % 180.0;
                        diff <= 2.0 || diff >= 178.0
                    })
                    .collect();
                if !r.is_empty() { r } else { vec![base_rot, norm_angle(base_rot + 180.0)] }
            } else {
                vec![base_rot, norm_angle(base_rot + 180.0)]
            }
        } else if let Some(rots) = &p.rotations {
            let mut r: Vec<f64> = rots.iter().map(|&d| norm_angle(d)).collect();
            if r.is_empty() { r.push(base_rot); }
            r
        } else if global_rot_div >= 4 || p.free_rotation == Some(true) {
            vec![base_rot, norm_angle(base_rot + 90.0), norm_angle(base_rot + 180.0), norm_angle(base_rot + 270.0)]
        } else {
            vec![base_rot, norm_angle(base_rot + 180.0)]
        };

        if (global_rot_div >= 4 || p.free_rotation == Some(true)) && !is_part_grain_locked {
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

        let raw_key = {
            use std::hash::{Hash, Hasher};
            let mut h = std::collections::hash_map::DefaultHasher::new();
            small_part.hash(&mut h);
            small_part_clearance.to_bits().hash(&mut h);
            p.contour.len().hash(&mut h);
            for pt in &p.contour {
                ((pt[0] * 100.0).round() as i64).hash(&mut h);
                ((pt[1] * 100.0).round() as i64).hash(&mut h);
            }
            if let Some(holes) = &effective_holes {
                holes.len().hash(&mut h);
                for hole in holes {
                    hole.len().hash(&mut h);
                    if let Some(fpt) = hole.first() {
                        ((fpt[0] * 100.0).round() as i64).hash(&mut h);
                        ((fpt[1] * 100.0).round() as i64).hash(&mut h);
                    }
                }
            }
            for &rot in &allowed_rotations {
                ((rot * 100.0).round() as i64).hash(&mut h);
            }
            h.finish()
        };

        let rotate_layers_for = |rot: f64, shift_x: f64, shift_y: f64| -> Vec<DrawLayerData> {
            let rad = rot * std::f64::consts::PI / 180.0;
            p.draw_layers.as_ref().map_or(Vec::new(), |layers| {
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
            })
        };

        let cached_geoms = if let Some(cg) = geom_cache.get(&raw_key) {
            cg.clone()
        } else {
            let base_poly = if p.manual_cluster_macro != Some(true) && poly.points.len() > 48 {
                crate::macro_comb::decimate_curved_polygon(&poly.points, 10.0)
            } else {
                poly.clone()
            };
            let mut geoms: Vec<(f64, f64, f64, Polygon, Vec<Polygon>, Polygon, Rect, Rect, bool)> = allowed_rotations.iter().map(|&rot| {
                let rotated = base_poly.rotate_degrees(rot, Point::new(0.0, 0.0));
                let (norm_poly, shift_x, shift_y) = rotated.normalize_to_origin();
                let bbox = norm_poly.bounding_box();
                let norm_holes: Vec<Polygon> = effective_holes.as_ref().map_or(Vec::new(), |holes_vec| {
                    holes_vec.iter().map(|raw_h| {
                        let h_poly = Polygon::from_raw(raw_h);
                        let h_rot = h_poly.rotate_degrees(rot, Point::new(0.0, 0.0));
                        h_rot.translate(shift_x, shift_y)
                    }).collect()
                });
                let is_rect = norm_poly.is_rectangular() && norm_holes.is_empty();
                let dilated_poly = offset_polygon(&norm_poly, spacing * 0.5);
                let dilated_bbox = dilated_poly.bounding_box();
                (rot, shift_x, shift_y, norm_poly, norm_holes, dilated_poly, dilated_bbox, bbox, is_rect)
            }).filter(|g| g.7.width() <= max_bw && g.7.height() <= max_bh).collect();

            if geoms.is_empty() {
                let rot = allowed_rotations[0];
                let rotated = poly.rotate_degrees(rot, Point::new(0.0, 0.0));
                let (norm_poly, shift_x, shift_y) = rotated.normalize_to_origin();
                let bbox = norm_poly.bounding_box();
                let norm_holes: Vec<Polygon> = effective_holes.as_ref().map_or(Vec::new(), |holes_vec| {
                    holes_vec.iter().map(|raw_h| {
                        let h_poly = Polygon::from_raw(raw_h);
                        let h_rot = h_poly.rotate_degrees(rot, Point::new(0.0, 0.0));
                        h_rot.translate(shift_x, shift_y)
                    }).collect()
                });
                let is_rect = norm_poly.is_rectangular() && norm_holes.is_empty();
                let dilated_poly = offset_polygon(&norm_poly, spacing * 0.5);
                let dilated_bbox = dilated_poly.bounding_box();
                geoms.push((rot, shift_x, shift_y, norm_poly, norm_holes, dilated_poly, dilated_bbox, bbox, is_rect));
            }
            geom_cache.insert(raw_key, geoms.clone());
            geoms
        };

        let final_variants: Vec<RotatedVariant> = cached_geoms.into_iter().map(|(rot, shift_x, shift_y, norm_poly, norm_holes, dilated_poly, dilated_bbox, bbox, is_rect)| {
            let norm_layers = rotate_layers_for(rot, shift_x, shift_y);
            RotatedVariant { rotation: rot, norm_poly, norm_holes, norm_layers, dilated_poly, dilated_bbox, bbox, is_rect }
        }).collect();
        let shape_key = raw_key;
        let mut orig_part = p.clone();
        if orig_part.holes.is_none() && effective_holes.is_some() {
            orig_part.holes = effective_holes;
        }

        ProcessedPart {
            original: orig_part,
            area,
            size_group,
            is_two_sided,
            is_anchor,
            is_irregular,
            is_rect,
            small_part,
            small_part_clearance,
            variants: final_variants,
            is_grain_locked: is_part_grain_locked,
            shape_key,
        }
    }).collect()
}

pub fn pack_material(
    material_state: &MaterialStateInput,
    strategy_id: usize,
) -> LayoutOutput {
    pack_material_streaming(material_state, strategy_id, None::<fn(&LayoutOutput)>)
}

pub fn pack_material_with_prepared(
    material_state: &MaterialStateInput,
    prepared: &[ProcessedPart],
    comb_count: usize,
    strategy_id: usize,
) -> LayoutOutput {
    pack_material_streaming_with_prepared(material_state, prepared, comb_count, strategy_id, None::<fn(&LayoutOutput)>)
}

/// Strict Implementation of MN Priority-Cavity Hybrid V35 (QUY_TRINH_NESTING_CHI_TIET.md)
pub fn pack_material_streaming<F>(
    material_state: &MaterialStateInput,
    strategy_id: usize,
    on_stream: Option<F>,
) -> LayoutOutput
where
    F: FnMut(&LayoutOutput),
{
    let config = material_state.configuration.clone().unwrap_or_default();
    let part_spacing = config.part_spacing.or(config.cut_gap).unwrap_or(6.0);
    let existing_macro_count = material_state.parts.iter().filter(|p| p.manual_cluster_macro == Some(true)).count();
    let (paired_parts, new_comb_count) = if existing_macro_count > 0 {
        (material_state.parts.clone(), 0)
    } else {
        crate::macro_comb::pair_comb_parts(&material_state.parts, &config, part_spacing)
    };
    let comb_count = existing_macro_count + new_comb_count;
    let processed = prepare_parts(&paired_parts, &config, part_spacing);
    pack_material_streaming_with_prepared(material_state, &processed, comb_count, strategy_id, on_stream)
}

pub fn pack_material_streaming_with_prepared<F>(
    material_state: &MaterialStateInput,
    prepared: &[ProcessedPart],
    comb_count: usize,
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
        optimization_seed: None,
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
    let processed = prepared.to_vec();

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

    let opt_seed = config.optimization_seed
        .or(material_state.optimization_seed)
        .unwrap_or(0);
    let seed_base = opt_seed.wrapping_add((strategy_id as u64).wrapping_mul(6364136223846793005));

    let mut ordered_parts = processed;
    match strategy_id % 10 {
        0 => {
            ordered_parts.sort_by(|a, b| {
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.size_group.is_filler() != b.size_group.is_filler() { return a.size_group.is_filler().cmp(&b.size_group.is_filler()); }
                if a.size_group != b.size_group { return a.size_group.cmp(&b.size_group); }
                let max_a = a.variants[0].bbox.width().max(a.variants[0].bbox.height());
                let max_b = b.variants[0].bbox.width().max(b.variants[0].bbox.height());
                let band_a = (max_a / 120.0).floor() as i64;
                let band_b = (max_b / 120.0).floor() as i64;
                if band_a != band_b { return band_b.cmp(&band_a); }
                b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
        1 => {
            ordered_parts.sort_by(|a, b| {
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.size_group.is_filler() != b.size_group.is_filler() { return a.size_group.is_filler().cmp(&b.size_group.is_filler()); }
                let h_a = a.variants[0].bbox.height().max(a.variants[0].bbox.width());
                let h_b = b.variants[0].bbox.height().max(b.variants[0].bbox.width());
                h_b.partial_cmp(&h_a).unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal))
            });
        }
        2 => {
            ordered_parts.sort_by(|a, b| {
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.size_group.is_filler() != b.size_group.is_filler() { return a.size_group.is_filler().cmp(&b.size_group.is_filler()); }
                let w_a = a.variants[0].bbox.width().max(a.variants[0].bbox.height());
                let w_b = b.variants[0].bbox.width().max(b.variants[0].bbox.height());
                w_b.partial_cmp(&w_a).unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal))
            });
        }
        3 => {
            ordered_parts.sort_by(|a, b| {
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.size_group.is_filler() != b.size_group.is_filler() { return a.size_group.is_filler().cmp(&b.size_group.is_filler()); }
                let perim_a = a.variants[0].bbox.width() + a.variants[0].bbox.height();
                let perim_b = b.variants[0].bbox.width() + b.variants[0].bbox.height();
                perim_b.partial_cmp(&perim_a).unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal))
            });
        }
        4 => {
            ordered_parts.sort_by(|a, b| {
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.size_group.is_filler() != b.size_group.is_filler() { return a.size_group.is_filler().cmp(&b.size_group.is_filler()); }
                let a_cluster = a.original.manual_cluster_macro == Some(true);
                let b_cluster = b.original.manual_cluster_macro == Some(true);
                if a_cluster != b_cluster { return b_cluster.cmp(&a_cluster); }
                let a_has_hole = a.original.holes.as_ref().map_or(false, |h| !h.is_empty());
                let b_has_hole = b.original.holes.as_ref().map_or(false, |h| !h.is_empty());
                if a_has_hole != b_has_hole { return b_has_hole.cmp(&a_has_hole); }
                if a.is_anchor != b.is_anchor { return b.is_anchor.cmp(&a.is_anchor); }
                b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
        5 => {
            ordered_parts.sort_by(|a, b| {
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.size_group.is_filler() != b.size_group.is_filler() { return a.size_group.is_filler().cmp(&b.size_group.is_filler()); }
                if a.is_rect != b.is_rect { return b.is_rect.cmp(&a.is_rect); }
                let a_cluster = a.original.manual_cluster_macro == Some(true);
                let b_cluster = b.original.manual_cluster_macro == Some(true);
                if a_cluster != b_cluster { return b_cluster.cmp(&a_cluster); }
                b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
        6 => {
            // Sắp diện tích tăng dần trong từng phân vùng (Skeleton trước, Filler sau)
            ordered_parts.sort_by(|a, b| {
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.size_group.is_filler() != b.size_group.is_filler() { return a.size_group.is_filler().cmp(&b.size_group.is_filler()); }
                a.area.partial_cmp(&b.area).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
        7 => {
            // Aspect ratio giảm dần (thanh dài trước, giữ Skeleton trước Filler)
            ordered_parts.sort_by(|a, b| {
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.size_group.is_filler() != b.size_group.is_filler() { return a.size_group.is_filler().cmp(&b.size_group.is_filler()); }
                let ar_a = (a.variants[0].bbox.width().max(a.variants[0].bbox.height())) / (a.variants[0].bbox.width().min(a.variants[0].bbox.height()).max(1.0));
                let ar_b = (b.variants[0].bbox.width().max(b.variants[0].bbox.height())) / (b.variants[0].bbox.width().min(b.variants[0].bbox.height()).max(1.0));
                ar_b.partial_cmp(&ar_a).unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal))
            });
        }
        8 => {
            // Interleaved trong từng nhóm (Skeleton trước, Filler sau)
            ordered_parts.sort_by(|a, b| {
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.size_group.is_filler() != b.size_group.is_filler() { return a.size_group.is_filler().cmp(&b.size_group.is_filler()); }
                b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
            });
            let split_idx = ordered_parts.partition_point(|p| !p.size_group.is_filler());
            if split_idx > 2 {
                let mut interleaved = Vec::with_capacity(ordered_parts.len());
                let mut l = 0;
                let mut r = split_idx - 1;
                while l <= r {
                    if l == r {
                        interleaved.push(ordered_parts[l].clone());
                        break;
                    }
                    interleaved.push(ordered_parts[l].clone());
                    interleaved.push(ordered_parts[r].clone());
                    l += 1;
                    r -= 1;
                }
                interleaved.extend_from_slice(&ordered_parts[split_idx..]);
                ordered_parts = interleaved;
            }
        }
        _ => {
            ordered_parts.sort_by(|a, b| {
                if a.is_two_sided != b.is_two_sided { return b.is_two_sided.cmp(&a.is_two_sided); }
                if a.size_group.is_filler() != b.size_group.is_filler() { return a.size_group.is_filler().cmp(&b.size_group.is_filler()); }
                let a_cluster = a.original.manual_cluster_macro == Some(true);
                let b_cluster = b.original.manual_cluster_macro == Some(true);
                if a_cluster != b_cluster { return b_cluster.cmp(&a_cluster); }
                if a.is_irregular != b.is_irregular { return b.is_irregular.cmp(&a.is_irregular); }
                b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
    }

    let is_room_strategy = strategy_id >= 100;
    let part_rooms: Vec<String> = if is_room_strategy {
        ordered_parts.iter().map(|p| crate::types::get_part_room_key(&p.original)).collect()
    } else {
        Vec::new()
    };

    if !is_room_strategy && ordered_parts.len() >= 120 {
        let filler_start = ordered_parts.partition_point(|p| !p.size_group.is_filler());
        let filler_len = ordered_parts.len().saturating_sub(filler_start);
        if filler_len >= 80 {
            ordered_parts[filler_start..].sort_by(|a, b| {
                if (a.area - b.area).abs() > 50.0 {
                    b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
                } else {
                    a.variants[0].bbox.height().partial_cmp(&b.variants[0].bbox.height()).unwrap_or(std::cmp::Ordering::Equal)
                }
            });
        }
    }

    let min_filler_area = ordered_parts.last().map_or(0.0, |p| p.area);
    let min_filler_h = ordered_parts.last().map_or(0.0, |p| p.variants[0].bbox.height());

    if !is_room_strategy && (strategy_id >= 10 || (opt_seed > 0 && strategy_id >= 6)) {
        let mut rng_state = seed_base.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let n = ordered_parts.len();
        if n > 1 {
            for i in (1..n).rev() {
                if ordered_parts[i].is_two_sided { continue; }
                let i_filler = ordered_parts[i].size_group.is_filler();
                let i_area = ordered_parts[i].area;
                rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let j = ((rng_state >> 32) as usize) % (i + 1);
                if !ordered_parts[j].is_two_sided
                    && ordered_parts[j].size_group.is_filler() == i_filler
                    && (ordered_parts[j].area - i_area).abs() <= i_area * 0.25
                {
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

    // Helper closure to try placing a part specifically into hole or concave pocket cavities on a SINGLE sheet
    let try_place_in_single_sheet_holes = |part: &ProcessedPart, sheet_idx: usize, sheets: &mut [SheetContext], sheet_placements: &mut [Vec<PlacementOutput>], reserve_for_smaller: bool| -> bool {
        if reserve_for_smaller
            && min_filler_area > 0.0
            && part.area > min_filler_area * 1.35
            && sheet_placements[sheet_idx].iter().any(|p| p.manual_cluster_macro == Some(true))
        {
            return false;
        }
        let sheet = &mut sheets[sheet_idx];
        let allow_pockets = sheet_in_sheet || part.size_group.is_filler();
        if !sheet.has_holes_or_pockets() {
            return false;
        }
        let mut best_variant = None;
        let mut best_pt = None;
        let mut best_score = f64::MAX;

        for variant in &part.variants {
            if let Some((px, py, score)) = sheet.find_best_hole_or_pocket_anchor(
                &variant.norm_poly,
                &variant.dilated_poly,
                &variant.dilated_bbox,
                variant.is_rect,
                part.small_part,
                part.small_part_clearance,
                bl_weight,
                allow_pockets,
            ) {
                if score < best_score {
                    best_score = score;
                    best_pt = Some((px, py));
                    best_variant = Some(variant);
                }
            }
        }

        if let (Some((px, py)), Some(variant)) = (best_pt, best_variant) {
            let col_poly = if part.original.manual_cluster_macro != Some(true) && variant.norm_poly.points.len() > 64 { crate::macro_comb::decimate_curved_polygon(&variant.norm_poly.points, 16.0) } else { variant.norm_poly.clone() };
            let col_holes = variant.norm_holes.iter().map(|h| if h.points.len() > 64 { crate::macro_comb::decimate_curved_polygon(&h.points, 16.0) } else { h.clone() }).collect();
            sheet.add_placed(col_poly, col_holes, px, py, variant.rotation, part.small_part, part.small_part_clearance);

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
                color: if part.is_two_sided { Some("#CC99FF".to_string()) } else { part.original.color.clone() },
                manual_cluster_macro: part.original.manual_cluster_macro,
                manual_cluster_children: part.original.manual_cluster_children.clone(),
                logical_part_count: part.original.logical_part_count,
                has_grain_label: if part.is_grain_locked { Some(true) } else { part.original.has_grain_label },
                grain_arrow_degrees: part.original.grain_arrow_degrees,
                grain_locked: Some(part.is_grain_locked),
                free_rotation: part.original.free_rotation,
                small_part: Some(part.small_part),
                small_part_clearance: Some(part.small_part_clearance),
                small_part_edge_protected: Some(sheet.small_part_edge_protected(
                    part.small_part,
                    px,
                    py,
                    variant.bbox.width(),
                    variant.bbox.height(),
                )),
                two_sided: Some(part.is_two_sided),
            };

            sheet_placements[sheet_idx].push(placement);
            true
        } else {
            false
        }
    };

    // Helper closure to try placing a part onto a SINGLE sheet
    let try_place_on_single_sheet = |part: &ProcessedPart, sheet_idx: usize, sheets: &mut [SheetContext], sheet_placements: &mut [Vec<PlacementOutput>], failed_hole_keys: &mut std::collections::HashSet<u64>, reserve_for_smaller: bool| -> bool {
        let min_dim = part.variants[0].bbox.width().min(part.variants[0].bbox.height());
        let max_dim = part.variants[0].bbox.width().max(part.variants[0].bbox.height());
        let has_macro_on_sheet = sheet_placements[sheet_idx].iter().any(|p| p.manual_cluster_macro == Some(true));
        let is_larger_filler = reserve_for_smaller && min_filler_area > 0.0 && part.area > min_filler_area * 1.35 && part.size_group.is_filler();
        if (sheet_in_sheet || part.size_group.is_filler() || (min_dim <= 220.0 && max_dim <= 600.0))
            && !(has_macro_on_sheet && is_larger_filler)
            && sheets[sheet_idx].has_holes_or_pockets()
            && !failed_hole_keys.contains(&part.shape_key)
        {
            if try_place_in_single_sheet_holes(part, sheet_idx, sheets, sheet_placements, reserve_for_smaller) {
                if !part.variants[0].norm_holes.is_empty() {
                    failed_hole_keys.clear();
                }
                return true;
            } else {
                failed_hole_keys.insert(part.shape_key);
            }
        }

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
                if has_macro_on_sheet && is_larger_filler && min_filler_h > 0.0 {
                    let vh = variant.bbox.height();
                    let rem_h = (board_height - edge_margin) - py;
                    if vh > min_filler_h + 8.0
                        && rem_h < vh * 2.0 + part_spacing
                        && rem_h - vh - part_spacing < min_filler_h
                        && rem_h >= min_filler_h * 2.0 + part_spacing
                    {
                        continue;
                    }
                }
                if score < best_score {
                    best_score = score;
                    best_pt = Some((px, py));
                    best_variant = Some(variant);
                }
            }
        }

        if let (Some((px, py)), Some(variant)) = (best_pt, best_variant) {
            let col_poly = if part.original.manual_cluster_macro != Some(true) && variant.norm_poly.points.len() > 64 { crate::macro_comb::decimate_curved_polygon(&variant.norm_poly.points, 16.0) } else { variant.norm_poly.clone() };
            let col_holes = variant.norm_holes.iter().map(|h| if h.points.len() > 64 { crate::macro_comb::decimate_curved_polygon(&h.points, 16.0) } else { h.clone() }).collect();
            sheet.add_placed(col_poly, col_holes, px, py, variant.rotation, part.small_part, part.small_part_clearance);

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
                color: if part.is_two_sided { Some("#CC99FF".to_string()) } else { part.original.color.clone() },
                manual_cluster_macro: part.original.manual_cluster_macro,
                manual_cluster_children: part.original.manual_cluster_children.clone(),
                logical_part_count: part.original.logical_part_count,
                has_grain_label: if part.is_grain_locked { Some(true) } else { part.original.has_grain_label },
                grain_arrow_degrees: part.original.grain_arrow_degrees,
                grain_locked: Some(part.is_grain_locked),
                free_rotation: part.original.free_rotation,
                small_part: Some(part.small_part),
                small_part_clearance: Some(part.small_part_clearance),
                small_part_edge_protected: Some(sheet.small_part_edge_protected(
                    part.small_part,
                    px,
                    py,
                    variant.bbox.width(),
                    variant.bbox.height(),
                )),
                two_sided: Some(part.is_two_sided),
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
            board_width, board_height, edge_margin, part_spacing,
            small_part_edge_zone, merge_cut_paths, sheet_in_sheet, compact_directions.clone(),
        );
        sheets.push(new_sheet);
        sheet_placements.push(Vec::new());

        let mut placed_any_on_this_sheet = false;
        let mut failed_hole_keys: std::collections::HashSet<u64> = std::collections::HashSet::new();
        let mut failed_sheet_keys: std::collections::HashSet<u64> = std::collections::HashSet::new();
        let mut pass_count = 0;

        if is_room_strategy {
            let mut active_sheet_room = String::new();
            for i in 0..total_parts {
                if !placed_flags[i] && !part_rooms[i].is_empty() {
                    active_sheet_room = part_rooms[i].clone();
                    break;
                }
            }
            if !active_sheet_room.is_empty() {
                for i in 0..total_parts {
                    if placed_flags[i] || part_rooms[i] != active_sheet_room { continue; }
                    let part = &ordered_parts[i];
                    if failed_sheet_keys.contains(&part.shape_key) { continue; }
                    if try_place_on_single_sheet(part, current_sheet_idx, &mut sheets, &mut sheet_placements, &mut failed_hole_keys, false) {
                        placed_flags[i] = true;
                        placed_any_on_this_sheet = true;
                        if !part.variants[0].norm_holes.is_empty() {
                            failed_sheet_keys.clear();
                            failed_hole_keys.clear();
                            for j in 0..total_parts {
                                if placed_flags[j] || part_rooms[j] != active_sheet_room { continue; }
                                let cand = &ordered_parts[j];
                                if !cand.variants[0].norm_holes.is_empty() || failed_hole_keys.contains(&cand.shape_key) { continue; }
                                if try_place_in_single_sheet_holes(cand, current_sheet_idx, &mut sheets, &mut sheet_placements, false) {
                                    placed_flags[j] = true;
                                } else {
                                    failed_hole_keys.insert(cand.shape_key);
                                }
                            }
                        }
                    } else {
                        failed_sheet_keys.insert(part.shape_key);
                    }
                }
                failed_sheet_keys.clear();
                failed_hole_keys.clear();
            }
        }

        loop {
            pass_count += 1;
            let mut placed_in_this_pass = false;
            let mut holes_added_in_pass = false;

            if sheets[current_sheet_idx].has_holes_or_pockets() {
                for i in 0..total_parts {
                    if placed_flags[i] { continue; }
                    let part = &ordered_parts[i];
                    if failed_hole_keys.contains(&part.shape_key) { continue; }
                    let reserve_for_smaller = total_parts >= 40 && !placed_flags[total_parts - 34];
                    if try_place_in_single_sheet_holes(part, current_sheet_idx, &mut sheets, &mut sheet_placements, reserve_for_smaller) {
                        placed_flags[i] = true;
                        placed_in_this_pass = true;
                        placed_any_on_this_sheet = true;
                        if !part.variants[0].norm_holes.is_empty() {
                            failed_hole_keys.clear();
                            holes_added_in_pass = true;
                        }
                    } else {
                        failed_hole_keys.insert(part.shape_key);
                    }
                }
            }

            for i in 0..total_parts {
                if placed_flags[i] { continue; }
                let part = &ordered_parts[i];
                if failed_sheet_keys.contains(&part.shape_key) { continue; }
                let reserve_for_smaller = total_parts >= 40 && !placed_flags[total_parts - 34];
                if try_place_on_single_sheet(part, current_sheet_idx, &mut sheets, &mut sheet_placements, &mut failed_hole_keys, reserve_for_smaller) {
                    placed_flags[i] = true;
                    placed_in_this_pass = true;
                    placed_any_on_this_sheet = true;
                    if !part.variants[0].norm_holes.is_empty() {
                        failed_sheet_keys.clear();
                        failed_hole_keys.clear();
                        holes_added_in_pass = true;
                        if part.original.manual_cluster_macro != Some(true) {
                            for j in 0..total_parts {
                                if placed_flags[j] { continue; }
                                let cand = &ordered_parts[j];
                                if !cand.variants[0].norm_holes.is_empty() || failed_hole_keys.contains(&cand.shape_key) { continue; }
                                let reserve_inner = total_parts >= 40 && !placed_flags[total_parts - 34];
                                if try_place_in_single_sheet_holes(cand, current_sheet_idx, &mut sheets, &mut sheet_placements, reserve_inner) {
                                    placed_flags[j] = true;
                                } else {
                                    failed_hole_keys.insert(cand.shape_key);
                                }
                            }
                        }
                    }
                } else {
                    failed_sheet_keys.insert(part.shape_key);
                }
            }

            if !placed_in_this_pass || !holes_added_in_pass || pass_count >= 2 {
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
