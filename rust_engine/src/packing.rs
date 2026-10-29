// src/packing.rs
use crate::geometry::{Polygon, Rect, offset_polygon, rotate_shape_full};
use crate::nfp::SheetContext;
use crate::types::{ConfigurationInput, LayoutOutput, MaterialStateInput, PartInput, PlacementOutput, SheetOutput};

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
    pub norm_holes: Option<Vec<Vec<[f64; 2]>>>,
    pub norm_draw_layers: Option<Vec<serde_json::Value>>,
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
    pub is_anchor: bool,
    pub is_two_sided: bool,
    pub is_irregular: bool,
    pub variants: Vec<RotatedVariant>,
    pub allowed_rotations: Vec<f64>,
}

pub fn prepare_parts(parts: &[PartInput], config: &ConfigurationInput, spacing: f64) -> Vec<ProcessedPart> {
    let global_rot_div = config.rotation_divisions.unwrap_or(1);
    let board_w = config.board_width.unwrap_or(2440.0);
    let board_h = config.board_height.unwrap_or(1220.0);
    let margin = config.edge_margin.unwrap_or(10.0);
    let max_bw = (board_w - 2.0 * margin).max(10.0);
    let max_bh = (board_h - 2.0 * margin).max(10.0);

    // 1. Complementary interlocking pair detection (L-shapes, Triangles at 180 deg)
    let mut paired_parts = Vec::new();
    let mut used = vec![false; parts.len()];

    for i in 0..parts.len() {
        if used[i] { continue; }
        for j in (i + 1)..parts.len() {
            if used[j] { continue; }
            if let Some((macro_part, _saving)) = crate::macro_comb::try_build_comb_pair(&parts[i], &parts[j], spacing) {
                used[i] = true;
                used[j] = true;
                paired_parts.push(macro_part);
                break;
            }
        }
        if !used[i] {
            paired_parts.push(parts[i].clone());
            used[i] = true;
        }
    }

    paired_parts.into_iter().map(|p| {
        let poly = Polygon::from_raw(&p.contour);
        let area = p.area.unwrap_or_else(|| poly.area());
        let base_rot = p.base_rotation_degrees.unwrap_or(0.0);
        let orig_bbox = poly.bounding_box();
        let size_group = SizeGroup::from_dimensions(orig_bbox.width(), orig_bbox.height());
        let is_anchor = p.holes.as_ref().map_or(false, |h| !h.is_empty()) || !poly.is_rectangular();
        let is_two_sided = p.two_sided == Some(true) || p.grain_locked == Some(true);
        let is_irregular = !poly.is_rectangular();

        let mut allowed_rotations = if p.manual_cluster_macro == Some(true) {
            vec![
                norm_angle(base_rot + 0.0),
                norm_angle(base_rot + 90.0),
                norm_angle(base_rot + 180.0),
                norm_angle(base_rot + 270.0),
            ]
        } else if let Some(ref rots) = p.rotations {
            rots.clone()
        } else if p.free_rotation == Some(true) {
            vec![
                norm_angle(base_rot + 0.0),
                norm_angle(base_rot + 90.0),
                norm_angle(base_rot + 180.0),
                norm_angle(base_rot + 270.0),
            ]
        } else if global_rot_div == 1 || p.grain_locked == Some(true) || (p.has_grain_label == Some(true) && p.free_rotation != Some(true)) {
            vec![
                norm_angle(base_rot + 0.0),
                norm_angle(base_rot + 180.0),
            ]
        } else if global_rot_div >= 4 {
            vec![
                norm_angle(base_rot + 0.0),
                norm_angle(base_rot + 90.0),
                norm_angle(base_rot + 180.0),
                norm_angle(base_rot + 270.0),
            ]
        } else {
            vec![
                norm_angle(base_rot + 0.0),
                norm_angle(base_rot + 180.0),
            ]
        };

        let fits_initially = allowed_rotations.iter().any(|&rot| {
            let rot_poly = poly.rotate_deg(rot);
            let bbox = rot_poly.bounding_box();
            bbox.width() <= max_bw && bbox.height() <= max_bh
        });

        if !fits_initially && orig_bbox.width() <= max_bh && orig_bbox.height() <= max_bw {
            allowed_rotations.push(norm_angle(base_rot + 90.0));
            allowed_rotations.push(norm_angle(base_rot + 270.0));
        }

        let variants: Vec<RotatedVariant> = allowed_rotations.iter().map(|&rot| {
            let (norm_poly, norm_holes, norm_draw_layers) = rotate_shape_full(
                &p.contour,
                p.holes.as_ref(),
                p.draw_layers.as_ref(),
                rot,
            );
            let bbox = norm_poly.bounding_box();
            let is_rect = norm_poly.is_rectangular();
            let dilated_poly = offset_polygon(&norm_poly, spacing * 0.5);
            let dilated_bbox = dilated_poly.bounding_box();
            RotatedVariant {
                rotation: rot,
                norm_poly,
                norm_holes,
                norm_draw_layers,
                dilated_poly,
                dilated_bbox,
                bbox,
                is_rect,
            }
        }).filter(|v| v.bbox.width() <= max_bw && v.bbox.height() <= max_bh).collect();

        let final_variants = if variants.is_empty() {
            let (norm_poly, norm_holes, norm_draw_layers) = rotate_shape_full(
                &p.contour,
                p.holes.as_ref(),
                p.draw_layers.as_ref(),
                allowed_rotations[0],
            );
            let bbox = norm_poly.bounding_box();
            let is_rect = norm_poly.is_rectangular();
            let dilated_poly = offset_polygon(&norm_poly, spacing * 0.5);
            let dilated_bbox = dilated_poly.bounding_box();
            vec![RotatedVariant {
                rotation: allowed_rotations[0],
                norm_poly,
                norm_holes,
                norm_draw_layers,
                dilated_poly,
                dilated_bbox,
                bbox,
                is_rect,
            }]
        } else {
            variants
        };

        ProcessedPart {
            original: p,
            area,
            size_group,
            is_anchor,
            is_two_sided,
            is_irregular,
            variants: final_variants,
            allowed_rotations,
        }
    }).collect()
}

pub fn expand_layout_clusters(layout: &mut LayoutOutput) {
    let board_w = layout.board_width;
    let board_h = layout.board_height;
    let board_area = board_w * board_h;

    for sheet in &mut layout.sheets {
        let mut expanded = Vec::new();

        for p in std::mem::take(&mut sheet.placements) {
            if p.manual_cluster_macro == Some(true) && p.manual_cluster_children.as_ref().map_or(false, |c| !c.is_empty()) {
                let children = p.manual_cluster_children.unwrap_or_default();
                let macro_rot = p.rotation_degrees;
                let rad = macro_rot * std::f64::consts::PI / 180.0;
                let cos = rad.cos();
                let sin = rad.sin();

                for child in children {
                    let ox = child.offset_x.unwrap_or(0.0);
                    let oy = child.offset_y.unwrap_or(0.0);
                    let child_rot = child.rotation_degrees.unwrap_or(0.0);
                    let rx = ox * cos - oy * sin;
                    let ry = ox * sin + oy * cos;

                    let final_x = p.x + rx;
                    let final_y = p.y + ry;
                    let final_rot = norm_angle(child_rot + macro_rot);

                    let raw_contour = child.contour.unwrap_or_else(|| {
                        let w = child.width.unwrap_or(100.0);
                        let h = child.height.unwrap_or(100.0);
                        vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]]
                    });
                    let poly = Polygon::from_raw(&raw_contour);
                    let rotated = poly.rotate_deg(macro_rot);
                    let norm_poly = rotated.normalize_to_origin();
                    let bbox = norm_poly.bounding_box();

                    expanded.push(PlacementOutput {
                        entity_id: child.entity_id.unwrap_or_default(),
                        id: child.id.unwrap_or_default(),
                        name: child.name,
                        x: (final_x * 10.0).round() / 10.0,
                        y: (final_y * 10.0).round() / 10.0,
                        rotation_degrees: final_rot,
                        base_rotation_degrees: Some(0.0),
                        width: bbox.width(),
                        height: bbox.height(),
                        packed_width: bbox.width(),
                        packed_height: bbox.height(),
                        area: child.area.unwrap_or_else(|| poly.area()),
                        contour: norm_poly.to_raw(),
                        holes: child.holes,
                        color: child.color.or_else(|| p.color.clone()),
                        manual_cluster_macro: None,
                        manual_cluster_children: None,
                        logical_part_count: Some(1),
                        has_grain_label: p.has_grain_label,
                        grain_arrow_degrees: p.grain_arrow_degrees,
                        special_shape: p.special_shape,
                        draw_layers: p.draw_layers.clone(),
                        source_entity_ids: p.source_entity_ids.clone(),
                    });
                }
            } else {
                expanded.push(p);
            }
        }

        sheet.width = board_w;
        sheet.height = board_h;
        sheet.placements = expanded;
        let used_area: f64 = sheet.placements.iter().map(|p| p.area).sum();
        let util = if board_area > 0.0 { (used_area / board_area) * 100.0 } else { 0.0 };
        sheet.utilization = (util * 10.0).round() / 10.0;
        sheet.used_percent = (util * 10.0).round() / 10.0;
    }
}

pub fn pack_material(
    material_state: &MaterialStateInput,
    strategy_id: usize,
) -> LayoutOutput {
    pack_material_streaming(material_state, strategy_id, None::<fn(&LayoutOutput)>)
}

/// Strict Implementation of Priority-Cavity Hybrid V35 Pipeline
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
        priority_zone_nesting: Some(true),
        selective_repack: Some(false),
        max_nesting_time_ms: Some(60000.0),
    });

    let board_width = config.board_width.unwrap_or(2440.0);
    let board_height = config.board_height.unwrap_or(1220.0);
    let edge_margin = config.edge_margin.unwrap_or(10.0);
    let part_spacing = config.part_spacing.or(config.cut_gap).unwrap_or(6.0);
    let target_sheet_utilization = config.target_sheet_utilization.unwrap_or(90.0);
    let sheet_in_sheet = config.sheet_in_sheet.unwrap_or(false) || config.priority_zone_nesting.unwrap_or(true);
    let compact_directions = config.compact_directions.clone().unwrap_or_else(|| vec!["left".to_string(), "bottom".to_string()]);

    let processed = prepare_parts(&material_state.parts, &config, part_spacing);

    // Multi-tier Priority Queue Ordering:
    // 1. Two-Sided First
    // 2. Priority Anchor (frames with holes or concave pockets)
    // 3. Comb Macro (interlocking pairs)
    // 4. Irregular Shapes
    // 5. Size Group (Group 1 > Group 2 > Group 3 > Group 4)
    // 6. Area descending (-area)
    let mut ordered_parts = processed;
    ordered_parts.sort_by(|a, b| {
        let a_cluster = a.original.manual_cluster_macro == Some(true);
        let b_cluster = b.original.manual_cluster_macro == Some(true);
        if a_cluster != b_cluster {
            return b_cluster.cmp(&a_cluster);
        }
        if a.is_two_sided != b.is_two_sided {
            return b.is_two_sided.cmp(&a.is_two_sided);
        }
        if a.is_anchor != b.is_anchor {
            return b.is_anchor.cmp(&a.is_anchor);
        }
        if a.is_irregular != b.is_irregular {
            return b.is_irregular.cmp(&a.is_irregular);
        }
        if a.size_group != b.size_group {
            return a.size_group.cmp(&b.size_group);
        }
        b.area.partial_cmp(&a.area).unwrap_or(std::cmp::Ordering::Equal)
    });

    let bl_weight = if strategy_id % 2 == 0 { 1.0 } else { 0.2 };
    let mut sheets: Vec<SheetContext> = Vec::new();
    let mut sheet_placements: Vec<Vec<PlacementOutput>> = Vec::new();

    let total_parts = ordered_parts.len();
    let material_name = material_state.material.clone().unwrap_or_else(|| "default".to_string());
    let thickness = material_state.thickness.unwrap_or(18.0);
    let board_area = board_width * board_height;

    // Helper closure to try placing a part onto existing sheets
    let try_place_on_existing = |part: &ProcessedPart, sheets: &mut [SheetContext], sheet_placements: &mut [Vec<PlacementOutput>]| -> bool {
        for (sheet_idx, sheet) in sheets.iter_mut().enumerate() {
            let mut best_variant = None;
            let mut best_pt = None;
            let mut best_score = f64::MAX;

            for variant in &part.variants {
                if let Some((px, py, score)) = sheet.find_best_anchor(
                    &variant.norm_poly,
                    &variant.dilated_poly,
                    &variant.dilated_bbox,
                    variant.is_rect,
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
                sheet.add_placed(variant.norm_poly.clone(), variant.norm_holes.clone(), px, py, variant.rotation);

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
                    holes: variant.norm_holes.clone(),
                    color: part.original.color.clone(),
                    manual_cluster_macro: part.original.manual_cluster_macro,
                    manual_cluster_children: part.original.manual_cluster_children.clone(),
                    logical_part_count: part.original.logical_part_count,
                    has_grain_label: part.original.has_grain_label,
                    grain_arrow_degrees: part.original.grain_arrow_degrees,
                    special_shape: part.original.special_shape,
                    draw_layers: variant.norm_draw_layers.clone(),
                    source_entity_ids: part.original.source_entity_ids.clone(),
                };

                sheet_placements[sheet_idx].push(placement);
                return true;
            }
        }
        false
    };

    let mut placed_flags = vec![false; total_parts];

    // --- BƯỚC 1: Dựng Khung Xương (Skeleton Stage — Nhóm 1 & 2) ---
    // Duy nhất giai đoạn này được phép mở Sheet mới.
    for i in 0..total_parts {
        let part = &ordered_parts[i];
        if !part.size_group.is_skeleton() {
            continue;
        }

        let placed = try_place_on_existing(part, &mut sheets, &mut sheet_placements);
        if placed {
            placed_flags[i] = true;
        } else {
            // Open new sheet for skeleton parts
            let mut new_sheet = SheetContext::new(board_width, board_height, edge_margin, part_spacing, sheet_in_sheet, compact_directions.clone());
            let valid_variant = &part.variants[0];
            let pt = (edge_margin, edge_margin);
            new_sheet.add_placed(valid_variant.norm_poly.clone(), valid_variant.norm_holes.clone(), pt.0, pt.1, valid_variant.rotation);

            let placement = PlacementOutput {
                entity_id: part.original.entity_id.clone().unwrap_or_default(),
                id: part.original.id.clone().unwrap_or_default(),
                name: part.original.name.clone(),
                x: pt.0,
                y: pt.1,
                rotation_degrees: valid_variant.rotation,
                base_rotation_degrees: part.original.base_rotation_degrees,
                width: valid_variant.bbox.width(),
                height: valid_variant.bbox.height(),
                packed_width: valid_variant.bbox.width(),
                packed_height: valid_variant.bbox.height(),
                area: part.area,
                contour: valid_variant.norm_poly.to_raw(),
                holes: valid_variant.norm_holes.clone(),
                color: part.original.color.clone(),
                manual_cluster_macro: part.original.manual_cluster_macro,
                manual_cluster_children: part.original.manual_cluster_children.clone(),
                logical_part_count: part.original.logical_part_count,
                has_grain_label: part.original.has_grain_label,
                grain_arrow_degrees: part.original.grain_arrow_degrees,
                special_shape: part.original.special_shape,
                draw_layers: valid_variant.norm_draw_layers.clone(),
                source_entity_ids: part.original.source_entity_ids.clone(),
            };

            sheets.push(new_sheet);
            sheet_placements.push(vec![placement]);
            placed_flags[i] = true;
        }
    }

    // --- BƯỚC 2 & 3: Lấp Hốc Rỗng, Ghép Cặp & Lấp Khoảng Trống (Nhóm 3 & 4) ---
    // TUYỆT ĐỐI KHÔNG ĐƯỢC PHÉP MỞ SHEET MỚI trong các bước này!
    for i in 0..total_parts {
        if placed_flags[i] { continue; }
        let part = &ordered_parts[i];

        if try_place_on_existing(part, &mut sheets, &mut sheet_placements) {
            placed_flags[i] = true;
        }
    }

    // --- BƯỚC 4: Xử Lý Phần Dư & Hậu Cần Dồn Tấm (Logistics Repack) ---
    for i in 0..total_parts {
        if placed_flags[i] { continue; }
        let part = &ordered_parts[i];

        // If part still couldn't fit on earlier sheets, open tail sheet
        let placed = try_place_on_existing(part, &mut sheets, &mut sheet_placements);
        if placed {
            placed_flags[i] = true;
        } else {
            let mut new_sheet = SheetContext::new(board_width, board_height, edge_margin, part_spacing, sheet_in_sheet, compact_directions.clone());
            let valid_variant = &part.variants[0];
            let pt = (edge_margin, edge_margin);
            new_sheet.add_placed(valid_variant.norm_poly.clone(), valid_variant.norm_holes.clone(), pt.0, pt.1, valid_variant.rotation);

            let placement = PlacementOutput {
                entity_id: part.original.entity_id.clone().unwrap_or_default(),
                id: part.original.id.clone().unwrap_or_default(),
                name: part.original.name.clone(),
                x: pt.0,
                y: pt.1,
                rotation_degrees: valid_variant.rotation,
                base_rotation_degrees: part.original.base_rotation_degrees,
                width: valid_variant.bbox.width(),
                height: valid_variant.bbox.height(),
                packed_width: valid_variant.bbox.width(),
                packed_height: valid_variant.bbox.height(),
                area: part.area,
                contour: valid_variant.norm_poly.to_raw(),
                holes: valid_variant.norm_holes.clone(),
                color: part.original.color.clone(),
                manual_cluster_macro: part.original.manual_cluster_macro,
                manual_cluster_children: part.original.manual_cluster_children.clone(),
                logical_part_count: part.original.logical_part_count,
                has_grain_label: part.original.has_grain_label,
                grain_arrow_degrees: part.original.grain_arrow_degrees,
                special_shape: part.original.special_shape,
                draw_layers: valid_variant.norm_draw_layers.clone(),
                source_entity_ids: part.original.source_entity_ids.clone(),
            };

            sheets.push(new_sheet);
            sheet_placements.push(vec![placement]);
            placed_flags[i] = true;
        }
    }

    let placed_count = placed_flags.iter().filter(|&&f| f).count();

    if let Some(ref mut stream_cb) = on_stream {
        let partial_sheets: Vec<SheetOutput> = sheet_placements.iter().enumerate().map(|(idx, placements)| {
            let used_area: f64 = placements.iter().map(|p| p.area).sum();
            let util = if board_area > 0.0 { (used_area / board_area) * 100.0 } else { 0.0 };
            SheetOutput {
                index: idx,
                width: board_width,
                height: board_height,
                utilization: (util * 10.0).round() / 10.0,
                used_percent: (util * 10.0).round() / 10.0,
                placements: placements.clone(),
            }
        }).collect();

        let total_used: f64 = partial_sheets.iter().map(|s| s.placements.iter().map(|p| p.area).sum::<f64>()).sum();
        let total_board = (partial_sheets.len() as f64) * board_area;
        let avg_used = if total_board > 0.0 { (total_used / total_board) * 100.0 } else { 0.0 };

        let mut snap_layout = LayoutOutput {
            key: material_state.key.clone(),
            material: material_name.clone(),
            thickness,
            group_index: material_state.group_index,
            part_count: total_parts,
            placed_count,
            board_width,
            board_height,
            edge_margin,
            cut_gap: part_spacing,
            part_spacing,
            used_percent: (avg_used * 10.0).round() / 10.0,
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
            comb_pair_placements: 0,
            staircase_pattern_placements: 0,
            priority_zone_fill_placements: 0,
            pocket_evacuated_sheets: 0,
            priority_zone_fill_area: 0.0,
            safety_certified: true,
            safety_version: 1,
        };
        expand_layout_clusters(&mut snap_layout);
        stream_cb(&snap_layout);
    }

    let mut sheet_outputs = Vec::new();

    for (idx, placements) in sheet_placements.into_iter().enumerate() {
        let used_area: f64 = placements.iter().map(|p| p.area).sum();
        let util = if board_area > 0.0 { (used_area / board_area) * 100.0 } else { 0.0 };

        sheet_outputs.push(SheetOutput {
            index: idx,
            width: board_width,
            height: board_height,
            utilization: (util * 10.0).round() / 10.0,
            used_percent: (util * 10.0).round() / 10.0,
            placements,
        });
    }

    let total_used: f64 = sheet_outputs.iter().map(|s| s.placements.iter().map(|p| p.area).sum::<f64>()).sum();
    let total_board = (sheet_outputs.len() as f64) * board_area;
    let avg_used = if total_board > 0.0 { (total_used / total_board) * 100.0 } else { 0.0 };

    let mut layout = LayoutOutput {
        key: material_state.key.clone(),
        material: material_name,
        thickness,
        group_index: material_state.group_index,
        part_count: total_parts,
        placed_count,
        board_width,
        board_height,
        edge_margin,
        cut_gap: part_spacing,
        part_spacing,
        used_percent: (avg_used * 10.0).round() / 10.0,
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
        comb_pair_placements: 0,
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
