// src/epochs.rs
use rayon::prelude::*;
use crate::geometry::{Point, Polygon};
use crate::nfp::SheetContext;
use crate::packing::{pack_material, pack_material_streaming, norm_angle};
use crate::scoring::LayoutScore;
use crate::types::{LayoutOutput, MaterialStateInput};

pub fn consolidate_sheets(
    layout: &mut LayoutOutput,
    global_rot_div: usize,
    sheet_in_sheet: bool,
    compact_directions: &[String],
) -> bool {
    let board_w = layout.board_width;
    let board_h = layout.board_height;
    let margin = layout.edge_margin;
    let spacing = layout.part_spacing;
    let small_part_edge_zone = layout.small_part_edge_zone;
    let merge_cut_paths = layout.merge_cut_paths;
    let board_area = board_w * board_h;

    let mut changed = false;

    if layout.sheets.len() > 1 {

        let last_idx = layout.sheets.len() - 1;
        let last_sheet_area: f64 = layout.sheets[last_idx].placements.iter().map(|p| p.area).sum();
        let total_free_area: f64 = layout.sheets[0..last_idx].iter().map(|s| {
            let used: f64 = s.placements.iter().map(|p| p.area).sum();
            (board_area - used).max(0.0)
        }).sum();

        if total_free_area < last_sheet_area {
            return false;
        }

        // Rebuild contexts for all current sheets
        let mut contexts: Vec<SheetContext> = layout.sheets.iter().map(|s| {
            let mut ctx = SheetContext::new(
                board_w,
                board_h,
                margin,
                spacing,
                small_part_edge_zone,
                merge_cut_paths,
                sheet_in_sheet,
                compact_directions.to_vec(),
            );
            for p in &s.placements {
                let poly = Polygon::from_raw(&p.contour);
                let holes_vec: Vec<Polygon> = p.holes.as_ref().map_or(Vec::new(), |raw_holes| {
                    raw_holes.iter().map(|raw| Polygon::from_raw(raw)).collect()
                });
                ctx.add_placed(
                    poly,
                    holes_vec,
                    p.x,
                    p.y,
                    p.rotation_degrees,
                    p.small_part.unwrap_or(false),
                    p.small_part_clearance.unwrap_or(0.0),
                );
            }
            ctx
        }).collect();

        let mut sheet_used_area: Vec<f64> = layout.sheets.iter().map(|s| {
            s.placements.iter().map(|p| p.area).sum()
        }).collect();

        let src_idx = last_idx;
        if layout.sheets[src_idx].placements.is_empty() {
            layout.sheets.remove(src_idx);
            changed = true;
        } else {
            let mut remaining_placements = Vec::new();
            let src_placements = std::mem::take(&mut layout.sheets[src_idx].placements);

            // Try moving each placement from src_sheet to an earlier sheet (0..src_idx)
            for p in src_placements.into_iter() {
                let poly = Polygon::from_raw(&p.contour);
                let p_area = p.area;
                let curr_rot = p.rotation_degrees;
                let is_grain = (global_rot_div <= 2)
                    || (p.has_grain_label == Some(true))
                    || (p.grain_locked == Some(true))
                    || (p.manual_cluster_macro == Some(true));

                let allowed_rotations = if is_grain {
                    if global_rot_div == 1 && p.has_grain_label != Some(true) && p.grain_locked != Some(true) {
                        vec![0.0]
                    } else {
                        vec![0.0, 180.0]
                    }
                } else {
                    vec![0.0, 90.0, 180.0, 270.0]
                };

                let mut moved = false;

                for target_idx in 0..src_idx {
                    if board_area - sheet_used_area[target_idx] < p_area * 1.05 {
                        continue;
                    }
                    let ctx = &mut contexts[target_idx];

                    for &delta_rot in &allowed_rotations {
                        let rotated = poly.rotate_degrees(delta_rot, Point::new(0.0, 0.0));
                        let (norm_poly, shift_x, shift_y) = rotated.normalize_to_origin();
                        let bbox = norm_poly.bounding_box();
                        let norm_holes: Vec<Polygon> = p.holes.as_ref().map_or(Vec::new(), |raw_holes| {
                            raw_holes.iter().map(|raw| {
                                let h_poly = Polygon::from_raw(raw);
                                let h_rot = h_poly.rotate_degrees(delta_rot, Point::new(0.0, 0.0));
                                h_rot.translate(shift_x, shift_y)
                            }).collect()
                        });
                        let is_rect = norm_poly.is_rectangular() && norm_holes.is_empty();
                        let dilated_poly = crate::geometry::offset_polygon(&norm_poly, spacing * 0.5);
                        let dilated_bbox = dilated_poly.bounding_box();

                        if let Some((px, py, _)) = ctx.find_best_anchor(
                            &norm_poly,
                            &dilated_poly,
                            &dilated_bbox,
                            is_rect,
                            p.small_part.unwrap_or(false),
                            p.small_part_clearance.unwrap_or(0.0),
                            1.0,
                        ) {
                            let new_rot = norm_angle(curr_rot + delta_rot);
                            ctx.add_placed(
                                norm_poly.clone(),
                                norm_holes.clone(),
                                px,
                                py,
                                new_rot,
                                p.small_part.unwrap_or(false),
                                p.small_part_clearance.unwrap_or(0.0),
                            );
                            let mut new_p = p.clone();
                            new_p.x = px;
                            new_p.y = py;
                            new_p.rotation_degrees = new_rot;
                            new_p.packed_width = bbox.width();
                            new_p.packed_height = bbox.height();
                            new_p.contour = norm_poly.to_raw();
                            new_p.holes = if norm_holes.is_empty() { None } else { Some(norm_holes.iter().map(|h| h.to_raw()).collect()) };
                            new_p.small_part_edge_protected = Some(ctx.small_part_edge_protected(
                                p.small_part.unwrap_or(false),
                                px,
                                py,
                                bbox.width(),
                                bbox.height(),
                            ));
                            let norm_layers: Option<Vec<crate::types::DrawLayerData>> = p.draw_layers.as_ref().map(|layers| {
                                let rad = delta_rot * std::f64::consts::PI / 180.0;
                                layers.iter().map(|layer| {
                                    let norm_paths = layer.paths.as_ref().map(|paths| {
                                        paths.iter().map(|path| {
                                            let rot_pts = path.points.iter().map(|pt_raw| {
                                                let pt = Point::new(pt_raw[0], pt_raw[1]).rotate(rad, Point::new(0.0, 0.0));
                                                [(pt.x + shift_x).round(), (pt.y + shift_y).round()]
                                            }).collect();
                                            crate::types::DrawPathData {
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
                                    crate::types::DrawLayerData {
                                        name: layer.name.clone(),
                                        layer_name: layer.layer_name.clone(),
                                        color: layer.color.clone(),
                                        side: layer.side.clone(),
                                        r#type: layer.r#type.clone(),
                                        paths: norm_paths,
                                    }
                                }).collect()
                            });
                            new_p.draw_layers = norm_layers;

                            layout.sheets[target_idx].placements.push(new_p);
                            moved = true;
                            changed = true;
                            break;
                        }
                    }
                    if moved {
                        sheet_used_area[target_idx] += p_area;
                        sheet_used_area[src_idx] -= p_area;
                        break;
                    }
                }

                if !moved {
                    remaining_placements.push(p);
                }
            }

            if remaining_placements.is_empty() {
                // Whole sheet absorbed! Remove it
                layout.sheets.remove(src_idx);
                changed = true;
            } else {
                layout.sheets[src_idx].placements = remaining_placements;
            }
        }
    }

    // Recompute sheet metadata
    for (idx, sheet) in layout.sheets.iter_mut().enumerate() {
        sheet.index = idx + 1;
        let used_area: f64 = sheet.placements.iter().map(|p| p.area).sum();
        let util = if board_area > 0.0 { (used_area / board_area) * 100.0 } else { 0.0 };
        sheet.utilization = (util * 10.0).round() / 10.0;
        sheet.used_percent = (util * 10.0).round() / 10.0;
    }

    changed
}

pub fn optimize_material_layout<F>(
    material_state: &MaterialStateInput,
    mut on_progress: F,
) -> LayoutOutput
where
    F: FnMut(&str, f64, usize, usize, &LayoutOutput),
{
    let config = material_state.configuration.as_ref();
    let global_rot_div = config.and_then(|c| c.rotation_divisions).unwrap_or(1);
    let sheet_in_sheet = config.and_then(|c| c.sheet_in_sheet).unwrap_or(false);
    let compact_directions = config.and_then(|c| c.compact_directions.clone()).unwrap_or_else(|| vec!["left".to_string(), "bottom".to_string()]);

    let config_obj = material_state.configuration.clone().unwrap_or_default();
    let part_spacing = config_obj.part_spacing.or(config_obj.cut_gap).unwrap_or(6.0);

    // Phase 1: Pre-pair comb parts into rectangular macros ONCE for the entire material
    let (paired_parts, _comb_count) = crate::macro_comb::pair_comb_parts(
        &material_state.parts,
        &config_obj,
        part_spacing,
    );
    let mut paired_material_state = material_state.clone();
    paired_material_state.parts = paired_parts;
    let total_parts = paired_material_state.parts.len();

    // 1. Initial baseline with progressive sheet-by-sheet live streaming
    let mut progressive_sheet = 0;
    let mut initial_layout = pack_material_streaming(
        &paired_material_state,
        0,
        Some(|snap: &LayoutOutput| {
            progressive_sheet = snap.sheets.len();
            let placed_count: usize = snap.sheets.iter().map(|s| s.placements.len()).sum();
            let pct = if total_parts > 0 {
                (placed_count as f64 / total_parts as f64) * 60.0
            } else {
                30.0
            };
            on_progress("Xếp sơ đồ ván", pct, progressive_sheet, 0, snap);
        }),
    );

    consolidate_sheets(&mut initial_layout, global_rot_div, sheet_in_sheet, &compact_directions);
    on_progress("Tối ưu sơ bộ", 60.0, initial_layout.sheets.len(), 0, &initial_layout);
    let mut best_layout = initial_layout;

    // 2. Parallel multi-strategy exploration (bounded 4-8 strategies for high speed)
    let num_threads = rayon::current_num_threads();
    let num_strategies = num_threads.clamp(4, 8);

    let (tx, rx) = std::sync::mpsc::channel();
    (1..=num_strategies).into_par_iter().for_each_with(tx, |s, strategy_id| {
        let mut layout = pack_material(&paired_material_state, strategy_id);
        consolidate_sheets(&mut layout, global_rot_div, sheet_in_sheet, &compact_directions);
        let _ = s.send((strategy_id, layout));
    });

    let mut completed_count = 0;
    while let Ok((_strat_id, layout)) = rx.recv() {
        completed_count += 1;
        let score_cand = LayoutScore::compute(&layout);
        let score_best = LayoutScore::compute(&best_layout);
        let strategy_pct = 60.0 + (completed_count as f64 / num_strategies as f64) * 35.0;
        if score_cand < score_best {
            best_layout = layout;
            on_progress("Tối ưu đa luồng", strategy_pct, completed_count, num_strategies, &best_layout);
        } else {
            on_progress("Tối ưu đa luồng", strategy_pct, completed_count, num_strategies, &best_layout);
        }
    }

    // 3. Final consolidation & scoring
    consolidate_sheets(&mut best_layout, global_rot_div, sheet_in_sheet, &compact_directions);

    let final_score = LayoutScore::compute(&best_layout);
    best_layout.waste_area = final_score.waste_area;
    best_layout.compact_area = final_score.compact_area;
    best_layout.last_sheet_compact_area = final_score.last_sheet_compact_area;
    best_layout.front_load_score = final_score.front_load_score;
    best_layout.target_deficit_score = final_score.target_deficit;

    best_layout
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{PlacementOutput, SheetOutput};

    #[test]
    fn test_consolidate_sheets_strictly_preserves_grain_rotation() {
        let contour = vec![
            [0.0, 0.0],
            [400.0, 0.0],
            [400.0, 200.0],
            [0.0, 200.0],
        ];

        let p_sheet1 = PlacementOutput {
            entity_id: "e1".to_string(),
            id: "1".to_string(),
            name: Some("Part 1".to_string()),
            x: 10.0,
            y: 10.0,
            rotation_degrees: 0.0,
            base_rotation_degrees: Some(0.0),
            width: 400.0,
            height: 200.0,
            packed_width: 400.0,
            packed_height: 200.0,
            area: 80000.0,
            contour: contour.clone(),
            holes: None,
            draw_layers: None,
            color: None,
            manual_cluster_macro: None,
            manual_cluster_children: None,
            logical_part_count: Some(1),
            has_grain_label: Some(true),
            grain_arrow_degrees: Some(0.0),
            grain_locked: Some(true),
            small_part: Some(false),
            small_part_clearance: Some(0.0),
            small_part_edge_protected: Some(false),
        };

        let mut p_sheet2 = p_sheet1.clone();
        p_sheet2.entity_id = "e2".to_string();
        p_sheet2.id = "2".to_string();

        let mut layout = LayoutOutput {
            material: "MDF".to_string(),
            thickness: 17.0,
            board_width: 2440.0,
            board_height: 1220.0,
            edge_margin: 10.0,
            part_spacing: 6.0,
            cut_gap: Some(6.0),
            small_part_threshold: 0.0,
            small_part_clearance: 0.0,
            small_part_edge_zone: 0.0,
            merge_cut_paths: false,
            sheets: vec![
                SheetOutput {
                    index: 1,
                    utilization: 10.0,
                    used_percent: 10.0,
                    placements: vec![p_sheet1],
                },
                SheetOutput {
                    index: 2,
                    utilization: 10.0,
                    used_percent: 10.0,
                    placements: vec![p_sheet2],
                },
            ],
            compact_directions: vec!["left".to_string(), "bottom".to_string()],
            target_sheet_utilization: 90.0,
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
            safety_certified: false,
            safety_version: 1,
        };

        // Even with global_rot_div = 4 (free rotation), grain-locked part consolidated from Sheet 2 to Sheet 1
        // MUST ONLY be rotated 0 or 180 degrees, NEVER 90 or 270!
        let compact_dirs = vec!["left".to_string(), "bottom".to_string()];
        consolidate_sheets(&mut layout, 4, false, &compact_dirs);

        assert_eq!(layout.sheets.len(), 1, "Sheet 2 should be consolidated into Sheet 1");
        for p in &layout.sheets[0].placements {
            let rot = p.rotation_degrees % 360.0;
            assert!(
                (rot - 0.0).abs() < 1e-3 || (rot - 180.0).abs() < 1e-3,
                "Consolidated placement {} has invalid rotation {}. Grain parts must ONLY be 0 or 180 deg!",
                p.id,
                rot
            );
        }
    }
}
