// src/epochs.rs
use rayon::prelude::*;
use crate::geometry::{Polygon, rotate_shape_full};
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
    let board_area = board_w * board_h;

    let mut changed = false;
    let mut max_rounds = 10;

    while max_rounds > 0 {
        max_rounds -= 1;
        let mut removed_sheet = false;

        if layout.sheets.len() <= 1 {
            break;
        }

        // Rebuild contexts for all current sheets
        let mut contexts: Vec<SheetContext> = layout.sheets.iter().map(|s| {
            let mut ctx = SheetContext::new(
                board_w,
                board_h,
                margin,
                spacing,
                sheet_in_sheet,
                compact_directions.to_vec(),
            );
            for p in &s.placements {
                let poly = Polygon::from_raw(&p.contour);
                ctx.add_placed(poly, p.holes.clone(), p.x, p.y, p.rotation_degrees);
            }
            ctx
        }).collect();

        // Iterate from last sheet backwards
        for src_idx in (1..layout.sheets.len()).rev() {
            if layout.sheets[src_idx].placements.is_empty() {
                layout.sheets.remove(src_idx);
                contexts.remove(src_idx);
                removed_sheet = true;
                changed = true;
                break;
            }

            let mut remaining_placements = Vec::new();
            let src_placements = std::mem::take(&mut layout.sheets[src_idx].placements);

            // Try moving each placement from src_sheet to an earlier sheet (0..src_idx)
            for p in src_placements.into_iter() {
                let base_rot = p.base_rotation_degrees.unwrap_or(0.0);
                let is_grain = global_rot_div == 1 || p.has_grain_label == Some(true);

                let allowed_rotations = if is_grain {
                    vec![
                        norm_angle(base_rot + 0.0),
                        norm_angle(base_rot + 180.0),
                    ]
                } else {
                    vec![
                        norm_angle(base_rot + 0.0),
                        norm_angle(base_rot + 90.0),
                        norm_angle(base_rot + 180.0),
                        norm_angle(base_rot + 270.0),
                    ]
                };

                let mut moved = false;

                for target_idx in 0..src_idx {
                    let ctx = &mut contexts[target_idx];

                    for &rot in &allowed_rotations {
                        let (norm_poly, norm_holes, norm_draw_layers) = rotate_shape_full(
                            &p.contour,
                            p.holes.as_ref(),
                            p.draw_layers.as_ref(),
                            rot,
                        );
                        let bbox = norm_poly.bounding_box();
                        let is_rect = norm_poly.is_rectangular();
                        let dilated_poly = crate::geometry::offset_polygon(&norm_poly, spacing * 0.5);
                        let dilated_bbox = dilated_poly.bounding_box();

                        if let Some((px, py, _)) = ctx.find_best_anchor(
                            &norm_poly,
                            &dilated_poly,
                            &dilated_bbox,
                            is_rect,
                            1.0,
                        ) {
                            ctx.add_placed(norm_poly.clone(), norm_holes.clone(), px, py, rot);
                            let mut new_p = p.clone();
                            new_p.x = px;
                            new_p.y = py;
                            new_p.rotation_degrees = rot;
                            new_p.packed_width = bbox.width();
                            new_p.packed_height = bbox.height();
                            new_p.contour = norm_poly.to_raw();
                            new_p.holes = norm_holes;
                            new_p.draw_layers = norm_draw_layers;

                            layout.sheets[target_idx].placements.push(new_p);
                            moved = true;
                            changed = true;
                            break;
                        }
                    }
                    if moved {
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
                contexts.remove(src_idx);
                removed_sheet = true;
                changed = true;
                break;
            } else {
                layout.sheets[src_idx].placements = remaining_placements;
            }
        }

        if !removed_sheet {
            break;
        }
    }

    // Recompute sheet metadata
    for (idx, sheet) in layout.sheets.iter_mut().enumerate() {
        sheet.index = idx;
        sheet.width = board_w;
        sheet.height = board_h;
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
    F: FnMut(usize, &LayoutOutput),
{
    let config = material_state.configuration.as_ref();
    let global_rot_div = config.and_then(|c| c.rotation_divisions).unwrap_or(1);
    let sheet_in_sheet = config.and_then(|c| c.sheet_in_sheet).unwrap_or(false);
    let compact_directions = config.and_then(|c| c.compact_directions.clone()).unwrap_or_else(|| vec!["left".to_string(), "bottom".to_string()]);

    // 1. Initial baseline with progressive sheet-by-sheet live streaming
    let mut progressive_step = 1;
    let mut initial_layout = pack_material_streaming(
        material_state,
        0,
        Some(|partial_layout: &LayoutOutput| {
            on_progress(progressive_step, partial_layout);
            progressive_step += 1;
        }),
    );

    // Consolidate initial layout
    consolidate_sheets(&mut initial_layout, global_rot_div, sheet_in_sheet, &compact_directions);
    on_progress(progressive_step, &initial_layout);
    let mut best_layout = initial_layout;

    // 2. Parallel multi-strategy exploration
    let candidate_layouts: Vec<LayoutOutput> = (1..6)
        .into_par_iter()
        .map(|strategy_id| {
            let mut layout = pack_material(material_state, strategy_id);
            consolidate_sheets(&mut layout, global_rot_div, sheet_in_sheet, &compact_directions);
            layout
        })
        .collect();

    for (idx, layout) in candidate_layouts.into_iter().enumerate() {
        let score_cand = LayoutScore::compute(&layout);
        let score_best = LayoutScore::compute(&best_layout);
        if score_cand < score_best {
            best_layout = layout;
            on_progress(progressive_step + 1 + idx, &best_layout);
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

    crate::packing::expand_layout_clusters(&mut best_layout);

    best_layout
}
