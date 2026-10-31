// src/epochs.rs
use rayon::prelude::*;
use crate::geometry::{Point, Polygon};
use crate::nfp::SheetContext;
use crate::packing::{pack_material, norm_angle};
use crate::scoring::LayoutScore;
use crate::types::{LayoutOutput, MaterialStateInput, SheetOutput};

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
            let mut failed_target_keys: Vec<std::collections::HashSet<u64>> = vec![std::collections::HashSet::new(); src_idx];

            // Try moving each placement from src_sheet to an earlier sheet (0..src_idx)
            for p in src_placements.into_iter() {
                let poly = Polygon::from_raw(&p.contour);
                let p_area = p.area;
                let curr_rot = p.rotation_degrees;
                let has_direct_grain = p.has_grain_label == Some(true)
                    || p.grain_locked == Some(true)
                    || p.grain_arrow_degrees.is_some();
                let is_free = p.free_rotation == Some(true) || global_rot_div >= 4;
                let is_grain = if is_free {
                    false
                } else {
                    (global_rot_div <= 2)
                        || has_direct_grain
                        || (p.manual_cluster_macro == Some(true))
                };

                let allowed_rotations = if is_grain {
                    if global_rot_div == 1 && !has_direct_grain { vec![0.0] } else { vec![0.0, 180.0] }
                } else {
                    vec![0.0, 90.0, 180.0, 270.0]
                };

                let p_shape_key = {
                    use std::hash::{Hash, Hasher};
                    let mut h = std::collections::hash_map::DefaultHasher::new();
                    is_grain.hash(&mut h);
                    p.contour.len().hash(&mut h);
                    ((p.packed_width * 10.0).round() as i64).hash(&mut h);
                    ((p.packed_height * 10.0).round() as i64).hash(&mut h);
                    (p_area.round() as i64).hash(&mut h);
                    h.finish()
                };

                let mut moved = false;
                let p_room = crate::types::get_placement_room_key(&p);
                let mut target_order: Vec<usize> = (0..src_idx).collect();
                if !p_room.is_empty() {
                    target_order.sort_by_key(|&t_idx| {
                        let has_same = layout.sheets[t_idx].placements.iter().any(|ep| crate::types::get_placement_room_key(ep) == p_room);
                        let has_other = layout.sheets[t_idx].placements.iter().any(|ep| {
                            let er = crate::types::get_placement_room_key(ep);
                            !er.is_empty() && er != p_room
                        });
                        if has_same && !has_other {
                            0usize
                        } else if has_same {
                            1usize
                        } else {
                            2usize
                        }
                    });
                }

                for target_idx in target_order {
                    if board_area - sheet_used_area[target_idx] < p_area * 1.05 {
                        continue;
                    }
                    if failed_target_keys[target_idx].contains(&p_shape_key) {
                        continue;
                    }
                    let ctx = &mut contexts[target_idx];

                    for &delta_rot in &allowed_rotations {
                        let new_rot = norm_angle(curr_rot + delta_rot);
                        if is_grain {
                            let base_rot = norm_angle(p.base_rotation_degrees.unwrap_or(0.0));
                            let diff = (new_rot - base_rot).abs() % 180.0;
                            if diff > 2.0 && diff < 178.0 {
                                continue;
                            }
                        }
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

                        let best_pos = if sheet_in_sheet && ctx.has_holes_or_pockets() {
                            ctx.find_best_hole_or_pocket_anchor(
                                &norm_poly, &dilated_poly, &dilated_bbox, is_rect,
                                p.small_part.unwrap_or(false), p.small_part_clearance.unwrap_or(0.0), 1.0, true,
                            ).or_else(|| ctx.find_best_anchor(
                                &norm_poly, &dilated_poly, &dilated_bbox, is_rect,
                                p.small_part.unwrap_or(false), p.small_part_clearance.unwrap_or(0.0), 1.0,
                            ))
                        } else {
                            ctx.find_best_anchor(
                                &norm_poly, &dilated_poly, &dilated_bbox, is_rect,
                                p.small_part.unwrap_or(false), p.small_part_clearance.unwrap_or(0.0), 1.0,
                            )
                        };
                        if let Some((px, py, _)) = best_pos {
                            ctx.add_placed(
                                norm_poly.clone(), norm_holes.clone(), px, py, new_rot,
                                p.small_part.unwrap_or(false), p.small_part_clearance.unwrap_or(0.0),
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
                                p.small_part.unwrap_or(false), px, py, bbox.width(), bbox.height(),
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
                                                points: rot_pts, closed: path.closed, surface_face: path.surface_face,
                                                surface_face_id: path.surface_face_id.clone(), surface_loop_type: path.surface_loop_type.clone(),
                                                geometry_kind: path.geometry_kind.clone(), _nesting_packed_local: Some(true),
                                            }
                                        }).collect()
                                    });
                                    crate::types::DrawLayerData {
                                        name: layer.name.clone(), layer_name: layer.layer_name.clone(),
                                        color: layer.color.clone(), side: layer.side.clone(),
                                        r#type: layer.r#type.clone(), paths: norm_paths,
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
                        failed_target_keys[target_idx].clear();
                        break;
                    } else {
                        failed_target_keys[target_idx].insert(p_shape_key);
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

fn prioritize_two_sided_sheets(sheets: &mut Vec<SheetOutput>) {
    sheets.sort_by(|a, b| {
        let a_2s = a.placements.iter().any(|p| p.two_sided == Some(true));
        let b_2s = b.placements.iter().any(|p| p.two_sided == Some(true));
        if a_2s != b_2s {
            return b_2s.cmp(&a_2s);
        }
        let a_count = a.placements.iter().filter(|p| p.two_sided == Some(true)).count();
        let b_count = b.placements.iter().filter(|p| p.two_sided == Some(true)).count();
        if a_count != b_count {
            return b_count.cmp(&a_count);
        }
        a.index.cmp(&b.index)
    });
    for (i, sheet) in sheets.iter_mut().enumerate() {
        sheet.index = i + 1;
    }
}

fn collect_placed_source_keys(sheets: &[SheetOutput]) -> std::collections::BTreeSet<String> {
    let mut keys = std::collections::BTreeSet::new();
    for s in sheets {
        for p in &s.placements {
            if !p.entity_id.is_empty() {
                keys.insert(format!("e:{}", p.entity_id));
            }
            if !p.id.is_empty() {
                keys.insert(format!("i:{}", p.id));
            }
            if let Some(children) = &p.manual_cluster_children {
                for c in children {
                    if let Some(eid) = &c.entity_id {
                        if !eid.is_empty() {
                            keys.insert(format!("e:{}", eid));
                        }
                    }
                    if let Some(pid) = &c.id {
                        if !pid.is_empty() {
                            keys.insert(format!("i:{}", pid));
                        }
                    }
                }
            }
        }
    }
    keys
}

fn is_part_in_placed_keys(part: &crate::types::PartInput, keys: &std::collections::BTreeSet<String>) -> bool {
    if let Some(eid) = &part.entity_id {
        if !eid.is_empty() && keys.contains(&format!("e:{}", eid)) {
            return true;
        }
    }
    if let Some(pid) = &part.id {
        if !pid.is_empty() && keys.contains(&format!("i:{}", pid)) {
            return true;
        }
    }
    if let Some(children) = &part.manual_cluster_children {
        if !children.is_empty() && children.iter().all(|c| {
            c.entity_id.as_ref().map_or(false, |e| !e.is_empty() && keys.contains(&format!("e:{}", e)))
                || c.id.as_ref().map_or(false, |i| !i.is_empty() && keys.contains(&format!("i:{}", i)))
        }) {
            return true;
        }
    }
    false
}

fn pack_sub_state_best(
    sub_state: &MaterialStateInput,
    strategies: &[usize],
    global_rot_div: usize,
    sheet_in_sheet: bool,
    compact_directions: &[String],
) -> LayoutOutput {
    let config = sub_state.configuration.clone().unwrap_or_default();
    let part_spacing = config.part_spacing.or(config.cut_gap).unwrap_or(6.0);
    let existing_macro_count = sub_state.parts.iter().filter(|p| p.manual_cluster_macro == Some(true)).count();
    let (paired_parts, new_comb_count) = if existing_macro_count > 0 {
        (sub_state.parts.clone(), 0)
    } else {
        crate::macro_comb::pair_comb_parts(&sub_state.parts, &config, part_spacing)
    };
    let comb_count = existing_macro_count + new_comb_count;
    let prepared = crate::packing::prepare_parts(&paired_parts, &config, part_spacing);

    let has_room_strat = strategies.iter().any(|&s| s >= 100);
    let mut best: Option<LayoutOutput> = None;
    for &strat_id in strategies {
        let mut cand = crate::packing::pack_material_with_prepared(sub_state, &prepared, comb_count, strat_id);
        consolidate_sheets(&mut cand, global_rot_div, sheet_in_sheet, compact_directions);
        prioritize_two_sided_sheets(&mut cand.sheets);
        match &best {
            None => best = Some(cand),
            Some(curr) => {
                let take = if has_room_strat {
                    let max_s = curr.sheets.len().max(cand.sheets.len());
                    let max_uf = crate::scoring::effective_room_underfilled(curr).max(crate::scoring::effective_room_underfilled(&cand));
                    crate::scoring::is_room_candidate_better(&cand, curr, max_s, max_uf)
                } else {
                    LayoutScore::compute(&cand) < LayoutScore::compute(curr)
                };
                if take {
                    best = Some(cand);
                }
            }
        }
    }
    best.unwrap_or_else(|| crate::packing::pack_material_with_prepared(sub_state, &prepared, comb_count, 0))
}

fn pack_sub_state_parallel(
    sub_state: &MaterialStateInput,
    strategies: &[usize],
    global_rot_div: usize,
    sheet_in_sheet: bool,
    compact_directions: &[String],
) -> LayoutOutput {
    let config = sub_state.configuration.clone().unwrap_or_default();
    let part_spacing = config.part_spacing.or(config.cut_gap).unwrap_or(6.0);
    let existing_macro_count = sub_state.parts.iter().filter(|p| p.manual_cluster_macro == Some(true)).count();
    let (paired_parts, new_comb_count) = if existing_macro_count > 0 {
        (sub_state.parts.clone(), 0)
    } else {
        crate::macro_comb::pair_comb_parts(&sub_state.parts, &config, part_spacing)
    };
    let comb_count = existing_macro_count + new_comb_count;
    let prepared = crate::packing::prepare_parts(&paired_parts, &config, part_spacing);

    let cands: Vec<LayoutOutput> = strategies
        .par_iter()
        .map(|&strat_id| {
            let mut cand = crate::packing::pack_material_with_prepared(sub_state, &prepared, comb_count, strat_id);
            consolidate_sheets(&mut cand, global_rot_div, sheet_in_sheet, compact_directions);
            prioritize_two_sided_sheets(&mut cand.sheets);
            cand
        })
        .collect();

    let has_room_strat = strategies.iter().any(|&s| s >= 100);
    let mut best: Option<LayoutOutput> = None;
    for cand in cands {
        match &best {
            None => best = Some(cand),
            Some(curr) => {
                let take = if has_room_strat {
                    let max_s = curr.sheets.len().max(cand.sheets.len());
                    let max_uf = crate::scoring::effective_room_underfilled(curr).max(crate::scoring::effective_room_underfilled(&cand));
                    crate::scoring::is_room_candidate_better(&cand, curr, max_s, max_uf)
                } else {
                    LayoutScore::compute(&cand) < LayoutScore::compute(curr)
                };
                if take {
                    best = Some(cand);
                }
            }
        }
    }
    best.unwrap_or_else(|| crate::packing::pack_material_with_prepared(sub_state, &prepared, comb_count, 0))
}

#[allow(dead_code)]
fn try_room_subbatched_layout(
    material_state: &MaterialStateInput,
    paired_material_state: &MaterialStateInput,
    baseline_layout: &LayoutOutput,
    global_rot_div: usize,
    sheet_in_sheet: bool,
    compact_directions: &[String],
    start_mat: &std::time::Instant,
) -> Option<LayoutOutput> {
    use std::collections::BTreeMap;

    if material_state.parts.len() < 2 || baseline_layout.sheets.is_empty() {
        return None;
    }

    let mut room_buckets: BTreeMap<String, Vec<crate::types::PartInput>> = BTreeMap::new();
    let mut unassigned_parts: Vec<crate::types::PartInput> = Vec::new();

    for p in &material_state.parts {
        let r = crate::types::get_part_room_key(p);
        if r.is_empty() {
            unassigned_parts.push(p.clone());
        } else {
            room_buckets.entry(r).or_default().push(p.clone());
        }
    }

    if room_buckets.len() < 2 {
        return None;
    }

    let baseline_sheets = baseline_layout.sheets.len();
    let baseline_underfilled = LayoutScore::compute(baseline_layout).underfilled_sheets;
    let mut best_cand = baseline_layout.clone();
    let mut improved = false;

    let config_obj = material_state.configuration.clone().unwrap_or_default();
    let part_spacing = config_obj.part_spacing.or(config_obj.cut_gap).unwrap_or(6.0);
    let board_area = baseline_layout.board_width * baseline_layout.board_height;

    let mut ordered_rooms: Vec<(String, Vec<crate::types::PartInput>, i64)> = room_buckets
        .into_iter()
        .map(|(k, parts)| {
            let area_sum: f64 = parts
                .iter()
                .map(|p| p.area.unwrap_or_else(|| Polygon::from_raw(&p.contour).area()))
                .sum();
            (k, parts, area_sum.round() as i64)
        })
        .collect();
    ordered_rooms.sort_by(|a, b| b.2.cmp(&a.2).then_with(|| a.0.cmp(&b.0)));

    let mut per_room_results: Vec<(String, Vec<crate::types::PartInput>, LayoutOutput)> =
        Vec::with_capacity(ordered_rooms.len());

    for (r_key, r_parts, _area) in ordered_rooms {
        let (paired_r_parts, _) = crate::macro_comb::pair_comb_parts(&r_parts, &config_obj, part_spacing);
        let mut r_state = material_state.clone();
        r_state.parts = paired_r_parts;
        let r_layout = pack_sub_state_best(&r_state, &[0, 1, 2, 4], global_rot_div, sheet_in_sheet, compact_directions);
        per_room_results.push((r_key, r_parts, r_layout));
    }

    for lock_mode in [0usize, 1usize] {
        if start_mat.elapsed().as_secs_f64() >= 9.4 {
            break;
        }
        let mut locked_sheets: Vec<SheetOutput> = Vec::new();
        let mut tail_pool: Vec<crate::types::PartInput> = Vec::new();

        for (_r_key, r_orig_parts, r_layout) in &per_room_results {
            let s_len = r_layout.sheets.len();
            let keep_count = if s_len > 1 {
                s_len - 1
            } else if s_len == 1 && lock_mode == 0 && r_layout.sheets[0].used_percent >= 78.0 {
                1
            } else {
                0
            };

            if keep_count > 0 {
                let kept_slice = &r_layout.sheets[0..keep_count];
                for s in kept_slice {
                    locked_sheets.push(s.clone());
                }
                let placed_keys = collect_placed_source_keys(kept_slice);
                for orig_p in r_orig_parts {
                    if !is_part_in_placed_keys(orig_p, &placed_keys) {
                        tail_pool.push(orig_p.clone());
                    }
                }
            } else {
                for orig_p in r_orig_parts {
                    tail_pool.push(orig_p.clone());
                }
            }
        }

        for u_p in &unassigned_parts {
            tail_pool.push(u_p.clone());
        }

        let mut merged_sheets = locked_sheets.clone();
        if !tail_pool.is_empty() {
            let (paired_tail, _) = crate::macro_comb::pair_comb_parts(&tail_pool, &config_obj, part_spacing);
            let mut tail_state = material_state.clone();
            tail_state.parts = paired_tail;
            let tail_layout = pack_sub_state_best(
                &tail_state,
                &[100, 101, 0, 1, 4],
                global_rot_div,
                sheet_in_sheet,
                compact_directions,
            );
            for ts in &tail_layout.sheets {
                merged_sheets.push(ts.clone());
            }
        }

        for (idx, s) in merged_sheets.iter_mut().enumerate() {
            s.index = idx + 1;
            let used_area: f64 = s.placements.iter().map(|p| p.area).sum();
            let util = if board_area > 0.0 { (used_area / board_area) * 100.0 } else { 0.0 };
            s.utilization = (util * 10.0).round() / 10.0;
            s.used_percent = (util * 10.0).round() / 10.0;
        }

        let mut cand_layout = baseline_layout.clone();
        cand_layout.sheets = merged_sheets;
        consolidate_sheets(&mut cand_layout, global_rot_div, sheet_in_sheet, compact_directions);
        prioritize_two_sided_sheets(&mut cand_layout.sheets);

        if crate::scoring::is_room_candidate_better(&cand_layout, &best_cand, baseline_sheets, baseline_underfilled) {
            best_cand = cand_layout.clone();
            improved = true;
        }

        if cand_layout.sheets.len() > baseline_sheets && !locked_sheets.is_empty() && !tail_pool.is_empty() {
            let mut backfill_layout = cand_layout;
            while backfill_layout.sheets.len() > locked_sheets.len() {
                if !consolidate_sheets(&mut backfill_layout, global_rot_div, sheet_in_sheet, compact_directions) {
                    break;
                }
            }
            if backfill_layout.sheets.len() > locked_sheets.len() + 1 {
                let lead_slice = &backfill_layout.sheets[0..locked_sheets.len()];
                let absorbed_keys = collect_placed_source_keys(lead_slice);
                let mut rem_tail_parts = Vec::new();
                for tp in &tail_pool {
                    if !is_part_in_placed_keys(tp, &absorbed_keys) {
                        rem_tail_parts.push(tp.clone());
                    }
                }
                if !rem_tail_parts.is_empty() && rem_tail_parts.len() < tail_pool.len() {
                    let (paired_rem, _) = crate::macro_comb::pair_comb_parts(&rem_tail_parts, &config_obj, part_spacing);
                    let mut rem_state = material_state.clone();
                    rem_state.parts = paired_rem;
                    let rem_layout = pack_sub_state_best(
                        &rem_state,
                        &[100, 0, 1],
                        global_rot_div,
                        sheet_in_sheet,
                        compact_directions,
                    );
                    let mut rebuilt = lead_slice.to_vec();
                    for rs in rem_layout.sheets {
                        rebuilt.push(rs);
                    }
                    for (idx, s) in rebuilt.iter_mut().enumerate() {
                        s.index = idx + 1;
                    }
                    backfill_layout.sheets = rebuilt;
                    consolidate_sheets(&mut backfill_layout, global_rot_div, sheet_in_sheet, compact_directions);
                    prioritize_two_sided_sheets(&mut backfill_layout.sheets);
                }
            }
            if crate::scoring::is_room_candidate_better(&backfill_layout, &best_cand, baseline_sheets, baseline_underfilled) {
                best_cand = backfill_layout;
                improved = true;
            }
        }
    }

    if start_mat.elapsed().as_secs_f64() < 9.2 {
        for strat_id in [100usize, 101, 102, 104] {
            if start_mat.elapsed().as_secs_f64() >= 9.5 {
                break;
            }
            let mut aff_layout = pack_material(paired_material_state, strat_id);
            consolidate_sheets(&mut aff_layout, global_rot_div, sheet_in_sheet, compact_directions);
            prioritize_two_sided_sheets(&mut aff_layout.sheets);
            if crate::scoring::is_room_candidate_better(&aff_layout, &best_cand, baseline_sheets, baseline_underfilled) {
                best_cand = aff_layout;
                improved = true;
            }
        }
    }

    if improved {
        Some(best_cand)
    } else {
        None
    }
}

fn try_room_subbatched_layout_fast(
    _material_state: &MaterialStateInput,
    paired_material_state: &MaterialStateInput,
    baseline_layout: &LayoutOutput,
    global_rot_div: usize,
    sheet_in_sheet: bool,
    compact_directions: &[String],
    start_mat: &std::time::Instant,
) -> Option<LayoutOutput> {
    use std::collections::BTreeMap;

    if paired_material_state.parts.len() < 2 || baseline_layout.sheets.is_empty() {
        return None;
    }

    let mut room_buckets: BTreeMap<String, Vec<crate::types::PartInput>> = BTreeMap::new();
    let mut unassigned_parts: Vec<crate::types::PartInput> = Vec::new();

    for p in &paired_material_state.parts {
        let r = crate::types::get_part_room_key(p);
        if r.is_empty() {
            unassigned_parts.push(p.clone());
        } else {
            room_buckets.entry(r).or_default().push(p.clone());
        }
    }

    if room_buckets.len() < 2 {
        return None;
    }

    let baseline_sheets = baseline_layout.sheets.len();
    let baseline_underfilled = LayoutScore::compute(baseline_layout).underfilled_sheets;
    let mut best_cand = baseline_layout.clone();
    let mut improved = false;
    let board_area = baseline_layout.board_width * baseline_layout.board_height;

    let mut ordered_rooms: Vec<(String, Vec<crate::types::PartInput>, i64)> = room_buckets
        .into_iter()
        .map(|(k, parts)| {
            let area_sum: f64 = parts
                .iter()
                .map(|p| p.area.unwrap_or_else(|| Polygon::from_raw(&p.contour).area()))
                .sum();
            (k, parts, area_sum.round() as i64)
        })
        .collect();
    ordered_rooms.sort_by(|a, b| b.2.cmp(&a.2).then_with(|| a.0.cmp(&b.0)));

    let per_room_results: Vec<(String, Vec<crate::types::PartInput>, LayoutOutput)> = ordered_rooms
        .into_par_iter()
        .map(|(r_key, r_parts, _area)| {
            let mut r_state = paired_material_state.clone();
            r_state.parts = r_parts.clone();
            let r_layout = pack_sub_state_best(&r_state, &[0, 1], global_rot_div, sheet_in_sheet, compact_directions);
            (r_key, r_parts, r_layout)
        })
        .collect();

    let has_single_dense_room = per_room_results
        .iter()
        .any(|(_, _, r_layout)| r_layout.sheets.len() == 1 && r_layout.sheets[0].used_percent >= 78.0);

    for lock_mode in [0usize, 1usize] {
        if lock_mode == 1 && (!has_single_dense_room || (improved && best_cand.sheets.len() <= baseline_sheets)) {
            break;
        }
        if lock_mode == 1 && start_mat.elapsed().as_secs_f64() >= 9.6 {
            break;
        }
        let mut locked_sheets: Vec<SheetOutput> = Vec::new();
        let mut tail_pool: Vec<crate::types::PartInput> = Vec::new();

        for (_r_key, r_orig_parts, r_layout) in &per_room_results {
            let s_len = r_layout.sheets.len();
            let keep_count = if s_len > 1 {
                s_len - 1
            } else if s_len == 1 && lock_mode == 0 && r_layout.sheets[0].used_percent >= 65.0 {
                1
            } else {
                0
            };

            if keep_count > 0 {
                let kept_slice = &r_layout.sheets[0..keep_count];
                for s in kept_slice {
                    locked_sheets.push(s.clone());
                }
                let placed_keys = collect_placed_source_keys(kept_slice);
                for orig_p in r_orig_parts {
                    if !is_part_in_placed_keys(orig_p, &placed_keys) {
                        tail_pool.push(orig_p.clone());
                    }
                }
            } else {
                for orig_p in r_orig_parts {
                    tail_pool.push(orig_p.clone());
                }
            }
        }

        for u_p in &unassigned_parts {
            tail_pool.push(u_p.clone());
        }

        let mut merged_sheets = locked_sheets.clone();
        if !tail_pool.is_empty() {
            let mut tail_state = paired_material_state.clone();
            tail_state.parts = tail_pool.clone();
            let tail_layout = pack_sub_state_parallel(
                &tail_state,
                &[100, 101, 103, 0],
                global_rot_div,
                sheet_in_sheet,
                compact_directions,
            );
            for ts in &tail_layout.sheets {
                merged_sheets.push(ts.clone());
            }
        }

        for (idx, s) in merged_sheets.iter_mut().enumerate() {
            s.index = idx + 1;
            let used_area: f64 = s.placements.iter().map(|p| p.area).sum();
            let util = if board_area > 0.0 { (used_area / board_area) * 100.0 } else { 0.0 };
            s.utilization = (util * 10.0).round() / 10.0;
            s.used_percent = (util * 10.0).round() / 10.0;
        }

        let mut cand_layout = baseline_layout.clone();
        cand_layout.sheets = merged_sheets;
        while cand_layout.sheets.len() > baseline_sheets {
            if !consolidate_sheets(&mut cand_layout, global_rot_div, sheet_in_sheet, compact_directions) {
                break;
            }
        }
        prioritize_two_sided_sheets(&mut cand_layout.sheets);

        if crate::scoring::is_room_candidate_better(&cand_layout, &best_cand, baseline_sheets, baseline_underfilled) {
            best_cand = cand_layout.clone();
            improved = true;
        }

        if cand_layout.sheets.len() > baseline_sheets && !locked_sheets.is_empty() && !tail_pool.is_empty() {
            let mut backfill_layout = cand_layout;
            while backfill_layout.sheets.len() > locked_sheets.len() {
                if !consolidate_sheets(&mut backfill_layout, global_rot_div, sheet_in_sheet, compact_directions) {
                    break;
                }
            }
            if backfill_layout.sheets.len() > locked_sheets.len() + 1 {
                let lead_slice = &backfill_layout.sheets[0..locked_sheets.len()];
                let absorbed_keys = collect_placed_source_keys(lead_slice);
                let mut rem_tail_parts = Vec::new();
                for tp in &tail_pool {
                    if !is_part_in_placed_keys(tp, &absorbed_keys) {
                        rem_tail_parts.push(tp.clone());
                    }
                }
                if !rem_tail_parts.is_empty() && rem_tail_parts.len() < tail_pool.len() {
                    let mut rem_state = paired_material_state.clone();
                    rem_state.parts = rem_tail_parts;
                    let rem_layout = pack_sub_state_parallel(
                        &rem_state,
                        &[100, 101, 103, 0],
                        global_rot_div,
                        sheet_in_sheet,
                        compact_directions,
                    );
                    let mut rebuilt = lead_slice.to_vec();
                    for rs in rem_layout.sheets {
                        rebuilt.push(rs);
                    }
                    for (idx, s) in rebuilt.iter_mut().enumerate() {
                        s.index = idx + 1;
                    }
                    backfill_layout.sheets = rebuilt;
                    consolidate_sheets(&mut backfill_layout, global_rot_div, sheet_in_sheet, compact_directions);
                    prioritize_two_sided_sheets(&mut backfill_layout.sheets);
                }
            }
            if crate::scoring::is_room_candidate_better(&backfill_layout, &best_cand, baseline_sheets, baseline_underfilled) {
                best_cand = backfill_layout;
                improved = true;
            }
        }
    }

    if start_mat.elapsed().as_secs_f64() < 9.5 {
        let aff_layout = pack_sub_state_parallel(
            paired_material_state,
            &[100, 101, 102, 103],
            global_rot_div,
            sheet_in_sheet,
            compact_directions,
        );
        if crate::scoring::is_room_candidate_better(&aff_layout, &best_cand, baseline_sheets, baseline_underfilled) {
            best_cand = aff_layout;
            improved = true;
        }
    }

    if improved {
        Some(best_cand)
    } else {
        None
    }
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
    let existing_macro_count = material_state.parts.iter().filter(|p| p.manual_cluster_macro == Some(true)).count();
    let (paired_parts, new_comb_count) = crate::macro_comb::pair_comb_parts(
        &material_state.parts,
        &config_obj,
        part_spacing,
    );
    let comb_count = existing_macro_count + new_comb_count;
    let mut paired_material_state = material_state.clone();
    paired_material_state.parts = paired_parts;
    let total_parts = paired_material_state.parts.len();
    let start_mat = std::time::Instant::now();
    let prepared_parts = crate::packing::prepare_parts(&paired_material_state.parts, &config_obj, part_spacing);
    let has_complex_shapes = comb_count > 0 || prepared_parts.iter().any(|p| p.is_irregular || !p.variants[0].norm_holes.is_empty());

    let has_multi_rooms = {
        let mut seen = std::collections::BTreeSet::new();
        for p in &paired_material_state.parts {
            let r = crate::types::get_part_room_key(p);
            if !r.is_empty() {
                seen.insert(r);
                if seen.len() >= 2 {
                    break;
                }
            }
        }
        seen.len() >= 2
    };

    // 1. Initial baseline with progressive sheet-by-sheet live streaming
    let mut progressive_sheet = 0;
    let mut initial_layout = crate::packing::pack_material_streaming_with_prepared(
        &paired_material_state,
        &prepared_parts,
        comb_count,
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
    prioritize_two_sided_sheets(&mut initial_layout.sheets);
    on_progress("Tối ưu sơ bộ", 60.0, initial_layout.sheets.len(), 0, &initial_layout);
    let mut best_layout = initial_layout;

    // 2. Parallel multi-strategy exploration (bounded for 10s response time rule)
    let baseline_elapsed = start_mat.elapsed().as_secs_f64();
    let num_threads = rayon::current_num_threads();
    let is_selective = config_obj.selective_repack == Some(true);
    let num_strategies = if is_selective {
        (num_threads * 2).clamp(16, 32)
    } else if baseline_elapsed > 4.0 {
        2.min(num_threads)
    } else if best_layout.sheets.len() <= 1 && !has_complex_shapes && !has_multi_rooms {
        4.min(num_threads)
    } else {
        num_threads.clamp(6, 16)
    };

    if num_strategies > 0 && baseline_elapsed < 8.0 {
        let (tx, rx) = std::sync::mpsc::channel();
        (1..=num_strategies).into_par_iter().for_each_with(tx, |s, idx| {
            let strategy_id = if has_multi_rooms && num_strategies >= 4 && idx == num_strategies - 2 {
                103
            } else if has_multi_rooms && num_strategies >= 4 && idx == num_strategies - 1 {
                100
            } else if has_multi_rooms && num_strategies >= 4 && idx == num_strategies {
                101
            } else {
                idx
            };
            let mut layout = crate::packing::pack_material_with_prepared(&paired_material_state, &prepared_parts, comb_count, strategy_id);
            consolidate_sheets(&mut layout, global_rot_div, sheet_in_sheet, &compact_directions);
            let _ = s.send((strategy_id, layout));
        });

        let mut completed_count = 0;
        let phase2_budget = if has_multi_rooms { 7.8 } else { 9.5 };
        while let Ok((_strat_id, mut layout)) = rx.recv() {
            completed_count += 1;
            prioritize_two_sided_sheets(&mut layout.sheets);
            let take = if has_multi_rooms {
                let max_s = best_layout.sheets.len().max(layout.sheets.len());
                let max_uf = LayoutScore::compute(&best_layout).underfilled_sheets;
                crate::scoring::is_room_candidate_better(&layout, &best_layout, max_s, max_uf)
            } else {
                LayoutScore::compute(&layout) < LayoutScore::compute(&best_layout)
            };
            let strategy_pct = 60.0 + (completed_count as f64 / num_strategies as f64) * 35.0;
            if take {
                best_layout = layout;
            }
            on_progress("Tối ưu đa luồng", strategy_pct, completed_count, num_strategies, &best_layout);

            if start_mat.elapsed().as_secs_f64() >= phase2_budget {
                break;
            }
        }
    }

    // 3. Final consolidation & scoring
    consolidate_sheets(&mut best_layout, global_rot_div, sheet_in_sheet, &compact_directions);
    prioritize_two_sided_sheets(&mut best_layout.sheets);

    if let Some(room_layout) = try_room_subbatched_layout_fast(
        material_state,
        &paired_material_state,
        &best_layout,
        global_rot_div,
        sheet_in_sheet,
        &compact_directions,
        &start_mat,
    ) {
        best_layout = room_layout;
    }

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
            free_rotation: Some(false),
            small_part: Some(false),
            small_part_clearance: Some(0.0),
            small_part_edge_protected: Some(false),
            two_sided: None,
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

    #[test]
    fn test_triangle_grain_never_rotated_270_or_90() {
        use crate::types::{MaterialStateInput, ConfigurationInput, PartInput};
        let tri_contour = vec![
            [0.0, 0.0],
            [600.0, 0.0],
            [0.0, 400.0],
        ];
        let poly = Polygon::from_raw(&tri_contour);
        let area = poly.area();

        let make_tri = |id: &str| PartInput {
            id: Some(id.to_string()),
            entity_id: Some(format!("ent_{}", id)),
            name: Some(format!("Triangle {}", id)),
            width: Some(600.0),
            height: Some(400.0),
            area: Some(area),
            contour: tri_contour.clone(),
            holes: None,
            render_contour: None,
            render_holes: None,
            collision_contour: None,
            collision_holes: None,
            draw_layers: None,
            rotations: None,
            base_rotation_degrees: Some(0.0),
            rotation_degrees: Some(0.0),
            grain_locked: Some(true),
            has_grain_label: Some(true),
            grain_arrow_degrees: Some(0.0),
            free_rotation: Some(false),
            rotation_divisions: Some(2),
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
            two_sided: None,
            has_circular_arc: None,
        };

        // Even with global configuration rotation_divisions = 4 (free 4-direction rotation)
        let config = ConfigurationInput {
            board_width: Some(2440.0),
            board_height: Some(1220.0),
            edge_margin: Some(10.0),
            cut_gap: Some(6.0),
            part_spacing: Some(6.0),
            rotation_divisions: Some(4),
            rotate_step: Some(90.0),
            compact_directions: Some(vec!["left".to_string(), "bottom".to_string()]),
            target_sheet_utilization: Some(90.0),
            sheet_in_sheet: Some(false),
            selective_repack: Some(false),
            small_part_threshold: Some(0.0),
            small_part_clearance: Some(0.0),
            small_part_edge_zone: Some(0.0),
            merge_cut_paths: Some(false),
            optimization_seed: None,
        };

        let mat_state = MaterialStateInput {
            key: Some("mdf|18".to_string()),
            material: Some("MDF".to_string()),
            thickness: Some(18.0),
            group_index: Some(0),
            configuration: Some(config),
            original_part_count: Some(2),
            parts: vec![make_tri("1"), make_tri("2")],
            learning_key: None,
            optimization_seed: None,
        };

        let layout = optimize_material_layout(&mat_state, |_phase, _pct, _iter, _tot, _lay| {});
        for sheet in &layout.sheets {
            for p in &sheet.placements {
                let rot = p.rotation_degrees.rem_euclid(360.0);
                assert!(
                    (rot - 0.0).abs() < 1e-2 || (rot - 180.0).abs() < 1e-2,
                    "Placement {} rotated by {:.1} degrees! Grain direction violation: must only be 0 or 180 deg.",
                    p.id,
                    rot
                );
            }
        }
    }

    #[test]
    fn test_multi_room_nesting_groups_rooms_without_extra_sheets() {
        use crate::types::{clear_room_registry, register_part_room, get_placement_room_key, MaterialStateInput, ConfigurationInput, PartInput};
        clear_room_registry();

        let make_rect = |id: &str, w: f64, h: f64, room: &str| {
            let eid = format!("ent_{}", id);
            register_part_room(Some(id), Some(&eid), Some(room));
            PartInput {
                id: Some(id.to_string()),
                entity_id: Some(eid),
                name: Some(format!("Panel {}", id)),
                width: Some(w),
                height: Some(h),
                area: Some(w * h),
                contour: vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]],
                holes: None,
                render_contour: None,
                render_holes: None,
                collision_contour: None,
                collision_holes: None,
                draw_layers: None,
                rotations: Some(vec![0.0, 180.0]),
                base_rotation_degrees: Some(0.0),
                rotation_degrees: Some(0.0),
                grain_locked: Some(true),
                has_grain_label: Some(false),
                grain_arrow_degrees: None,
                free_rotation: Some(false),
                rotation_divisions: Some(2),
                color: None,
                logical_part_count: Some(1),
                manual_cluster_macro: None,
                manual_cluster_children: None,
                two_sided: None,
                has_circular_arc: None,
            }
        };

        // Interleave parts of Pn 01 and Pn 02 with varied sizes so global area sort would mix them across sheets
        let parts = vec![
            make_rect("101", 1180.0, 580.0, "Pn 01"),
            make_rect("201", 1170.0, 580.0, "Pn 02"),
            make_rect("102", 1160.0, 580.0, "Pn 01"),
            make_rect("202", 1150.0, 580.0, "Pn 02"),
            make_rect("103", 1140.0, 580.0, "Pn 01"),
            make_rect("203", 1130.0, 580.0, "Pn 02"),
        ];

        let config = ConfigurationInput {
            board_width: Some(2440.0),
            board_height: Some(1220.0),
            edge_margin: Some(10.0),
            cut_gap: Some(6.0),
            part_spacing: Some(6.0),
            rotation_divisions: Some(2),
            rotate_step: Some(180.0),
            compact_directions: Some(vec!["left".to_string(), "bottom".to_string()]),
            target_sheet_utilization: Some(90.0),
            sheet_in_sheet: Some(false),
            selective_repack: Some(false),
            small_part_threshold: Some(0.0),
            small_part_clearance: Some(0.0),
            small_part_edge_zone: Some(0.0),
            merge_cut_paths: Some(false),
            optimization_seed: None,
        };

        let mat_state = MaterialStateInput {
            key: Some("mdf|17.5".to_string()),
            material: Some("MDF".to_string()),
            thickness: Some(17.5),
            group_index: Some(0),
            configuration: Some(config),
            original_part_count: Some(parts.len()),
            parts,
            learning_key: None,
            optimization_seed: None,
        };

        let layout = optimize_material_layout(&mat_state, |_phase, _pct, _iter, _tot, _lay| {});
        assert_eq!(layout.sheets.len(), 2, "Total sheets must stay at 2");
        let (_score, splits, _span) = crate::scoring::compute_room_dispersion(&layout);
        assert_eq!(splits, 0, "Each room (Pn 01 and Pn 02) should be grouped onto its own sheet with 0 splits");
        for sheet in &layout.sheets {
            let rooms: std::collections::BTreeSet<String> = sheet.placements.iter().map(get_placement_room_key).collect();
            assert_eq!(rooms.len(), 1, "Sheet {} should contain exactly 1 room, got {:?}", sheet.index, rooms);
        }
    }
}

