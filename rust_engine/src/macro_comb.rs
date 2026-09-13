#![allow(dead_code)]
// src/macro_comb.rs
use std::f64::consts::PI;
use crate::geometry::{
    ensure_ccw, extract_edges, is_truly_irregular,
    polygons_collide_with_spacing, simplify_collinear, Edge, Point, Polygon,
};
use crate::types::{ClusterChild, ConfigurationInput, PartInput};

pub struct CombMatingResult {
    pub macro_w: f64,
    pub macro_h: f64,
    pub fill_rate: f64,
    pub child_a: ClusterChild,
    pub child_b: ClusterChild,
}

pub fn find_best_mating(
    part_a: &PartInput,
    part_b: &PartInput,
    spacing: f64,
    config: Option<&ConfigurationInput>,
) -> Option<CombMatingResult> {
    if part_a.contour.len() < 3 || part_b.contour.len() < 3 {
        return None;
    }

    let board_w = config.and_then(|c| c.board_width).unwrap_or(2440.0);
    let board_h = config.and_then(|c| c.board_height).unwrap_or(1220.0);
    let margin = config.and_then(|c| c.edge_margin).unwrap_or(10.0);
    let max_bw = (board_w - margin * 2.0).max(100.0);
    let max_bh = (board_h - margin * 2.0).max(100.0);
    let global_grain_locked = config.and_then(|c| c.rotation_divisions).map_or(true, |d| d <= 2);
    let is_grain_locked = global_grain_locked
        || part_a.grain_locked == Some(true)
        || part_b.grain_locked == Some(true)
        || part_a.has_grain_label == Some(true)
        || part_b.has_grain_label == Some(true);

    let poly_a = Polygon::from_raw(&part_a.contour);
    let poly_b = Polygon::from_raw(&part_b.contour);
    if poly_a.is_circular() || poly_b.is_circular() {
        return None;
    }
    let poly_orig_ccw_a = ensure_ccw(&poly_a);
    let poly_orig_ccw_b = ensure_ccw(&poly_b);

    let clean_a = simplify_collinear(&poly_a.points, 0.5);
    let clean_b = simplify_collinear(&poly_b.points, 0.5);
    if clean_a.len() < 3 || clean_b.len() < 3 {
        return None;
    }

    let poly_clean_a = ensure_ccw(&Polygon::new(clean_a));
    let poly_clean_b = ensure_ccw(&Polygon::new(clean_b));

    let bbox_a = poly_orig_ccw_a.bounding_box();
    let bbox_b = poly_orig_ccw_b.bounding_box();

    let area_a = part_a.area.unwrap_or_else(|| poly_orig_ccw_a.area());
    let area_b = part_b.area.unwrap_or_else(|| poly_orig_ccw_b.area());

    if !is_truly_irregular(&poly_orig_ccw_a, &bbox_a, area_a)
        || !is_truly_irregular(&poly_orig_ccw_b, &bbox_b, area_b)
    {
        return None;
    }

    let edges_a = extract_edges(&poly_clean_a.points);
    let edges_b = extract_edges(&poly_clean_b.points);

    let mut candidate_edges_a: Vec<&Edge> = edges_a.iter().filter(|e| e.length >= 15.0).collect();
    if candidate_edges_a.is_empty() {
        candidate_edges_a = edges_a.iter().collect();
    }
    let mut candidate_edges_b: Vec<&Edge> = edges_b.iter().filter(|e| e.length >= 15.0).collect();
    if candidate_edges_b.is_empty() {
        candidate_edges_b = edges_b.iter().collect();
    }

    let total_area = area_a + area_b;
    let mut best_score = -1e9;
    let mut best_candidate = None;

    for ea in &candidate_edges_a {
        for eb in &candidate_edges_b {
            let len_a = ea.length;
            let len_b = eb.length;
            let len_diff = (len_a - len_b).abs();
            if len_diff > 2.0 && !(len_diff <= 80.0 && len_a.min(len_b) >= 25.0) {
                continue;
            }

            let rot_angle = (ea.angle + PI) - eb.angle;
            let deg = (rot_angle * 180.0 / PI).rem_euclid(360.0);
            let is_180 = (deg - 180.0).abs() <= 2.0;
            let is_0 = deg <= 2.0 || deg >= 358.0;
            if !is_180 && !is_0 {
                continue;
            }

            let grain_a = part_a.has_grain_label == Some(true) || part_a.grain_locked == Some(true);
            let grain_b = part_b.has_grain_label == Some(true) || part_b.grain_locked == Some(true);
            let is_mating_grain_locked = global_grain_locked || grain_a || grain_b;
            if is_mating_grain_locked {
                let base_a = part_a.base_rotation_degrees.or(part_a.rotation_degrees).unwrap_or(0.0);
                let base_b = part_b.base_rotation_degrees.or(part_b.rotation_degrees).unwrap_or(0.0);
                let diff = (crate::packing::norm_angle(base_a + deg) - crate::packing::norm_angle(base_b)).abs() % 180.0;
                let grain_aligned = diff <= 2.0 || diff >= 178.0;
                if !grain_aligned {
                    continue;
                }
            }

            let cos_r = rot_angle.cos();
            let sin_r = rot_angle.sin();
            let origin_bx = eb.p1.x;
            let origin_by = eb.p1.y;
            let target_x0 = ea.p2.x;
            let target_y0 = ea.p2.y;

            let transform_b_pt = |p: Point| -> Point {
                let rx = p.x - origin_bx;
                let ry = p.y - origin_by;
                let tx = rx * cos_r - ry * sin_r + target_x0;
                let ty = rx * sin_r + ry * cos_r + target_y0;
                Point::new(tx, ty)
            };

            let tb_0_points: Vec<Point> = poly_orig_ccw_b.points.iter().map(|&p| transform_b_pt(p)).collect();
            let poly_b_0 = Polygon::new(tb_0_points);
            let bbox_b0 = poly_b_0.bounding_box();

            let ux = (ea.p2.x - ea.p1.x) / len_a;
            let uy = (ea.p2.y - ea.p1.y) / len_a;

            let (off_x, off_y) = if spacing > 1e-4 {
                let norm_x = uy;
                let norm_y = -ux;
                (norm_x * spacing, norm_y * spacing)
            } else {
                (0.0, 0.0)
            };

            let mut candidate_s = vec![0.0];
            if len_diff > 1.0 {
                candidate_s.push(len_a - len_b);
                candidate_s.push((len_a - len_b) * 0.5);
            }
            if spacing > 1e-4 {
                candidate_s.push(spacing);
                candidate_s.push(-spacing);
                if len_diff > 1.0 {
                    candidate_s.push((len_a - len_b) + spacing);
                    candidate_s.push((len_a - len_b) - spacing);
                }
            }
            if ux.abs() > 0.05 {
                let s_min_x = (bbox_a.min_x - bbox_b0.min_x) / ux;
                let s_max_x = (bbox_a.max_x - bbox_b0.max_x) / ux;
                candidate_s.push(s_max_x);
                candidate_s.push(s_min_x);
                candidate_s.push(((bbox_a.max_x + bbox_a.min_x) - (bbox_b0.max_x + bbox_b0.min_x)) / (2.0 * ux));

                let s_start = s_min_x.min(s_max_x) - 10.0;
                let s_end = s_min_x.max(s_max_x) + 10.0;
                let mut curr_s = s_start;
                while curr_s <= s_end {
                    candidate_s.push(curr_s);
                    curr_s += 10.0;
                }
            }
            if uy.abs() > 0.05 {
                let s_min_y = (bbox_a.min_y - bbox_b0.min_y) / uy;
                let s_max_y = (bbox_a.max_y - bbox_b0.max_y) / uy;
                candidate_s.push(s_max_y);
                candidate_s.push(s_min_y);
                candidate_s.push(((bbox_a.max_y + bbox_a.min_y) - (bbox_b0.max_y + bbox_b0.min_y)) / (2.0 * uy));

                let s_start = s_min_y.min(s_max_y) - 10.0;
                let s_end = s_min_y.max(s_max_y) + 10.0;
                let mut curr_s = s_start;
                while curr_s <= s_end {
                    candidate_s.push(curr_s);
                    curr_s += 10.0;
                }
            }

            candidate_s.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            candidate_s.dedup_by(|a, b| (*a - *b).abs() < 0.1);

            for s in candidate_s {
                let shift_x = s * ux + off_x;
                let shift_y = s * uy + off_y;
                let poly_b_s = poly_b_0.translate(shift_x, shift_y);

                if polygons_collide_with_spacing(&poly_orig_ccw_a, &poly_b_s, spacing) {
                    continue;
                }

                let bbox_b_s = poly_b_s.bounding_box();
                let comb_min_x = bbox_a.min_x.min(bbox_b_s.min_x);
                let comb_max_x = bbox_a.max_x.max(bbox_b_s.max_x);
                let comb_min_y = bbox_a.min_y.min(bbox_b_s.min_y);
                let comb_max_y = bbox_a.max_y.max(bbox_b_s.max_y);

                let w = comb_max_x - comb_min_x;
                let h = comb_max_y - comb_min_y;

                // Sheet boundary guard
                if is_grain_locked {
                    let base_a = part_a.base_rotation_degrees.or(part_a.rotation_degrees).unwrap_or(0.0);
                    let m_base = crate::packing::norm_angle(base_a);
                    let (sheet_dim_w, sheet_dim_h) = if (m_base.round() as i64).abs() % 180 == 90 {
                        (h, w)
                    } else {
                        (w, h)
                    };
                    if sheet_dim_w > max_bw || sheet_dim_h > max_bh {
                        continue;
                    }
                } else {
                    let vw = w.min(h);
                    let vh = w.max(h);
                    if vw > max_bw.min(max_bh) || vh > max_bw.max(max_bh) {
                        continue;
                    }
                }

                let box_area = w * h;
                if box_area <= 1.0 {
                    continue;
                }

                let orig_boxes_area = bbox_a.area() + bbox_b.area();
                let is_compact_saving = orig_boxes_area > 1.0 && box_area <= orig_boxes_area * 0.85;
                let fill_rate = total_area / box_area;
                let min_fill = if is_compact_saving { 0.25 } else { 0.60 };
                if fill_rate < min_fill || fill_rate > 1.01 {
                    continue;
                }

                let score = if is_compact_saving {
                    let saving_ratio = (orig_boxes_area - box_area) / orig_boxes_area;
                    saving_ratio * 2000.0 + fill_rate * 500.0 - w.min(h) * 0.1
                } else {
                    fill_rate * 1000.0 - w.min(h) * 0.1 - box_area * 0.0005
                };
                if score > best_score {
                    best_score = score;
                    best_candidate = Some((
                        w,
                        h,
                        fill_rate,
                        comb_min_x,
                        comb_min_y,
                        rot_angle,
                        origin_bx,
                        origin_by,
                        target_x0,
                        target_y0,
                        shift_x,
                        shift_y,
                    ));
                }
            }
        }
    }

    let (
        w,
        h,
        fill_rate,
        comb_min_x,
        comb_min_y,
        rot_angle,
        origin_bx,
        origin_by,
        target_x0,
        target_y0,
        shift_x,
        shift_y,
    ) = best_candidate?;

    let cos_r = rot_angle.cos();
    let sin_r = rot_angle.sin();
    let kx = target_x0 + shift_x - (origin_bx * cos_r - origin_by * sin_r);
    let ky = target_y0 + shift_y - (origin_bx * sin_r + origin_by * cos_r);

    let child_rot_a: f64 = 0.0;
    let child_ox_a: f64 = -comb_min_x;
    let child_oy_a: f64 = -comb_min_y;

    let child_rot_b: f64 = (rot_angle * 180.0 / PI).rem_euclid(360.0);
    let child_ox_b: f64 = kx - comb_min_x;
    let child_oy_b: f64 = ky - comb_min_y;

    let (macro_w, macro_h) = (w, h);

    // Strict SAT collision verification between children in final assembly coordinates
    let origin_zero = Point::new(0.0, 0.0);
    let poly_a_final = poly_orig_ccw_a
        .rotate_degrees(child_rot_a, origin_zero)
        .translate(child_ox_a, child_oy_a);
    let poly_b_final = poly_orig_ccw_b
        .rotate_degrees(child_rot_b, origin_zero)
        .translate(child_ox_b, child_oy_b);
    let check_spacing = if spacing <= 1e-4 { 0.0 } else { spacing * 0.5 };
    let collides_final = polygons_collide_with_spacing(&poly_a_final, &poly_b_final, check_spacing);
    if collides_final {
        return None;
    }

    let round3 = |v: f64| (v * 1000.0).round() / 1000.0;

    let child_a = ClusterChild {
        entity_id: part_a.entity_id.clone(),
        id: part_a.id.clone(),
        name: part_a.name.clone(),
        offset_x: Some(round3(child_ox_a)),
        offset_y: Some(round3(child_oy_a)),
        rotation_degrees: Some(round3(child_rot_a)),
        width: part_a.width,
        height: part_a.height,
        area: Some(round3(area_a)),
        contour: Some(part_a.contour.clone()),
        holes: part_a.holes.clone(),
        render_contour: part_a.render_contour.clone(),
        render_holes: part_a.render_holes.clone(),
        collision_contour: part_a.collision_contour.clone(),
        collision_holes: part_a.collision_holes.clone(),
        color: part_a.color.clone(),
        draw_layers: part_a.draw_layers.clone(),
        has_grain_label: part_a.has_grain_label,
        grain_arrow_degrees: part_a.grain_arrow_degrees,
        base_rotation_degrees: part_a.base_rotation_degrees,
    };

    let child_b = ClusterChild {
        entity_id: part_b.entity_id.clone(),
        id: part_b.id.clone(),
        name: part_b.name.clone(),
        offset_x: Some(round3(child_ox_b)),
        offset_y: Some(round3(child_oy_b)),
        rotation_degrees: Some(round3(child_rot_b)),
        width: part_b.width,
        height: part_b.height,
        area: Some(round3(area_b)),
        contour: Some(part_b.contour.clone()),
        holes: part_b.holes.clone(),
        render_contour: part_b.render_contour.clone(),
        render_holes: part_b.render_holes.clone(),
        collision_contour: part_b.collision_contour.clone(),
        collision_holes: part_b.collision_holes.clone(),
        color: part_b.color.clone(),
        draw_layers: part_b.draw_layers.clone(),
        has_grain_label: part_b.has_grain_label,
        grain_arrow_degrees: part_b.grain_arrow_degrees,
        base_rotation_degrees: part_b.base_rotation_degrees,
    };

    Some(CombMatingResult {
        macro_w: round3(macro_w),
        macro_h: round3(macro_h),
        fill_rate: round3(fill_rate),
        child_a,
        child_b,
    })
}

pub fn try_build_comb_pair(
    part_a: &PartInput,
    part_b: &PartInput,
    spacing: f64,
    config: Option<&ConfigurationInput>,
) -> Option<(PartInput, f64)> {
    let mating = find_best_mating(part_a, part_b, spacing, config)?;
    let id_a = part_a.id.as_deref().unwrap_or("A");
    let id_b = part_b.id.as_deref().unwrap_or("B");
    let name_a = part_a.name.as_deref().unwrap_or("Part A");
    let name_b = part_b.name.as_deref().unwrap_or("Part B");
    let area_a = part_a.area.unwrap_or(0.0);
    let area_b = part_b.area.unwrap_or(0.0);
    let grain_a = part_a.has_grain_label == Some(true) || part_a.grain_locked == Some(true);
    let grain_b = part_b.has_grain_label == Some(true) || part_b.grain_locked == Some(true);
    let global_grain_locked = config.and_then(|c| c.rotation_divisions).map_or(true, |d| d <= 2);
    let is_grain_locked = global_grain_locked || grain_a || grain_b;

    let (macro_base_rot, ref_grain_arrow) = if grain_a {
        let base_a = part_a.base_rotation_degrees.or(part_a.rotation_degrees).unwrap_or(0.0);
        let child_rot_a = mating.child_a.rotation_degrees.unwrap_or(0.0);
        (crate::packing::norm_angle(base_a - child_rot_a), part_a.grain_arrow_degrees.or(Some(90.0)))
    } else if grain_b {
        let base_b = part_b.base_rotation_degrees.or(part_b.rotation_degrees).unwrap_or(0.0);
        let child_rot_b = mating.child_b.rotation_degrees.unwrap_or(0.0);
        (crate::packing::norm_angle(base_b - child_rot_b), part_b.grain_arrow_degrees.or(Some(90.0)))
    } else {
        let base_a = part_a.base_rotation_degrees.or(part_a.rotation_degrees).unwrap_or(0.0);
        let child_rot_a = mating.child_a.rotation_degrees.unwrap_or(0.0);
        (crate::packing::norm_angle(base_a - child_rot_a), if is_grain_locked { Some(90.0) } else { None })
    };

    let macro_part = PartInput {
        entity_id: Some(format!("comb-macro:{}_{}", id_a, id_b)),
        id: Some(format!("{}+{}", id_a, id_b)),
        name: Some(format!("Cụm răng lược: {} + {}", name_a, name_b)),
        width: Some(mating.macro_w),
        height: Some(mating.macro_h),
        area: Some(area_a + area_b),
        contour: vec![
            [0.0, 0.0],
            [mating.macro_w, 0.0],
            [mating.macro_w, mating.macro_h],
            [0.0, mating.macro_h],
        ],
        holes: None,
        render_contour: None,
        render_holes: None,
        collision_contour: None,
        collision_holes: None,
        draw_layers: None,
        rotations: if is_grain_locked {
            Some(vec![macro_base_rot, crate::packing::norm_angle(macro_base_rot + 180.0)])
        } else {
            Some(vec![
                macro_base_rot,
                crate::packing::norm_angle(macro_base_rot + 90.0),
                crate::packing::norm_angle(macro_base_rot + 180.0),
                crate::packing::norm_angle(macro_base_rot + 270.0),
            ])
        },
        base_rotation_degrees: Some(macro_base_rot),
        rotation_degrees: Some(macro_base_rot),
        grain_locked: Some(is_grain_locked),
        has_grain_label: Some(grain_a || grain_b),
        grain_arrow_degrees: if is_grain_locked { ref_grain_arrow } else { None },
        free_rotation: Some(!is_grain_locked),
        rotation_divisions: Some(if is_grain_locked { 2 } else { 4 }),
        color: part_a.color.clone(),
        logical_part_count: Some(2),
        manual_cluster_macro: Some(true),
        manual_cluster_children: Some(vec![mating.child_a, mating.child_b]),
    };

    Some((macro_part, mating.fill_rate))
}

#[derive(Clone, Debug)]
pub struct MatingOffsets {
    pub macro_w: f64,
    pub macro_h: f64,
    pub fill_rate: f64,
    pub ox_a: f64,
    pub oy_a: f64,
    pub rot_a: f64,
    pub ox_b: f64,
    pub oy_b: f64,
    pub rot_b: f64,
}

impl CombMatingResult {
    pub fn to_offsets(&self) -> MatingOffsets {
        MatingOffsets {
            macro_w: self.macro_w,
            macro_h: self.macro_h,
            fill_rate: self.fill_rate,
            ox_a: self.child_a.offset_x.unwrap_or(0.0),
            oy_a: self.child_a.offset_y.unwrap_or(0.0),
            rot_a: self.child_a.rotation_degrees.unwrap_or(0.0),
            ox_b: self.child_b.offset_x.unwrap_or(0.0),
            oy_b: self.child_b.offset_y.unwrap_or(0.0),
            rot_b: self.child_b.rotation_degrees.unwrap_or(0.0),
        }
    }
}

pub fn create_macro_part(
    part_a: &PartInput,
    part_b: &PartInput,
    offsets: &MatingOffsets,
    config: Option<&ConfigurationInput>,
) -> PartInput {
    let id_a = part_a.id.as_deref().unwrap_or("A");
    let id_b = part_b.id.as_deref().unwrap_or("B");
    let name_a = part_a.name.as_deref().unwrap_or("Part A");
    let name_b = part_b.name.as_deref().unwrap_or("Part B");
    let area_a = part_a.area.unwrap_or(0.0);
    let area_b = part_b.area.unwrap_or(0.0);
    let round3 = |v: f64| (v * 1000.0).round() / 1000.0;

    let child_a = ClusterChild {
        entity_id: part_a.entity_id.clone(),
        id: part_a.id.clone(),
        name: part_a.name.clone(),
        offset_x: Some(round3(offsets.ox_a)),
        offset_y: Some(round3(offsets.oy_a)),
        rotation_degrees: Some(round3(offsets.rot_a)),
        width: part_a.width,
        height: part_a.height,
        area: Some(round3(area_a)),
        contour: Some(part_a.contour.clone()),
        holes: part_a.holes.clone(),
        render_contour: part_a.render_contour.clone(),
        render_holes: part_a.render_holes.clone(),
        collision_contour: part_a.collision_contour.clone(),
        collision_holes: part_a.collision_holes.clone(),
        color: part_a.color.clone(),
        draw_layers: part_a.draw_layers.clone(),
        has_grain_label: part_a.has_grain_label,
        grain_arrow_degrees: part_a.grain_arrow_degrees,
        base_rotation_degrees: part_a.base_rotation_degrees,
    };

    let child_b = ClusterChild {
        entity_id: part_b.entity_id.clone(),
        id: part_b.id.clone(),
        name: part_b.name.clone(),
        offset_x: Some(round3(offsets.ox_b)),
        offset_y: Some(round3(offsets.oy_b)),
        rotation_degrees: Some(round3(offsets.rot_b)),
        width: part_b.width,
        height: part_b.height,
        area: Some(round3(area_b)),
        contour: Some(part_b.contour.clone()),
        holes: part_b.holes.clone(),
        render_contour: part_b.render_contour.clone(),
        render_holes: part_b.render_holes.clone(),
        collision_contour: part_b.collision_contour.clone(),
        collision_holes: part_b.collision_holes.clone(),
        color: part_b.color.clone(),
        draw_layers: part_b.draw_layers.clone(),
        has_grain_label: part_b.has_grain_label,
        grain_arrow_degrees: part_b.grain_arrow_degrees,
        base_rotation_degrees: part_b.base_rotation_degrees,
    };

    let grain_a = part_a.has_grain_label == Some(true) || part_a.grain_locked == Some(true);
    let grain_b = part_b.has_grain_label == Some(true) || part_b.grain_locked == Some(true);
    let global_grain_locked = config.and_then(|c| c.rotation_divisions).map_or(true, |d| d <= 2);
    let is_grain_locked = global_grain_locked || grain_a || grain_b;

    let (macro_base_rot, ref_grain_arrow) = if grain_a {
        let base_a = part_a.base_rotation_degrees.or(part_a.rotation_degrees).unwrap_or(0.0);
        let child_rot_a = offsets.rot_a;
        (crate::packing::norm_angle(base_a - child_rot_a), part_a.grain_arrow_degrees.or(Some(90.0)))
    } else if grain_b {
        let base_b = part_b.base_rotation_degrees.or(part_b.rotation_degrees).unwrap_or(0.0);
        let child_rot_b = offsets.rot_b;
        (crate::packing::norm_angle(base_b - child_rot_b), part_b.grain_arrow_degrees.or(Some(90.0)))
    } else {
        let base_a = part_a.base_rotation_degrees.or(part_a.rotation_degrees).unwrap_or(0.0);
        let child_rot_a = offsets.rot_a;
        (crate::packing::norm_angle(base_a - child_rot_a), if is_grain_locked { Some(90.0) } else { None })
    };

    PartInput {
        entity_id: Some(format!("comb-macro:{}_{}", id_a, id_b)),
        id: Some(format!("{}+{}", id_a, id_b)),
        name: Some(format!("Cụm răng lược: {} + {}", name_a, name_b)),
        width: Some(offsets.macro_w),
        height: Some(offsets.macro_h),
        area: Some(area_a + area_b),
        contour: vec![
            [0.0, 0.0],
            [offsets.macro_w, 0.0],
            [offsets.macro_w, offsets.macro_h],
            [0.0, offsets.macro_h],
        ],
        holes: None,
        render_contour: None,
        render_holes: None,
        collision_contour: None,
        collision_holes: None,
        draw_layers: None,
        rotations: if is_grain_locked {
            Some(vec![macro_base_rot, crate::packing::norm_angle(macro_base_rot + 180.0)])
        } else {
            Some(vec![
                macro_base_rot,
                crate::packing::norm_angle(macro_base_rot + 90.0),
                crate::packing::norm_angle(macro_base_rot + 180.0),
                crate::packing::norm_angle(macro_base_rot + 270.0),
            ])
        },
        base_rotation_degrees: Some(macro_base_rot),
        rotation_degrees: Some(macro_base_rot),
        grain_locked: Some(is_grain_locked),
        has_grain_label: Some(grain_a || grain_b || is_grain_locked),
        grain_arrow_degrees: if is_grain_locked { ref_grain_arrow } else { None },
        free_rotation: Some(!is_grain_locked),
        rotation_divisions: Some(if is_grain_locked { 2 } else { 4 }),
        color: part_a.color.clone(),
        logical_part_count: Some(2),
        manual_cluster_macro: Some(true),
        manual_cluster_children: Some(vec![child_a, child_b]),
    }
}

pub fn compute_shape_signature(part: &PartInput) -> String {
    let poly = Polygon::from_raw(&part.contour);
    let clean = simplify_collinear(&poly.points, 0.5);
    let poly_ccw = ensure_ccw(&Polygon::new(clean));
    let (norm_poly, _, _) = poly_ccw.normalize_to_origin();
    let bbox = norm_poly.bounding_box();
    let area = part.area.unwrap_or_else(|| poly.area());

    let mut sig = format!(
        "{:.1}_{:.1}_{:.0}_{}_",
        (bbox.width() * 2.0).round() / 2.0,
        (bbox.height() * 2.0).round() / 2.0,
        area.round(),
        norm_poly.points.len()
    );
    for pt in &norm_poly.points {
        sig.push_str(&format!("{:.1},{:.1};", (pt.x * 2.0).round() / 2.0, (pt.y * 2.0).round() / 2.0));
    }
    sig
}

pub fn pair_comb_parts(
    parts: &[PartInput],
    config: &ConfigurationInput,
    spacing: f64,
) -> (Vec<PartInput>, usize) {
    if parts.len() < 2 {
        return (parts.to_vec(), 0);
    }

    // Step 1: Pre-filter irregular parts that are candidates for comb pairing
    let mut candidate_indices = Vec::new();
    let mut signatures: Vec<String> = Vec::with_capacity(parts.len());
    let global_grain_locked = config.rotation_divisions.map_or(true, |d| d <= 2);

    for (i, p) in parts.iter().enumerate() {
        if p.manual_cluster_macro == Some(true) {
            signatures.push(String::new());
            continue;
        }
        let poly = Polygon::from_raw(&p.contour);
        if poly.is_circular() {
            signatures.push(String::new());
            continue;
        }
        let bbox = poly.bounding_box();
        let area = p.area.unwrap_or_else(|| poly.area());
        if is_truly_irregular(&poly, &bbox, area) {
            candidate_indices.push(i);
            let is_p_grain = global_grain_locked || p.grain_locked == Some(true) || p.has_grain_label == Some(true);
            let base_sig = compute_shape_signature(p);
            if is_p_grain {
                let g = (p.base_rotation_degrees.or(p.rotation_degrees).unwrap_or(0.0).round() as i64).rem_euclid(180);
                signatures.push(format!("{}_g{}", base_sig, g));
            } else {
                signatures.push(base_sig);
            }
        } else {
            signatures.push(String::new());
        }
    }

    if candidate_indices.len() < 2 {
        return (parts.to_vec(), 0);
    }

    use std::collections::HashMap;
    let mut groups_by_sig: HashMap<String, Vec<usize>> = HashMap::new();
    for &idx in &candidate_indices {
        let sig = &signatures[idx];
        groups_by_sig.entry(sig.clone()).or_default().push(idx);
    }

    let mut used = vec![false; parts.len()];
    let mut macro_parts = Vec::new();
    let mut comb_count = 0;

    let mut mating_cache: HashMap<(String, String), Option<MatingOffsets>> = HashMap::new();

    let get_mating = |idx_a: usize, idx_b: usize, cache: &mut HashMap<(String, String), Option<MatingOffsets>>| -> Option<MatingOffsets> {
        let sig_a = &signatures[idx_a];
        let sig_b = &signatures[idx_b];
        let key = if sig_a <= sig_b {
            (sig_a.clone(), sig_b.clone())
        } else {
            (sig_b.clone(), sig_a.clone())
        };

        if let Some(cached) = cache.get(&key) {
            return cached.clone();
        }

        let res = find_best_mating(&parts[idx_a], &parts[idx_b], spacing, Some(config));
        let offsets = res.map(|m| m.to_offsets());
        cache.insert(key, offsets.clone());
        offsets
    };

    let is_pair_grain_compatible = |idx_a: usize, idx_b: usize, offsets: &MatingOffsets| -> bool {
        let is_grain = global_grain_locked
            || parts[idx_a].grain_locked == Some(true)
            || parts[idx_b].grain_locked == Some(true)
            || parts[idx_a].has_grain_label == Some(true)
            || parts[idx_b].has_grain_label == Some(true);
        if !is_grain {
            return true;
        }
        let base_a = parts[idx_a].base_rotation_degrees.or(parts[idx_a].rotation_degrees).unwrap_or(0.0);
        let base_b = parts[idx_b].base_rotation_degrees.or(parts[idx_b].rotation_degrees).unwrap_or(0.0);
        let diff = ((base_a - base_b) - (offsets.rot_a - offsets.rot_b)).round().abs() as i64 % 180;
        diff == 0
    };

    // Phase 1: Self-mating within identical shape families O(N)
    for (_sig, indices) in groups_by_sig.iter() {
        if indices.len() >= 2 {
            if let Some(offsets) = get_mating(indices[0], indices[1], &mut mating_cache) {
                let p_a = Polygon::from_raw(&parts[indices[0]].contour);
                let p_b = Polygon::from_raw(&parts[indices[1]].contour);
                let orig_boxes_area = p_a.bounding_box().area() + p_b.bounding_box().area();
                let macro_box = offsets.macro_w * offsets.macro_h;
                let is_compact_saving = orig_boxes_area > 1.0 && macro_box <= orig_boxes_area * 0.85;
                let min_threshold = if is_compact_saving { 0.25 } else { 0.65 };
                if offsets.fill_rate >= min_threshold {
                    let mut i = 0;
                    while i + 1 < indices.len() {
                        let idx_a = indices[i];
                        let idx_b = indices[i + 1];
                        if !is_pair_grain_compatible(idx_a, idx_b, &offsets) {
                            i += 1;
                            continue;
                        }
                        let macro_p = create_macro_part(&parts[idx_a], &parts[idx_b], &offsets, Some(config));
                        macro_parts.push(macro_p);
                        used[idx_a] = true;
                        used[idx_b] = true;
                        comb_count += 1;
                        i += 2;
                    }
                }
            }
        }
    }

    // Phase 2: Cross-family mating among remaining candidates
    let remaining_candidates: Vec<usize> = candidate_indices
        .into_iter()
        .filter(|&idx| !used[idx])
        .collect();

    if remaining_candidates.len() >= 2 {
        let mut pair_matches: Vec<(usize, usize, f64, MatingOffsets)> = Vec::new();
        let mut sorted_rem = remaining_candidates.clone();
        sorted_rem.sort_by(|&a, &b| {
            let area_a = parts[a].area.unwrap_or(0.0);
            let area_b = parts[b].area.unwrap_or(0.0);
            area_b.partial_cmp(&area_a).unwrap_or(std::cmp::Ordering::Equal)
        });

        let window = 40.min(sorted_rem.len());
        for i_pos in 0..sorted_rem.len() {
            let i = sorted_rem[i_pos];
            if used[i] { continue; }
            let max_j = (i_pos + window).min(sorted_rem.len());
            for j_pos in (i_pos + 1)..max_j {
                let j = sorted_rem[j_pos];
                if used[j] { continue; }
                if let Some(offsets) = get_mating(i, j, &mut mating_cache) {
                    let p_a = Polygon::from_raw(&parts[i].contour);
                    let p_b = Polygon::from_raw(&parts[j].contour);
                    let orig_boxes_area = p_a.bounding_box().area() + p_b.bounding_box().area();
                    let macro_box = offsets.macro_w * offsets.macro_h;
                    let is_compact_saving = orig_boxes_area > 1.0 && macro_box <= orig_boxes_area * 0.85;
                    let min_threshold = if is_compact_saving { 0.25 } else { 0.65 };
                    if offsets.fill_rate >= min_threshold && is_pair_grain_compatible(i, j, &offsets) {
                        pair_matches.push((i, j, offsets.fill_rate, offsets));
                    }
                }
            }
        }

        pair_matches.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));

        for (i, j, _, offsets) in pair_matches {
            if !used[i] && !used[j] {
                used[i] = true;
                used[j] = true;
                let macro_p = create_macro_part(&parts[i], &parts[j], &offsets, Some(config));
                macro_parts.push(macro_p);
                comb_count += 1;
            }
        }
    }

    // Step 4: Assemble final parts list
    let mut result = Vec::with_capacity(parts.len());
    for m in macro_parts {
        result.push(m);
    }
    for (i, p) in parts.iter().enumerate() {
        if !used[i] {
            result.push(p.clone());
        }
    }

    (result, comb_count)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_comb_contour() -> Vec<[f64; 2]> {
        let mut pts = Vec::new();
        pts.push([0.0, 0.0]);
        pts.push([600.0, 0.0]);
        pts.push([600.0, 100.0]);

        let teeth = [
            (550.0, 450.0, 100.0, 180.0),
            (350.0, 250.0, 100.0, 180.0),
            (150.0, 50.0,  100.0, 180.0),
        ];

        let mut current_x = 600.0;
        for t in teeth {
            if current_x > t.0 {
                pts.push([t.0, t.2]);
            }
            pts.push([t.0, t.3]);
            pts.push([t.1, t.3]);
            pts.push([t.1, t.2]);
            current_x = t.1;
        }
        pts.push([0.0, 100.0]);
        pts
    }

    #[test]
    fn test_irregular_comb_detection() {
        let comb = make_test_comb_contour();
        let poly = Polygon::from_raw(&comb);
        let bbox = poly.bounding_box();
        let area = poly.area();
        assert!(is_truly_irregular(&poly, &bbox, area));

        let rect_pts = vec![[0.0, 0.0], [500.0, 0.0], [500.0, 300.0], [0.0, 300.0]];
        let rect_poly = Polygon::from_raw(&rect_pts);
        let rect_bbox = rect_poly.bounding_box();
        assert!(!is_truly_irregular(&rect_poly, &rect_bbox, rect_poly.area()));
    }

    #[test]
    fn test_comb_pair_spacing_6() {
        let comb = make_test_comb_contour();
        let poly = Polygon::from_raw(&comb);
        let area = poly.area();

        let part_a = PartInput {
            id: Some("1".to_string()),
            entity_id: Some("e1".to_string()),
            name: Some("Comb_1".to_string()),
            width: Some(600.0),
            height: Some(180.0),
            area: Some(area),
            contour: comb.clone(),
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
            has_grain_label: Some(true),
            grain_arrow_degrees: Some(0.0),
            free_rotation: Some(false),
            rotation_divisions: Some(2),
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
        };

        let part_b = PartInput {
            id: Some("2".to_string()),
            entity_id: Some("e2".to_string()),
            name: Some("Comb_2".to_string()),
            width: Some(600.0),
            height: Some(180.0),
            area: Some(area),
            contour: comb.clone(),
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
            has_grain_label: Some(true),
            grain_arrow_degrees: Some(0.0),
            free_rotation: Some(false),
            rotation_divisions: Some(2),
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
        };

        let res = try_build_comb_pair(&part_a, &part_b, 6.0, None);
        assert!(res.is_some(), "Comb pair should succeed with spacing 6.0");
        let (macro_p, fill) = res.unwrap();
        assert!(fill >= 0.70, "Fill rate should be >= 70%, got {}", fill);

        let mw = macro_p.width.unwrap();
        let mh = macro_p.height.unwrap();

        let children = macro_p.manual_cluster_children.as_ref().unwrap();
        assert_eq!(children.len(), 2);

        // Verify that transformed points of both children are within macro bounds
        for child in children {
            let rot = child.rotation_degrees.unwrap_or(0.0) * PI / 180.0;
            let ox = child.offset_x.unwrap_or(0.0);
            let oy = child.offset_y.unwrap_or(0.0);
            let c_pts = child.contour.as_ref().unwrap();
            for p in c_pts {
                let mx = (p[0] * rot.cos() - p[1] * rot.sin()) + ox;
                let my = (p[0] * rot.sin() + p[1] * rot.cos()) + oy;
                assert!(
                    mx >= -0.1 && mx <= mw + 0.1,
                    "Child point X {} should be in [0, {}]",
                    mx,
                    mw
                );
                assert!(
                    my >= -0.1 && my <= mh + 0.1,
                    "Child point Y {} should be in [0, {}]",
                    my,
                    mh
                );
            }
        }
    }

    #[test]
    fn test_comb_pair_spacing_0() {
        let comb = make_test_comb_contour();
        let poly = Polygon::from_raw(&comb);
        let area = poly.area();

        let part_a = PartInput {
            id: Some("1".to_string()),
            entity_id: Some("e1".to_string()),
            name: Some("Comb_1".to_string()),
            width: Some(600.0),
            height: Some(180.0),
            area: Some(area),
            contour: comb.clone(),
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
            has_grain_label: Some(true),
            grain_arrow_degrees: Some(0.0),
            free_rotation: Some(false),
            rotation_divisions: Some(2),
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
        };

        let part_b = part_a.clone();
        let res = try_build_comb_pair(&part_a, &part_b, 0.0, None);
        assert!(res.is_some(), "Comb pair should succeed with spacing 0.0 (Value = 0 rule)");
        let (_macro_p, fill) = res.unwrap();
        assert!(fill >= 0.70);
    }

    #[test]
    fn test_trapezoid_diagonal_mating_180_degrees() {
        let trap_contour = vec![
            [0.0, 0.0],
            [380.0, 0.0],
            [380.0, 400.0],
            [0.0, 1100.0],
        ];
        let poly = Polygon::from_raw(&trap_contour);
        let area = poly.area();

        let part_a = PartInput {
            id: Some("trap_1".to_string()),
            entity_id: Some("ent_trap_1".to_string()),
            name: Some("Trapezoid 1".to_string()),
            width: Some(380.0),
            height: Some(1100.0),
            area: Some(area),
            contour: trap_contour.clone(),
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
            has_grain_label: Some(true),
            grain_arrow_degrees: Some(0.0),
            free_rotation: Some(false),
            rotation_divisions: Some(2),
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
        };
        let part_b = PartInput {
            id: Some("trap_2".to_string()),
            entity_id: Some("ent_trap_2".to_string()),
            name: Some("Trapezoid 2".to_string()),
            ..part_a.clone()
        };

        let config = ConfigurationInput {
            board_width: Some(1220.0),
            board_height: Some(2440.0),
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
        };

        let res = try_build_comb_pair(&part_a, &part_b, 6.0, Some(&config));
        assert!(res.is_some(), "Two trapezoids must mate along diagonal edge!");
        let (macro_p, fill) = res.unwrap();
        println!("Mated trapezoid macro: w={}, h={}, fill={}", macro_p.width.unwrap(), macro_p.height.unwrap(), fill);
        let children = macro_p.manual_cluster_children.as_ref().unwrap();
        assert_eq!(children.len(), 2);
        println!("Child 0: rot={:?}, ox={:?}, oy={:?}", children[0].rotation_degrees, children[0].offset_x, children[0].offset_y);
        println!("Child 1: rot={:?}, ox={:?}, oy={:?}", children[1].rotation_degrees, children[1].offset_x, children[1].offset_y);
        println!("Macro rotations={:?}, base_rot={:?}", macro_p.rotations, macro_p.base_rotation_degrees);
        assert!(fill >= 0.85, "Mated rectangle fill rate should be high, got {}", fill);
        let rot_diff = (children[1].rotation_degrees.unwrap() - children[0].rotation_degrees.unwrap()).abs();
        assert_eq!(rot_diff.round() as i64 % 360, 180, "Child 2 must be rotated 180 degrees");
    }

    #[test]
    fn test_trapezoid_mating_with_base_rotation_90() {
        let trap_contour = vec![
            [0.0, 0.0],
            [1100.0, 0.0],
            [400.0, 380.0],
            [0.0, 380.0],
        ];
        let poly = Polygon::from_raw(&trap_contour);
        let area = poly.area();

        let part_a = PartInput {
            id: Some("trap_x1".to_string()),
            entity_id: Some("ent_trap_x1".to_string()),
            name: Some("Trapezoid X1".to_string()),
            width: Some(1100.0),
            height: Some(380.0),
            area: Some(area),
            contour: trap_contour.clone(),
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
            has_grain_label: Some(true),
            grain_arrow_degrees: Some(90.0),
            free_rotation: Some(false),
            rotation_divisions: Some(2),
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
        };
        let part_b = PartInput {
            id: Some("trap_x2".to_string()),
            entity_id: Some("ent_trap_x2".to_string()),
            name: Some("Trapezoid X2".to_string()),
            ..part_a.clone()
        };

        let config = ConfigurationInput {
            board_width: Some(1220.0),
            board_height: Some(2440.0),
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
        };

        let res = try_build_comb_pair(&part_a, &part_b, 6.0, Some(&config));
        assert!(res.is_some(), "Trapezoids with base_rot=0 must mate!");
        let (macro_p, fill) = res.unwrap();
        assert!(fill >= 0.85);
        assert_eq!(macro_p.base_rotation_degrees, Some(0.0));
        assert_eq!(macro_p.rotations, Some(vec![0.0, 180.0]));
    }

    #[test]
    fn test_fast_comb_pairing_500_parts() {
        let comb = make_test_comb_contour();
        let poly = Polygon::from_raw(&comb);
        let area = poly.area();

        let mut parts = Vec::with_capacity(500);
        for i in 0..500 {
            parts.push(PartInput {
                id: Some(format!("p_{}", i)),
                entity_id: Some(format!("ent_{}", i)),
                name: Some(format!("Comb_{}", i)),
                width: Some(600.0),
                height: Some(180.0),
                area: Some(area),
                contour: comb.clone(),
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
                has_grain_label: Some(true),
                grain_arrow_degrees: Some(0.0),
                free_rotation: Some(false),
                rotation_divisions: Some(2),
                color: None,
                logical_part_count: Some(1),
                manual_cluster_macro: None,
                manual_cluster_children: None,
            });
        }

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
        };

        let start = std::time::Instant::now();
        let (paired, count) = pair_comb_parts(&parts, &config, 6.0);
        let duration = start.elapsed();

        assert_eq!(count, 250, "500 parts should form exactly 250 pairs");
        assert_eq!(paired.len(), 250, "Output should contain 250 macro parts");
        assert!(duration.as_millis() < 500, "Pairing 500 parts should take < 500ms, took {:?}", duration);
    }

    #[test]
    fn test_fast_comb_pairing_2000_parts() {
        let comb = make_test_comb_contour();
        let poly = Polygon::from_raw(&comb);
        let area = poly.area();

        let mut parts = Vec::with_capacity(2000);
        for i in 0..2000 {
            parts.push(PartInput {
                id: Some(format!("p_{}", i)),
                entity_id: Some(format!("ent_{}", i)),
                name: Some(format!("Comb_{}", i)),
                width: Some(600.0),
                height: Some(180.0),
                area: Some(area),
                contour: comb.clone(),
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
                has_grain_label: Some(true),
                grain_arrow_degrees: Some(0.0),
                free_rotation: Some(false),
                rotation_divisions: Some(2),
                color: None,
                logical_part_count: Some(1),
                manual_cluster_macro: None,
                manual_cluster_children: None,
            });
        }

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
        };

        let start = std::time::Instant::now();
        let (paired, count) = pair_comb_parts(&parts, &config, 6.0);
        let duration = start.elapsed();

        assert_eq!(count, 1000, "2000 parts should form exactly 1000 pairs");
        assert_eq!(paired.len(), 1000, "Output should contain 1000 macro parts");
        assert!(duration.as_millis() < 2000, "Pairing 2000 parts should take < 2s, took {:?}", duration);
    }

    #[test]
    fn test_fillet_contour_clearance_at_least_6mm() {
        let comb = make_test_comb_contour();
        let poly = Polygon::from_raw(&comb);
        let area = poly.area();

        let part_a = PartInput {
            id: Some("p_a".to_string()),
            entity_id: Some("ent_a".to_string()),
            name: Some("Comb A".to_string()),
            width: Some(600.0),
            height: Some(180.0),
            area: Some(area),
            contour: comb.clone(),
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
            has_grain_label: Some(true),
            grain_arrow_degrees: Some(0.0),
            free_rotation: Some(false),
            rotation_divisions: Some(2),
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
        };
        let part_b = part_a.clone();

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
        };

        let mating = find_best_mating(&part_a, &part_b, 6.0, Some(&config)).expect("Must find mating");
        let ox_a = mating.child_a.offset_x.unwrap();
        let oy_a = mating.child_a.offset_y.unwrap();
        let rot_a = mating.child_a.rotation_degrees.unwrap();
        let ox_b = mating.child_b.offset_x.unwrap();
        let oy_b = mating.child_b.offset_y.unwrap();
        let rot_b = mating.child_b.rotation_degrees.unwrap();

        let placed_a = poly.rotate_degrees(rot_a, Point::new(0.0, 0.0)).translate(ox_a, oy_a);
        let placed_b = poly.rotate_degrees(rot_b, Point::new(0.0, 0.0)).translate(ox_b, oy_b);

        // Verify min distance between any edge of A and B is >= 6.0mm (minus tiny float tolerance 1e-3)
        let n_a = placed_a.points.len();
        let n_b = placed_b.points.len();
        let mut min_dist = f64::MAX;
        for i in 0..n_a {
            let p1 = placed_a.points[i];
            let p2 = placed_a.points[(i + 1) % n_a];
            for j in 0..n_b {
                let q1 = placed_b.points[j];
                let q2 = placed_b.points[(j + 1) % n_b];
                let d = crate::geometry::segment_to_segment_distance(p1, p2, q1, q2);
                if d < min_dist {
                    min_dist = d;
                }
            }
        }
        assert!(min_dist >= 6.0 - 1e-3, "Minimum clearance must be >= 6.0mm, got {}", min_dist);
    }

    #[test]
    fn test_wood_grain_rotation_rules() {
        let part = PartInput {
            entity_id: Some("P1".to_string()),
            id: Some("1".to_string()),
            name: Some("Test Part".to_string()),
            width: Some(400.0),
            height: Some(600.0),
            area: Some(240000.0),
            contour: vec![
                [0.0, 0.0],
                [400.0, 0.0],
                [400.0, 600.0],
                [0.0, 600.0],
            ],
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
            grain_arrow_degrees: Some(90.0),
            free_rotation: Some(false),
            rotation_divisions: None,
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
        };

        // Case 1: Dropdown "Vân Gỗ" (rotation_divisions = 1)
        let config_van_go = ConfigurationInput {
            board_width: Some(1220.0),
            board_height: Some(2440.0),
            edge_margin: Some(10.0),
            cut_gap: Some(6.0),
            part_spacing: Some(6.0),
            rotation_divisions: Some(1),
            rotate_step: Some(180.0),
            compact_directions: Some(vec!["left".to_string(), "bottom".to_string()]),
            target_sheet_utilization: Some(90.0),
            sheet_in_sheet: Some(false),
            selective_repack: Some(false),
            small_part_threshold: Some(0.0),
            small_part_clearance: Some(0.0),
            small_part_edge_zone: Some(0.0),
            merge_cut_paths: Some(false),
        };

        let prepared_vg = crate::packing::prepare_parts(&[part.clone()], &config_van_go, 6.0);
        let rots_vg: Vec<f64> = prepared_vg[0].variants.iter().map(|v| v.rotation).collect();
        // MUST ONLY have 0 deg and 180 deg relative to base_rot (0.0 and 180.0)
        assert_eq!(rots_vg, vec![0.0, 180.0]);

        // Case 2: Dropdown "Xoay Tự Do" (rotation_divisions = 4) nhưng chi tiết CÓ VÂN GỖ
        // Quy tắc tuyệt đối: chi tiết có vân gỗ VẪN CHỈ ĐƯỢC XOAY 0 và 180 độ (0.0 và 180.0)
        let mut config_tu_do = config_van_go.clone();
        config_tu_do.rotation_divisions = Some(4);
        let prepared_td = crate::packing::prepare_parts(&[part.clone()], &config_tu_do, 6.0);
        let rots_td: Vec<f64> = prepared_td[0].variants.iter().map(|v| v.rotation).collect();
        assert_eq!(rots_td, vec![0.0, 180.0]);

        // Case 3: Chi tiết KHÔNG CÓ VÂN GỖ khi chọn "Xoay Tự Do" (rotation_divisions = 4)
        // Chi tiết không vân gỗ mới được phép xoay cả 4 hướng
        let mut non_grain_part = part;
        non_grain_part.has_grain_label = None;
        non_grain_part.grain_locked = None;
        let prepared_non_grain = crate::packing::prepare_parts(&[non_grain_part], &config_tu_do, 6.0);
        let rots_non_grain: Vec<f64> = prepared_non_grain[0].variants.iter().map(|v| v.rotation).collect();
        assert_eq!(rots_non_grain, vec![0.0, 90.0, 180.0, 270.0]);
    }

    #[test]
    fn test_comb_grain_signature_separation_and_macro_lock() {
        let comb = make_test_comb_contour();
        let poly = Polygon::from_raw(&comb);
        let area = poly.area();

        // Part 1: grain along length (base_rotation_degrees = 0.0)
        let part_0 = PartInput {
            id: Some("p_0".to_string()),
            entity_id: Some("ent_0".to_string()),
            name: Some("Comb 0".to_string()),
            width: Some(600.0),
            height: Some(180.0),
            area: Some(area),
            contour: comb.clone(),
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
            has_grain_label: Some(true),
            grain_arrow_degrees: Some(0.0),
            free_rotation: Some(false),
            rotation_divisions: Some(2),
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
        };

        // Part 2: identical shape, but grain along width (base_rotation_degrees = 90.0)
        let mut part_90 = part_0.clone();
        part_90.id = Some("p_90".to_string());
        part_90.entity_id = Some("ent_90".to_string());
        part_90.base_rotation_degrees = Some(90.0);
        part_90.rotation_degrees = Some(90.0);
        part_90.rotations = Some(vec![90.0, 270.0]);

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
        };

        // Pairing part_0 and part_90 must NOT pair because their grains are perpendicular!
        let (paired_incompatible, count_incompatible) = pair_comb_parts(&[part_0.clone(), part_90.clone()], &config, 6.0);
        assert_eq!(count_incompatible, 0, "Perpendicular grain parts must NOT be paired together!");
        assert_eq!(paired_incompatible.len(), 2);

        // Pairing two part_0 parts MUST succeed and result in a grain-locked macro with rotations [0, 180]
        let mut part_0_b = part_0.clone();
        part_0_b.id = Some("p_0_b".to_string());
        part_0_b.entity_id = Some("ent_0_b".to_string());
        let (paired_ok, count_ok) = pair_comb_parts(&[part_0.clone(), part_0_b], &config, 6.0);
        assert_eq!(count_ok, 1, "Compatible grain parts must be paired");
        let macro_part = &paired_ok[0];
        assert_eq!(macro_part.grain_locked, Some(true));
        assert_eq!(macro_part.base_rotation_degrees, Some(0.0));
        assert_eq!(macro_part.rotations, Some(vec![0.0, 180.0]));

        // Check prepare_parts on this macro: must strictly allow ONLY 0.0 and 180.0
        let prepared = crate::packing::prepare_parts(&[macro_part.clone()], &config, 6.0);
        let rots: Vec<f64> = prepared[0].variants.iter().map(|v| v.rotation).collect();
        assert_eq!(rots, vec![0.0, 180.0], "Macro must ONLY rotate 0 and 180 deg along sheet length");
    }

    #[test]
    fn test_002_corner_mating() {
        let contour = vec![
            [210.0, 185.0], [210.0, 400.0], [0.0, 400.0], [0.0, 165.772],
            [1.418, 144.134], [5.648, 122.867], [12.618, 102.334], [22.209, 82.886],
            [34.256, 64.857], [48.553, 48.554], [64.856, 34.256], [82.886, 22.209],
            [102.333, 12.619], [122.867, 5.649], [144.134, 1.418], [165.771, 0.0],
            [400.0, 0.0], [400.0, 185.0]
        ];
        let part_a = PartInput {
            id: Some("002_1".to_string()),
            entity_id: Some("ent_1".to_string()),
            name: Some("002_Group 1".to_string()),
            width: Some(400.0),
            height: Some(400.0),
            area: Some(113191.12),
            contour: contour.clone(),
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
            has_grain_label: Some(true),
            grain_arrow_degrees: Some(90.0),
            free_rotation: Some(false),
            rotation_divisions: Some(2),
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
        };
        let part_b = PartInput {
            id: Some("002_2".to_string()),
            entity_id: Some("ent_2".to_string()),
            name: Some("002_Group 2".to_string()),
            ..part_a.clone()
        };
        let config = ConfigurationInput {
            board_width: Some(1220.0),
            board_height: Some(2440.0),
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
        };
        let res = try_build_comb_pair(&part_a, &part_b, 6.0, Some(&config));
        assert!(res.is_some(), "002_Group corner parts must be successfully comb mated");
        let (macro_p, fill) = res.unwrap();
        assert!(fill > 0.85, "Fill rate should be high (>85%), got {}", fill);
        assert_eq!(macro_p.manual_cluster_macro, Some(true));
        assert_eq!(macro_p.manual_cluster_children.as_ref().map(|c| c.len()), Some(2));
    }
}


