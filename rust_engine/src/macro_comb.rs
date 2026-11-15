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

pub fn is_contour_curved(poly: &Polygon, _has_circular_arc: Option<bool>) -> bool {
    poly.is_circular()
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
    if is_contour_curved(&poly_a, part_a.has_circular_arc) || is_contour_curved(&poly_b, part_b.has_circular_arc) {
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
            let grain_a = part_a.has_grain_label == Some(true) || part_a.grain_locked == Some(true);
            let grain_b = part_b.has_grain_label == Some(true) || part_b.grain_locked == Some(true);
            let is_mating_grain_locked = global_grain_locked || grain_a || grain_b;
            let is_90 = !is_mating_grain_locked && (deg - 90.0).abs() <= 2.0;
            let is_270 = !is_mating_grain_locked && (deg - 270.0).abs() <= 2.0;
            if !is_180 && !is_0 && !is_90 && !is_270 {
                continue;
            }
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
                let is_arc_or_hollow = part_a.has_circular_arc == Some(true)
                    || part_b.has_circular_arc == Some(true)
                    || (orig_boxes_area > 1.0 && total_area / orig_boxes_area < 0.50);
                let saving_threshold = if is_arc_or_hollow { 0.90 } else { 0.85 };
                let is_compact_saving = orig_boxes_area > 1.0 && box_area <= orig_boxes_area * saving_threshold;
                let fill_rate = total_area / box_area;
                let min_fill = if is_compact_saving {
                    if is_arc_or_hollow { 0.18 } else { 0.25 }
                } else {
                    0.60
                };
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
    let check_spacing = spacing;
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
        two_sided: part_a.two_sided,
        manual_cluster_child: None,
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
        two_sided: part_b.two_sided,
        manual_cluster_child: None,
    };

    Some(CombMatingResult {
        macro_w: round3(macro_w),
        macro_h: round3(macro_h),
        fill_rate: round3(fill_rate),
        child_a,
        child_b,
    })
}

pub fn extract_macro_cavity_holes(
    children: &[ClusterChild],
    macro_w: f64,
    macro_h: f64,
) -> Option<Vec<Vec<[f64; 2]>>> {
    if children.is_empty() || macro_w <= 60.0 || macro_h <= 60.0 {
        return None;
    }
    let round3 = |v: f64| (v * 1000.0).round() / 1000.0;
    let mut child_polys: Vec<Polygon> = Vec::with_capacity(children.len());
    for child in children {
        let raw = match child.contour.as_ref() {
            Some(c) if c.len() >= 3 => c,
            _ => continue,
        };
        let ox = child.offset_x.unwrap_or(0.0);
        let oy = child.offset_y.unwrap_or(0.0);
        let rot = if child.manual_cluster_child == Some(true) {
            0.0
        } else {
            child.rotation_degrees.unwrap_or(0.0)
        };
        let base_poly = Polygon::from_raw(raw);
        let cand = base_poly.rotate_degrees(rot, Point::new(0.0, 0.0)).translate(ox, oy);
        let cb = cand.bounding_box();
        let poly = if cb.min_x >= -5.0 && cb.min_y >= -5.0 && cb.max_x <= macro_w + 5.0 && cb.max_y <= macro_h + 5.0 {
            cand
        } else {
            let (norm_p, _, _) = base_poly.rotate_degrees(rot, Point::new(0.0, 0.0)).normalize_to_origin();
            norm_p.translate(ox, oy)
        };
        let decimated = if poly.points.len() > 64 {
            decimate_curved_polygon(&poly.points, 6.0)
        } else {
            poly
        };
        child_polys.push(decimated);
    }
    if child_polys.is_empty() {
        return None;
    }

    let is_rect_free = |rx1: f64, ry1: f64, rx2: f64, ry2: f64| -> bool {
        if rx1 < -1e-4 || ry1 < -1e-4 || rx2 > macro_w + 1e-4 || ry2 > macro_h + 1e-4 {
            return false;
        }
        if rx2 - rx1 < 20.0 || ry2 - ry1 < 20.0 {
            return false;
        }
        let eps = 0.15;
        let rect_poly = Polygon::new(vec![
            Point::new(rx1 + eps, ry1 + eps),
            Point::new(rx2 - eps, ry1 + eps),
            Point::new(rx2 - eps, ry2 - eps),
            Point::new(rx1 + eps, ry2 - eps),
        ]);
        for cp in &child_polys {
            if polygons_collide_concave_safe(&rect_poly, cp, 0.0) {
                return false;
            }
        }
        true
    };

    let nx = 28usize;
    let ny = 28usize;
    let dx = macro_w / (nx as f64);
    let dy = macro_h / (ny as f64);
    let mut free_grid = vec![vec![false; nx]; ny];
    for iy in 0..ny {
        let cy0 = (iy as f64) * dy;
        for ix in 0..nx {
            let cx0 = (ix as f64) * dx;
            let sample_pts = [
                Point::new(cx0 + dx * 0.5, cy0 + dy * 0.5),
                Point::new(cx0 + dx * 0.2, cy0 + dy * 0.2),
                Point::new(cx0 + dx * 0.8, cy0 + dy * 0.2),
                Point::new(cx0 + dx * 0.8, cy0 + dy * 0.8),
                Point::new(cx0 + dx * 0.2, cy0 + dy * 0.8),
            ];
            free_grid[iy][ix] = sample_pts.iter().all(|pt| {
                child_polys.iter().all(|cp| !cp.contains_point_strict(*pt, 0.0))
            });
        }
    }

    let mut heights = vec![0usize; nx];
    let mut raw_candidates: Vec<(f64, f64, f64, f64, f64)> = Vec::new();
    for iy in 0..ny {
        for ix in 0..nx {
            heights[ix] = if free_grid[iy][ix] { heights[ix] + 1 } else { 0 };
        }
        for ix1 in 0..nx {
            if heights[ix1] == 0 {
                continue;
            }
            let mut min_h = heights[ix1];
            for ix2 in ix1..nx {
                if heights[ix2] == 0 {
                    break;
                }
                if heights[ix2] < min_h {
                    min_h = heights[ix2];
                }
                let rw = ((ix2 - ix1 + 1) as f64) * dx;
                let rh = (min_h as f64) * dy;
                let area = rw * rh;
                if rw.min(rh) >= 32.0 && rw.max(rh) >= 80.0 && area >= 4000.0 {
                    let rx1 = (ix1 as f64) * dx;
                    let ry1 = ((iy + 1 - min_h) as f64) * dy;
                    let rx2 = ((ix2 + 1) as f64) * dx;
                    let ry2 = ((iy + 1) as f64) * dy;
                    raw_candidates.push((area, rx1, ry1, rx2, ry2));
                }
            }
        }
    }
    raw_candidates.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

    let expand_rect = |mut x1: f64, mut y1: f64, mut x2: f64, mut y2: f64, x_first: bool| -> (f64, f64, f64, f64) {
        for &step in &[8.0, 4.0, 2.0, 1.0, 0.5, 0.25] {
            if x_first {
                while x1 - step >= 0.0 && is_rect_free(x1 - step, y1, x2, y2) { x1 -= step; }
                while x2 + step <= macro_w && is_rect_free(x1, y1, x2 + step, y2) { x2 += step; }
                while y1 - step >= 0.0 && is_rect_free(x1, y1 - step, x2, y2) { y1 -= step; }
                while y2 + step <= macro_h && is_rect_free(x1, y1, x2, y2 + step) { y2 += step; }
            } else {
                while y1 - step >= 0.0 && is_rect_free(x1, y1 - step, x2, y2) { y1 -= step; }
                while y2 + step <= macro_h && is_rect_free(x1, y1, x2, y2 + step) { y2 += step; }
                while x1 - step >= 0.0 && is_rect_free(x1 - step, y1, x2, y2) { x1 -= step; }
                while x2 + step <= macro_w && is_rect_free(x1, y1, x2 + step, y2) { x2 += step; }
            }
        }
        (x1, y1, x2, y2)
    };

    let mut refined: Vec<(f64, f64, f64, f64)> = Vec::new();
    let push_refined = |refined: &mut Vec<(f64, f64, f64, f64)>, fx1: f64, fy1: f64, fx2: f64, fy2: f64| {
        let fw = fx2 - fx1;
        let fh = fy2 - fy1;
        let f_area = fw * fh;
        if fw < 62.0 || fh < 140.0 || f_area < 9000.0 {
            return;
        }
        let dup = refined.iter().any(|&(ex1, ey1, ex2, ey2)| {
            let iw = (fx2.min(ex2) - fx1.max(ex1)).max(0.0);
            let ih = (fy2.min(ey2) - fy1.max(ey1)).max(0.0);
            let e_area = (ex2 - ex1) * (ey2 - ey1);
            iw * ih >= f_area.min(e_area) * 0.90
        });
        if !dup {
            refined.push((fx1, fy1, fx2, fy2));
        }
    };

    for &h_target in &[149.5, 297.5, 445.5, 593.5] {
        if h_target <= macro_h * 0.45 && macro_w >= 80.0 {
            if is_rect_free(0.0, 0.0, 36.0, h_target) {
                let (ax1, ay1, ax2, ay2) = expand_rect(0.0, 0.0, 36.0, h_target, true);
                push_refined(&mut refined, ax1, ay1, ax2, ay2);
            }
            if is_rect_free(macro_w - 36.0, 0.0, macro_w, h_target) {
                let (ax1, ay1, ax2, ay2) = expand_rect(macro_w - 36.0, 0.0, macro_w, h_target, true);
                push_refined(&mut refined, ax1, ay1, ax2, ay2);
            }
            let ty1 = macro_h - h_target;
            if is_rect_free(0.0, ty1, 36.0, macro_h) {
                let (ax1, ay1, ax2, ay2) = expand_rect(0.0, ty1, 36.0, macro_h, true);
                push_refined(&mut refined, ax1, ay1, ax2, ay2);
            }
            if is_rect_free(macro_w - 36.0, ty1, macro_w, macro_h) {
                let (ax1, ay1, ax2, ay2) = expand_rect(macro_w - 36.0, ty1, macro_w, macro_h, true);
                push_refined(&mut refined, ax1, ay1, ax2, ay2);
            }
        }
    }

    for &w_target in &[73.5, 146.0, 218.0, 290.0, 362.0, 434.0] {
        if w_target <= macro_w * 0.45 && macro_h >= 200.0 {
            let sy1 = macro_h * 0.46;
            let sy2 = macro_h * 0.54;
            if is_rect_free(macro_w - w_target, sy1, macro_w, sy2) {
                let (bx1, by1, bx2, by2) = expand_rect(macro_w - w_target, sy1, macro_w, sy2, false);
                push_refined(&mut refined, bx1, by1, bx2, by2);
            }
            if is_rect_free(0.0, sy1, w_target, sy2) {
                let (bx1, by1, bx2, by2) = expand_rect(0.0, sy1, w_target, sy2, false);
                push_refined(&mut refined, bx1, by1, bx2, by2);
            }
        }
    }

    for &(f_lo, f_hi) in &[
        (0.43, 0.57),
        (0.37, 0.63),
        (0.31, 0.69),
        (0.25, 0.75),
        (0.19, 0.81),
        (0.0, 0.09),
        (0.0, 0.15),
        (0.0, 0.21),
        (0.0, 0.28),
        (0.91, 1.0),
        (0.85, 1.0),
        (0.79, 1.0),
        (0.72, 1.0),
    ] {
        let sy1 = macro_h * f_lo;
        let sy2 = macro_h * f_hi;
        if sy2 - sy1 >= 140.0 && macro_w >= 80.0 {
            if is_rect_free(0.0, sy1, 36.0, sy2) {
                let (ax1, ay1, ax2, ay2) = expand_rect(0.0, sy1, 36.0, sy2, true);
                push_refined(&mut refined, ax1, ay1, ax2, ay2);
            }
            if is_rect_free(macro_w - 36.0, sy1, macro_w, sy2) {
                let (ax1, ay1, ax2, ay2) = expand_rect(macro_w - 36.0, sy1, macro_w, sy2, true);
                push_refined(&mut refined, ax1, ay1, ax2, ay2);
            }
        }
    }

    for &(fx_lo, fx_hi) in &[(0.42, 0.74), (0.26, 0.58)] {
        let sx1 = macro_w * fx_lo;
        let sx2 = macro_w * fx_hi;
        if sx2 - sx1 >= 80.0 && macro_h >= 200.0 {
            if is_rect_free(sx1, 0.0, sx2, 60.0) {
                let (bx1, by1, bx2, by2) = expand_rect(sx1, 0.0, sx2, 60.0, false);
                push_refined(&mut refined, bx1, by1, bx2, by2);
            }
            if is_rect_free(sx1, macro_h - 60.0, sx2, macro_h) {
                let (bx1, by1, bx2, by2) = expand_rect(sx1, macro_h - 60.0, sx2, macro_h, false);
                push_refined(&mut refined, bx1, by1, bx2, by2);
            }
        }
    }

    for &(_, mut x1, mut y1, mut x2, mut y2) in raw_candidates.iter() {
        if refined.len() >= 32 {
            break;
        }
        let c_area = (x2 - x1) * (y2 - y1);
        if refined.iter().any(|&(ex1, ey1, ex2, ey2)| {
            let iw = (x2.min(ex2) - x1.max(ex1)).max(0.0);
            let ih = (y2.min(ey2) - y1.max(ey1)).max(0.0);
            iw * ih >= c_area * 0.85
        }) {
            continue;
        }
        let mut valid = is_rect_free(x1, y1, x2, y2);
        for _ in 0..4 {
            if valid { break; }
            x1 += 2.0; y1 += 2.0; x2 -= 2.0; y2 -= 2.0;
            valid = is_rect_free(x1, y1, x2, y2);
        }
        if !valid {
            continue;
        }
        let (ax1, ay1, ax2, ay2) = expand_rect(x1, y1, x2, y2, true);
        push_refined(&mut refined, ax1, ay1, ax2, ay2);
        let (bx1, by1, bx2, by2) = expand_rect(x1, y1, x2, y2, false);
        push_refined(&mut refined, bx1, by1, bx2, by2);
    }

    refined.sort_by(|a, b| {
        let area_a = (a.2 - a.0) * (a.3 - a.1);
        let area_b = (b.2 - b.0) * (b.3 - b.1);
        area_b.partial_cmp(&area_a).unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut primary_pockets: Vec<(f64, f64, f64, f64)> = Vec::new();
    for (x1, y1, x2, y2) in refined {
        let area = (x2 - x1) * (y2 - y1);
        let overlaps = primary_pockets.iter().any(|&(ex1, ey1, ex2, ey2)| {
            let iw = (x2.min(ex2) - x1.max(ex1)).max(0.0);
            let ih = (y2.min(ey2) - y1.max(ey1)).max(0.0);
            iw * ih > area * 0.90
        });
        if !overlaps {
            primary_pockets.push((x1, y1, x2, y2));
            if primary_pockets.len() >= 24 {
                break;
            }
        }
    }

    let mut holes: Vec<Vec<[f64; 2]>> = Vec::new();
    for &(x1, y1, x2, y2) in &primary_pockets {
        let rx1 = if x1 <= 1.5 { 0.0 } else { round3(x1) };
        let ry1 = if y1 <= 1.5 { 0.0 } else { round3(y1) };
        let rx2 = if (macro_w - x2) <= 1.5 { round3(macro_w) } else { round3(x2) };
        let ry2 = if (macro_h - y2) <= 1.5 { round3(macro_h) } else { round3(y2) };
        holes.push(vec![[rx1, ry1], [rx2, ry1], [rx2, ry2], [rx1, ry2]]);
    }

    if holes.is_empty() {
        None
    } else {
        holes.truncate(24);
        Some(holes)
    }
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

    let children = vec![mating.child_a, mating.child_b];
    let macro_holes = extract_macro_cavity_holes(&children, mating.macro_w, mating.macro_h);

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
        holes: macro_holes,
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
        manual_cluster_children: Some(children),
        two_sided: Some(part_a.two_sided == Some(true) || part_b.two_sided == Some(true)),
        has_circular_arc: Some(false),
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
        two_sided: part_a.two_sided,
        manual_cluster_child: None,
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
        two_sided: part_b.two_sided,
        manual_cluster_child: None,
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

    let children = vec![child_a, child_b];
    let macro_holes = extract_macro_cavity_holes(&children, offsets.macro_w, offsets.macro_h);

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
        holes: macro_holes,
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
        manual_cluster_children: Some(children),
        two_sided: Some(part_a.two_sided == Some(true) || part_b.two_sided == Some(true)),
        has_circular_arc: Some(false),
    }
}

pub fn polygons_collide_concave_safe(poly1: &Polygon, poly2: &Polygon, spacing: f64) -> bool {
    let bbox1 = poly1.bounding_box();
    let bbox2 = poly2.bounding_box();
    let pad = spacing.max(0.0) - 1e-4;
    if bbox1.min_x >= bbox2.max_x + pad
        || bbox1.max_x <= bbox2.min_x - pad
        || bbox1.min_y >= bbox2.max_y + pad
        || bbox1.max_y <= bbox2.min_y - pad
    {
        return false;
    }
    if poly1.is_rectangular() && poly2.is_rectangular() {
        return true;
    }
    let n1 = poly1.points.len();
    let n2 = poly2.points.len();
    if n1 < 3 || n2 < 3 {
        return false;
    }
    for i in 0..n1 {
        let p1 = poly1.points[i];
        let p2 = poly1.points[(i + 1) % n1];
        let p_min_x = p1.x.min(p2.x);
        let p_max_x = p1.x.max(p2.x);
        if p_max_x <= bbox2.min_x - pad || p_min_x >= bbox2.max_x + pad {
            continue;
        }
        let p_min_y = p1.y.min(p2.y);
        let p_max_y = p1.y.max(p2.y);
        if p_max_y <= bbox2.min_y - pad || p_min_y >= bbox2.max_y + pad {
            continue;
        }
        for j in 0..n2 {
            let q1 = poly2.points[j];
            let q2 = poly2.points[(j + 1) % n2];
            if p_max_x <= q1.x.min(q2.x) - pad
                || p_min_x >= q1.x.max(q2.x) + pad
                || p_max_y <= q1.y.min(q2.y) - pad
                || p_min_y >= q1.y.max(q2.y) + pad
            {
                continue;
            }
            if spacing > 1e-4 {
                if crate::geometry::segment_to_segment_distance(p1, p2, q1, q2) < pad {
                    return true;
                }
            } else if crate::geometry::segments_intersect_proper(p1, p2, q1, q2) {
                return true;
            }
        }
    }
    if spacing > 1e-4 {
        if bbox2.contains_rect(&bbox1, 1e-4) && poly2.contains_point_strict(poly1.points[0], 0.0) {
            return true;
        }
        if bbox1.contains_rect(&bbox2, 1e-4) && poly1.contains_point_strict(poly2.points[0], 0.0) {
            return true;
        }
        return false;
    }
    for pt in &poly1.points {
        if pt.x > bbox2.min_x && pt.x < bbox2.max_x && pt.y > bbox2.min_y && pt.y < bbox2.max_y && poly2.contains_point_strict(*pt, 0.0) {
            return true;
        }
    }
    for pt in &poly2.points {
        if pt.x > bbox1.min_x && pt.x < bbox1.max_x && pt.y > bbox1.min_y && pt.y < bbox1.max_y && poly1.contains_point_strict(*pt, 0.0) {
            return true;
        }
    }
    let c2 = poly2.centroid();
    if c2.x > bbox1.min_x && c2.x < bbox1.max_x && c2.y > bbox1.min_y && c2.y < bbox1.max_y && poly2.contains_point_strict(c2, 0.0) && poly1.contains_point_strict(c2, 0.0) {
        return true;
    }
    let c1 = poly1.centroid();
    if c1.x > bbox2.min_x && c1.x < bbox2.max_x && c1.y > bbox2.min_y && c1.y < bbox2.max_y && poly1.contains_point_strict(c1, 0.0) && poly2.contains_point_strict(c1, 0.0) {
        return true;
    }
    false
}

pub fn decimate_curved_polygon(points: &[Point], max_chord: f64) -> Polygon {
    if points.len() <= 40 {
        return Polygon::new(points.to_vec());
    }
    let n = points.len();
    let mut out = Vec::with_capacity(48);
    out.push(points[0]);
    let max_chord_sq = max_chord * max_chord;
    for i in 1..n {
        let prev = points[i - 1];
        let curr = points[i];
        let next = points[(i + 1) % n];
        let last_kept = *out.last().unwrap();
        let dist_sq = (curr.x - last_kept.x).powi(2) + (curr.y - last_kept.y).powi(2);
        let ux = curr.x - prev.x;
        let uy = curr.y - prev.y;
        let vx = next.x - curr.x;
        let vy = next.y - curr.y;
        let len_u = ux.hypot(uy);
        let len_v = vx.hypot(vy);
        let is_sharp_corner = if len_u > 1e-5 && len_v > 1e-5 {
            (ux * vx + uy * vy) / (len_u * len_v) < 0.96
        } else {
            false
        };
        if is_sharp_corner || dist_sq >= max_chord_sq {
            out.push(curr);
        }
    }
    if out.len() >= 3 {
        Polygon::new(out)
    } else {
        Polygon::new(points.to_vec())
    }
}

pub fn find_parallel_nesting_vector(
    poly: &Polygon,
    spacing: f64,
    max_bw: f64,
    max_bh: f64,
) -> Option<(f64, f64, f64)> {
    let bbox = poly.bounding_box();
    let w0 = bbox.width();
    let h0 = bbox.height();
    let area = poly.area();
    let bbox_area = w0 * h0;
    if bbox_area <= 1.0 || w0 < 20.0 || h0 < 20.0 {
        return None;
    }

    let fill_ratio = area / bbox_area;
    // Must have hollow/concavity or be curved (elbows, 1/4 arc, 1/2 arc, chevrons)
    let is_candidate = fill_ratio < 0.88 || poly.points.len() >= 6;
    if !is_candidate {
        return None;
    }

    let mut best_vector: Option<(f64, f64, f64)> = None;
    let mut min_score = f64::MAX;

    let sp = spacing.max(0.0);
    let r_max = (w0.hypot(h0)).min(w0.max(h0) * 1.5);
    if r_max < sp {
        return None;
    }

    let num_angles = 120;
    let dr = (sp * 0.5).clamp(2.0, 6.0);
    let coarse_poly = decimate_curved_polygon(&poly.points, 16.0);

    for step in 0..num_angles {
        let angle = (step as f64) * 2.0 * PI / (num_angles as f64);
        let ux = angle.cos();
        let uy = angle.sin();

        // 1. Ray march to find the first collision-free distance along this ray
        let mut found_r = None;
        let mut r = sp.max(1.0);
        while r <= r_max {
            let shifted = coarse_poly.translate(r * ux, r * uy);
            if !polygons_collide_concave_safe(&coarse_poly, &shifted, sp) {
                found_r = Some(r);
                break;
            }
            r += dr;
        }

        let r_coarse = match found_r {
            Some(rc) => rc,
            None => continue,
        };

        // 2. Binary search refine to within 0.1mm
        let mut lo = (r_coarse - dr).max(sp);
        let mut hi = r_coarse;
        for _ in 0..8 {
            let mid = (lo + hi) * 0.5;
            let shifted = coarse_poly.translate(mid * ux, mid * uy);
            if polygons_collide_concave_safe(&coarse_poly, &shifted, sp) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let r_exact = hi + 0.35;
        let dx = r_exact * ux;
        let dy = r_exact * uy;

        // 3. Check bounding box overlap condition:
        // For true nested stack, parts penetrate each other's bounding box
        if dx.abs() >= w0 - sp && dy.abs() >= h0 - sp {
            continue;
        }

        // 4. Compute bounding box of pair & multi-part stack
        let min_x = bbox.min_x.min(bbox.min_x + dx);
        let max_x = bbox.max_x.max(bbox.max_x + dx);
        let min_y = bbox.min_y.min(bbox.min_y + dy);
        let max_y = bbox.max_y.max(bbox.max_y + dy);
        let pair_w = max_x - min_x;
        let pair_h = max_y - min_y;

        if (pair_w > max_bw || pair_h > max_bh) && (pair_w > max_bh || pair_h > max_bw) {
            continue;
        }

        let pair_area = pair_w * pair_h;
        let ratio = pair_area / bbox_area;
        if ratio >= 1.82 {
            continue;
        }

        let (stack6_w, stack6_h) = compute_stack_dimensions(poly, 6, dx, dy);
        let stack6_area_ratio = (stack6_w * stack6_h) / (6.0 * bbox_area);
        let cardinal_bonus = if dx.abs() < 0.1 || dy.abs() < 0.1 {
            0.70
        } else if dx.abs() < 0.25 || dy.abs() < 0.25 {
            0.88
        } else {
            1.0
        };
        let stack_fit_bonus = if (stack6_w <= max_bw && stack6_h <= max_bh) || (stack6_w <= max_bh && stack6_h <= max_bw) {
            1.0
        } else {
            1.25
        };
        let score = (ratio * 0.35 + stack6_area_ratio * 0.65) * cardinal_bonus * stack_fit_bonus;

        if score < min_score {
            // Verify on exact poly that 1 step and 2 steps remain collision-free
            let mut r_ver = r_exact;
            let mut verified = None;
            for _ in 0..4 {
                let vdx = r_ver * ux;
                let vdy = r_ver * uy;
                let s1 = poly.translate(vdx, vdy);
                let s2 = poly.translate(2.0 * vdx, 2.0 * vdy);
                if !polygons_collide_concave_safe(poly, &s1, sp)
                    && !polygons_collide_concave_safe(poly, &s2, sp)
                {
                    verified = Some((vdx, vdy));
                    break;
                }
                r_ver += 0.5;
            }
            if let Some((vdx, vdy)) = verified {
                min_score = score;
                best_vector = Some((vdx, vdy, ratio));
            }
        }
    }

    best_vector
}

pub fn compute_stack_dimensions(poly: &Polygon, count: usize, dx: f64, dy: f64) -> (f64, f64) {
    let bbox = poly.bounding_box();
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    for k in 0..count {
        let shift_x = k as f64 * dx;
        let shift_y = k as f64 * dy;
        min_x = min_x.min(bbox.min_x + shift_x);
        max_x = max_x.max(bbox.max_x + shift_x);
        min_y = min_y.min(bbox.min_y + shift_y);
        max_y = max_y.max(bbox.max_y + shift_y);
    }
    (max_x - min_x, max_y - min_y)
}

fn build_stack_profile_contour(
    base_poly: &Polygon,
    count: usize,
    dx: f64,
    dy: f64,
    min_x: f64,
    min_y: f64,
    macro_w: f64,
    macro_h: f64,
) -> Vec<[f64; 2]> {
    let _round3 = |v: f64| (v * 1000.0).round() / 1000.0;

    let shifted_polys: Vec<Polygon> = (0..count)
        .map(|k| base_poly.translate((k as f64 * dx) - min_x, (k as f64 * dy) - min_y))
        .collect();

    build_polys_profile_contour(&shifted_polys, macro_w, macro_h)
}

pub fn build_polys_profile_contour(
    shifted_polys: &[Polygon],
    macro_w: f64,
    macro_h: f64,
) -> Vec<[f64; 2]> {
    let round3 = |v: f64| (v * 1000.0).round() / 1000.0;
    let rect_fallback = vec![
        [0.0, 0.0],
        [macro_w, 0.0],
        [macro_w, macro_h],
        [0.0, macro_h],
    ];
    if shifted_polys.len() < 2 || macro_w < 20.0 || macro_h < 20.0 {
        return rect_fallback;
    }

    let num_slices = 36usize;

    // 1. X-slice envelope (vertical slices along [0, macro_w])
    let step_x = macro_w / (num_slices as f64);
    let mut bot_y = vec![macro_h; num_slices];
    let mut top_y = vec![0.0f64; num_slices];
    for sp in shifted_polys {
        let n = sp.points.len();
        for i in 0..n {
            let p1 = sp.points[i];
            let p2 = sp.points[(i + 1) % n];
            let seg_min_x = p1.x.min(p2.x);
            let seg_max_x = p1.x.max(p2.x);
            let s_start = ((seg_min_x / step_x).floor() as isize).clamp(0, (num_slices - 1) as isize) as usize;
            let s_end = ((seg_max_x / step_x).floor() as isize).clamp(0, (num_slices - 1) as isize) as usize;
            for s in s_start..=s_end {
                let x_lo = (s as f64) * step_x;
                let x_hi = if s + 1 == num_slices { macro_w } else { ((s + 1) as f64) * step_x };
                if p1.x >= x_lo - 1e-4 && p1.x <= x_hi + 1e-4 {
                    bot_y[s] = bot_y[s].min(p1.y);
                    top_y[s] = top_y[s].max(p1.y);
                }
                if p2.x >= x_lo - 1e-4 && p2.x <= x_hi + 1e-4 {
                    bot_y[s] = bot_y[s].min(p2.y);
                    top_y[s] = top_y[s].max(p2.y);
                }
                let seg_dx = p2.x - p1.x;
                if seg_dx.abs() > 1e-6 {
                    for &bx in &[x_lo, x_hi] {
                        let t = (bx - p1.x) / seg_dx;
                        if t >= 0.0 && t <= 1.0 {
                            let y_at = p1.y + t * (p2.y - p1.y);
                            bot_y[s] = bot_y[s].min(y_at);
                            top_y[s] = top_y[s].max(y_at);
                        }
                    }
                }
            }
        }
    }

    let mut x_pts = Vec::with_capacity(num_slices * 4);
    for s in 0..num_slices {
        if bot_y[s] <= top_y[s] {
            let x_lo = if s == 0 { 0.0 } else { (s as f64) * step_x };
            let x_hi = if s + 1 == num_slices { macro_w } else { ((s + 1) as f64) * step_x };
            let by = bot_y[s].clamp(0.0, macro_h);
            x_pts.push(Point::new(round3(x_lo), round3(by)));
            x_pts.push(Point::new(round3(x_hi), round3(by)));
        }
    }
    for s in (0..num_slices).rev() {
        if bot_y[s] <= top_y[s] {
            let x_lo = if s == 0 { 0.0 } else { (s as f64) * step_x };
            let x_hi = if s + 1 == num_slices { macro_w } else { ((s + 1) as f64) * step_x };
            let ty = top_y[s].clamp(0.0, macro_h);
            x_pts.push(Point::new(round3(x_hi), round3(ty)));
            x_pts.push(Point::new(round3(x_lo), round3(ty)));
        }
    }
    let x_simp = simplify_collinear(&x_pts, 0.5);
    let x_poly = Polygon::new(x_simp);

    // 2. Y-slice envelope (horizontal slices along [0, macro_h])
    let step_y = macro_h / (num_slices as f64);
    let mut left_x = vec![macro_w; num_slices];
    let mut right_x = vec![0.0f64; num_slices];
    for sp in shifted_polys {
        let n = sp.points.len();
        for i in 0..n {
            let p1 = sp.points[i];
            let p2 = sp.points[(i + 1) % n];
            let seg_min_y = p1.y.min(p2.y);
            let seg_max_y = p1.y.max(p2.y);
            let s_start = ((seg_min_y / step_y).floor() as isize).clamp(0, (num_slices - 1) as isize) as usize;
            let s_end = ((seg_max_y / step_y).floor() as isize).clamp(0, (num_slices - 1) as isize) as usize;
            for s in s_start..=s_end {
                let y_lo = (s as f64) * step_y;
                let y_hi = if s + 1 == num_slices { macro_h } else { ((s + 1) as f64) * step_y };
                if p1.y >= y_lo - 1e-4 && p1.y <= y_hi + 1e-4 {
                    left_x[s] = left_x[s].min(p1.x);
                    right_x[s] = right_x[s].max(p1.x);
                }
                if p2.y >= y_lo - 1e-4 && p2.y <= y_hi + 1e-4 {
                    left_x[s] = left_x[s].min(p2.x);
                    right_x[s] = right_x[s].max(p2.x);
                }
                let seg_dy = p2.y - p1.y;
                if seg_dy.abs() > 1e-6 {
                    for &by in &[y_lo, y_hi] {
                        let t = (by - p1.y) / seg_dy;
                        if t >= 0.0 && t <= 1.0 {
                            let x_at = p1.x + t * (p2.x - p1.x);
                            left_x[s] = left_x[s].min(x_at);
                            right_x[s] = right_x[s].max(x_at);
                        }
                    }
                }
            }
        }
    }

    let mut y_pts = Vec::with_capacity(num_slices * 4);
    for s in 0..num_slices {
        if left_x[s] <= right_x[s] {
            let y_lo = if s == 0 { 0.0 } else { (s as f64) * step_y };
            let y_hi = if s + 1 == num_slices { macro_h } else { ((s + 1) as f64) * step_y };
            let rx = right_x[s].clamp(0.0, macro_w);
            y_pts.push(Point::new(round3(rx), round3(y_lo)));
            y_pts.push(Point::new(round3(rx), round3(y_hi)));
        }
    }
    for s in (0..num_slices).rev() {
        if left_x[s] <= right_x[s] {
            let y_lo = if s == 0 { 0.0 } else { (s as f64) * step_y };
            let y_hi = if s + 1 == num_slices { macro_h } else { ((s + 1) as f64) * step_y };
            let lx = left_x[s].clamp(0.0, macro_w);
            y_pts.push(Point::new(round3(lx), round3(y_hi)));
            y_pts.push(Point::new(round3(lx), round3(y_lo)));
        }
    }
    let y_simp = simplify_collinear(&y_pts, 0.5);
    let y_poly = Polygon::new(y_simp);

    let rect_area = macro_w * macro_h;
    let x_area = x_poly.area();
    let y_area = y_poly.area();

    let best_poly = if macro_h >= macro_w * 1.15 && y_poly.points.len() >= 4 && y_area > 1.0 && y_area < rect_area * 0.96 {
        Some((y_poly, y_area))
    } else if macro_w >= macro_h * 1.15 && x_poly.points.len() >= 4 && x_area > 1.0 && x_area < rect_area * 0.96 {
        Some((x_poly, x_area))
    } else if x_poly.points.len() >= 4 && x_area > 1.0 && (x_area <= y_area || y_poly.points.len() < 4) {
        Some((x_poly, x_area))
    } else if y_poly.points.len() >= 4 && y_area > 1.0 {
        Some((y_poly, y_area))
    } else {
        None
    };

    if let Some((bp, b_area)) = best_poly {
        let bb = bp.bounding_box();
        if b_area < rect_area * 0.96
            && (bb.width() - macro_w).abs() < 1.0
            && (bb.height() - macro_h).abs() < 1.0
        {
            return bp.to_raw();
        }
    }

    rect_fallback
}

pub fn create_parallel_nested_stack_macro(
    stack_parts: &[PartInput],
    dx: f64,
    dy: f64,
    config: Option<&ConfigurationInput>,
) -> PartInput {
    let round3 = |v: f64| (v * 1000.0).round() / 1000.0;
    let first = &stack_parts[0];
    let poly = Polygon::from_raw(&first.contour);
    let bbox = poly.bounding_box();

    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    for k in 0..stack_parts.len() {
        let shift_x = k as f64 * dx;
        let shift_y = k as f64 * dy;
        min_x = min_x.min(bbox.min_x + shift_x);
        max_x = max_x.max(bbox.max_x + shift_x);
        min_y = min_y.min(bbox.min_y + shift_y);
        max_y = max_y.max(bbox.max_y + shift_y);
    }

    let stack_w = max_x - min_x;
    let stack_h = max_y - min_y;

    let mut children = Vec::with_capacity(stack_parts.len());
    for (k, p) in stack_parts.iter().enumerate() {
        let ox = (k as f64 * dx) - min_x;
        let oy = (k as f64 * dy) - min_y;
        children.push(ClusterChild {
            entity_id: p.entity_id.clone(),
            id: p.id.clone(),
            name: p.name.clone(),
            offset_x: Some(round3(ox)),
            offset_y: Some(round3(oy)),
            rotation_degrees: Some(0.0),
            width: p.width,
            height: p.height,
            area: p.area,
            contour: Some(p.contour.clone()),
            holes: p.holes.clone(),
            render_contour: p.render_contour.clone(),
            render_holes: p.render_holes.clone(),
            collision_contour: p.collision_contour.clone(),
            collision_holes: p.collision_holes.clone(),
            color: p.color.clone(),
            draw_layers: p.draw_layers.clone(),
            has_grain_label: p.has_grain_label,
            grain_arrow_degrees: p.grain_arrow_degrees,
            base_rotation_degrees: p.base_rotation_degrees,
            two_sided: p.two_sided,
            manual_cluster_child: None,
        });
    }

    let global_grain_locked = config.and_then(|c| c.rotation_divisions).map_or(true, |d| d <= 2);
    let is_grain_locked = global_grain_locked
        || stack_parts.iter().any(|p| {
            p.grain_locked == Some(true)
                || p.has_grain_label == Some(true)
                || p.grain_arrow_degrees.is_some()
        });

    let macro_base_rot = 0.0;
    let ref_grain_arrow = first.grain_arrow_degrees;

    let total_area: f64 = stack_parts.iter().map(|p| p.area.unwrap_or(0.0)).sum();
    let first_name = first.name.as_deref().unwrap_or("Part");
    let macro_name = format!("Cụm cong xếp lồng: {} ({} chi tiết)", first_name, stack_parts.len());
    let child_ids: Vec<&str> = stack_parts.iter().map(|p| p.id.as_deref().unwrap_or("")).collect();
    let macro_id = child_ids.join("+");
    let macro_eid = format!("arc-macro:{}", macro_id);

    let macro_w = round3(stack_w);
    let macro_h = round3(stack_h);
    let macro_holes = extract_macro_cavity_holes(&children, macro_w, macro_h);
    let profile_contour = if macro_holes.is_some() {
        vec![
            [0.0, 0.0],
            [macro_w, 0.0],
            [macro_w, macro_h],
            [0.0, macro_h],
        ]
    } else {
        build_stack_profile_contour(
            &poly,
            stack_parts.len(),
            dx,
            dy,
            min_x,
            min_y,
            macro_w,
            macro_h,
        )
    };

    PartInput {
        entity_id: Some(macro_eid),
        id: Some(macro_id),
        name: Some(macro_name),
        width: Some(macro_w),
        height: Some(macro_h),
        area: Some(round3(total_area)),
        contour: profile_contour,
        holes: macro_holes,
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
        has_grain_label: Some(is_grain_locked),
        grain_arrow_degrees: if is_grain_locked { ref_grain_arrow } else { None },
        free_rotation: Some(!is_grain_locked),
        rotation_divisions: Some(if is_grain_locked { 2 } else { 4 }),
        color: first.color.clone(),
        logical_part_count: Some(stack_parts.len()),
        manual_cluster_macro: Some(true),
        manual_cluster_children: Some(children),
        two_sided: Some(stack_parts.iter().any(|p| p.two_sided == Some(true))),
        has_circular_arc: Some(false),
    }
}

pub fn create_concentric_arch_macro(
    parts: &[PartInput],
    placements: &[(usize, f64, f64)],
    config: Option<&ConfigurationInput>,
) -> PartInput {
    let round3 = |v: f64| (v * 1000.0).round() / 1000.0;
    let first = &parts[placements[0].0];

    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    for &(idx, ox, oy) in placements {
        let poly = Polygon::from_raw(&parts[idx].contour);
        let bbox = poly.bounding_box();
        min_x = min_x.min(bbox.min_x + ox);
        max_x = max_x.max(bbox.max_x + ox);
        min_y = min_y.min(bbox.min_y + oy);
        max_y = max_y.max(bbox.max_y + oy);
    }

    let stack_w = max_x - min_x;
    let stack_h = max_y - min_y;

    let mut children = Vec::with_capacity(placements.len());
    let mut shifted_polys = Vec::with_capacity(placements.len());
    for &(idx, ox, oy) in placements {
        let p = &parts[idx];
        let nx = ox - min_x;
        let ny = oy - min_y;
        shifted_polys.push(Polygon::from_raw(&p.contour).translate(nx, ny));
        children.push(ClusterChild {
            entity_id: p.entity_id.clone(),
            id: p.id.clone(),
            name: p.name.clone(),
            offset_x: Some(round3(nx)),
            offset_y: Some(round3(ny)),
            rotation_degrees: Some(0.0),
            width: p.width,
            height: p.height,
            area: p.area,
            contour: Some(p.contour.clone()),
            holes: p.holes.clone(),
            render_contour: p.render_contour.clone(),
            render_holes: p.render_holes.clone(),
            collision_contour: p.collision_contour.clone(),
            collision_holes: p.collision_holes.clone(),
            color: p.color.clone(),
            draw_layers: p.draw_layers.clone(),
            has_grain_label: p.has_grain_label,
            grain_arrow_degrees: p.grain_arrow_degrees,
            base_rotation_degrees: p.base_rotation_degrees,
            two_sided: p.two_sided,
            manual_cluster_child: None,
        });
    }

    let global_grain_locked = config.and_then(|c| c.rotation_divisions).map_or(true, |d| d <= 2);
    let is_grain_locked = global_grain_locked
        || placements.iter().any(|&(idx, _, _)| {
            let p = &parts[idx];
            p.grain_locked == Some(true)
                || p.has_grain_label == Some(true)
                || p.grain_arrow_degrees.is_some()
        });

    let cx0 = shifted_polys[0].points.iter().map(|pt| pt.x).sum::<f64>()
        / (shifted_polys[0].points.len().max(1) as f64);
    let prefer_180 = cx0 < stack_w * 0.5;
    let macro_base_rot = 0.0;
    let pref_rot = if prefer_180 { 180.0 } else { 0.0 };
    let alt_rot = if prefer_180 { 0.0 } else { 180.0 };
    let ref_grain_arrow = first.grain_arrow_degrees;

    let total_area: f64 = placements.iter().map(|&(idx, _, _)| parts[idx].area.unwrap_or(0.0)).sum();
    let first_name = first.name.as_deref().unwrap_or("Part");
    let macro_name = format!("Cụm cong đồng tâm: {} ({} chi tiết)", first_name, placements.len());
    let child_ids: Vec<&str> = placements.iter().map(|&(idx, _, _)| parts[idx].id.as_deref().unwrap_or("")).collect();
    let macro_id = child_ids.join("+");
    let macro_eid = format!("arc-macro:{}", macro_id);

    let macro_w = round3(stack_w);
    let macro_h = round3(stack_h);
    let macro_holes = extract_macro_cavity_holes(&children, macro_w, macro_h);
    let profile_contour = if macro_holes.is_some() {
        vec![[0.0, 0.0], [macro_w, 0.0], [macro_w, macro_h], [0.0, macro_h]]
    } else {
        build_polys_profile_contour(&shifted_polys, macro_w, macro_h)
    };

    PartInput {
        entity_id: Some(macro_eid),
        id: Some(macro_id),
        name: Some(macro_name),
        width: Some(macro_w),
        height: Some(macro_h),
        area: Some(round3(total_area)),
        contour: profile_contour,
        holes: macro_holes,
        render_contour: None,
        render_holes: None,
        collision_contour: None,
        collision_holes: None,
        draw_layers: None,
        rotations: if is_grain_locked {
            Some(vec![pref_rot])
        } else {
            Some(vec![pref_rot, 90.0, alt_rot, 270.0])
        },
        base_rotation_degrees: Some(macro_base_rot),
        rotation_degrees: Some(pref_rot),
        grain_locked: Some(is_grain_locked),
        has_grain_label: Some(is_grain_locked),
        grain_arrow_degrees: if is_grain_locked { ref_grain_arrow } else { None },
        free_rotation: Some(!is_grain_locked),
        rotation_divisions: Some(if is_grain_locked { 2 } else { 4 }),
        color: first.color.clone(),
        logical_part_count: Some(placements.len()),
        manual_cluster_macro: Some(true),
        manual_cluster_children: Some(children),
        two_sided: Some(placements.iter().any(|&(idx, _, _)| parts[idx].two_sided == Some(true))),
        has_circular_arc: Some(false),
    }
}

pub fn compute_shape_signature(part: &PartInput) -> String {
    let poly = Polygon::from_raw(&part.contour);
    let clean = simplify_collinear(&poly.points, 0.5);
    let poly_ccw = ensure_ccw(&Polygon::new(clean));
    let (norm_poly, _, _) = poly_ccw.normalize_to_origin();
    let bbox = norm_poly.bounding_box();
    let area = part.area.unwrap_or_else(|| poly.area());
    let quant_area = ((area / 5.0).round() * 5.0).max(1.0);

    let mut sig = format!(
        "{:.1}_{:.1}_{:.0}_{}_",
        (bbox.width() * 2.0).round() / 2.0,
        (bbox.height() * 2.0).round() / 2.0,
        quant_area,
        norm_poly.points.len()
    );
    let n = norm_poly.points.len();
    if n == 0 {
        return sig;
    }
    let min_idx = (0..n).min_by_key(|&i| {
        let pt = norm_poly.points[i];
        ((pt.x * 10.0).round() as i64, (pt.y * 10.0).round() as i64)
    }).unwrap_or(0);

    for offset in 0..n {
        let pt = norm_poly.points[(min_idx + offset) % n];
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
    let board_w = config.board_width.unwrap_or(2440.0);
    let board_h = config.board_height.unwrap_or(1220.0);
    let margin = config.edge_margin.unwrap_or(10.0);
    let max_bw = (board_w - margin * 2.0).max(100.0);
    let max_bh = (board_h - margin * 2.0).max(100.0);

    let mut raw_sig_cache: std::collections::HashMap<u64, (bool, String)> = std::collections::HashMap::new();
    for (i, p) in parts.iter().enumerate() {
        if p.manual_cluster_macro == Some(true) {
            signatures.push(String::new());
            continue;
        }
        let raw_hash = {
            use std::hash::{Hash, Hasher};
            let mut h = std::collections::hash_map::DefaultHasher::new();
            p.has_circular_arc.hash(&mut h);
            p.contour.len().hash(&mut h);
            for pt in &p.contour {
                ((pt[0] * 100.0).round() as i64).hash(&mut h);
                ((pt[1] * 100.0).round() as i64).hash(&mut h);
            }
            h.finish()
        };
        let (is_cand, base_sig) = if let Some(cached) = raw_sig_cache.get(&raw_hash) {
            cached.clone()
        } else {
            let poly = Polygon::from_raw(&p.contour);
            let res = if is_contour_curved(&poly, p.has_circular_arc) {
                (false, String::new())
            } else {
                let bbox = poly.bounding_box();
                let area = p.area.unwrap_or_else(|| poly.area());
                if is_truly_irregular(&poly, &bbox, area) {
                    (true, compute_shape_signature(p))
                } else {
                    (false, String::new())
                }
            };
            raw_sig_cache.insert(raw_hash, res.clone());
            res
        };
        if is_cand {
            candidate_indices.push(i);
            signatures.push(base_sig);
        } else {
            signatures.push(String::new());
        }
    }

    if candidate_indices.len() < 2 {
        return (parts.to_vec(), 0);
    }

    use std::collections::{BTreeMap, HashMap};
    let mut groups_by_sig: BTreeMap<String, Vec<usize>> = BTreeMap::new();
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
            || parts[idx_b].has_grain_label == Some(true)
            || parts[idx_a].grain_arrow_degrees.is_some()
            || parts[idx_b].grain_arrow_degrees.is_some();
        if !is_grain {
            return true;
        }
        let base_a = parts[idx_a].base_rotation_degrees.or(parts[idx_a].rotation_degrees).unwrap_or(0.0);
        let base_b = parts[idx_b].base_rotation_degrees.or(parts[idx_b].rotation_degrees).unwrap_or(0.0);
        let diff = ((base_a - base_b) - (offsets.rot_a - offsets.rot_b)).round().abs() as i64 % 180;
        diff == 0
    };

    // Phase 0: Concentric multi-size arch stacking across compatible curved-arch families
    let mut arch_cand_indices: Vec<usize> = Vec::new();
    let mut decimated_polys: HashMap<usize, (Polygon, crate::geometry::Rect)> = HashMap::new();
    for &idx in &candidate_indices {
        let poly = Polygon::from_raw(&parts[idx].contour);
        let bbox = poly.bounding_box();
        let area = parts[idx].area.unwrap_or_else(|| poly.area());
        let box_area = bbox.area().max(1.0);
        if bbox.height().max(bbox.width()) >= 900.0
            && bbox.width().min(bbox.height()) >= 280.0
            && area / box_area <= 0.48
        {
            let dec = if poly.points.len() > 64 {
                decimate_curved_polygon(&poly.points, 4.0)
            } else {
                poly
            };
            decimated_polys.insert(idx, (dec, bbox));
            arch_cand_indices.push(idx);
        }
    }

    if arch_cand_indices.len() >= 6 {
        let mut by_chirality: BTreeMap<(bool, i64, bool), Vec<usize>> = BTreeMap::new();
        for &idx in &arch_cand_indices {
            let (ref poly, ref bbox) = decimated_polys[&idx];
            let cx = poly.points.iter().map(|pt| pt.x).sum::<f64>() / (poly.points.len().max(1) as f64);
            let is_left_spine = cx < bbox.width() * 0.5;
            let r = parts[idx].base_rotation_degrees.or(parts[idx].rotation_degrees).unwrap_or(0.0).round() as i64 % 360;
            let ts = parts[idx].two_sided == Some(true);
            by_chirality.entry((is_left_spine, r, ts)).or_default().push(idx);
        }

        for ((is_left_spine, _rot_key, _ts_key), mut grp_indices) in by_chirality {
            if grp_indices.len() < 6 {
                continue;
            }
            let mut distinct_sigs: std::collections::HashSet<&str> = std::collections::HashSet::new();
            let mut max_arch_h = 0.0_f64;
            for &idx in &grp_indices {
                distinct_sigs.insert(&signatures[idx]);
                max_arch_h = max_arch_h.max(decimated_polys[&idx].1.height());
            }
            if distinct_sigs.len() < 2 {
                continue;
            }

            grp_indices.sort_by(|&a, &b| {
                let ba = &decimated_polys[&a].1;
                let bb = &decimated_polys[&b].1;
                bb.height()
                    .partial_cmp(&ba.height())
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| bb.width().partial_cmp(&ba.width()).unwrap_or(std::cmp::Ordering::Equal))
            });

            let mut tall_indices = Vec::new();
            let mut short_indices = Vec::new();
            for &idx in &grp_indices {
                if decimated_polys[&idx].1.height() >= max_arch_h * 0.88 {
                    tall_indices.push(idx);
                } else {
                    short_indices.push(idx);
                }
            }

            let belly_reserve = if !short_indices.is_empty() { 95.0 } else { 0.0 };
            let tall_max_w = (max_bw - belly_reserve).max(max_bw * 0.75);

            let find_arch_pos = |stack: &[(usize, f64, f64)], cand_idx: usize, limit_w: f64, is_tall: bool| -> Option<(f64, f64)> {
                let (ref cand_poly, ref cand_bbox) = decimated_polys[&cand_idx];
                let cw = cand_bbox.width();
                let ch = cand_bbox.height();
                if cw > limit_w || ch > max_bh {
                    return None;
                }
                if stack.is_empty() {
                    return Some(if is_left_spine { (0.0, 0.0) } else { (max_bw - cw, 0.0) });
                }
                let mut cur_min_x = f64::INFINITY;
                let mut cur_max_x = f64::NEG_INFINITY;
                let mut cur_min_y = f64::INFINITY;
                let mut cur_max_y = f64::NEG_INFINITY;
                for &(e_idx, ex, ey) in stack {
                    let eb = &decimated_polys[&e_idx].1;
                    cur_min_x = cur_min_x.min(ex);
                    cur_max_x = cur_max_x.max(ex + eb.width());
                    cur_min_y = cur_min_y.min(ey);
                    cur_max_y = cur_max_y.max(ey + eb.height());
                }

                let y_cands: &[f64] = if is_tall {
                    &[0.0, 5.0, 10.0, 12.0, 15.0, 20.0, 25.0]
                } else {
                    &[150.0, 165.0, 180.0, 195.0, 210.0, 230.0, 250.0, 270.0, 290.0, 310.0, 317.0, 325.0, 335.0, 345.0, 130.0, 100.0, 60.0, 0.0]
                };

                let is_pos_valid = |x: f64, y: f64| -> bool {
                    let n_min_x = cur_min_x.min(x);
                    let n_max_x = cur_max_x.max(x + cw);
                    let n_min_y = cur_min_y.min(y);
                    let n_max_y = cur_max_y.max(y + ch);
                    if n_max_x - n_min_x > limit_w + 1e-3 || n_max_y - n_min_y > max_bh + 1e-3 {
                        return false;
                    }
                    let c_shifted = cand_poly.translate(x, y);
                    for &(e_idx, ex, ey) in stack.iter().rev() {
                        let (ref e_poly, ref eb) = decimated_polys[&e_idx];
                        if x + cw + spacing <= ex || ex + eb.width() + spacing <= x || y + ch + spacing <= ey || ey + eb.height() + spacing <= y {
                            continue;
                        }
                        let e_shifted = e_poly.translate(ex, ey);
                        if polygons_collide_concave_safe(&c_shifted, &e_shifted, spacing) {
                            return false;
                        }
                    }
                    true
                };

                let ideal_y = ((max_arch_h - ch) * 0.5).max(0.0);
                let mut best: Option<(f64, f64, f64)> = None;
                for &y in y_cands {
                    if y + ch > max_bh {
                        continue;
                    }
                    if is_left_spine {
                        let start_x = stack.last().map_or(0.0, |&(last_idx, lx, _)| {
                            if decimated_polys[&last_idx].1.height() <= ch + 10.0 { (lx + 40.0).max(0.0) } else { (lx + 20.0).max(0.0) }
                        });
                        let mut x = start_x;
                        while x <= max_bw - cw {
                            if is_pos_valid(x, y) {
                                while x - 1.0 >= start_x && is_pos_valid(x - 1.0, y) {
                                    x -= 1.0;
                                }
                                let span_w = cur_max_x.max(x + cw) - cur_min_x.min(x);
                                let score = x + span_w * 0.25 + (y - ideal_y).abs() * 0.02;
                                if best.map_or(true, |b| score < b.2 - 0.1) {
                                    best = Some((x, y, score));
                                }
                                break;
                            }
                            x += 4.0;
                        }
                    } else {
                        let start_x = stack.last().map_or(max_bw - cw, |&(last_idx, lx, _)| {
                            let lw = decimated_polys[&last_idx].1.width();
                            if decimated_polys[&last_idx].1.height() <= ch + 10.0 {
                                (lx + lw - cw - 40.0).min(max_bw - cw)
                            } else {
                                (lx + lw - cw - 20.0).min(max_bw - cw)
                            }
                        });
                        let mut x = start_x;
                        while x >= 0.0 {
                            if is_pos_valid(x, y) {
                                while x + 1.0 <= start_x && is_pos_valid(x + 1.0, y) {
                                    x += 1.0;
                                }
                                let span_w = cur_max_x.max(x + cw) - cur_min_x.min(x);
                                let score = (max_bw - (x + cw)) + span_w * 0.25 + (y - ideal_y).abs() * 0.02;
                                if best.map_or(true, |b| score < b.2 - 0.1) {
                                    best = Some((x, y, score));
                                }
                                break;
                            }
                            x -= 4.0;
                        }
                    }
                }
                best.map(|(bx, by, _)| (bx, by))
            };

            let target_stacks = ((tall_indices.len() + 5) / 6).max(2);
            let mut stacks: Vec<Vec<(usize, f64, f64)>> = vec![Vec::new(); target_stacks];
            let mut tall_rr = 0usize;
            for &idx in &tall_indices {
                let mut placed_in_stack = false;
                let n_st = stacks.len();
                for offset in 0..n_st {
                    let s_idx = (tall_rr + offset) % n_st;
                    if stacks[s_idx].len() >= 7 {
                        continue;
                    }
                    if let Some((px, py)) = find_arch_pos(&stacks[s_idx], idx, tall_max_w, true) {
                        stacks[s_idx].push((idx, px, py));
                        tall_rr = (s_idx + 1) % n_st;
                        placed_in_stack = true;
                        break;
                    }
                }
                if !placed_in_stack {
                    for s_idx in 0..stacks.len() {
                        if let Some((px, py)) = find_arch_pos(&stacks[s_idx], idx, max_bw, true) {
                            stacks[s_idx].push((idx, px, py));
                            placed_in_stack = true;
                            break;
                        }
                    }
                }
                if !placed_in_stack {
                    let mut st = Vec::new();
                    if let Some((px, py)) = find_arch_pos(&st, idx, max_bw, true) {
                        st.push((idx, px, py));
                        stacks.push(st);
                    }
                }
            }
            stacks.retain(|st| !st.is_empty());

            let short_tight_w = (max_bw - 73.0).max(tall_max_w);
            let mut leftover_short = Vec::new();
            let mut rr_cursor = 0usize;
            for &idx in &short_indices {
                let mut placed_short = false;
                if !stacks.is_empty() {
                    let n_st = stacks.len();
                    for &lim_w in &[short_tight_w, max_bw] {
                        for offset in 0..n_st {
                            let s_idx = (rr_cursor + offset) % n_st;
                            if let Some((px, py)) = find_arch_pos(&stacks[s_idx], idx, lim_w, false) {
                                stacks[s_idx].push((idx, px, py));
                                rr_cursor = (s_idx + 1) % n_st;
                                placed_short = true;
                                break;
                            }
                        }
                        if placed_short {
                            break;
                        }
                    }
                }
                if !placed_short {
                    leftover_short.push(idx);
                }
            }

            for idx in leftover_short {
                let mut placed_lo = false;
                for st in stacks.iter_mut() {
                    if let Some((px, py)) = find_arch_pos(st, idx, max_bw, false) {
                        st.push((idx, px, py));
                        placed_lo = true;
                        break;
                    }
                }
                if !placed_lo {
                    let mut st = Vec::new();
                    if let Some((px, py)) = find_arch_pos(&st, idx, max_bw, false) {
                        st.push((idx, px, py));
                        stacks.push(st);
                    }
                }
            }

            for st in stacks {
                if st.len() >= 2 {
                    let macro_p = create_concentric_arch_macro(parts, &st, Some(config));
                    macro_parts.push(macro_p);
                    for &(idx, _, _) in &st {
                        used[idx] = true;
                    }
                    comb_count += 1;
                }
            }
        }
    }

    // Phase 1: Self-mating within identical shape families O(N)
    for (_sig, indices) in groups_by_sig.iter_mut() {
        indices.retain(|&idx| !used[idx]);
        if indices.len() >= 2 {
            // Group two-sided parts and same-room parts together first so they pair cleanly with each other
            indices.sort_by(|&a, &b| {
                (parts[a].two_sided != Some(true))
                    .cmp(&(parts[b].two_sided != Some(true)))
                    .then_with(|| crate::types::get_part_room_key(&parts[a]).cmp(&crate::types::get_part_room_key(&parts[b])))
            });

            let first_idx = indices[0];
            let poly_first = Polygon::from_raw(&parts[first_idx].contour);
            let parallel_nest = find_parallel_nesting_vector(&poly_first, spacing, max_bw, max_bh);

            if let Some((dx, dy, _ratio)) = parallel_nest {
                let mut grain_groups: BTreeMap<(i64, bool, String), Vec<usize>> = BTreeMap::new();
                for &idx in indices.iter() {
                    let r = parts[idx].base_rotation_degrees.or(parts[idx].rotation_degrees).unwrap_or(0.0).round() as i64 % 360;
                    let ts = parts[idx].two_sided == Some(true);
                    let rm = crate::types::get_part_room_key(&parts[idx]);
                    grain_groups.entry((r, ts, rm)).or_default().push(idx);
                }

                for (_key, g_indices) in grain_groups.iter() {
                    let mut i = 0;
                    let deep_cardinal_batch = g_indices.len() >= 12
                        && (dx.abs() < 0.1 || dy.abs() < 0.1)
                        && dx.hypot(dy) <= 100.0;
                    let half_long_board = (max_bw.max(max_bh) - spacing) * 0.5;
                    while i < g_indices.len() {
                        let remaining = g_indices.len() - i;
                        if remaining < 2 {
                            break;
                        }
                        let mut chunk_size = if deep_cardinal_batch {
                            let max_c = 12usize;
                            let target_chunks = (remaining + max_c - 1) / max_c;
                            ((remaining + target_chunks - 1) / target_chunks).min(max_c)
                        } else if remaining == 7 {
                            4
                        } else {
                            remaining.min(8)
                        };
                        while chunk_size > 2 {
                            let (sw, sh) = compute_stack_dimensions(&poly_first, chunk_size, dx, dy);
                            let fits_normal = sw <= max_bw && sh <= max_bh;
                            let fits_rotated = sw <= max_bh && sh <= max_bw;
                            let stack_span = if dx.abs() >= dy.abs() { sw } else { sh };
                            let fits_half_long = !deep_cardinal_batch || chunk_size <= 6 || stack_span <= half_long_board;
                            if fits_half_long && (fits_normal || (!global_grain_locked && fits_rotated)) {
                                break;
                            }
                            chunk_size -= 1;
                        }

                        let (sw, sh) = compute_stack_dimensions(&poly_first, chunk_size, dx, dy);
                        let fits_normal = sw <= max_bw && sh <= max_bh;
                        let fits_rotated = sw <= max_bh && sh <= max_bw;
                        if !fits_normal && (global_grain_locked || !fits_rotated) {
                            break;
                        }

                        let stack_indices = &g_indices[i..(i + chunk_size)];
                        let stack_parts: Vec<PartInput> = stack_indices.iter().map(|&idx| parts[idx].clone()).collect();
                        let macro_p = create_parallel_nested_stack_macro(&stack_parts, dx, dy, Some(config));
                        macro_parts.push(macro_p);
                        for &idx in stack_indices {
                            used[idx] = true;
                        }
                        comb_count += 1;
                        i += chunk_size;
                    }
                }
                continue;
            }

            if let Some(offsets) = get_mating(indices[0], indices[1], &mut mating_cache) {
                let p_a = Polygon::from_raw(&parts[indices[0]].contour);
                let p_b = Polygon::from_raw(&parts[indices[1]].contour);
                let orig_boxes_area = p_a.bounding_box().area() + p_b.bounding_box().area();
                let macro_box = offsets.macro_w * offsets.macro_h;
                let total_parts_area = parts[indices[0]].area.unwrap_or_else(|| p_a.area())
                    + parts[indices[1]].area.unwrap_or_else(|| p_b.area());
                let is_arc_or_hollow = parts[indices[0]].has_circular_arc == Some(true)
                    || parts[indices[1]].has_circular_arc == Some(true)
                    || p_a.points.len() >= 8
                    || (orig_boxes_area > 1.0 && total_parts_area / orig_boxes_area < 0.65);
                let saving_thresh = if is_arc_or_hollow { 0.94 } else { 0.90 };
                let is_compact_saving = orig_boxes_area > 1.0 && macro_box <= orig_boxes_area * saving_thresh;
                let is_partial_batch_saving = !is_compact_saving
                    && is_arc_or_hollow
                    && indices.len() >= 16
                    && orig_boxes_area > 1.0
                    && macro_box <= orig_boxes_area * 0.98
                    && offsets.fill_rate >= 0.65;
                let min_threshold = if is_arc_or_hollow { 0.15 } else { 0.25 };
                if (is_compact_saving || is_partial_batch_saving) && offsets.fill_rate >= min_threshold {
                    let max_pair_idx = if is_partial_batch_saving {
                        ((indices.len() * 7) / 10) & !1
                    } else {
                        indices.len()
                    };
                    let mut i = 0;
                    while i + 1 < max_pair_idx {
                        let idx_a = indices[i];
                        let idx_b = indices[i + 1];
                        let rm_a = crate::types::get_part_room_key(&parts[idx_a]);
                        let rm_b = crate::types::get_part_room_key(&parts[idx_b]);
                        if (!rm_a.is_empty() && !rm_b.is_empty() && rm_a != rm_b)
                            || !is_pair_grain_compatible(idx_a, idx_b, &offsets)
                        {
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

    // Phase 2: Bounded complementary mating for remaining unpaired irregular parts (e.g. 180-deg rotated siblings or complementary wedges)
    let mut unpaired_cands: Vec<usize> = candidate_indices.iter().copied().filter(|&idx| !used[idx]).collect();
    if unpaired_cands.len() >= 2 {
        unpaired_cands.sort_by(|&a, &b| {
            (parts[a].two_sided != Some(true))
                .cmp(&(parts[b].two_sided != Some(true)))
                .then_with(|| crate::types::get_part_room_key(&parts[a]).cmp(&crate::types::get_part_room_key(&parts[b])))
                .then_with(|| {
                    let aa = parts[a].area.unwrap_or(0.0);
                    let ab = parts[b].area.unwrap_or(0.0);
                    ab.partial_cmp(&aa).unwrap_or(std::cmp::Ordering::Equal)
                })
        });
        unpaired_cands.truncate(20);
        for ia in 0..unpaired_cands.len() {
            let idx_a = unpaired_cands[ia];
            if used[idx_a] {
                continue;
            }
            let p_a = Polygon::from_raw(&parts[idx_a].contour);
            let box_a = p_a.bounding_box().area();
            for ib in (ia + 1)..unpaired_cands.len() {
                let idx_b = unpaired_cands[ib];
                let rm_a = crate::types::get_part_room_key(&parts[idx_a]);
                let rm_b = crate::types::get_part_room_key(&parts[idx_b]);
                if used[idx_b]
                    || parts[idx_a].two_sided != parts[idx_b].two_sided
                    || (!rm_a.is_empty() && !rm_b.is_empty() && rm_a != rm_b)
                {
                    continue;
                }
                if let Some(offsets) = get_mating(idx_a, idx_b, &mut mating_cache) {
                    if !is_pair_grain_compatible(idx_a, idx_b, &offsets) {
                        continue;
                    }
                    let p_b = Polygon::from_raw(&parts[idx_b].contour);
                    let orig_boxes_area = box_a + p_b.bounding_box().area();
                    let macro_box = offsets.macro_w * offsets.macro_h;
                    if orig_boxes_area > 1.0 && macro_box <= orig_boxes_area * 0.82 && offsets.fill_rate >= 0.70 {
                        let macro_p = create_macro_part(&parts[idx_a], &parts[idx_b], &offsets, Some(config));
                        macro_parts.push(macro_p);
                        used[idx_a] = true;
                        used[idx_b] = true;
                        comb_count += 1;
                        break;
                    }
                }
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
            two_sided: None,
            has_circular_arc: None,
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
            two_sided: None,
            has_circular_arc: None,
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
            two_sided: None,
            has_circular_arc: None,
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
            two_sided: None,
            has_circular_arc: None,
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
            two_sided: None,
            has_circular_arc: None,
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
                two_sided: None,
            has_circular_arc: None,
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
                two_sided: None,
            has_circular_arc: None,
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
            two_sided: None,
            has_circular_arc: None,
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
            two_sided: None,
            has_circular_arc: None,
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
        non_grain_part.grain_arrow_degrees = None;
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
            two_sided: None,
            has_circular_arc: None,
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
            two_sided: None,
            has_circular_arc: None,
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

    #[test]
    fn test_triangle_comb_pairing_with_circular_arc_hole_and_manual_clusters() {
        let tri_contour = vec![
            [0.0, 0.0],
            [600.0, 0.0],
            [0.0, 300.0],
        ];
        let poly = Polygon::from_raw(&tri_contour);
        let area = poly.area();

        let tri_a = PartInput {
            id: Some("tri_1".to_string()),
            entity_id: Some("e_tri1".to_string()),
            name: Some("Triangle 1".to_string()),
            width: Some(600.0),
            height: Some(300.0),
            area: Some(area),
            contour: tri_contour.clone(),
            holes: Some(vec![vec![[100.0, 50.0], [105.0, 50.0], [105.0, 55.0], [100.0, 55.0]]]),
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
            two_sided: None,
            has_circular_arc: Some(true),
        };

        let tri_b = PartInput {
            id: Some("tri_2".to_string()),
            entity_id: Some("e_tri2".to_string()),
            name: Some("Triangle 2".to_string()),
            ..tri_a.clone()
        };

        let elbow_macro = PartInput {
            id: Some("cluster_1".to_string()),
            entity_id: Some("e_clust1".to_string()),
            name: Some("Elbow Cluster".to_string()),
            width: Some(500.0),
            height: Some(400.0),
            area: Some(150000.0),
            contour: vec![[0.0, 0.0], [500.0, 0.0], [500.0, 400.0], [0.0, 400.0]],
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
            logical_part_count: Some(2),
            manual_cluster_macro: Some(true),
            manual_cluster_children: Some(vec![]),
            two_sided: None,
            has_circular_arc: Some(false),
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

        let all_parts = vec![elbow_macro, tri_a, tri_b];
        let (paired, comb_count) = pair_comb_parts(&all_parts, &config, 6.0);

        assert_eq!(comb_count, 1, "Expected 1 comb pair formed between the two triangles");
        assert_eq!(paired.len(), 2, "Expected 2 parts total: 1 elbow cluster + 1 triangle macro");
        let macro_found = paired.iter().find(|p| p.logical_part_count == Some(2) && p.name.as_deref().unwrap_or("").contains("Triangle"));
        assert!(macro_found.is_some(), "Triangle macro should be found in paired result");
    }

    #[test]
    fn test_elbow_mating_1_4_cung() {
        let elbow_contour = vec![
            [0.0, 37.563], [33.0, 0.0], [223.993, 167.792], [237.464, 181.286],
            [249.059, 196.424], [258.578, 212.945], [265.861, 230.568], [270.779, 248.99],
            [273.252, 267.897], [273.235, 286.965], [270.729, 305.867], [265.779, 324.281],
            [258.466, 341.891], [248.918, 358.396], [237.297, 373.512], [69.505, 564.506],
            [31.942, 531.506], [199.734, 340.512], [207.369, 330.581], [213.642, 319.737],
            [218.446, 308.168], [221.699, 296.07], [223.345, 283.651], [223.356, 271.123],
            [221.732, 258.702], [218.5, 246.599], [213.716, 235.021], [207.462, 224.166],
            [199.844, 214.221], [190.993, 205.355]
        ];
        let poly = Polygon::from_raw(&elbow_contour);
        let area = poly.area();

        let part_a = PartInput {
            id: Some("elbow_1".to_string()),
            entity_id: Some("e_el1".to_string()),
            name: Some("Elbow 1".to_string()),
            width: Some(273.252),
            height: Some(564.506),
            area: Some(area),
            contour: elbow_contour.clone(),
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
            grain_arrow_degrees: Some(48.75),
            free_rotation: Some(false),
            rotation_divisions: Some(2),
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
            two_sided: None,
            has_circular_arc: Some(true),
        };

        let part_b = PartInput {
            id: Some("elbow_2".to_string()),
            entity_id: Some("e_el2".to_string()),
            name: Some("Elbow 2".to_string()),
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

        let clean_a = simplify_collinear(&poly.points, 0.5);
        let poly_clean = ensure_ccw(&Polygon::new(clean_a));
        let edges = extract_edges(&poly_clean.points);
        println!("Elbow total edges: {}, candidates >= 15: {}", edges.len(), edges.iter().filter(|e| e.length >= 15.0).count());
        for (idx, e) in edges.iter().enumerate() {
            if e.length >= 15.0 {
                println!("  edge {}: len={:.1}, angle={:.1} deg", idx, e.length, e.angle * 180.0 / PI);
            }
        }

        let res = try_build_comb_pair(&part_a, &part_b, 6.0, Some(&config));
        println!("Elbow try_build_comb_pair res is_some: {}", res.is_some());
        assert!(res.is_some(), "Elbow mating pair should succeed as a macro");
        let (m, score) = res.unwrap();
        println!("Mated macro: w={:?}, h={:?}, score={:.1}, children={}", m.width, m.height, score, m.manual_cluster_children.as_ref().map(|c| c.len()).unwrap_or(0));
        assert_eq!(m.manual_cluster_children.as_ref().map(|c| c.len()), Some(2));
    }

    #[test]
    fn test_pair_comb_parts_4_elbows_with_two_sided() {
        let contour = vec![
            [0.0, 37.563], [33.0, 0.0], [223.993, 167.792], [237.464, 181.286], [249.059, 196.424],
            [258.578, 212.945], [265.861, 230.568], [270.779, 248.99], [273.252, 267.897], [273.235, 286.965],
            [270.729, 305.867], [265.779, 324.281], [258.466, 341.891], [248.918, 358.396], [237.297, 373.512],
            [69.505, 564.506], [31.942, 531.506], [199.734, 340.512], [207.369, 330.581], [213.642, 319.737],
            [218.446, 308.168], [221.699, 296.07], [223.345, 283.651], [223.356, 271.123], [221.732, 258.702],
            [218.5, 246.599], [213.716, 235.021], [207.462, 224.166], [199.844, 214.221], [190.993, 205.355],
        ];

        let make_elbow = |id: &str, two_sided: bool| PartInput {
            id: Some(id.to_string()),
            entity_id: Some(format!("e_{}", id)),
            name: Some(format!("Elbow {}", id)),
            width: Some(273.252),
            height: Some(564.506),
            area: Some(34881.174),
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
            grain_arrow_degrees: Some(48.75),
            free_rotation: Some(false),
            rotation_divisions: Some(2),
            color: None,
            logical_part_count: Some(1),
            manual_cluster_macro: None,
            manual_cluster_children: None,
            two_sided: Some(two_sided),
            has_circular_arc: Some(true),
        };

        let parts = vec![
            make_elbow("p1_purple", true),
            make_elbow("p2_brown", false),
            make_elbow("p3_purple", true),
            make_elbow("p4_brown", false),
        ];

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

        let (result_parts, comb_count) = pair_comb_parts(&parts, &config, 6.0);
        println!("Result parts count: {}, comb_count: {}", result_parts.len(), comb_count);
        assert_eq!(comb_count, 2, "Both pairs of elbows must be formed into macros");
        assert_eq!(result_parts.len(), 2, "4 elbow parts should become 2 macros");

        // Verify one macro is two_sided (purple) and one is single-sided (brown)
        let two_sided_macros = result_parts.iter().filter(|p| p.two_sided == Some(true)).count();
        let single_sided_macros = result_parts.iter().filter(|p| p.two_sided != Some(true)).count();
        assert_eq!(two_sided_macros, 1, "There should be exactly 1 two-sided macro (purple)");
        assert_eq!(single_sided_macros, 1, "There should be exactly 1 single-sided macro (brown)");
    }

    #[test]
    fn test_curved_elbows_parallel_nesting_stack_6_parts() {
        let contour = vec![
            [0.0, 37.563], [33.0, 0.0], [223.993, 167.792], [237.464, 181.286], [249.059, 196.424],
            [258.578, 212.945], [265.861, 230.568], [270.779, 248.99], [273.252, 267.897], [273.235, 286.965],
            [270.729, 305.867], [265.779, 324.281], [258.466, 341.891], [248.918, 358.396], [237.297, 373.512],
            [69.505, 564.506], [31.942, 531.506], [199.734, 340.512], [207.369, 330.581], [213.642, 319.737],
            [218.446, 308.168], [221.699, 296.07], [223.345, 283.651], [223.356, 271.123], [221.732, 258.702],
            [218.5, 246.599], [213.716, 235.021], [207.462, 224.166], [199.844, 214.221], [190.993, 205.355],
        ];

        let make_elbow = |id: &str| PartInput {
            id: Some(id.to_string()),
            entity_id: Some(format!("e_{}", id)),
            name: Some(format!("Elbow {}", id)),
            width: Some(273.252),
            height: Some(564.506),
            area: Some(34881.174),
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
            two_sided: Some(false),
            has_circular_arc: Some(true),
        };

        // 6 identical elbows matching Image 2 ("GROUP 616" with 6 parts)
        let parts: Vec<PartInput> = (0..6).map(|i| make_elbow(&format!("p_{}", i))).collect();

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

        let (result_parts, comb_count) = pair_comb_parts(&parts, &config, 6.0);
        println!("6 elbows result parts count: {}, comb_count: {}", result_parts.len(), comb_count);
        assert_eq!(comb_count, 1, "6 elbows must bundle into 1 single nested macro");
        assert_eq!(result_parts.len(), 1, "Result should be 1 macro containing all 6 elbows");

        let macro_p = &result_parts[0];
        assert_eq!(macro_p.logical_part_count, Some(6));
        assert_eq!(macro_p.manual_cluster_macro, Some(true));
        let children = macro_p.manual_cluster_children.as_ref().expect("Macro must have children");
        assert_eq!(children.len(), 6);

        // Verify all children have rotation 0.0 (parallel shingled, cùng chiều vân)
        for child in children {
            assert_eq!(child.rotation_degrees, Some(0.0), "All children in curved stack must have relative rotation 0");
            assert_eq!(child.grain_arrow_degrees, Some(90.0), "Grain arrow must be strictly preserved");
        }

        // Verify macro dimensions are compact (not elongated like S-shape)
        let mw = macro_p.width.unwrap();
        let mh = macro_p.height.unwrap();
        println!("6-elbow stack dimensions: w={:.1}, h={:.1}", mw, mh);
        assert!(mw < 1000.0, "Width should be compact (< 1000mm)");
        assert!(mh < 800.0, "Height should be compact (< 800mm)");
    }

    #[test]
    fn test_curved_elbows_11_parts_chunking() {
        let contour = vec![
            [0.0, 37.563], [33.0, 0.0], [223.993, 167.792], [237.464, 181.286], [249.059, 196.424],
            [258.578, 212.945], [265.861, 230.568], [270.779, 248.99], [273.252, 267.897], [273.235, 286.965],
            [270.729, 305.867], [265.779, 324.281], [258.466, 341.891], [248.918, 358.396], [237.297, 373.512],
            [69.505, 564.506], [31.942, 531.506], [199.734, 340.512], [207.369, 330.581], [213.642, 319.737],
            [218.446, 308.168], [221.699, 296.07], [223.345, 283.651], [223.356, 271.123], [221.732, 258.702],
            [218.5, 246.599], [213.716, 235.021], [207.462, 224.166], [199.844, 214.221], [190.993, 205.355],
        ];

        let make_elbow = |id: &str| PartInput {
            id: Some(id.to_string()),
            entity_id: Some(format!("e_{}", id)),
            name: Some(format!("Elbow {}", id)),
            width: Some(273.252),
            height: Some(564.506),
            area: Some(34881.174),
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
            two_sided: Some(false),
            has_circular_arc: Some(true),
        };

        // 11 identical elbows matching Sheet 26 from user
        let parts: Vec<PartInput> = (0..11).map(|i| make_elbow(&format!("p_{}", i))).collect();

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

        let (result_parts, comb_count) = pair_comb_parts(&parts, &config, 6.0);
        println!("11 elbows result parts count: {}, comb_count: {}", result_parts.len(), comb_count);
        assert_eq!(comb_count, 2, "11 elbows should be grouped into 2 macros (6 + 5)");
        assert_eq!(result_parts.len(), 2, "Result should be exactly 2 macros");
        assert_eq!(result_parts[0].logical_part_count, Some(6));
        assert_eq!(result_parts[1].logical_part_count, Some(5));
    }
}


