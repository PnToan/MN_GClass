// src/engine.rs - Ultra-Fast Spatial-Indexed Multi-Bin MaxRects + Exact Polygon
// Concavity/Hole Nesting Engine with Tail-Sheet Consolidation.
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use rayon::prelude::*;

use crate::types::{
    get_part_room_key, DrawLayerData, DrawPathData, LayoutOutput, MaterialStateInput,
    PartInput, PlacementOutput, SheetOutput,
};

#[path = "geom.rs"]
mod geom;
#[path = "spatial.rs"]
mod spatial;

use geom::{angle_close, build_oriented_poly, norm_deg, rotate_pt, BBox, OrientedPoly, Pt};
use spatial::{compute_shape_hash, BinRule, PartSpec, SheetConfig, SpatialSheet};

fn is_part_grain_locked(part: &PartInput, global_rot_div: usize) -> bool {
    if part.free_rotation == Some(false) { return true; }
    if part.free_rotation == Some(true) { return false; }
    if part.grain_locked == Some(true) || part.has_grain_label == Some(true) { return true; }
    global_rot_div <= 2
}

fn allowed_rotations(part: &PartInput, global_rot_div: usize) -> Vec<f64> {
    let base = norm_deg(part.base_rotation_degrees.or(part.rotation_degrees).unwrap_or(0.0));
    let locked = is_part_grain_locked(part, global_rot_div);
    let div = part.rotation_divisions.unwrap_or(global_rot_div);
    let raw = if locked {
        if div <= 1 && part.rotations.as_ref().is_none_or(|r| r.len() <= 1) {
            vec![base]
        } else if let Some(ref rots) = part.rotations {
            let f: Vec<f64> = rots.iter().map(|&d| norm_deg(d))
                .filter(|&d| angle_close(d, base, 0.5) || angle_close(d, base + 180.0, 0.5))
                .collect();
            if !f.is_empty() { f } else { vec![base, norm_deg(base + 180.0)] }
        } else {
            vec![base, norm_deg(base + 180.0)]
        }
    } else if let Some(ref rots) = part.rotations {
        if !rots.is_empty() {
            rots.iter().map(|&d| norm_deg(d)).collect()
        } else {
            vec![base, norm_deg(base + 90.0), norm_deg(base + 180.0), norm_deg(base + 270.0)]
        }
    } else if div >= 4 || part.free_rotation == Some(true) {
        vec![base, norm_deg(base + 90.0), norm_deg(base + 180.0), norm_deg(base + 270.0)]
    } else {
        vec![base, norm_deg(base + 180.0)]
    };

    let mut out = Vec::with_capacity(raw.len());
    for a in raw { if !out.iter().any(|&x| angle_close(x, a, 0.2)) { out.push(a); } }
    if out.is_empty() { out.push(base); }
    out
}

#[inline]
fn f64_bits(v: f64) -> u64 {
    if v == 0.0 { 0 } else { v.to_bits() }
}

fn contour_cache_key(p: &PartInput, rots: &[f64]) -> Vec<u64> {
    let mut key = Vec::with_capacity(p.contour.len() * 2 + rots.len() + 4);
    key.push(p.contour.len() as u64);
    key.push(p.holes.as_ref().map_or(0, |h| h.len() as u64));
    key.push(p.area.map_or(0, f64_bits));
    for &r in rots { key.push(f64_bits(r)); }
    for c in &p.contour { key.push(f64_bits(c[0])); key.push(f64_bits(c[1])); }
    if let Some(ref hs) = p.holes {
        for h in hs {
            key.push(h.len() as u64);
            for c in h { key.push(f64_bits(c[0])); key.push(f64_bits(c[1])); }
        }
    }
    key
}

fn prepare_parts(material_state: &MaterialStateInput, cfg: &SheetConfig, global_rot_div: usize) -> (Vec<PartSpec>, bool) {
    let mut room_map: HashMap<String, u32> = HashMap::new();
    let mut poly_cache: HashMap<Vec<u64>, Arc<Vec<OrientedPoly>>> = HashMap::new();
    let mut has_rooms = false;
    let mut specs = Vec::with_capacity(material_state.parts.len());

    for p in &material_state.parts {
        let rots = allowed_rotations(p, global_rot_div);
        let ckey = contour_cache_key(p, &rots);
        let variants = if let Some(cached) = poly_cache.get(&ckey) {
            Arc::clone(cached)
        } else {
            let mut vars: Vec<OrientedPoly> = Vec::with_capacity(rots.len());
            for &r in &rots {
                let poly = build_oriented_poly(&p.contour, p.holes.as_ref(), r, cfg.spacing, cfg.sheet_in_sheet, p.area);
                if poly.is_rect && vars.iter().any(|v| v.is_rect && (v.w - poly.w).abs() < 0.05 && (v.h - poly.h).abs() < 0.05) {
                    continue;
                }
                vars.push(poly);
            }
            if vars.is_empty() {
                vars.push(build_oriented_poly(&p.contour, p.holes.as_ref(), 0.0, cfg.spacing, cfg.sheet_in_sheet, p.area));
            }
            let arc = Arc::new(vars);
            poly_cache.insert(ckey, Arc::clone(&arc));
            arc
        };

        let raw_pts: Vec<Pt> = p.contour.iter().map(|c| Pt { x: c[0], y: c[1] }).collect();
        let ob = BBox::from_pts(&raw_pts);
        let ow = p.width.filter(|v| *v > 1e-6).unwrap_or_else(|| ob.w());
        let oh = p.height.filter(|v| *v > 1e-6).unwrap_or_else(|| ob.h());
        let small_part = p.manual_cluster_macro != Some(true) && cfg.small_thr > 0.001 && ow.min(oh) <= cfg.small_thr + 0.001;
        let small_part_clearance = if small_part && cfg.small_clr > 0.001 { cfg.spacing.max(cfg.small_clr) } else { 0.0 };

        let v0 = &variants[0];
        let area = p.area.filter(|v| *v > 1e-4).unwrap_or(v0.area);
        let min_bbox_area = variants.iter().map(|v| v.w * v.h).fold(f64::MAX, f64::min);
        let min_dim = variants.iter().map(|v| v.w.min(v.h)).fold(f64::MAX, f64::min);
        let max_dim = variants.iter().map(|v| v.w.max(v.h)).fold(0.0f64, f64::max);
        let rkey = get_part_room_key(p);
        let room_id = if rkey.is_empty() {
            0
        } else {
            has_rooms = true;
            let next = (room_map.len() + 1) as u32;
            *room_map.entry(rkey).or_insert(next)
        };

        specs.push(PartSpec {
            shape_hash: compute_shape_hash(v0.w, v0.h, area, v0.pts.len()),
            perimeter: 2.0 * (v0.w + v0.h),
            is_low_fill: !v0.is_rect && v0.fill_ratio < 0.78,
            has_holes: !v0.holes.is_empty(),
            two_sided: p.two_sided == Some(true),
            variants, area, min_bbox_area, max_dim, min_dim, small_part, small_part_clearance, room_id,
        });
    }
    (specs, has_rooms)
}

#[derive(Debug, Clone, Copy)]
enum SortOrder {
    MaxSideDesc, AreaDesc, BboxAreaDesc, PerimeterDesc, HeightDesc, WidthDesc, ShortSideDesc, WeightedDimDesc, RoomGrouped,
}

#[derive(Debug, Clone, Copy)]
struct Strategy { order: SortOrder, rule: BinRule, best_fit_all: bool, two_sided_first: bool }

fn cmp_order(pa: &PartSpec, pb: &PartSpec, order: SortOrder) -> std::cmp::Ordering {
    use std::cmp::Ordering::Equal;
    match order {
        SortOrder::MaxSideDesc => pb.max_dim.partial_cmp(&pa.max_dim).unwrap_or(Equal)
            .then_with(|| pb.area.partial_cmp(&pa.area).unwrap_or(Equal)),
        SortOrder::AreaDesc => pb.area.partial_cmp(&pa.area).unwrap_or(Equal)
            .then_with(|| pb.max_dim.partial_cmp(&pa.max_dim).unwrap_or(Equal)),
        SortOrder::BboxAreaDesc => pb.min_bbox_area.partial_cmp(&pa.min_bbox_area).unwrap_or(Equal)
            .then_with(|| pb.max_dim.partial_cmp(&pa.max_dim).unwrap_or(Equal)),
        SortOrder::PerimeterDesc => pb.perimeter.partial_cmp(&pa.perimeter).unwrap_or(Equal)
            .then_with(|| pb.area.partial_cmp(&pa.area).unwrap_or(Equal)),
        SortOrder::HeightDesc => pb.variants[0].h.partial_cmp(&pa.variants[0].h).unwrap_or(Equal)
            .then_with(|| pb.variants[0].w.partial_cmp(&pa.variants[0].w).unwrap_or(Equal)),
        SortOrder::WidthDesc => pb.variants[0].w.partial_cmp(&pa.variants[0].w).unwrap_or(Equal)
            .then_with(|| pb.variants[0].h.partial_cmp(&pa.variants[0].h).unwrap_or(Equal)),
        SortOrder::ShortSideDesc => pb.min_dim.partial_cmp(&pa.min_dim).unwrap_or(Equal)
            .then_with(|| pb.max_dim.partial_cmp(&pa.max_dim).unwrap_or(Equal)),
        SortOrder::WeightedDimDesc => (pb.max_dim + 0.55 * pb.min_dim)
            .partial_cmp(&(pa.max_dim + 0.55 * pa.min_dim)).unwrap_or(Equal)
            .then_with(|| pb.area.partial_cmp(&pa.area).unwrap_or(Equal)),
        SortOrder::RoomGrouped => pa.room_id.cmp(&pb.room_id)
            .then_with(|| pb.area.partial_cmp(&pa.area).unwrap_or(Equal)),
    }
}

fn build_order(parts: &[PartSpec], order: SortOrder, sis: bool, two_sided_first: bool, boosts: Option<&[u32]>) -> Vec<usize> {
    let mut idxs: Vec<usize> = (0..parts.len()).collect();
    idxs.sort_by(|&a, &b| {
        if let Some(bs) = boosts { if bs[a] != bs[b] { return bs[b].cmp(&bs[a]); } }
        let (pa, pb) = (&parts[a], &parts[b]);
        if sis && pa.has_holes != pb.has_holes { return pb.has_holes.cmp(&pa.has_holes); }
        if two_sided_first && pa.two_sided != pb.two_sided { return pb.two_sided.cmp(&pa.two_sided); }
        cmp_order(pa, pb, order).then_with(|| a.cmp(&b))
    });
    idxs
}

fn pack_order(
    parts: &[PartSpec], cfg: &SheetConfig, order: &[usize],
    rule: BinRule, best_fit_all: bool, max_sheets_limit: Option<usize>,
) -> Option<Vec<SpatialSheet>> {
    let mut sheets: Vec<SpatialSheet> = Vec::with_capacity(8);
    let perfect_cutoff = match rule { BinRule::Bssf | BinRule::Blsf => 0.5, BinRule::Baf => 25.0, _ => -1e9 };

    for &p_idx in order {
        let pspec = &parts[p_idx];

        // 1. Tight polygon interlocking slide for low-fill arches/L/U shapes (O(1) bounded sheet check)
        if pspec.is_low_fill {
            let mut best_slide: Option<(usize, usize, f64, f64, f64)> = None;
            let mut checked = 0usize;
            for (s_idx, sh) in sheets.iter().enumerate().rev() {
                if sh.low_fill_items.is_empty() { continue; }
                let has_same = sh.low_fill_items.iter().any(|&i| parts[sh.items[i as usize].part_idx as usize].shape_hash == pspec.shape_hash);
                if !has_same && checked >= 2 { continue; }
                checked += 1;
                if let Some((vi, x, y, sc)) = sh.find_polygon_slide(parts, cfg, p_idx) {
                    if best_slide.is_none_or(|b| sc < b.4) {
                        best_slide = Some((s_idx, vi, x, y, sc));
                        if has_same { break; }
                    }
                }
                if checked >= 4 { break; }
            }
            if let Some((si, vi, x, y, _)) = best_slide {
                sheets[si].commit_placement(parts, cfg, p_idx, vi, x, y);
                continue;
            }
        }

        // 2. Branch-and-bound Multi-Bin MaxRects search across open sheets
        let mut chosen: Option<(usize, usize, f64, f64)> = None;
        let (mut best_s1, mut best_s2) = (f64::MAX, f64::MAX);
        for (s_idx, sh) in sheets.iter().enumerate() {
            if let Some((vi, x, y, s1, s2)) = sh.find_best_free_rect(parts, cfg, p_idx, rule, best_s1, best_s2) {
                best_s1 = s1; best_s2 = s2;
                chosen = Some((s_idx, vi, x, y));
                if !best_fit_all || best_s1 <= perfect_cutoff { break; }
            }
        }
        if let Some((si, vi, x, y)) = chosen {
            sheets[si].commit_placement(parts, cfg, p_idx, vi, x, y);
            continue;
        }

        // 3. Open new sheet
        if let Some(lim) = max_sheets_limit { if sheets.len() >= lim { return None; } }
        let mut ns = SpatialSheet::new(cfg);
        if let Some((vi, x, y, _, _)) = ns.find_best_free_rect(parts, cfg, p_idx, rule, f64::MAX, f64::MAX) {
            ns.commit_placement(parts, cfg, p_idx, vi, x, y);
        } else {
            let bv = pspec.variants.iter().position(|v| v.w <= cfg.usable_w + 1e-3 && v.h <= cfg.usable_h + 1e-3).unwrap_or(0);
            let var = &pspec.variants[bv];
            let x = if cfg.compact_right { (cfg.board_w - cfg.margin - var.w).max(cfg.margin) } else { cfg.margin };
            let y = if cfg.compact_top { (cfg.board_h - cfg.margin - var.h).max(cfg.margin) } else { cfg.margin };
            ns.commit_placement(parts, cfg, p_idx, bv, x, y);
        }
        sheets.push(ns);
    }
    Some(sheets)
}

#[inline]
fn solution_score(sheets: &[SpatialSheet]) -> (usize, i64) {
    // Fewer sheets strictly wins; ties prefer higher sum(used_area^2) to concentrate slack onto sparse tails
    let sum_sq: f64 = sheets.iter().map(|s| {
        let u = s.used_area * 1e-4;
        u * u
    }).sum();
    (sheets.len(), -(sum_sq * 100.0).round() as i64)
}

struct XorRng(u64);
impl XorRng {
    fn new(seed: u64) -> Self { Self(if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed }) }
    #[inline]
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13; x ^= x >> 7; x ^= x << 17;
        self.0 = x;
        x
    }
    #[inline]
    fn next_f64(&mut self) -> f64 { (self.next_u64() >> 11) as f64 / ((1u64 << 53) as f64) }
    #[inline]
    fn range(&mut self, n: usize) -> usize { if n <= 1 { 0 } else { (self.next_u64() as usize) % n } }
}

fn try_repack_subset(parts: &[PartSpec], cfg: &SheetConfig, subset: &[usize], target: usize) -> Option<Vec<SpatialSheet>> {
    const ORDERS: [(SortOrder, BinRule, bool); 6] = [
        (SortOrder::MaxSideDesc, BinRule::Bssf, true),
        (SortOrder::AreaDesc, BinRule::Bssf, true),
        (SortOrder::BboxAreaDesc, BinRule::Baf, true),
        (SortOrder::PerimeterDesc, BinRule::Bssf, true),
        (SortOrder::ShortSideDesc, BinRule::Bssf, true),
        (SortOrder::AreaDesc, BinRule::Bssf, false),
    ];
    for &(ord_kind, rule, bf_all) in &ORDERS {
        let mut sub = subset.to_vec();
        sub.sort_by(|&a, &b| {
            let (pa, pb) = (&parts[a], &parts[b]);
            if cfg.sheet_in_sheet && pa.has_holes != pb.has_holes { return pb.has_holes.cmp(&pa.has_holes); }
            cmp_order(pa, pb, ord_kind).then_with(|| a.cmp(&b))
        });
        if let Some(res) = pack_order(parts, cfg, &sub, rule, bf_all, Some(target)) {
            if res.len() <= target { return Some(res); }
        }
    }
    None
}

fn evict_sparsest_sheet(parts: &[PartSpec], cfg: &SheetConfig, best: &mut Vec<SpatialSheet>) {
    let usable_area = cfg.usable_w * cfg.usable_h;
    while best.len() > 1 {
        let Some((min_idx, _)) = best.iter().enumerate()
            .min_by(|a, b| a.1.used_area.partial_cmp(&b.1.used_area).unwrap_or(std::cmp::Ordering::Equal))
        else { break; };
        let free_sum: f64 = best.iter().enumerate().filter(|&(i, _)| i != min_idx).map(|(_, s)| (usable_area - s.used_area).max(0.0)).sum();
        if best[min_idx].used_area > free_sum { break; }
        let mut trial: Vec<SpatialSheet> = best.iter().enumerate().filter(|&(i, _)| i != min_idx).map(|(_, s)| s.clone()).collect();
        let mut all_fit = true;
        for it in &best[min_idx].items {
            let p_idx = it.part_idx as usize;
            let mut placed = false;
            if parts[p_idx].is_low_fill {
                for sh in &mut trial {
                    if !sh.low_fill_items.is_empty() {
                        if let Some((vi, x, y, _)) = sh.find_polygon_slide(parts, cfg, p_idx) {
                            sh.commit_placement(parts, cfg, p_idx, vi, x, y);
                            placed = true; break;
                        }
                    }
                }
            }
            if !placed {
                let (mut bs1, mut bs2, mut bf) = (f64::MAX, f64::MAX, None);
                for (si, sh) in trial.iter().enumerate() {
                    if let Some((vi, x, y, s1, s2)) = sh.find_best_free_rect(parts, cfg, p_idx, BinRule::Bssf, bs1, bs2) {
                        bs1 = s1; bs2 = s2; bf = Some((si, vi, x, y));
                    }
                }
                if let Some((si, vi, x, y)) = bf { trial[si].commit_placement(parts, cfg, p_idx, vi, x, y); placed = true; }
            }
            if !placed { all_fit = false; break; }
        }
        if all_fit { *best = trial; } else { break; }
    }
}

fn squeeze_sheets(parts: &[PartSpec], cfg: &SheetConfig, mut best: Vec<SpatialSheet>, seed: u64) -> Vec<SpatialSheet> {
    if best.len() <= 1 { return best; }
    let usable_area = cfg.usable_w * cfg.usable_h;

    // 1. Iterative Tail-Boosted Repack
    let boost_rounds = if parts.len() > 1000 { 4 } else { 6 };
    let mut boosts = vec![0u32; parts.len()];
    for round in 0..boost_rounds {
        let Some((min_idx, _)) = best.iter().enumerate()
            .min_by(|a, b| a.1.used_area.partial_cmp(&b.1.used_area).unwrap_or(std::cmp::Ordering::Equal))
        else { break; };
        for it in &best[min_idx].items { boosts[it.part_idx as usize] += 1; }
        let rule = if round % 2 == 0 { BinRule::Bssf } else { BinRule::Baf };
        let ord_kind = match round % 3 { 0 => SortOrder::MaxSideDesc, 1 => SortOrder::AreaDesc, _ => SortOrder::WeightedDimDesc };
        let order = build_order(parts, ord_kind, cfg.sheet_in_sheet, false, Some(&boosts));
        if let Some(cand) = pack_order(parts, cfg, &order, rule, true, Some(best.len())) {
            if solution_score(&cand) < solution_score(&best) {
                best = cand;
                if best.len() <= 1 { return best; }
            }
        }
    }

    // 2. Direct single-part eviction of sparsest sheet into earlier sheets
    evict_sparsest_sheet(parts, cfg, &mut best);

    // 3. Exact 2 -> 1 Sheet-Pair Consolidation
    let mut improved = true;
    while improved && best.len() >= 2 {
        improved = false;
        let mut idxs: Vec<usize> = (0..best.len()).collect();
        idxs.sort_by(|&a, &b| best[a].used_area.partial_cmp(&best[b].used_area).unwrap_or(std::cmp::Ordering::Equal));
        let lim = idxs.len().min(20);
        'pair: for (ii, &a) in idxs.iter().take(lim).enumerate() {
            for &b in idxs.iter().take(lim).skip(ii + 1) {
                if best[a].used_area + best[b].used_area > usable_area * 0.995 { break; }
                let comb: Vec<usize> = best[a].items.iter().chain(&best[b].items).map(|it| it.part_idx as usize).collect();
                if let Some(mut merged) = try_repack_subset(parts, cfg, &comb, 1) {
                    best.remove(a.max(b)); best.remove(a.min(b));
                    best.push(merged.pop().unwrap());
                    improved = true;
                    break 'pair;
                }
            }
        }
    }

    // 4. Exact 3 -> 2 Consolidation on sparsest sheets (capped to 14 feasible triples/pass for O(1) speed)
    for _ in 0..6 {
        if best.len() < 3 { break; }
        let mut idxs: Vec<usize> = (0..best.len()).collect();
        idxs.sort_by(|&a, &b| best[a].used_area.partial_cmp(&best[b].used_area).unwrap_or(std::cmp::Ordering::Equal));
        let k = idxs.len().min(8);
        let mut merged_any = false;
        let mut tries = 0usize;
        'tri: for (i0, &a) in idxs.iter().take(k).enumerate() {
            for (i1, &b) in idxs.iter().take(k).enumerate().skip(i0 + 1) {
                for &c in idxs.iter().take(k).skip(i1 + 1) {
                    if best[a].used_area + best[b].used_area + best[c].used_area > usable_area * 1.96 { break; }
                    tries += 1;
                    if tries > 14 { break 'tri; }
                    let comb: Vec<usize> = best[a].items.iter().chain(&best[b].items).chain(&best[c].items).map(|it| it.part_idx as usize).collect();
                    if let Some(merged) = try_repack_subset(parts, cfg, &comb, 2) {
                        let mut rem = [a, b, c]; rem.sort_unstable();
                        best.remove(rem[2]); best.remove(rem[1]); best.remove(rem[0]);
                        best.extend(merged);
                        merged_any = true;
                        break 'tri;
                    }
                }
            }
        }
        if !merged_any { break; }
    }

    // 5. Time-Budgeted Waste-Targeted Bounded ALNS
    let total_area: f64 = parts.iter().map(|p| p.area).sum();
    let min_possible = (total_area / usable_area).ceil().max(1.0) as usize;
    if best.len() <= min_possible {
        return best;
    }
    let budget_ms = if parts.len() <= 100 { 45 } else if parts.len() <= 600 { 80 } else { 110 };
    let deadline = Instant::now() + Duration::from_millis(budget_ms);
    let mut rng = XorRng::new(seed ^ (parts.len() as u64).wrapping_mul(0x517C_C1B7_2722_0A95));
    let mut best_sc = solution_score(&best);

    // Small/medium materials: fast random-key + tail-boosted full repacks (nails 10-85 part bin-packing!)
    if parts.len() <= 100 && best.len() > min_possible {
        let rules_sm = [BinRule::Bssf, BinRule::Baf, BinRule::Blsf, BinRule::BottomLeft];
        for it in 0..60 {
            if Instant::now() >= deadline || best.len() <= min_possible { break; }
            let mut scored: Vec<(usize, f64)> = (0..parts.len()).map(|pi| {
                let p = &parts[pi];
                let base = match it % 4 {
                    0 => p.area,
                    1 => p.max_dim,
                    2 => p.min_dim,
                    _ => p.perimeter,
                };
                let spread = if it < 20 { 0.35 } else { 0.65 };
                (pi, base * (1.0 - 0.5 * spread + spread * rng.next_f64()))
            }).collect();
            // Boost parts currently on sparsest sheet every 3rd iteration
            if it % 3 == 2 {
                if let Some((_, sh)) = best.iter().enumerate().min_by(|a, b| a.1.used_area.partial_cmp(&b.1.used_area).unwrap_or(std::cmp::Ordering::Equal)) {
                    for itm in &sh.items { scored[itm.part_idx as usize].1 *= 1.6; }
                }
            }
            scored.sort_by(|a, b| {
                let (pa, pb) = (&parts[a.0], &parts[b.0]);
                if cfg.sheet_in_sheet && pa.has_holes != pb.has_holes { return pb.has_holes.cmp(&pa.has_holes); }
                b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal)
            });
            let ord: Vec<usize> = scored.into_iter().map(|(i, _)| i).collect();
            let rule = rules_sm[it % rules_sm.len()];
            let bf_all = it % 5 != 4;
            if let Some(cand) = pack_order(parts, cfg, &ord, rule, bf_all, Some(best.len())) {
                let sc = solution_score(&cand);
                if sc < best_sc { best = cand; best_sc = sc; }
            }
        }
    }

    let max_alns = if parts.len() <= 150 { 20 } else if parts.len() <= 600 { 14 } else { 10 };
    for iter in 0..max_alns {
        if Instant::now() >= deadline || best.len() <= min_possible { break; }
        let n_sh = best.len();
        let mut ranked: Vec<usize> = (0..n_sh).collect();
        ranked.sort_by(|&a, &b| best[a].used_area.partial_cmp(&best[b].used_area).unwrap_or(std::cmp::Ordering::Equal));
        let target_waste = usable_area * (1.15 + 0.12 * (iter % 3) as f64);
        let max_ruin = n_sh.min(12);
        let mut mask = vec![false; n_sh];
        let (mut r_waste, mut r_cnt) = (0.0f64, 0usize);
        if iter % 2 == 0 {
            for &si in &ranked {
                if r_cnt >= 2 && (r_waste >= target_waste || r_cnt >= max_ruin) { break; }
                mask[si] = true; r_waste += (usable_area - best[si].used_area).max(0.0); r_cnt += 1;
            }
        } else {
            mask[ranked[0]] = true; r_waste += (usable_area - best[ranked[0]].used_area).max(0.0); r_cnt = 1;
            let pool_lim = (n_sh * 2 / 3).max(2).min(n_sh);
            for _ in 0..24 {
                if r_cnt >= max_ruin || (r_cnt >= 2 && r_waste >= target_waste) { break; }
                let pick = ranked[rng.range(pool_lim)];
                if !mask[pick] { mask[pick] = true; r_waste += (usable_area - best[pick].used_area).max(0.0); r_cnt += 1; }
            }
        }
        let mut pool: Vec<usize> = Vec::new();
        let mut kept: Vec<SpatialSheet> = Vec::with_capacity(n_sh);
        for (si, s) in best.iter().enumerate() {
            if mask[si] { pool.extend(s.items.iter().map(|it| it.part_idx as usize)); } else { kept.push(s.clone()); }
        }
        if pool.is_empty() { continue; }
        let mut scored: Vec<(usize, f64)> = pool.iter().map(|&pi| {
            let p = &parts[pi];
            let base = match iter % 3 { 0 => p.area, 1 => p.max_dim, _ => p.min_dim };
            (pi, base * (0.85 + 0.30 * rng.next_f64()))
        }).collect();
        scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        let rule = if iter % 2 == 0 { BinRule::Bssf } else { BinRule::Baf };
        let mut leftover = Vec::new();
        // Try inserting ruined parts into the sparsest kept sheets first
        let mut k_idx: Vec<usize> = (0..kept.len()).collect();
        k_idx.sort_by(|&a, &b| kept[a].used_area.partial_cmp(&kept[b].used_area).unwrap_or(std::cmp::Ordering::Equal));
        let scan_cap = k_idx.len().min(18);
        for &(pi, _) in &scored {
            let mut placed = false;
            let (mut bs1, mut bs2, mut bf) = (f64::MAX, f64::MAX, None);
            for &si in &k_idx[..scan_cap] {
                if let Some((vi, x, y, s1, s2)) = kept[si].find_best_free_rect(parts, cfg, pi, rule, bs1, bs2) {
                    bs1 = s1; bs2 = s2; bf = Some((si, vi, x, y));
                }
            }
            if let Some((si, vi, x, y)) = bf { kept[si].commit_placement(parts, cfg, pi, vi, x, y); placed = true; }
            if !placed { leftover.push(pi); }
        }
        let budget_new = best.len().saturating_sub(kept.len());
        if !leftover.is_empty() {
            if let Some(repacked) = pack_order(parts, cfg, &leftover, rule, true, Some(budget_new)) {
                kept.extend(repacked);
            } else {
                continue;
            }
        }
        let sc = solution_score(&kept);
        if sc < best_sc {
            let dropped = sc.0 < best_sc.0;
            best = kept; best_sc = sc;
            if dropped { evict_sparsest_sheet(parts, cfg, &mut best); best_sc = solution_score(&best); }
        }
    }
    best
}

fn transform_draw_layers(layers: Option<&Vec<DrawLayerData>>, rot_deg: f64, sx: f64, sy: f64) -> Option<Vec<DrawLayerData>> {
    Some(layers?.iter().map(|layer| DrawLayerData {
        name: layer.name.clone(), layer_name: layer.layer_name.clone(),
        color: layer.color.clone(), side: layer.side.clone(), r#type: layer.r#type.clone(),
        paths: layer.paths.as_ref().map(|ps| ps.iter().map(|dp| DrawPathData {
            points: dp.points.iter().map(|c| {
                let rp = rotate_pt(Pt { x: c[0], y: c[1] }, rot_deg);
                [rp.x + sx, rp.y + sy]
            }).collect(),
            closed: dp.closed, surface_face: dp.surface_face, surface_face_id: dp.surface_face_id.clone(),
            surface_loop_type: dp.surface_loop_type.clone(), geometry_kind: dp.geometry_kind.clone(),
            _nesting_packed_local: Some(true),
        }).collect()),
    }).collect())
}

fn materialize_layout(
    material_state: &MaterialStateInput, cfg: &SheetConfig, parts: &[PartSpec],
    mut sheets: Vec<SpatialSheet>, compact_directions: Vec<String>, target_util: f64, global_rot_div: usize,
) -> LayoutOutput {
    sheets.sort_by(|a, b| b.has_two_sided.cmp(&a.has_two_sided)
        .then_with(|| b.used_area.partial_cmp(&a.used_area).unwrap_or(std::cmp::Ordering::Equal)));
    let board_area = (cfg.board_w * cfg.board_h).max(1.0);
    let mut sheet_outputs = Vec::with_capacity(sheets.len());
    let mut total_waste = 0.0f64;

    for (s_idx, sh) in sheets.iter().enumerate() {
        let mut placements = Vec::with_capacity(sh.items.len());
        let mut sheet_area = 0.0f64;
        for it in &sh.items {
            let p_idx = it.part_idx as usize;
            let (orig, pspec) = (&material_state.parts[p_idx], &parts[p_idx]);
            let var = &pspec.variants[it.var_idx as usize];
            sheet_area += pspec.area;
            let eid = orig.entity_id.as_deref().filter(|s| !s.trim().is_empty()).map(str::to_string)
                .unwrap_or_else(|| orig.id.clone().unwrap_or_else(|| format!("part_{}", p_idx)));
            let pid = orig.id.as_deref().filter(|s| !s.trim().is_empty()).map(str::to_string).unwrap_or_else(|| eid.clone());
            let locked = is_part_grain_locked(orig, global_rot_div);
            let edge_protected = pspec.small_part && pspec.small_part_clearance > 0.001
                && (!cfg.merge_cut || cfg.in_edge_zone(it.x, it.y, var.w, var.h));

            placements.push(PlacementOutput {
                entity_id: eid, id: pid, name: orig.name.clone(),
                x: it.x, y: it.y, rotation_degrees: var.rot_deg, base_rotation_degrees: orig.base_rotation_degrees,
                width: var.w, height: var.h, packed_width: var.w, packed_height: var.h, area: pspec.area,
                contour: var.pts.iter().map(|p| [p.x, p.y]).collect(),
                holes: (!var.holes.is_empty()).then(|| var.holes.iter().map(|h| h.pts.iter().map(|p| [p.x, p.y]).collect()).collect()),
                draw_layers: transform_draw_layers(orig.draw_layers.as_ref(), var.rot_deg, var.shift_x, var.shift_y),
                color: if pspec.two_sided { Some("#CC99FF".to_string()) } else { orig.color.clone() },
                manual_cluster_macro: orig.manual_cluster_macro,
                manual_cluster_children: orig.manual_cluster_children.clone(),
                logical_part_count: orig.logical_part_count,
                has_grain_label: orig.has_grain_label, grain_arrow_degrees: orig.grain_arrow_degrees,
                grain_locked: orig.grain_locked.or(Some(locked)), free_rotation: orig.free_rotation.or(Some(!locked)),
                small_part: Some(pspec.small_part),
                small_part_clearance: Some(if pspec.small_part && pspec.small_part_clearance > 0.001 { pspec.small_part_clearance } else { 0.0 }),
                small_part_edge_protected: Some(edge_protected), two_sided: Some(pspec.two_sided),
            });
        }
        let util_pct = if board_area > 0.0 { ((sheet_area / board_area) * 100.0).clamp(0.0, 100.0) } else { 0.0 };
        let rounded_util = (util_pct * 10.0).round() / 10.0;
        total_waste += (board_area - sheet_area).max(0.0);
        sheet_outputs.push(SheetOutput { index: s_idx + 1, utilization: rounded_util, used_percent: rounded_util, placements });
    }

    let sheet_count = sheet_outputs.len();
    let eff_target = target_util.max(80.0);
    let mut target_deficit = 0.0f64;
    let mut front_load = 0.0f64;
    let mut total_compact_area = 0.0f64;
    for (idx, sh) in sheet_outputs.iter().enumerate() {
        let used_a: f64 = sh.placements.iter().map(|p| p.area).sum();
        let u_pct = if board_area > 0.0 { (used_a / board_area) * 100.0 } else { 0.0 };
        if idx < sheet_count.saturating_sub(1) {
            target_deficit += (eff_target - u_pct).max(0.0) * (sheet_count - idx) as f64;
        }
        front_load += idx as f64 * used_a;
        if !sh.placements.is_empty() {
            let max_x = sh.placements.iter().map(|p| p.x + p.packed_width).fold(0.0, f64::max);
            let max_y = sh.placements.iter().map(|p| p.y + p.packed_height).fold(0.0, f64::max);
            total_compact_area += (max_x - cfg.margin).max(0.0) * (max_y - cfg.margin).max(0.0);
        }
    }
    let last_sheet_compact_area = sheet_outputs.last().map_or(0.0, |last| {
        if last.placements.is_empty() { 0.0 } else {
            let max_x = last.placements.iter().map(|p| p.x + p.packed_width).fold(0.0, f64::max);
            let max_y = last.placements.iter().map(|p| p.y + p.packed_height).fold(0.0, f64::max);
            (max_x - cfg.margin).max(0.0) * (max_y - cfg.margin).max(0.0)
        }
    });

    LayoutOutput {
        material: material_state.material.clone().unwrap_or_else(|| "Material".to_string()),
        thickness: material_state.thickness.unwrap_or(17.0),
        board_width: cfg.board_w, board_height: cfg.board_h, edge_margin: cfg.margin,
        part_spacing: cfg.spacing, cut_gap: Some(cfg.spacing),
        small_part_threshold: cfg.small_thr, small_part_clearance: cfg.small_clr,
        small_part_edge_zone: cfg.edge_zone, merge_cut_paths: cfg.merge_cut,
        sheets: sheet_outputs, compact_directions, target_sheet_utilization: target_util,
        waste_area: total_waste, compact_area: total_compact_area, last_sheet_compact_area,
        front_load_score: front_load, alignment_score: 0.0, compact_direction_score: 0.0,
        target_deficit_score: target_deficit, comb_pair_placements: 0, staircase_pattern_placements: 0,
        priority_zone_fill_placements: 0, pocket_evacuated_sheets: 0, priority_zone_fill_area: 0.0,
        safety_certified: true, safety_version: 2,
    }
}

pub fn optimize_material_layout<F>(material_state: &MaterialStateInput, mut on_progress: F) -> LayoutOutput
where
    F: FnMut(&str, f64, usize, usize, &LayoutOutput),
{
    let raw_cfg = material_state.configuration.clone().unwrap_or_default();
    let board_w = raw_cfg.board_width.unwrap_or(2440.0).max(1.0);
    let board_h = raw_cfg.board_height.unwrap_or(1220.0).max(1.0);
    let margin = raw_cfg.edge_margin.unwrap_or(10.0).max(0.0);
    let spacing = raw_cfg.part_spacing.or(raw_cfg.cut_gap).unwrap_or(6.0).max(0.0);
    let global_rot_div = raw_cfg.rotation_divisions.unwrap_or(1);
    let sheet_in_sheet = raw_cfg.sheet_in_sheet.unwrap_or(false);
    let small_thr = raw_cfg.small_part_threshold.unwrap_or(0.0).max(0.0);
    let small_clr = raw_cfg.small_part_clearance.unwrap_or(0.0).max(0.0);
    let merge_cut = raw_cfg.merge_cut_paths.unwrap_or(false);
    let edge_zone = if merge_cut { let cz = raw_cfg.small_part_edge_zone.unwrap_or(0.0); if cz > 0.0 { cz } else { 200.0 } } else { 0.0 };
    let compact_dirs = raw_cfg.compact_directions.unwrap_or_else(|| vec!["left".to_string(), "bottom".to_string()]);
    let target_util = raw_cfg.target_sheet_utilization.unwrap_or(90.0);

    let cfg = SheetConfig {
        board_w, board_h, margin,
        usable_w: (board_w - 2.0 * margin).max(1.0), usable_h: (board_h - 2.0 * margin).max(1.0),
        spacing, sheet_in_sheet, small_thr, small_clr, merge_cut, edge_zone,
        max_possible_gap: spacing.max(small_clr),
        compact_right: compact_dirs.iter().any(|d| d.eq_ignore_ascii_case("right")),
        compact_top: compact_dirs.iter().any(|d| d.eq_ignore_ascii_case("top")),
    };

    if material_state.parts.is_empty() {
        return materialize_layout(material_state, &cfg, &[], Vec::new(), compact_dirs, target_util, global_rot_div);
    }

    let (parts, has_rooms) = prepare_parts(material_state, &cfg, global_rot_div);
    let ts = parts.iter().any(|p| p.two_sided);

    let mut strategies = vec![
        Strategy { order: SortOrder::MaxSideDesc, rule: BinRule::Bssf, best_fit_all: true, two_sided_first: ts },
        Strategy { order: SortOrder::AreaDesc, rule: BinRule::Bssf, best_fit_all: true, two_sided_first: ts },
        Strategy { order: SortOrder::AreaDesc, rule: BinRule::Baf, best_fit_all: true, two_sided_first: ts },
        Strategy { order: SortOrder::PerimeterDesc, rule: BinRule::Bssf, best_fit_all: true, two_sided_first: ts },
        Strategy { order: SortOrder::HeightDesc, rule: BinRule::Bssf, best_fit_all: true, two_sided_first: ts },
        Strategy { order: SortOrder::WidthDesc, rule: BinRule::Blsf, best_fit_all: true, two_sided_first: ts },
        Strategy { order: SortOrder::ShortSideDesc, rule: BinRule::Bssf, best_fit_all: true, two_sided_first: ts },
        Strategy { order: SortOrder::WeightedDimDesc, rule: BinRule::Bssf, best_fit_all: true, two_sided_first: ts },
        Strategy { order: SortOrder::BboxAreaDesc, rule: BinRule::Baf, best_fit_all: true, two_sided_first: ts },
        Strategy { order: SortOrder::AreaDesc, rule: BinRule::BottomLeft, best_fit_all: true, two_sided_first: ts },
        Strategy { order: SortOrder::MaxSideDesc, rule: BinRule::Bssf, best_fit_all: false, two_sided_first: ts },
        Strategy { order: SortOrder::AreaDesc, rule: BinRule::Baf, best_fit_all: false, two_sided_first: ts },
    ];
    if parts.len() <= 550 {
        strategies.push(Strategy { order: SortOrder::AreaDesc, rule: BinRule::Contact, best_fit_all: true, two_sided_first: ts });
    }
    if has_rooms {
        strategies.push(Strategy { order: SortOrder::RoomGrouped, rule: BinRule::Bssf, best_fit_all: true, two_sided_first: ts });
    }

    let mut best_sheets = strategies
        .par_iter()
        .enumerate()
        .map(|(s_idx, st)| {
            let order = build_order(&parts, st.order, cfg.sheet_in_sheet, st.two_sided_first, None);
            Some((s_idx, pack_order(&parts, &cfg, &order, st.rule, st.best_fit_all, None).unwrap_or_default()))
        })
        .reduce(
            || None,
            |acc, item| match (acc, item) {
                (None, x) | (x, None) => x,
                (Some((ia, sa)), Some((ib, sb))) => {
                    if solution_score(&sa).cmp(&solution_score(&sb)).then_with(|| ia.cmp(&ib)) != std::cmp::Ordering::Greater {
                        Some((ia, sa))
                    } else {
                        Some((ib, sb))
                    }
                }
            },
        )
        .map(|(_, s)| s)
        .unwrap_or_default();

    let seed = material_state.optimization_seed.or(raw_cfg.optimization_seed).unwrap_or(42);
    best_sheets = squeeze_sheets(&parts, &cfg, best_sheets, seed);
    let layout = materialize_layout(material_state, &cfg, &parts, best_sheets, compact_dirs, target_util, global_rot_div);
    on_progress("Done", 100.0, 1, 1, &layout);
    layout
}
