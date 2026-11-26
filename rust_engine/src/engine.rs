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

use geom::{angle_close, build_oriented_poly, norm_deg, poly_contained_in_hole_exact, polys_conflict_exact, rotate_pt, BBox, OrientedPoly, Pt};
use spatial::{compute_shape_hash, BinRule, PartSpec, PlacedItem, SheetConfig, SpatialSheet};

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

fn contour_cache_key(contour: &[[f64; 2]], p: &PartInput, rots: &[f64]) -> Vec<u64> {
    let mut key = Vec::with_capacity(contour.len() * 2 + rots.len() + 4);
    key.push(contour.len() as u64);
    key.push(p.holes.as_ref().map_or(0, |h| h.len() as u64));
    key.push(p.area.map_or(0, f64_bits));
    for &r in rots { key.push(f64_bits(r)); }
    for c in contour { key.push(f64_bits(c[0])); key.push(f64_bits(c[1])); }
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
        let finite_cnt = p.contour.iter().filter(|c| c[0].is_finite() && c[1].is_finite()).count();
        let fallback_contour: Option<Vec<[f64; 2]>> = if finite_cnt < 3 {
            match (p.width, p.height) {
                (Some(w), Some(h)) if w.is_finite() && h.is_finite() && w > 0.0 && h > 0.0 => {
                    Some(vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]])
                }
                _ => None,
            }
        } else {
            None
        };
        let eff_contour: &[[f64; 2]] = fallback_contour.as_deref().unwrap_or(&p.contour);

        let rots = allowed_rotations(p, global_rot_div);
        let ckey = contour_cache_key(eff_contour, p, &rots);
        let variants = if let Some(cached) = poly_cache.get(&ckey) {
            Arc::clone(cached)
        } else {
            let mut vars: Vec<OrientedPoly> = Vec::with_capacity(rots.len());
            for &r in &rots {
                let poly = build_oriented_poly(eff_contour, p.holes.as_ref(), r, cfg.spacing, cfg.sheet_in_sheet, p.area);
                if poly.is_rect && vars.iter().any(|v| v.is_rect && (v.w - poly.w).abs() < 0.05 && (v.h - poly.h).abs() < 0.05) {
                    continue;
                }
                vars.push(poly);
            }
            if vars.is_empty() {
                vars.push(build_oriented_poly(eff_contour, p.holes.as_ref(), 0.0, cfg.spacing, cfg.sheet_in_sheet, p.area));
            }
            let arc = Arc::new(vars);
            poly_cache.insert(ckey, Arc::clone(&arc));
            arc
        };

        let raw_pts: Vec<Pt> = eff_contour
            .iter()
            .filter(|c| c[0].is_finite() && c[1].is_finite())
            .map(|c| Pt { x: c[0], y: c[1] })
            .collect();
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
            two_sided: p.two_sided == Some(true)
                || p.manual_cluster_children.as_ref().is_some_and(|cs| cs.iter().any(|c| c.two_sided == Some(true))),
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

fn build_order_subset(
    parts: &[PartSpec], subset: &[usize], order: SortOrder, sis: bool, two_sided_first: bool, boosts: Option<&[u32]>,
) -> Vec<usize> {
    let mut idxs: Vec<usize> = subset.to_vec();
    idxs.sort_by(|&a, &b| {
        let (pa, pb) = (&parts[a], &parts[b]);
        if two_sided_first && pa.two_sided != pb.two_sided { return pb.two_sided.cmp(&pa.two_sided); }
        if let Some(bs) = boosts { if bs[a] != bs[b] { return bs[b].cmp(&bs[a]); } }
        if sis && pa.has_holes != pb.has_holes { return pb.has_holes.cmp(&pa.has_holes); }
        cmp_order(pa, pb, order).then_with(|| a.cmp(&b))
    });
    idxs
}

fn build_order(
    parts: &[PartSpec], order: SortOrder, sis: bool, two_sided_first: bool, boosts: Option<&[u32]>,
) -> Vec<usize> {
    let all: Vec<usize> = (0..parts.len()).collect();
    build_order_subset(parts, &all, order, sis, two_sided_first, boosts)
}

fn pack_order_onto(
    parts: &[PartSpec], cfg: &SheetConfig, order: &[usize],
    rule: BinRule, best_fit_all: bool, max_sheets_limit: Option<usize>,
    initial_sheets: Vec<SpatialSheet>,
) -> Option<Vec<SpatialSheet>> {
    let mut sheets = initial_sheets;
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

fn pack_order(
    parts: &[PartSpec], cfg: &SheetConfig, order: &[usize],
    rule: BinRule, best_fit_all: bool, max_sheets_limit: Option<usize>,
) -> Option<Vec<SpatialSheet>> {
    pack_order_onto(parts, cfg, order, rule, best_fit_all, max_sheets_limit, Vec::with_capacity(8))
}

#[inline]
fn count_two_sided_sheets(sheets: &[SpatialSheet]) -> usize {
    sheets.iter().filter(|s| s.has_two_sided).count()
}

#[inline]
fn solution_score(sheets: &[SpatialSheet], strict_two_sided: bool) -> (usize, usize, i64) {
    // Fewer sheets strictly wins (or fewer 2-sided sheets first when strict_two_sided);
    // ties prefer higher sum(used_area^2) to concentrate slack onto sparse tails
    let sum_sq: f64 = sheets.iter().map(|s| {
        let u = s.used_area * 1e-4;
        u * u
    }).sum();
    let neg_sq = -(sum_sq * 100.0).round() as i64;
    let ts_sheets = count_two_sided_sheets(sheets);
    if strict_two_sided {
        (ts_sheets, sheets.len(), neg_sq)
    } else {
        (sheets.len(), ts_sheets, neg_sq)
    }
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

fn try_repack_subset(
    parts: &[PartSpec], cfg: &SheetConfig, subset: &[usize], target: usize,
    two_sided_first: bool, max_ts_sheets: Option<usize>,
) -> Option<Vec<SpatialSheet>> {
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
            if two_sided_first && pa.two_sided != pb.two_sided { return pb.two_sided.cmp(&pa.two_sided); }
            if cfg.sheet_in_sheet && pa.has_holes != pb.has_holes { return pb.has_holes.cmp(&pa.has_holes); }
            cmp_order(pa, pb, ord_kind).then_with(|| a.cmp(&b))
        });
        if let Some(res) = pack_order(parts, cfg, &sub, rule, bf_all, Some(target)) {
            if res.len() <= target && max_ts_sheets.is_none_or(|lim| count_two_sided_sheets(&res) <= lim) {
                return Some(res);
            }
        }
    }
    None
}

fn try_evict_sheet(
    parts: &[PartSpec], cfg: &SheetConfig, best: &mut Vec<SpatialSheet>,
    min_idx: usize, protect_two_sided: bool,
) -> bool {
    let usable_area = cfg.usable_w * cfg.usable_h;
    let free_sum: f64 = best.iter().enumerate()
        .filter(|&(i, _)| i != min_idx)
        .map(|(_, s)| (usable_area - s.used_area).max(0.0))
        .sum();
    if best[min_idx].used_area > free_sum { return false; }
    let mut trial: Vec<SpatialSheet> = best.iter().enumerate()
        .filter(|&(i, _)| i != min_idx)
        .map(|(_, s)| s.clone())
        .collect();
    for it in &best[min_idx].items {
        let p_idx = it.part_idx as usize;
        let mut placed = false;
        if parts[p_idx].is_low_fill {
            for sh in &mut trial {
                if protect_two_sided && parts[p_idx].two_sided && !sh.has_two_sided { continue; }
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
                if protect_two_sided && parts[p_idx].two_sided && !sh.has_two_sided { continue; }
                if let Some((vi, x, y, s1, s2)) = sh.find_best_free_rect(parts, cfg, p_idx, BinRule::Bssf, bs1, bs2) {
                    bs1 = s1; bs2 = s2; bf = Some((si, vi, x, y));
                }
            }
            if let Some((si, vi, x, y)) = bf { trial[si].commit_placement(parts, cfg, p_idx, vi, x, y); placed = true; }
        }
        if !placed { return false; }
    }
    *best = trial;
    true
}

fn evict_sparsest_sheet(parts: &[PartSpec], cfg: &SheetConfig, best: &mut Vec<SpatialSheet>, protect_two_sided: bool) {
    while best.len() > 1 {
        let Some((min_idx, _)) = best.iter().enumerate()
            .min_by(|a, b| a.1.used_area.partial_cmp(&b.1.used_area).unwrap_or(std::cmp::Ordering::Equal))
        else { break; };
        if try_evict_sheet(parts, cfg, best, min_idx, protect_two_sided) { continue; }
        if protect_two_sided && best[min_idx].has_two_sided {
            if let Some((min_1s, _)) = best.iter().enumerate()
                .filter(|(_, s)| !s.has_two_sided)
                .min_by(|a, b| a.1.used_area.partial_cmp(&b.1.used_area).unwrap_or(std::cmp::Ordering::Equal))
            {
                if try_evict_sheet(parts, cfg, best, min_1s, true) { continue; }
            }
        }
        break;
    }
}

fn squeeze_sheets_opts(
    parts: &[PartSpec], cfg: &SheetConfig, mut best: Vec<SpatialSheet>, seed: u64,
    active_subset: Option<&[usize]>, seed_sheets: Option<&[SpatialSheet]>,
    two_sided_first: bool, strict_two_sided: bool,
) -> Vec<SpatialSheet> {
    let all_idxs_storage: Option<Vec<usize>> = if active_subset.is_none() {
        Some((0..parts.len()).collect())
    } else {
        None
    };
    let active_idxs: &[usize] = active_subset.unwrap_or_else(|| all_idxs_storage.as_deref().unwrap());
    let min_sheets_stop = seed_sheets.map_or(1, |s| s.len().max(1));
    if best.len() <= min_sheets_stop || active_idxs.is_empty() { return best; }
    let usable_area = cfg.usable_w * cfg.usable_h;

    // 1. Iterative Tail-Boosted Repack
    let boost_rounds = if active_idxs.len() > 1000 { 4 } else { 6 };
    let mut boosts = vec![0u32; parts.len()];
    for round in 0..boost_rounds {
        let min_idx_opt = if seed_sheets.is_some() {
            best.iter().enumerate()
                .filter(|(_, s)| !s.has_two_sided)
                .min_by(|a, b| a.1.used_area.partial_cmp(&b.1.used_area).unwrap_or(std::cmp::Ordering::Equal))
                .or_else(|| best.iter().enumerate().min_by(|a, b| a.1.used_area.partial_cmp(&b.1.used_area).unwrap_or(std::cmp::Ordering::Equal)))
        } else {
            best.iter().enumerate()
                .min_by(|a, b| a.1.used_area.partial_cmp(&b.1.used_area).unwrap_or(std::cmp::Ordering::Equal))
        };
        let Some((min_idx, _)) = min_idx_opt else { break; };
        for it in &best[min_idx].items {
            let p_idx = it.part_idx as usize;
            if seed_sheets.is_none() || !parts[p_idx].two_sided {
                boosts[p_idx] += 1;
            }
        }
        let rule = if round % 2 == 0 { BinRule::Bssf } else { BinRule::Baf };
        let ord_kind = match round % 3 { 0 => SortOrder::MaxSideDesc, 1 => SortOrder::AreaDesc, _ => SortOrder::WeightedDimDesc };
        let order = build_order_subset(parts, active_idxs, ord_kind, cfg.sheet_in_sheet, two_sided_first, Some(&boosts));
        let init_sheets = seed_sheets.map_or_else(|| Vec::with_capacity(8), |s| s.to_vec());
        if let Some(cand) = pack_order_onto(parts, cfg, &order, rule, true, Some(best.len()), init_sheets) {
            if solution_score(&cand, strict_two_sided) < solution_score(&best, strict_two_sided) {
                best = cand;
                if best.len() <= min_sheets_stop { return best; }
            }
        }
    }

    // 2. Direct single-part eviction of sparsest sheet into earlier sheets
    evict_sparsest_sheet(parts, cfg, &mut best, two_sided_first || strict_two_sided);
    if best.len() <= min_sheets_stop { return best; }

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
                let max_ts = 1usize.min(usize::from(best[a].has_two_sided) + usize::from(best[b].has_two_sided));
                if let Some(mut merged) = try_repack_subset(parts, cfg, &comb, 1, two_sided_first || strict_two_sided, Some(max_ts)) {
                    if merged.len() == 1 {
                        best.remove(a.max(b)); best.remove(a.min(b));
                        best.push(merged.pop().unwrap());
                        improved = true;
                        break 'pair;
                    }
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
                    let orig_ts = usize::from(best[a].has_two_sided) + usize::from(best[b].has_two_sided) + usize::from(best[c].has_two_sided);
                    let max_ts = if strict_two_sided || two_sided_first { Some(orig_ts) } else { None };
                    if let Some(merged) = try_repack_subset(parts, cfg, &comb, 2, two_sided_first || strict_two_sided, max_ts) {
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
    let total_area: f64 = active_idxs.iter().map(|&i| parts[i].area).sum::<f64>()
        + seed_sheets.map_or(0.0, |ss| ss.iter().map(|s| s.used_area).sum());
    let min_possible = (total_area / usable_area).ceil().max(min_sheets_stop as f64) as usize;
    if best.len() <= min_possible {
        return best;
    }
    let budget_ms = if active_idxs.len() <= 100 { 45 } else if active_idxs.len() <= 600 { 80 } else { 110 };
    let deadline = Instant::now() + Duration::from_millis(budget_ms);
    let mut rng = XorRng::new(seed ^ (active_idxs.len() as u64).wrapping_mul(0x517C_C1B7_2722_0A95));
    let mut best_sc = solution_score(&best, strict_two_sided);

    // 5A. Small/medium materials: fast 60 random-key + tail-boosted full repacks
    if active_idxs.len() <= 100 && best.len() > min_possible {
        let rules_sm = [BinRule::Bssf, BinRule::Baf, BinRule::Blsf, BinRule::BottomLeft];
        for it in 0..60 {
            if Instant::now() >= deadline || best.len() <= min_possible { break; }
            let mut scored: Vec<(usize, f64)> = active_idxs.iter().map(|&pi| {
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
                let sh_opt = if seed_sheets.is_some() {
                    best.iter().enumerate()
                        .filter(|(_, s)| !s.has_two_sided)
                        .min_by(|a, b| a.1.used_area.partial_cmp(&b.1.used_area).unwrap_or(std::cmp::Ordering::Equal))
                } else {
                    best.iter().enumerate()
                        .min_by(|a, b| a.1.used_area.partial_cmp(&b.1.used_area).unwrap_or(std::cmp::Ordering::Equal))
                };
                if let Some((_, sh)) = sh_opt {
                    for itm in &sh.items {
                        let pi = itm.part_idx as usize;
                        if seed_sheets.is_some() && parts[pi].two_sided { continue; }
                        if let Some(entry) = scored.iter_mut().find(|(epi, _)| *epi == pi) {
                            entry.1 *= 1.6;
                        }
                    }
                }
            }
            scored.sort_by(|a, b| {
                let (pa, pb) = (&parts[a.0], &parts[b.0]);
                if two_sided_first && pa.two_sided != pb.two_sided { return pb.two_sided.cmp(&pa.two_sided); }
                if cfg.sheet_in_sheet && pa.has_holes != pb.has_holes { return pb.has_holes.cmp(&pa.has_holes); }
                b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal)
            });
            let ord: Vec<usize> = scored.into_iter().map(|(i, _)| i).collect();
            let rule = rules_sm[it % rules_sm.len()];
            let bf_all = it % 5 != 4;
            let init_sheets = seed_sheets.map_or_else(|| Vec::with_capacity(8), |s| s.to_vec());
            if let Some(cand) = pack_order_onto(parts, cfg, &ord, rule, bf_all, Some(best.len()), init_sheets) {
                let sc = solution_score(&cand, strict_two_sided);
                if sc < best_sc { best = cand; best_sc = sc; }
            }
        }
    }

    // 5B. Bounded ALNS
    let max_alns = if active_idxs.len() <= 150 { 20 } else if active_idxs.len() <= 600 { 14 } else { 10 };
    for iter in 0..max_alns {
        if Instant::now() >= deadline || best.len() <= min_possible { break; }
        let n_sh = best.len();
        let mut ranked: Vec<usize> = if strict_two_sided {
            (0..n_sh).filter(|&i| !best[i].has_two_sided).collect()
        } else {
            (0..n_sh).collect()
        };
        if ranked.len() < 2 { break; }
        ranked.sort_by(|&a, &b| best[a].used_area.partial_cmp(&best[b].used_area).unwrap_or(std::cmp::Ordering::Equal));
        let target_waste = usable_area * (1.15 + 0.12 * (iter % 3) as f64);
        let max_ruin = ranked.len().min(12);
        let mut mask = vec![false; n_sh];
        let (mut r_waste, mut r_cnt) = (0.0f64, 0usize);
        if iter % 2 == 0 {
            for &si in &ranked {
                if r_cnt >= 2 && (r_waste >= target_waste || r_cnt >= max_ruin) { break; }
                mask[si] = true; r_waste += (usable_area - best[si].used_area).max(0.0); r_cnt += 1;
            }
        } else {
            mask[ranked[0]] = true; r_waste += (usable_area - best[ranked[0]].used_area).max(0.0); r_cnt = 1;
            let pool_lim = (ranked.len() * 2 / 3).max(2).min(ranked.len());
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
        scored.sort_by(|a, b| {
            if !strict_two_sided && two_sided_first {
                let (pa, pb) = (&parts[a.0], &parts[b.0]);
                if pa.two_sided != pb.two_sided { return pb.two_sided.cmp(&pa.two_sided); }
            }
            b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal)
        });
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
                if !strict_two_sided && two_sided_first && parts[pi].two_sided && !kept[si].has_two_sided {
                    continue;
                }
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
        let sc = solution_score(&kept, strict_two_sided);
        if sc < best_sc {
            let dropped = kept.len() < best.len();
            best = kept; best_sc = sc;
            if dropped {
                evict_sparsest_sheet(parts, cfg, &mut best, two_sided_first || strict_two_sided);
                best_sc = solution_score(&best, strict_two_sided);
            }
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

#[inline]
fn item_in_board_bounds(cfg: &SheetConfig, it: &PlacedItem, var: &OrientedPoly) -> bool {
    it.x >= cfg.margin - 1e-4
        && it.y >= cfg.margin - 1e-4
        && it.x + var.w <= cfg.board_w - cfg.margin + 1e-4
        && it.y + var.h <= cfg.board_h - cfg.margin + 1e-4
}

#[inline]
fn items_nested_in_hole(
    parts: &[PartSpec], cfg: &SheetConfig,
    a: &PlacedItem, va: &OrientedPoly,
    b: &PlacedItem, vb: &OrientedPoly,
) -> bool {
    let _ = parts;
    cfg.sheet_in_sheet && (
        vb.holes.iter().any(|h| poly_contained_in_hole_exact(va, a.x, a.y, h, b.x, b.y, 0.0))
        || va.holes.iter().any(|h| poly_contained_in_hole_exact(vb, b.x, b.y, h, a.x, a.y, 0.0))
    )
}

fn find_sheet_violation(parts: &[PartSpec], cfg: &SheetConfig, sh: &SpatialSheet) -> Option<usize> {
    let n = sh.items.len();
    for i in 0..n {
        let a = &sh.items[i];
        let va = &parts[a.part_idx as usize].variants[a.var_idx as usize];
        if !item_in_board_bounds(cfg, a, va) {
            if n > 1 || (a.x - cfg.margin).abs() > 1e-4 || (a.y - cfg.margin).abs() > 1e-4 {
                return Some(i);
            }
        }
        for j in (i + 1)..n {
            let b = &sh.items[j];
            let vb = &parts[b.part_idx as usize].variants[b.var_idx as usize];
            if !items_nested_in_hole(parts, cfg, a, va, b, vb)
                && polys_conflict_exact(va, a.x, a.y, vb, b.x, b.y, 0.0)
            {
                let area_a = parts[a.part_idx as usize].area;
                let area_b = parts[b.part_idx as usize].area;
                return Some(if area_a < area_b { i } else { j });
            }
        }
    }
    None
}

fn verify_and_sanitize_sheets(parts: &[PartSpec], cfg: &SheetConfig, sheets: &mut Vec<SpatialSheet>) -> bool {
    for _round in 0..3 {
        let mut evicted: Vec<PlacedItem> = Vec::new();
        for sh in sheets.iter_mut() {
            while let Some(victim_idx) = find_sheet_violation(parts, cfg, sh) {
                let victim = sh.items.remove(victim_idx);
                if victim_idx < sh.item_boxes.len() {
                    sh.item_boxes.remove(victim_idx);
                }
                sh.used_area = sh.items.iter().map(|it| parts[it.part_idx as usize].area).sum();
                sh.has_two_sided = sh.items.iter().any(|it| parts[it.part_idx as usize].two_sided);
                evicted.push(victim);
            }
        }
        sheets.retain(|sh| !sh.items.is_empty());
        if evicted.is_empty() {
            break;
        }
        for victim in evicted {
            let mut ns = SpatialSheet::new(cfg);
            ns.commit_placement(
                parts,
                cfg,
                victim.part_idx as usize,
                victim.var_idx as usize,
                cfg.margin,
                cfg.margin,
            );
            sheets.push(ns);
        }
    }

    let mut total_items = 0usize;
    for sh in sheets.iter() {
        let n = sh.items.len();
        total_items += n;
        for i in 0..n {
            let a = &sh.items[i];
            let va = &parts[a.part_idx as usize].variants[a.var_idx as usize];
            if !item_in_board_bounds(cfg, a, va) {
                return false;
            }
            for j in (i + 1)..n {
                let b = &sh.items[j];
                let vb = &parts[b.part_idx as usize].variants[b.var_idx as usize];
                if !items_nested_in_hole(parts, cfg, a, va, b, vb)
                    && polys_conflict_exact(va, a.x, a.y, vb, b.x, b.y, 0.0)
                {
                    return false;
                }
            }
        }
    }
    total_items == parts.len()
}

fn materialize_layout(
    material_state: &MaterialStateInput, cfg: &SheetConfig, parts: &[PartSpec],
    mut sheets: Vec<SpatialSheet>, compact_directions: Vec<String>, target_util: f64, global_rot_div: usize,
) -> LayoutOutput {
    let verified_clean = verify_and_sanitize_sheets(parts, cfg, &mut sheets);
    sheets.sort_by(|a, b| {
        let a_ts_cnt = a.items.iter().filter(|it| parts[it.part_idx as usize].two_sided).count();
        let b_ts_cnt = b.items.iter().filter(|it| parts[it.part_idx as usize].two_sided).count();
        let a_ts_area: f64 = a.items.iter().filter(|it| parts[it.part_idx as usize].two_sided).map(|it| parts[it.part_idx as usize].area).sum();
        let b_ts_area: f64 = b.items.iter().filter(|it| parts[it.part_idx as usize].two_sided).map(|it| parts[it.part_idx as usize].area).sum();
        b.has_two_sided.cmp(&a.has_two_sided)
            .then_with(|| b_ts_cnt.cmp(&a_ts_cnt))
            .then_with(|| b_ts_area.partial_cmp(&a_ts_area).unwrap_or(std::cmp::Ordering::Equal))
            .then_with(|| b.used_area.partial_cmp(&a.used_area).unwrap_or(std::cmp::Ordering::Equal))
    });
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
        safety_certified: verified_clean, safety_version: 2,
    }
}

fn run_tournament(
    parts: &[PartSpec], cfg: &SheetConfig, active_idxs: Option<&[usize]>,
    base_sheets: Option<&[SpatialSheet]>, strategies: &[Strategy], two_sided_first: bool,
    strict_two_sided: bool,
) -> Vec<SpatialSheet> {
    strategies
        .par_iter()
        .enumerate()
        .map(|(s_idx, st)| {
            let order = match active_idxs {
                Some(idxs) => build_order_subset(parts, idxs, st.order, cfg.sheet_in_sheet, two_sided_first && st.two_sided_first, None),
                None => build_order(parts, st.order, cfg.sheet_in_sheet, two_sided_first && st.two_sided_first, None),
            };
            let init = base_sheets.map_or_else(|| Vec::with_capacity(8), |bs| bs.to_vec());
            Some((s_idx, pack_order_onto(parts, cfg, &order, st.rule, st.best_fit_all, None, init).unwrap_or_default()))
        })
        .reduce(
            || None,
            |acc, item| match (acc, item) {
                (None, x) | (x, None) => x,
                (Some((ia, sa)), Some((ib, sb))) => {
                    if solution_score(&sa, strict_two_sided).cmp(&solution_score(&sb, strict_two_sided)).then_with(|| ia.cmp(&ib)) != std::cmp::Ordering::Greater {
                        Some((ia, sa))
                    } else {
                        Some((ib, sb))
                    }
                }
            },
        )
        .map(|(_, s)| s)
        .unwrap_or_default()
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
    let edge_zone = raw_cfg.small_part_edge_zone.unwrap_or(if merge_cut { 200.0 } else { 0.0 }).max(0.0);
    let compact_dirs = raw_cfg.compact_directions.clone().unwrap_or_else(|| vec!["left".to_string(), "bottom".to_string()]);
    let target_util = raw_cfg.target_sheet_utilization.unwrap_or(90.0);
    let nest_by_room = raw_cfg.nest_by_room.unwrap_or(true);

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
    let (two_sided_idxs, one_sided_idxs): (Vec<usize>, Vec<usize>) = (0..parts.len()).partition(|&i| parts[i].two_sided);
    let ts = !two_sided_idxs.is_empty();
    let strict_two_sided = ts && !nest_by_room;

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

    let seed = material_state.optimization_seed.or(raw_cfg.optimization_seed).unwrap_or(42);
    let best_sheets = if strict_two_sided && !one_sided_idxs.is_empty() {
        // Phase A: Consolidate all 2-sided parts onto the minimum number of leading sheets first
        let best_2s = run_tournament(&parts, &cfg, Some(&two_sided_idxs), None, &strategies, false, false);
        let best_2s = squeeze_sheets_opts(&parts, &cfg, best_2s, seed, Some(&two_sided_idxs), None, false, false);
        // Phase B: Backfill remaining free space on those leading sheets and pack 1-sided parts
        let best_sheets = run_tournament(&parts, &cfg, Some(&one_sided_idxs), Some(&best_2s), &strategies, false, true);
        // Phase C: Squeeze total sheet count without spilling 2-sided parts onto later sheets
        squeeze_sheets_opts(&parts, &cfg, best_sheets, seed, Some(&one_sided_idxs), Some(&best_2s), true, true)
    } else {
        let best_sheets = run_tournament(&parts, &cfg, None, None, &strategies, ts, false);
        squeeze_sheets_opts(&parts, &cfg, best_sheets, seed, None, None, ts, false)
    };
    let layout = materialize_layout(material_state, &cfg, &parts, best_sheets, compact_dirs, target_util, global_rot_div);
    on_progress("Done", 100.0, 1, 1, &layout);
    layout
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::ConfigurationInput;

    fn make_part(id: &str, contour: Vec<[f64; 2]>, holes: Option<Vec<Vec<[f64; 2]>>>) -> PartInput {
        PartInput {
            id: Some(id.to_string()),
            entity_id: Some(id.to_string()),
            name: Some(id.to_string()),
            width: None,
            height: None,
            area: None,
            contour,
            holes,
            render_contour: None,
            render_holes: None,
            collision_contour: None,
            collision_holes: None,
            draw_layers: None,
            rotations: Some(vec![0.0]),
            base_rotation_degrees: Some(0.0),
            rotation_degrees: Some(0.0),
            grain_locked: Some(true),
            has_grain_label: None,
            grain_arrow_degrees: None,
            free_rotation: Some(false),
            rotation_divisions: Some(1),
            color: None,
            logical_part_count: None,
            manual_cluster_macro: None,
            manual_cluster_children: None,
            two_sided: None,
            has_circular_arc: None,
        }
    }

    fn test_cfg(sheet_in_sheet: bool) -> SheetConfig {
        SheetConfig {
            board_w: 1220.0,
            board_h: 2440.0,
            margin: 10.0,
            usable_w: 1200.0,
            usable_h: 2420.0,
            spacing: 5.0,
            sheet_in_sheet,
            small_thr: 0.0,
            small_clr: 0.0,
            merge_cut: false,
            edge_zone: 0.0,
            max_possible_gap: 5.0,
            compact_right: false,
            compact_top: false,
        }
    }

    #[test]
    fn test_a_solid_arch_over_64_verts_vs_bottom_overlap() {
        // Vòm đặc > 64 đỉnh: outer 800x600, đáy khoét vòm từ x=150..650 lên đỉnh y=450
        let mut arch_contour = vec![
            [0.0, 0.0],
            [150.0, 0.0],
        ];
        let arc_steps = 72usize;
        for i in 0..=arc_steps {
            let theta = std::f64::consts::PI * (1.0 - (i as f64) / (arc_steps as f64));
            let x = 400.0 + 250.0 * theta.cos();
            let y = 200.0 + 250.0 * theta.sin();
            arch_contour.push([x, y]);
        }
        arch_contour.push([650.0, 0.0]);
        arch_contour.push([800.0, 0.0]);
        arch_contour.push([800.0, 600.0]);
        arch_contour.push([0.0, 600.0]);
        assert!(arch_contour.len() > 64);

        let arch = build_oriented_poly(&arch_contour, None, 0.0, 0.0, false, None);
        let small_box = build_oriented_poly(
            &[[0.0, 0.0], [200.0, 0.0], [200.0, 200.0], [0.0, 200.0]],
            None,
            0.0,
            0.0,
            false,
            None,
        );

        // Nằm gọn trong khoang lõm đáy vòm (x=300..500, y=0..200) -> không va chạm
        assert!(!polys_conflict_exact(&arch, 0.0, 0.0, &small_box, 300.0, 0.0, 0.0));
        // Đè lên chân đáy trái (x=0..200, y=0..200) -> phải báo va chạm
        assert!(polys_conflict_exact(&arch, 0.0, 0.0, &small_box, 0.0, 0.0, 0.0));
        // Đè lên đỉnh vòm (x=300..500, y=400..600) -> phải báo va chạm
        assert!(polys_conflict_exact(&arch, 0.0, 0.0, &small_box, 300.0, 400.0, 0.0));
    }

    #[test]
    fn test_b_multi_level_hole_nesting() {
        // Vòm/khung ngoài 1000x800 có lỗ 800x600 -> chứa khung giữa 700x500 có lỗ 500x300 -> chứa thanh con 300x150
        let outer = make_part(
            "outer",
            vec![[0.0, 0.0], [1000.0, 0.0], [1000.0, 800.0], [0.0, 800.0]],
            Some(vec![vec![[100.0, 100.0], [900.0, 100.0], [900.0, 700.0], [100.0, 700.0]]]),
        );
        let middle = make_part(
            "middle",
            vec![[0.0, 0.0], [700.0, 0.0], [700.0, 500.0], [0.0, 500.0]],
            Some(vec![vec![[100.0, 100.0], [600.0, 100.0], [600.0, 400.0], [100.0, 400.0]]]),
        );
        let inner_bar = make_part(
            "inner_bar",
            vec![[0.0, 0.0], [300.0, 0.0], [300.0, 150.0], [0.0, 150.0]],
            None,
        );

        let mat = MaterialStateInput {
            key: None,
            material: Some("MDF".to_string()),
            thickness: Some(17.0),
            group_index: None,
            configuration: Some(ConfigurationInput {
                board_width: Some(1050.0),
                board_height: Some(850.0),
                edge_margin: Some(10.0),
                part_spacing: Some(5.0),
                sheet_in_sheet: Some(true),
                rotation_divisions: Some(1),
                ..Default::default()
            }),
            original_part_count: Some(3),
            parts: vec![outer, middle, inner_bar],
            learning_key: None,
            optimization_seed: Some(42),
        };

        let layout = optimize_material_layout(&mat, |_, _, _, _, _| {});
        assert!(layout.safety_certified);
        assert_eq!(layout.sheets.len(), 1, "All 3 multi-level nested parts must fit on 1 sheet");
        assert_eq!(layout.sheets[0].placements.len(), 3);
    }

    #[test]
    fn test_c_concave_overlap_and_touching_rects_at_zero_gap() {
        // Chữ C lõm có centroid nằm ngoài phần thịt đa giác
        let c_shape = build_oriented_poly(
            &[
                [0.0, 0.0], [400.0, 0.0], [400.0, 40.0], [40.0, 40.0],
                [40.0, 460.0], [400.0, 460.0], [400.0, 500.0], [0.0, 500.0],
            ],
            None,
            0.0,
            0.0,
            false,
            None,
        );
        assert!(!c_shape.centroid_inside);
        // Hai đa giác lõm trùng khít tại gap = 0.0 phải báo conflict
        assert!(polys_conflict_exact(&c_shape, 100.0, 100.0, &c_shape, 100.0, 100.0, 0.0));

        // Hai hình chữ nhật chạm cạnh khít tại gap = 0.0 không báo giả
        let rect = build_oriented_poly(
            &[[0.0, 0.0], [200.0, 0.0], [200.0, 300.0], [0.0, 300.0]],
            None,
            0.0,
            0.0,
            false,
            None,
        );
        assert!(!polys_conflict_exact(&rect, 0.0, 0.0, &rect, 200.0, 0.0, 0.0));
        assert!(!polys_conflict_exact(&rect, 0.0, 0.0, &rect, 0.0, 300.0, 0.0));
        // Và đa giác lõm chạm cạnh hình chữ nhật tại gap = 0.0 không báo giả
        assert!(!polys_conflict_exact(&c_shape, 0.0, 0.0, &rect, 400.0, 0.0, 0.0));
    }

    #[test]
    fn test_d_gap_008_conflict() {
        // Đa giác không phải rect thuần để đi qua kiểm tra đa giác chính xác (không qua fast-path rect)
        let poly_a = build_oriented_poly(
            &[[0.0, 0.0], [100.0, 0.0], [100.0, 95.0], [95.0, 100.0], [0.0, 100.0]],
            None,
            0.0,
            0.0,
            false,
            None,
        );
        let small_b = build_oriented_poly(
            &[[0.0, 0.0], [10.0, 0.0], [10.0, 9.0], [9.0, 10.0], [0.0, 10.0]],
            None,
            0.0,
            0.0,
            false,
            None,
        );

        // Khoảng cách cạnh 0.04 < gap = 0.08 -> phải báo conflict
        assert!(polys_conflict_exact(&poly_a, 0.0, 0.0, &small_b, 100.04, 0.0, 0.08));
        // Nằm lọt bên trong cách biên 0.10 (trước đây pt_tol=0.14 bỏ sót) -> phải báo conflict tại gap = 0.08
        assert!(polys_conflict_exact(&poly_a, 0.0, 0.0, &small_b, 0.10, 0.10, 0.08));
        // Cách nhau 0.12 > gap = 0.08 -> không báo conflict
        assert!(!polys_conflict_exact(&poly_a, 0.0, 0.0, &small_b, 100.12, 0.0, 0.08));
    }

    #[test]
    fn test_e_verify_and_sanitize_sheets_evicts_overlap() {
        let big = make_part("big", vec![[0.0, 0.0], [500.0, 0.0], [500.0, 500.0], [0.0, 500.0]], None);
        let small = make_part("small", vec![[0.0, 0.0], [200.0, 0.0], [200.0, 200.0], [0.0, 200.0]], None);
        let mat = MaterialStateInput {
            key: None,
            material: Some("MDF".to_string()),
            thickness: Some(17.0),
            group_index: None,
            configuration: Some(ConfigurationInput {
                board_width: Some(1220.0),
                board_height: Some(2440.0),
                edge_margin: Some(10.0),
                part_spacing: Some(5.0),
                sheet_in_sheet: Some(false),
                rotation_divisions: Some(1),
                ..Default::default()
            }),
            original_part_count: Some(2),
            parts: vec![big, small],
            learning_key: None,
            optimization_seed: Some(42),
        };
        let cfg = test_cfg(false);
        let (specs, _) = prepare_parts(&mat, &cfg, 1);

        let mut sh = SpatialSheet::new(&cfg);
        sh.commit_placement(&specs, &cfg, 0, 0, 10.0, 10.0);
        sh.commit_placement(&specs, &cfg, 1, 0, 10.0, 10.0); // cố tình đè lên tấm lớn
        let mut sheets = vec![sh];

        assert!(verify_and_sanitize_sheets(&specs, &cfg, &mut sheets));
        assert_eq!(sheets.len(), 2);
        assert_eq!(sheets[0].items.len(), 1);
        assert_eq!(sheets[0].items[0].part_idx, 0);
        assert!((sheets[0].used_area - 250_000.0).abs() < 1e-3);
        assert_eq!(sheets[1].items.len(), 1);
        assert_eq!(sheets[1].items[0].part_idx, 1);
        assert!((sheets[1].items[0].x - cfg.margin).abs() < 1e-6);
        assert!((sheets[1].items[0].y - cfg.margin).abs() < 1e-6);
        assert!((sheets[1].used_area - 40_000.0).abs() < 1e-3);

        let layout = materialize_layout(&mat, &cfg, &specs, sheets, vec!["left".to_string()], 90.0, 1);
        assert!(layout.safety_certified);
        assert_eq!(layout.sheets.len(), 2);
    }

    fn make_rect_part(id: &str, w: f64, h: f64, two_sided: bool) -> PartInput {
        let mut p = make_part(id, vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]], None);
        p.width = Some(w);
        p.height = Some(h);
        p.area = Some(w * h);
        p.two_sided = Some(two_sided);
        p
    }

    #[test]
    fn test_f_two_sided_grouped_leading_sheets_small_job() {
        let mut parts = Vec::new();
        // 3 two-sided parts fitting easily on 1 sheet (~0.62 sheet area)
        parts.push(make_rect_part("ts_1", 550.0, 1200.0, true));
        parts.push(make_rect_part("ts_2", 350.0, 1400.0, true));
        parts.push(make_rect_part("ts_3", 250.0, 1100.0, true));
        // 30 varied one-sided parts (~4-5 sheets)
        for i in 0..30 {
            let (w, h) = match i % 5 {
                0 => (560.0, 1800.0),
                1 => (450.0, 1100.0),
                2 => (380.0, 900.0),
                3 => (280.0, 750.0),
                _ => (180.0, 620.0),
            };
            parts.push(make_rect_part(&format!("os_{}", i), w, h, false));
        }

        let mat = MaterialStateInput {
            key: None,
            material: Some("MDF".to_string()),
            thickness: Some(17.5),
            group_index: None,
            configuration: Some(ConfigurationInput {
                board_width: Some(1220.0),
                board_height: Some(2440.0),
                edge_margin: Some(3.0),
                part_spacing: Some(5.0),
                sheet_in_sheet: Some(false),
                rotation_divisions: Some(1),
                nest_by_room: Some(false),
                ..Default::default()
            }),
            original_part_count: Some(parts.len()),
            parts,
            learning_key: None,
            optimization_seed: Some(42),
        };

        let layout = optimize_material_layout(&mat, |_, _, _, _, _| {});
        assert!(layout.safety_certified);
        let total_placed: usize = layout.sheets.iter().map(|s| s.placements.len()).sum();
        assert_eq!(total_placed, 33);

        let ts_sheet_indices: Vec<usize> = layout.sheets.iter().enumerate()
            .filter(|(_, s)| s.placements.iter().any(|p| p.two_sided == Some(true)))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(ts_sheet_indices, vec![0], "All 3 two-sided parts must be grouped strictly onto Sheet 1");
        assert_eq!(
            layout.sheets[0].placements.iter().filter(|p| p.two_sided == Some(true)).count(),
            3
        );
    }

    #[test]
    fn test_g_two_sided_grouped_leading_sheets_large_job() {
        let mut parts = Vec::new();
        // 11 two-sided parts (~1.42 sheets -> must consolidate onto <= 2 leading sheets)
        for i in 0..11 {
            let (w, h) = if i < 3 { (560.0, 1100.0) } else if i < 7 { (420.0, 950.0) } else { (320.0, 820.0) };
            parts.push(make_rect_part(&format!("ts_{}", i), w, h, true));
        }
        // 90 one-sided parts (~13-15 sheets)
        for i in 0..90 {
            let (w, h) = match i % 6 {
                0 => (580.0, 2100.0),
                1 => (500.0, 1600.0),
                2 => (420.0, 1200.0),
                3 => (350.0, 850.0),
                4 => (240.0, 700.0),
                _ => (150.0, 550.0),
            };
            parts.push(make_rect_part(&format!("os_{}", i), w, h, false));
        }

        let mat = MaterialStateInput {
            key: None,
            material: Some("A150G".to_string()),
            thickness: Some(17.5),
            group_index: None,
            configuration: Some(ConfigurationInput {
                board_width: Some(1220.0),
                board_height: Some(2440.0),
                edge_margin: Some(3.0),
                part_spacing: Some(5.0),
                sheet_in_sheet: Some(false),
                rotation_divisions: Some(1),
                nest_by_room: Some(false),
                ..Default::default()
            }),
            original_part_count: Some(parts.len()),
            parts,
            learning_key: None,
            optimization_seed: Some(42),
        };

        let layout = optimize_material_layout(&mat, |_, _, _, _, _| {});
        assert!(layout.safety_certified);
        let total_placed: usize = layout.sheets.iter().map(|s| s.placements.len()).sum();
        assert_eq!(total_placed, 101);

        let ts_sheet_indices: Vec<usize> = layout.sheets.iter().enumerate()
            .filter(|(_, s)| s.placements.iter().any(|p| p.two_sided == Some(true)))
            .map(|(i, _)| i)
            .collect();
        assert!(!ts_sheet_indices.is_empty() && ts_sheet_indices.len() <= 2, "Expected <=2 leading 2-sided sheets, got {:?}", ts_sheet_indices);
        let expected_leading: Vec<usize> = (0..ts_sheet_indices.len()).collect();
        assert_eq!(ts_sheet_indices, expected_leading, "2-sided sheets must be strictly contiguous at the front");
    }

    fn build_mixed_two_sided_and_holed_parts() -> Vec<PartInput> {
        let mut parts = Vec::with_capacity(72);
        // 12 two-sided parts (~1.40 sheets of 1200x2420 usable area = 2,904,000)
        for i in 0..12 {
            let (w, h) = if i < 4 {
                (560.0, 1000.0)
            } else if i < 8 {
                (420.0, 700.0)
            } else {
                (300.0, 540.0)
            };
            parts.push(make_rect_part(&format!("ts_{}", i), w, h, true));
        }
        // 60 varied 1-sided parts including large panels & holed frames
        for i in 0..60 {
            match i % 6 {
                0 => {
                    let mut frame = make_part(
                        &format!("os_frame_{}", i),
                        vec![[0.0, 0.0], [900.0, 0.0], [900.0, 1200.0], [0.0, 1200.0]],
                        Some(vec![vec![[120.0, 120.0], [780.0, 120.0], [780.0, 1080.0], [120.0, 1080.0]]]),
                    );
                    frame.width = Some(900.0);
                    frame.height = Some(1200.0);
                    frame.two_sided = Some(false);
                    parts.push(frame);
                }
                1 => parts.push(make_rect_part(&format!("os_{}", i), 580.0, 2000.0, false)),
                2 => parts.push(make_rect_part(&format!("os_{}", i), 500.0, 1400.0, false)),
                3 => parts.push(make_rect_part(&format!("os_{}", i), 400.0, 800.0, false)),
                4 => parts.push(make_rect_part(&format!("os_{}", i), 300.0, 600.0, false)),
                _ => parts.push(make_rect_part(&format!("os_{}", i), 200.0, 400.0, false)),
            }
        }
        parts
    }

    #[test]
    fn test_strict_two_sided_consolidates_when_nest_by_room_false() {
        let parts = build_mixed_two_sided_and_holed_parts();
        let mat = MaterialStateInput {
            key: None,
            material: Some("MDF".to_string()),
            thickness: Some(17.5),
            group_index: None,
            configuration: Some(ConfigurationInput {
                board_width: Some(1220.0),
                board_height: Some(2440.0),
                edge_margin: Some(10.0),
                part_spacing: Some(5.0),
                sheet_in_sheet: Some(true),
                rotation_divisions: Some(1),
                nest_by_room: Some(false),
                ..Default::default()
            }),
            original_part_count: Some(parts.len()),
            parts,
            learning_key: None,
            optimization_seed: Some(42),
        };

        let layout = optimize_material_layout(&mat, |_, _, _, _, _| {});
        assert!(layout.safety_certified);
        let total_placed: usize = layout.sheets.iter().map(|s| s.placements.len()).sum();
        assert_eq!(total_placed, 72);

        let ts_sheet_indices: Vec<usize> = layout.sheets.iter()
            .filter(|s| s.placements.iter().any(|p| p.two_sided == Some(true)))
            .map(|s| s.index)
            .collect();
        assert!(
            !ts_sheet_indices.is_empty() && ts_sheet_indices.len() <= 2,
            "Expected <= 2 two-sided sheets, got {:?}",
            ts_sheet_indices
        );
        let expected_leading: Vec<usize> = (1..=ts_sheet_indices.len()).collect();
        assert_eq!(
            ts_sheet_indices, expected_leading,
            "Two-sided placements must occupy leading sheets 1..=K with zero on later sheets"
        );
    }

    #[test]
    fn test_nest_by_room_true_keeps_valid_layout() {
        let parts = build_mixed_two_sided_and_holed_parts();
        let mat = MaterialStateInput {
            key: None,
            material: Some("MDF".to_string()),
            thickness: Some(17.5),
            group_index: None,
            configuration: Some(ConfigurationInput {
                board_width: Some(1220.0),
                board_height: Some(2440.0),
                edge_margin: Some(10.0),
                part_spacing: Some(5.0),
                sheet_in_sheet: Some(true),
                rotation_divisions: Some(1),
                nest_by_room: Some(true),
                ..Default::default()
            }),
            original_part_count: Some(parts.len()),
            parts,
            learning_key: None,
            optimization_seed: Some(42),
        };

        let layout = optimize_material_layout(&mat, |_, _, _, _, _| {});
        assert!(layout.safety_certified);
        let total_placed: usize = layout.sheets.iter().map(|s| s.placements.len()).sum();
        assert_eq!(total_placed, 72);
    }
}
