// src/spatial.rs - Compact SpatialSheet with adaptive 2D grid, incremental MaxRects
// free-space manager, exact clearance/hole/bounds checks, and fast polygon sliding.
use std::sync::Arc;
use super::geom::{poly_contained_in_hole_exact, polys_conflict_exact, BBox, OrientedPoly};

#[derive(Debug, Clone, Copy)]
pub struct SheetConfig {
    pub board_w: f64,
    pub board_h: f64,
    pub margin: f64,
    pub usable_w: f64,
    pub usable_h: f64,
    pub spacing: f64,
    pub sheet_in_sheet: bool,
    pub small_thr: f64,
    pub small_clr: f64,
    pub merge_cut: bool,
    pub edge_zone: f64,
    pub max_possible_gap: f64,
    pub compact_right: bool,
    pub compact_top: bool,
}

impl SheetConfig {
    #[inline]
    pub fn in_edge_zone(&self, x: f64, y: f64, w: f64, h: f64) -> bool {
        if !self.merge_cut { return false; }
        let left = x.max(0.0);
        let right = (self.board_w - (x + w)).max(0.0);
        let top = (self.board_h - (y + h)).max(0.0);
        let bottom = y.max(0.0);
        let in_bottom = bottom < self.edge_zone && (self.edge_zone - bottom).min(h) >= h * 0.6 - 0.12;
        left.min(right).min(top) <= self.edge_zone + 0.15 || in_bottom
    }

    #[inline]
    pub fn req_clearance_at(&self, small_part: bool, part_sclr: f64, x: f64, y: f64, w: f64, h: f64) -> f64 {
        if !small_part || part_sclr <= 0.001 { return self.spacing; }
        if !self.merge_cut || self.in_edge_zone(x, y, w, h) {
            self.spacing.max(part_sclr)
        } else {
            self.spacing
        }
    }
}

pub struct PartSpec {
    pub variants: Arc<Vec<OrientedPoly>>,
    pub area: f64,
    pub min_bbox_area: f64,
    pub max_dim: f64,
    pub min_dim: f64,
    pub perimeter: f64,
    pub small_part: bool,
    pub small_part_clearance: f64,
    pub is_low_fill: bool,
    pub has_holes: bool,
    pub two_sided: bool,
    pub room_id: u32,
    pub shape_hash: u64,
}

#[derive(Debug, Clone, Copy)]
pub struct PlacedItem {
    pub part_idx: u32,
    pub var_idx: u8,
    pub x: f64,
    pub y: f64,
    pub w: f64,
    pub h: f64,
    pub req_clr: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct FreeRect { pub x: f64, pub y: f64, pub w: f64, pub h: f64 }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BinRule { Bssf, Baf, Blsf, BottomLeft, Contact }

const GRID_X: usize = 8;
const GRID_Y: usize = 16;
const GRID_ACTIVATE_THRESHOLD: usize = 24;

#[derive(Clone)]
struct SpatialGrid {
    cells: Vec<Vec<u16>>,
    inv_cw: f64,
    inv_ch: f64,
}

impl SpatialGrid {
    fn new(board_w: f64, board_h: f64) -> Self {
        Self {
            cells: vec![Vec::new(); GRID_X * GRID_Y],
            inv_cw: GRID_X as f64 / board_w.max(1.0),
            inv_ch: GRID_Y as f64 / board_h.max(1.0),
        }
    }

    #[inline]
    fn cell_range(&self, x0: f64, y0: f64, x1: f64, y1: f64) -> (usize, usize, usize, usize) {
        let gx0 = ((x0 * self.inv_cw).floor() as isize).clamp(0, (GRID_X - 1) as isize) as usize;
        let gx1 = ((x1 * self.inv_cw).floor() as isize).clamp(0, (GRID_X - 1) as isize) as usize;
        let gy0 = ((y0 * self.inv_ch).floor() as isize).clamp(0, (GRID_Y - 1) as isize) as usize;
        let gy1 = ((y1 * self.inv_ch).floor() as isize).clamp(0, (GRID_Y - 1) as isize) as usize;
        (gx0, gx1, gy0, gy1)
    }

    fn insert(&mut self, idx: u16, x0: f64, y0: f64, x1: f64, y1: f64) {
        let (gx0, gx1, gy0, gy1) = self.cell_range(x0, y0, x1, y1);
        for gy in gy0..=gy1 {
            let row = gy * GRID_X;
            for gx in gx0..=gx1 { self.cells[row + gx].push(idx); }
        }
    }
}

#[derive(Clone)]
pub struct SpatialSheet {
    pub items: Vec<PlacedItem>,
    pub item_boxes: Vec<[f64; 4]>,
    pub free_rects: Vec<FreeRect>,
    pub used_area: f64,
    pub max_free_area: f64,
    pub max_free_long: f64,
    pub max_free_short: f64,
    pub low_fill_items: Vec<u16>,
    pub has_two_sided: bool,
    pub all_pure_rect_uniform: bool,
    grid: Option<Box<SpatialGrid>>,
}

impl SpatialSheet {
    pub fn new(cfg: &SheetConfig) -> Self {
        Self {
            items: Vec::with_capacity(16),
            item_boxes: Vec::with_capacity(16),
            free_rects: vec![FreeRect { x: cfg.margin, y: cfg.margin, w: cfg.usable_w, h: cfg.usable_h }],
            used_area: 0.0,
            max_free_area: cfg.usable_w * cfg.usable_h,
            max_free_long: cfg.usable_w.max(cfg.usable_h),
            max_free_short: cfg.usable_w.min(cfg.usable_h),
            low_fill_items: Vec::new(),
            has_two_sided: false,
            all_pure_rect_uniform: true,
            grid: None,
        }
    }

    #[inline]
    pub fn may_fit_bbox(&self, min_area: f64, min_dim: f64, max_dim: f64) -> bool {
        self.max_free_area + 1e-3 >= min_area
            && self.max_free_short + 1e-3 >= min_dim
            && self.max_free_long + 1e-3 >= max_dim
    }

    #[allow(clippy::too_many_arguments)]
    #[inline]
    fn conflicts_with_item(
        &self, parts: &[PartSpec], cfg: &SheetConfig, var_a: &OrientedPoly,
        ax: f64, ay: f64, req_a: f64, item_idx: usize,
    ) -> bool {
        let ib = self.item_boxes[item_idx];
        let b_item = &self.items[item_idx];
        let req_gap = req_a.max(b_item.req_clr);
        let pad = if req_gap > 1e-6 { req_gap + 0.03 } else { -1e-4 };
        if ax + var_a.w + pad <= ib[0] || ib[2] + pad <= ax || ay + var_a.h + pad <= ib[1] || ib[3] + pad <= ay {
            return false;
        }
        let var_b = &parts[b_item.part_idx as usize].variants[b_item.var_idx as usize];
        if cfg.sheet_in_sheet {
            if var_b.holes.iter().any(|h| poly_contained_in_hole_exact(var_a, ax, ay, h, b_item.x, b_item.y, req_gap)) {
                return false;
            }
            if var_a.holes.iter().any(|h| poly_contained_in_hole_exact(var_b, b_item.x, b_item.y, h, ax, ay, req_gap)) {
                return false;
            }
        }
        polys_conflict_exact(var_a, ax, ay, var_b, b_item.x, b_item.y, req_gap)
    }

    pub fn can_place(&self, parts: &[PartSpec], cfg: &SheetConfig, part_idx: usize, var_idx: usize, x: f64, y: f64) -> bool {
        let pspec = &parts[part_idx];
        let var_a = &pspec.variants[var_idx];
        if x < cfg.margin - 1e-4 || y < cfg.margin - 1e-4
            || x + var_a.w > cfg.board_w - cfg.margin + 1e-4
            || y + var_a.h > cfg.board_h - cfg.margin + 1e-4
        { return false; }
        let req_a = cfg.req_clearance_at(pspec.small_part, pspec.small_part_clearance, x, y, var_a.w, var_a.h);
        let pad = cfg.max_possible_gap.max(req_a) + 0.1;
        if let Some(ref grid) = self.grid {
            let (gx0, gx1, gy0, gy1) = grid.cell_range(x - pad, y - pad, x + var_a.w + pad, y + var_a.h + pad);
            for gy in gy0..=gy1 {
                let row = gy * GRID_X;
                for gx in gx0..=gx1 {
                    for &idx in &grid.cells[row + gx] {
                        if self.conflicts_with_item(parts, cfg, var_a, x, y, req_a, idx as usize) { return false; }
                    }
                }
            }
            true
        } else {
            (0..self.items.len()).all(|i| !self.conflicts_with_item(parts, cfg, var_a, x, y, req_a, i))
        }
    }

    #[inline]
    fn contact_score(&self, cfg: &SheetConfig, x: f64, y: f64, w: f64, h: f64) -> f64 {
        let mut s = 0.0;
        let tol = cfg.spacing + 1.0;
        if (x - cfg.margin).abs() <= 1.0 || (x + w - (cfg.board_w - cfg.margin)).abs() <= 1.0 { s += h; }
        if (y - cfg.margin).abs() <= 1.0 || (y + h - (cfg.board_h - cfg.margin)).abs() <= 1.0 { s += w; }
        for ib in &self.item_boxes {
            if (ib[0] - (x + w)).abs() <= tol || (x - ib[2]).abs() <= tol {
                s += ((y + h).min(ib[3]) - y.max(ib[1])).max(0.0);
            }
            if (ib[1] - (y + h)).abs() <= tol || (y - ib[3]).abs() <= tol {
                s += ((x + w).min(ib[2]) - x.max(ib[0])).max(0.0);
            }
        }
        s
    }

    /// Score-first branch-and-bound free-rect search; only invokes `can_place` when candidate beats cutoff.
    pub fn find_best_free_rect(
        &self, parts: &[PartSpec], cfg: &SheetConfig, part_idx: usize,
        rule: BinRule, cutoff_s1: f64, cutoff_s2: f64,
    ) -> Option<(usize, f64, f64, f64, f64)> {
        let pspec = &parts[part_idx];
        if !self.may_fit_bbox(pspec.min_bbox_area, pspec.min_dim, pspec.max_dim) { return None; }
        let mut best: Option<(usize, f64, f64, f64, f64)> = None;
        let (mut cur_s1, mut cur_s2) = (cutoff_s1, cutoff_s2);
        let max_extra = if pspec.small_part && pspec.small_part_clearance > cfg.spacing + 0.001 {
            pspec.small_part_clearance - cfg.spacing
        } else { 0.0 };
        let fast_pure_rect = self.all_pure_rect_uniform && max_extra <= 1e-6 && !pspec.has_holes;

        for fr in &self.free_rects {
            let fr_area = fr.w * fr.h;
            if fr_area + 1e-3 < pspec.min_bbox_area { continue; }
            for (v_idx, var) in pspec.variants.iter().enumerate() {
                if var.w > fr.w + 1e-4 || var.h > fr.h + 1e-4 { continue; }
                let mut cx = if cfg.compact_right { fr.x + fr.w - var.w } else { fr.x };
                let mut cy = if cfg.compact_top { fr.y + fr.h - var.h } else { fr.y };
                cx = cx.clamp(cfg.margin, cfg.board_w - cfg.margin - var.w);
                cy = cy.clamp(cfg.margin, cfg.board_h - cfg.margin - var.h);

                let (rx, ry) = ((fr.w - var.w).max(0.0), (fr.h - var.h).max(0.0));
                let (s1, s2) = match rule {
                    BinRule::Bssf => (rx.min(ry), rx.max(ry)),
                    BinRule::Blsf => (rx.max(ry), rx.min(ry)),
                    BinRule::Baf => (fr_area - var.w * var.h, rx.min(ry)),
                    BinRule::BottomLeft => (
                        if cfg.compact_top { cfg.board_h - (cy + var.h) } else { cy },
                        if cfg.compact_right { cfg.board_w - (cx + var.w) } else { cx },
                    ),
                    BinRule::Contact => (-self.contact_score(cfg, cx, cy, var.w, var.h), rx.min(ry)),
                };
                let s1_lb = if cfg.merge_cut && max_extra > 0.001 { s1 - 5.0 } else { s1 };
                if s1_lb > cur_s1 + 1e-6 || ((s1_lb - cur_s1).abs() <= 1e-6 && s2 >= cur_s2 - 1e-6) {
                    continue;
                }

                // Pure-rect fast path: MaxRects invariant guarantees disjointness; verify exact once below!
                if fast_pure_rect && var.is_rect {
                    cur_s1 = s1; cur_s2 = s2;
                    best = Some((v_idx, cx, cy, s1, s2));
                    continue;
                }

                let mut ok = self.can_place(parts, cfg, part_idx, v_idx, cx, cy);
                if !ok && max_extra > 0.001 {
                    let lpad = if fr.x > cfg.margin + 0.05 { max_extra } else { 0.0 };
                    let bpad = if fr.y > cfg.margin + 0.05 { max_extra } else { 0.0 };
                    let rpad = if fr.x + fr.w < cfg.board_w - cfg.margin - 0.05 { max_extra } else { 0.0 };
                    let tpad = if fr.y + fr.h < cfg.board_h - cfg.margin - 0.05 { max_extra } else { 0.0 };
                    if var.w <= fr.w - lpad - rpad + 1e-4 && var.h <= fr.h - bpad - tpad + 1e-4 {
                        let ax = if cfg.compact_right { fr.x + fr.w - rpad - var.w } else { fr.x + lpad };
                        let ay = if cfg.compact_top { fr.y + fr.h - tpad - var.h } else { fr.y + bpad };
                        if self.can_place(parts, cfg, part_idx, v_idx, ax, ay) { cx = ax; cy = ay; ok = true; }
                    }
                    if !ok && cfg.merge_cut && cfg.edge_zone > 0.0 {
                        let ez = cfg.edge_zone + 0.2;
                        let ix = cx.max(ez).min(cfg.board_w - ez - var.w);
                        let iy = cy.max(ez).min(cfg.board_h - ez - var.h);
                        if ix >= fr.x - 1e-4 && ix + var.w <= fr.x + fr.w + 1e-4
                            && iy >= fr.y - 1e-4 && iy + var.h <= fr.y + fr.h + 1e-4
                            && self.can_place(parts, cfg, part_idx, v_idx, ix, iy)
                        { cx = ix; cy = iy; ok = true; }
                    }
                }
                if !ok { continue; }
                let s1_adj = if cfg.merge_cut && max_extra > 0.001 && !cfg.in_edge_zone(cx, cy, var.w, var.h) {
                    s1 - 5.0
                } else { s1 };
                if s1_adj < cur_s1 - 1e-6 || ((s1_adj - cur_s1).abs() <= 1e-6 && s2 < cur_s2 - 1e-6) {
                    cur_s1 = s1_adj; cur_s2 = s2;
                    best = Some((v_idx, cx, cy, s1_adj, s2));
                }
            }
        }
        if fast_pure_rect {
            if let Some((vi, cx, cy, _, _)) = best {
                if !self.can_place(parts, cfg, part_idx, vi, cx, cy) {
                    return None;
                }
            }
        }
        best
    }

    /// Fast 1-probe + 7-step binary search directional polygon slide for low-fill arches/L/U shapes.
    pub fn find_polygon_slide(&self, parts: &[PartSpec], cfg: &SheetConfig, part_idx: usize) -> Option<(usize, f64, f64, f64)> {
        let pspec = &parts[part_idx];
        if !pspec.is_low_fill || self.low_fill_items.is_empty() { return None; }
        let mut anchors = [0usize; 3];
        let mut n_anchors = 0usize;
        let mut first_same = None;
        for &idx_u16 in self.low_fill_items.iter().rev() {
            let idx = idx_u16 as usize;
            if parts[self.items[idx].part_idx as usize].shape_hash == pspec.shape_hash {
                if n_anchors < 2 {
                    anchors[n_anchors] = idx;
                    n_anchors += 1;
                }
                first_same = Some(idx);
            }
        }
        if let Some(fi) = first_same {
            if n_anchors < 3 && !anchors[..n_anchors].contains(&fi) {
                anchors[n_anchors] = fi;
                n_anchors += 1;
            }
        }
        if n_anchors == 0 { anchors[0] = *self.low_fill_items.last().unwrap() as usize; n_anchors = 1; }

        let gap = cfg.spacing.max(pspec.small_part_clearance);
        let (max_x, max_y) = (cfg.board_w - cfg.margin, cfg.board_h - cfg.margin);
        let mut best: Option<(usize, f64, f64, f64)> = None;

        for &a_idx in &anchors[..n_anchors] {
            let host = self.items[a_idx];
            let host_area = host.w * host.h;
            for (v_idx, var) in pspec.variants.iter().enumerate() {
                if var.w > cfg.usable_w + 1e-4 || var.h > cfg.usable_h + 1e-4 { continue; }
                let fy_n = (host.y + 0.05).clamp(cfg.margin, max_y - var.h);
                let fx_n = host.x.clamp(cfg.margin, max_x - var.w);
                let trials = [
                    (true, (host.x + host.w + gap).min(max_x - var.w), host.x.max(cfg.margin), fy_n),
                    (false, (host.y + host.h + gap).min(max_y - var.h), host.y.max(cfg.margin), fx_n),
                ];
                for &(is_x, start_c, deep_c, fixed_c) in &trials {
                    if (start_c - deep_c).abs() < 20.0 { continue; }
                    let probe = start_c + 0.25 * (deep_c - start_c);
                    let (px, py) = if is_x { (probe, fixed_c) } else { (fixed_c, probe) };
                    if !self.can_place(parts, cfg, part_idx, v_idx, px, py) { continue; }
                    let (mut valid, mut target) = (probe, deep_c);
                    for _ in 0..7 {
                        let mid = 0.5 * (valid + target);
                        let (mx, my) = if is_x { (mid, fixed_c) } else { (fixed_c, mid) };
                        if self.can_place(parts, cfg, part_idx, v_idx, mx, my) { valid = mid; } else { target = mid; }
                    }
                    let (fx, fy) = if is_x { (valid, fixed_c) } else { (fixed_c, valid) };
                    let uw = (fx + var.w).max(host.x + host.w) - fx.min(host.x);
                    let uh = (fy + var.h).max(host.y + host.h) - fy.min(host.y);
                    let added_area = (uw * uh - host_area).max(0.0);
                    if added_area < var.w * var.h * 0.78 {
                        let sc = added_area + 0.01 * (fx + fy);
                        if best.is_none_or(|b| sc < b.3) { best = Some((v_idx, fx, fy, sc)); }
                    }
                }
            }
        }
        best
    }

    fn inject_clipped_rect(&self, added: &mut Vec<FreeRect>, raw: BBox, ox: f64, oy: f64, prev_count: usize) {
        let mut fr_list = vec![FreeRect { x: ox + raw.min_x, y: oy + raw.min_y, w: raw.w(), h: raw.h() }];
        for prev_i in 0..prev_count {
            let pb = self.item_boxes[prev_i];
            let pg = self.items[prev_i].req_clr;
            fr_list = clip_rects_against_box(fr_list, pb[0] - pg, pb[1] - pg, pb[2] + pg, pb[3] + pg);
            if fr_list.is_empty() { return; }
        }
        for fr in fr_list { if fr.w >= 12.0 && fr.h >= 12.0 { added.push(fr); } }
    }

    pub fn commit_placement(&mut self, parts: &[PartSpec], cfg: &SheetConfig, part_idx: usize, var_idx: usize, x: f64, y: f64) {
        let pspec = &parts[part_idx];
        let var = &pspec.variants[var_idx];
        let req_clr = cfg.req_clearance_at(pspec.small_part, pspec.small_part_clearance, x, y, var.w, var.h);
        let item_idx = self.items.len();

        self.items.push(PlacedItem { part_idx: part_idx as u32, var_idx: var_idx as u8, x, y, w: var.w, h: var.h, req_clr });
        self.item_boxes.push([x, y, x + var.w, y + var.h]);
        self.used_area += pspec.area;
        if pspec.is_low_fill { self.low_fill_items.push(item_idx as u16); }
        if pspec.two_sided { self.has_two_sided = true; }
        if !var.is_rect || !var.holes.is_empty() || req_clr > cfg.spacing + 1e-6 || var.cover_strips.len() > 1 {
            self.all_pure_rect_uniform = false;
        }

        if let Some(ref mut grid) = self.grid {
            grid.insert(item_idx as u16, x, y, x + var.w, y + var.h);
        } else if self.items.len() > GRID_ACTIVATE_THRESHOLD {
            let mut g = Box::new(SpatialGrid::new(cfg.board_w, cfg.board_h));
            for (i, ib) in self.item_boxes.iter().enumerate() { g.insert(i as u16, ib[0], ib[1], ib[2], ib[3]); }
            self.grid = Some(g);
        }

        let cg = req_clr;
        if var.cover_strips.len() > 1 {
            for s in &var.cover_strips {
                let added = self.subtract_box_from_free_rects(x + s[0] - cg, y + s[1] - cg, x + s[0] + s[2] + cg, y + s[1] + s[3] + cg);
                self.merge_and_prune_added(added);
            }
        } else {
            let mut added = self.subtract_box_from_free_rects(x - cg, y - cg, x + var.w + cg, y + var.h + cg);
            if cfg.sheet_in_sheet {
                for h in &var.holes {
                    if let Some(ir) = h.inscribed_rect { self.inject_clipped_rect(&mut added, ir, x, y, item_idx); }
                }
            }
            self.merge_and_prune_added(added);
        }
    }

    fn subtract_box_from_free_rects(&mut self, bx0: f64, by0: f64, bx1: f64, by1: f64) -> Vec<FreeRect> {
        let mut added = Vec::with_capacity(16);
        let mut i = 0usize;
        while i < self.free_rects.len() {
            let fr = self.free_rects[i];
            let (fx1, fy1) = (fr.x + fr.w, fr.y + fr.h);
            if bx0 >= fx1 - 1e-6 || bx1 <= fr.x + 1e-6 || by0 >= fy1 - 1e-6 || by1 <= fr.y + 1e-6 {
                i += 1;
                continue;
            }
            self.free_rects.swap_remove(i);
            if bx0 > fr.x + 0.5 { added.push(FreeRect { x: fr.x, y: fr.y, w: bx0 - fr.x, h: fr.h }); }
            if bx1 < fx1 - 0.5 { added.push(FreeRect { x: bx1, y: fr.y, w: fx1 - bx1, h: fr.h }); }
            if by0 > fr.y + 0.5 { added.push(FreeRect { x: fr.x, y: fr.y, w: fr.w, h: by0 - fr.y }); }
            if by1 < fy1 - 0.5 { added.push(FreeRect { x: fr.x, y: by1, w: fr.w, h: fy1 - by1 }); }
        }
        added
    }

    fn merge_and_prune_added(&mut self, mut added: Vec<FreeRect>) {
        let mut i = 0usize;
        while i < added.len() {
            let a = added[i];
            let mut drop_a = false;
            let mut j = i + 1;
            while j < added.len() {
                if rect_contains(added[j], a) { drop_a = true; break; }
                if rect_contains(a, added[j]) { added.swap_remove(j); continue; }
                j += 1;
            }
            if drop_a { added.swap_remove(i); } else { i += 1; }
        }
        let mut ai = 0usize;
        while ai < added.len() {
            let a = added[ai];
            let mut drop_a = false;
            let mut fi = 0usize;
            while fi < self.free_rects.len() {
                if rect_contains(self.free_rects[fi], a) { drop_a = true; break; }
                if rect_contains(a, self.free_rects[fi]) { self.free_rects.swap_remove(fi); continue; }
                fi += 1;
            }
            if drop_a { added.swap_remove(ai); } else { ai += 1; }
        }
        self.free_rects.extend(added);
        if self.free_rects.len() > 64 {
            self.free_rects.sort_by(|a, b| (b.w * b.h).partial_cmp(&(a.w * a.h)).unwrap_or(std::cmp::Ordering::Equal));
            self.free_rects.truncate(64);
        }
        let (mut ma, mut ml, mut ms) = (0.0f64, 0.0f64, 0.0f64);
        for fr in &self.free_rects {
            ma = ma.max(fr.w * fr.h);
            ml = ml.max(fr.w.max(fr.h));
            ms = ms.max(fr.w.min(fr.h));
        }
        self.max_free_area = ma; self.max_free_long = ml; self.max_free_short = ms;
    }
}

#[inline]
fn rect_contains(o: FreeRect, i: FreeRect) -> bool {
    o.x <= i.x + 1e-4 && o.y <= i.y + 1e-4 && o.x + o.w >= i.x + i.w - 1e-4 && o.y + o.h >= i.y + i.h - 1e-4
}

fn clip_rects_against_box(rects: Vec<FreeRect>, bx0: f64, by0: f64, bx1: f64, by1: f64) -> Vec<FreeRect> {
    let mut out = Vec::with_capacity(rects.len());
    for fr in rects {
        let (fx1, fy1) = (fr.x + fr.w, fr.y + fr.h);
        if bx0 >= fx1 - 1e-6 || bx1 <= fr.x + 1e-6 || by0 >= fy1 - 1e-6 || by1 <= fr.y + 1e-6 {
            out.push(fr);
            continue;
        }
        if bx0 > fr.x + 1.0 { out.push(FreeRect { x: fr.x, y: fr.y, w: bx0 - fr.x, h: fr.h }); }
        if bx1 < fx1 - 1.0 { out.push(FreeRect { x: bx1, y: fr.y, w: fx1 - bx1, h: fr.h }); }
        if by0 > fr.y + 1.0 { out.push(FreeRect { x: fr.x, y: fr.y, w: fr.w, h: by0 - fr.y }); }
        if by1 < fy1 - 1.0 { out.push(FreeRect { x: fr.x, y: by1, w: fr.w, h: fy1 - by1 }); }
    }
    out
}

pub fn compute_shape_hash(w: f64, h: f64, area: f64, n_verts: usize) -> u64 {
    let a = (w.min(h) * 2.0).round() as u64;
    let b = (w.max(h) * 2.0).round() as u64;
    let c = (area * 0.1).round() as u64;
    a.wrapping_mul(1_000_003) ^ b.wrapping_mul(9_999_991) ^ c.wrapping_mul(31_337) ^ (n_verts as u64)
}
