// src/geom.rs - Exact 2D polygon geometry, chunked edge-AABB acceleration,
// concavity free-rect extraction, and hole containment verification.
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, Default)]
pub struct Pt { pub x: f64, pub y: f64 }

#[derive(Debug, Clone, Copy)]
pub struct BBox { pub min_x: f64, pub min_y: f64, pub max_x: f64, pub max_y: f64 }

impl BBox {
    #[inline]
    pub fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Self {
        Self { min_x, min_y, max_x, max_y }
    }
    #[inline]
    pub fn from_pts(pts: &[Pt]) -> Self {
        let (mut min_x, mut min_y, mut max_x, mut max_y) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for p in pts {
            if p.x < min_x { min_x = p.x; }
            if p.y < min_y { min_y = p.y; }
            if p.x > max_x { max_x = p.x; }
            if p.y > max_y { max_y = p.y; }
        }
        if min_x > max_x { Self::new(0.0, 0.0, 0.0, 0.0) } else { Self::new(min_x, min_y, max_x, max_y) }
    }
    #[inline] pub fn w(&self) -> f64 { (self.max_x - self.min_x).max(0.0) }
    #[inline] pub fn h(&self) -> f64 { (self.max_y - self.min_y).max(0.0) }
}

#[derive(Debug, Clone, Copy)]
pub struct EdgeChunk { pub start: u16, pub end: u16, pub bbox: BBox }

#[derive(Debug, Clone)]
pub struct OrientedHole {
    pub pts: Vec<Pt>,
    pub bbox: BBox,
    pub chunks: Vec<EdgeChunk>,
    pub inscribed_rect: Option<BBox>,
}

#[derive(Debug, Clone)]
pub struct OrientedPoly {
    pub rot_deg: f64,
    pub w: f64,
    pub h: f64,
    pub area: f64,
    pub fill_ratio: f64,
    pub is_rect: bool,
    pub shift_x: f64,
    pub shift_y: f64,
    pub pts: Vec<Pt>,
    pub chunks: Vec<EdgeChunk>,
    pub centroid: Pt,
    pub centroid_inside: bool,
    pub holes: Vec<OrientedHole>,
    pub cover_strips: Vec<[f64; 4]>,
}

#[inline]
pub fn norm_deg(deg: f64) -> f64 {
    let mut a = deg % 360.0;
    if a < 0.0 { a += 360.0; }
    if (a - 360.0).abs() < 0.05 { a = 0.0; }
    (a * 10.0).round() / 10.0
}

#[inline]
pub fn angle_close(a: f64, b: f64, tol: f64) -> bool {
    let da = (norm_deg(a) - norm_deg(b)).abs();
    da <= tol || (360.0 - da).abs() <= tol
}

#[inline]
pub fn rotate_pt(p: Pt, deg: f64) -> Pt {
    if deg.abs() < 1e-6 { return p; }
    let r = deg * PI / 180.0;
    let (s, c) = r.sin_cos();
    Pt { x: p.x * c - p.y * s, y: p.x * s + p.y * c }
}

pub fn poly_signed_area(pts: &[Pt]) -> f64 {
    let n = pts.len();
    if n < 3 { return 0.0; }
    let mut a = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        a += pts[i].x * pts[j].y - pts[j].x * pts[i].y;
    }
    a * 0.5
}

pub fn poly_centroid(pts: &[Pt]) -> Pt {
    let n = pts.len();
    if n == 0 { return Pt::default(); }
    let a = poly_signed_area(pts);
    if a.abs() < 1e-8 {
        let (mut sx, mut sy) = (0.0, 0.0);
        for p in pts { sx += p.x; sy += p.y; }
        return Pt { x: sx / n as f64, y: sy / n as f64 };
    }
    let (mut cx, mut cy) = (0.0, 0.0);
    for i in 0..n {
        let j = (i + 1) % n;
        let f = pts[i].x * pts[j].y - pts[j].x * pts[i].y;
        cx += (pts[i].x + pts[j].x) * f;
        cy += (pts[i].y + pts[j].y) * f;
    }
    Pt { x: cx / (6.0 * a), y: cy / (6.0 * a) }
}

pub fn build_chunks(pts: &[Pt]) -> Vec<EdgeChunk> {
    const CHUNK_SIZE: usize = 8;
    let n = pts.len();
    if n < 2 { return Vec::new(); }
    let mut chunks = Vec::with_capacity(n.div_ceil(CHUNK_SIZE));
    let mut start = 0usize;
    while start < n {
        let end = (start + CHUNK_SIZE).min(n);
        let mut b = BBox::new(f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for i in start..end {
            let (p1, p2) = (pts[i], pts[(i + 1) % n]);
            b.min_x = b.min_x.min(p1.x.min(p2.x));
            b.max_x = b.max_x.max(p1.x.max(p2.x));
            b.min_y = b.min_y.min(p1.y.min(p2.y));
            b.max_y = b.max_y.max(p1.y.max(p2.y));
        }
        chunks.push(EdgeChunk { start: start as u16, end: end as u16, bbox: b });
        start = end;
    }
    chunks
}

fn check_exact_rect(pts: &[Pt], w: f64, h: f64, area: f64, has_holes: bool) -> bool {
    if has_holes || w <= 1e-4 || h <= 1e-4 { return false; }
    let bbox_area = w * h;
    if (area - bbox_area).abs() > (bbox_area * 1e-4).max(0.05) { return false; }
    pts.iter().all(|p| {
        p.x.abs() <= 0.02 || (p.x - w).abs() <= 0.02 || p.y.abs() <= 0.02 || (p.y - h).abs() <= 0.02
    })
}

#[inline]
pub fn ccw(a: Pt, b: Pt, c: Pt) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

#[inline]
pub fn segs_intersect_proper(p1: Pt, p2: Pt, q1: Pt, q2: Pt) -> bool {
    let (d1, d2, d3, d4) = (ccw(q1, q2, p1), ccw(q1, q2, p2), ccw(p1, p2, q1), ccw(p1, p2, q2));
    ((d1 > 1e-7 && d2 < -1e-7) || (d1 < -1e-7 && d2 > 1e-7))
        && ((d3 > 1e-7 && d4 < -1e-7) || (d3 < -1e-7 && d4 > 1e-7))
}

#[inline]
pub fn point_to_seg_dist_sq(p: Pt, a: Pt, b: Pt) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len_sq = dx * dx + dy * dy;
    if len_sq <= 1e-12 {
        let (ex, ey) = (p.x - a.x, p.y - a.y);
        return ex * ex + ey * ey;
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq).clamp(0.0, 1.0);
    let (ex, ey) = (p.x - (a.x + t * dx), p.y - (a.y + t * dy));
    ex * ex + ey * ey
}

#[inline]
pub fn seg_to_seg_dist_sq(p1: Pt, p2: Pt, q1: Pt, q2: Pt) -> f64 {
    if segs_intersect_proper(p1, p2, q1, q2) { return 0.0; }
    point_to_seg_dist_sq(p1, q1, q2)
        .min(point_to_seg_dist_sq(p2, q1, q2))
        .min(point_to_seg_dist_sq(q1, p1, p2))
        .min(point_to_seg_dist_sq(q2, p1, p2))
}

pub fn point_in_poly_local(pt: Pt, poly: &[Pt], chunks: &[EdgeChunk], min_edge_dist: f64) -> bool {
    let n = poly.len();
    if n < 3 { return false; }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (pi, pj) = (poly[i], poly[j]);
        let dy = pj.y - pi.y;
        if dy.abs() > 1e-10 && ((pi.y > pt.y) != (pj.y > pt.y))
            && pt.x < (pj.x - pi.x) * (pt.y - pi.y) / dy + pi.x
        {
            inside = !inside;
        }
        j = i;
    }
    if !inside { return false; }
    if min_edge_dist > 0.0 {
        let d_sq = min_edge_dist * min_edge_dist;
        for ch in chunks {
            if pt.x < ch.bbox.min_x - min_edge_dist || pt.x > ch.bbox.max_x + min_edge_dist
                || pt.y < ch.bbox.min_y - min_edge_dist || pt.y > ch.bbox.max_y + min_edge_dist
            { continue; }
            for i in (ch.start as usize)..(ch.end as usize) {
                if point_to_seg_dist_sq(pt, poly[i], poly[(i + 1) % n]) < d_sq { return false; }
            }
        }
    }
    true
}

#[inline]
fn any_vertex_or_centroid_inside(src: &OrientedPoly, dst: &OrientedPoly, dx: f64, dy: f64, tol: f64, check_midpoints: bool) -> bool {
    let n = src.pts.len();
    for i in 0..n {
        let p = src.pts[i];
        let q = Pt { x: p.x + dx, y: p.y + dy };
        if q.x <= dst.w + 0.01 && q.y >= -0.01 && q.y <= dst.h + 0.01
            && point_in_poly_local(q, &dst.pts, &dst.chunks, tol)
        { return true; }
        if check_midpoints {
            let p2 = src.pts[(i + 1) % n];
            let m = Pt { x: 0.5 * (p.x + p2.x) + dx, y: 0.5 * (p.y + p2.y) + dy };
            if m.x <= dst.w + 0.01 && m.y >= -0.01 && m.y <= dst.h + 0.01
                && point_in_poly_local(m, &dst.pts, &dst.chunks, tol)
            { return true; }
        }
    }
    if src.centroid_inside {
        let c = Pt { x: src.centroid.x + dx, y: src.centroid.y + dy };
        if c.x <= dst.w + 0.01 && c.y >= -0.01 && c.y <= dst.h + 0.01
            && point_in_poly_local(c, &dst.pts, &dst.chunks, tol)
        { return true; }
    }
    false
}

/// Exact zero-allocation conflict test between two oriented polygons at (ax, ay) and (bx, by).
#[inline]
pub fn polys_conflict_exact(
    a: &OrientedPoly, ax: f64, ay: f64,
    b: &OrientedPoly, bx: f64, by: f64,
    req_gap: f64,
) -> bool {
    let eff_gap = if req_gap > 1e-6 { (req_gap - 1e-4).max(0.0) } else { 0.0 };
    let overlap_eps = if eff_gap == 0.0 { 1e-4 } else { 0.0 };
    if ax + a.w + eff_gap <= bx + overlap_eps || bx + b.w + eff_gap <= ax + overlap_eps
        || ay + a.h + eff_gap <= by + overlap_eps || by + b.h + eff_gap <= ay + overlap_eps
    { return false; }
    if a.is_rect && b.is_rect { return true; }

    let (dx, dy, eff_sq) = (ax - bx, ay - by, eff_gap * eff_gap);
    let (na, nb) = (a.pts.len(), b.pts.len());
    for ca in &a.chunks {
        let (ax0, ax1) = (ca.bbox.min_x + dx, ca.bbox.max_x + dx);
        let (ay0, ay1) = (ca.bbox.min_y + dy, ca.bbox.max_y + dy);
        if ax1 + eff_gap < 0.0 || ax0 - eff_gap > b.w || ay1 + eff_gap < 0.0 || ay0 - eff_gap > b.h {
            continue;
        }
        for cb in &b.chunks {
            if ax1 + eff_gap < cb.bbox.min_x || ax0 - eff_gap > cb.bbox.max_x
                || ay1 + eff_gap < cb.bbox.min_y || ay0 - eff_gap > cb.bbox.max_y
            { continue; }
            for i in (ca.start as usize)..(ca.end as usize) {
                let p1 = Pt { x: a.pts[i].x + dx, y: a.pts[i].y + dy };
                let p2 = Pt { x: a.pts[(i + 1) % na].x + dx, y: a.pts[(i + 1) % na].y + dy };
                if p1.x.max(p2.x) + eff_gap < cb.bbox.min_x || p1.x.min(p2.x) - eff_gap > cb.bbox.max_x
                    || p1.y.max(p2.y) + eff_gap < cb.bbox.min_y || p1.y.min(p2.y) - eff_gap > cb.bbox.max_y
                { continue; }
                for j in (cb.start as usize)..(cb.end as usize) {
                    let (q1, q2) = (b.pts[j], b.pts[(j + 1) % nb]);
                    if eff_gap > 0.0 {
                        if seg_to_seg_dist_sq(p1, p2, q1, q2) < eff_sq { return true; }
                    } else if segs_intersect_proper(p1, p2, q1, q2) {
                        return true;
                    }
                }
            }
        }
    }
    let pt_tol = if eff_gap > 0.0 { 0.14 } else { 0.05 };
    let mid = eff_gap == 0.0;
    any_vertex_or_centroid_inside(a, b, dx, dy, pt_tol, mid)
        || any_vertex_or_centroid_inside(b, a, -dx, -dy, pt_tol, mid)
}

/// Exact check: is `inner` at `(ix, iy)` strictly inside `hole` of host at `(hx, hy)` with `req_gap`?
pub fn poly_contained_in_hole_exact(
    inner: &OrientedPoly, ix: f64, iy: f64,
    hole: &OrientedHole, hx: f64, hy: f64,
    req_gap: f64,
) -> bool {
    let eff_gap = req_gap + 0.04;
    let (dx, dy) = (ix - hx, iy - hy);
    if dx < hole.bbox.min_x + eff_gap || dx + inner.w > hole.bbox.max_x - eff_gap
        || dy < hole.bbox.min_y + eff_gap || dy + inner.h > hole.bbox.max_y - eff_gap
    { return false; }
    for &p in &inner.pts {
        if !point_in_poly_local(Pt { x: p.x + dx, y: p.y + dy }, &hole.pts, &hole.chunks, eff_gap) {
            return false;
        }
    }
    let eff_sq = eff_gap * eff_gap;
    let (ni, nh) = (inner.pts.len(), hole.pts.len());
    for ci in &inner.chunks {
        let (ix0, ix1) = (ci.bbox.min_x + dx - eff_gap, ci.bbox.max_x + dx + eff_gap);
        let (iy0, iy1) = (ci.bbox.min_y + dy - eff_gap, ci.bbox.max_y + dy + eff_gap);
        for ch in &hole.chunks {
            if ix1 < ch.bbox.min_x || ix0 > ch.bbox.max_x || iy1 < ch.bbox.min_y || iy0 > ch.bbox.max_y {
                continue;
            }
            for i in (ci.start as usize)..(ci.end as usize) {
                let p1 = Pt { x: inner.pts[i].x + dx, y: inner.pts[i].y + dy };
                let p2 = Pt { x: inner.pts[(i + 1) % ni].x + dx, y: inner.pts[(i + 1) % ni].y + dy };
                for j in (ch.start as usize)..(ch.end as usize) {
                    if seg_to_seg_dist_sq(p1, p2, hole.pts[j], hole.pts[(j + 1) % nh]) < eff_sq {
                        return false;
                    }
                }
            }
        }
    }
    true
}

fn largest_grid_rect<const G: usize>(grid: &[[bool; G]; G], cw: f64, ch: f64) -> Option<(usize, usize, usize, usize, f64)> {
    let mut heights = [0usize; G];
    let mut best: Option<(usize, usize, usize, usize, f64)> = None;
    for (gy, row) in grid.iter().enumerate() {
        for gx in 0..G { heights[gx] = if row[gx] { heights[gx] + 1 } else { 0 }; }
        for x1 in 0..G {
            let mut mh = heights[x1];
            if mh == 0 { continue; }
            for (x2, &hx2) in heights.iter().enumerate().skip(x1) {
                if hx2 < mh { mh = hx2; }
                if mh == 0 { break; }
                let a = (x2 - x1 + 1) as f64 * cw * (mh as f64 * ch);
                if best.is_none_or(|b| a > b.4) {
                    best = Some((x1, gy + 1 - mh, x2 + 1, gy + 1, a));
                }
            }
        }
    }
    best
}

fn scanline_x_intervals(pts: &[Pt], y: f64) -> Vec<(f64, f64)> {
    let n = pts.len();
    let mut xs = Vec::new();
    let mut j = n - 1;
    for i in 0..n {
        let (pi, pj) = (pts[i], pts[j]);
        let dy = pj.y - pi.y;
        if dy.abs() > 1e-10 && ((pi.y > y) != (pj.y > y)) {
            xs.push((pj.x - pi.x) * (y - pi.y) / dy + pi.x);
        }
        j = i;
    }
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    xs.as_chunks::<2>().0.iter().filter(|p| p[1] > p[0] + 1e-4).map(|p| (p[0], p[1])).collect()
}

/// Decompose a concave polygon (without holes) into coalesced horizontal cover strips
/// so MaxRects preserves continuous concave space directly.
fn compute_cover_strips(pts: &[Pt], w: f64, h: f64, bands: usize) -> Vec<[f64; 4]> {
    if pts.len() < 3 || w <= 1e-4 || h <= 1e-4 { return vec![[0.0, 0.0, w, h]]; }
    let dy = h / bands as f64;
    let n = pts.len();
    let mut strips: Vec<[f64; 4]> = Vec::new();
    for b in 0..bands {
        let y0 = b as f64 * dy;
        let y1 = if b + 1 == bands { h } else { (b + 1) as f64 * dy };
        let mut ivs: Vec<(f64, f64)> = Vec::new();
        for frac in [0.05, 0.5, 0.95] { ivs.extend(scanline_x_intervals(pts, y0 + frac * (y1 - y0))); }
        for i in 0..n {
            let (p1, p2) = (pts[i], pts[(i + 1) % n]);
            if p1.y.max(p2.y) < y0 - 1e-6 || p1.y.min(p2.y) > y1 + 1e-6 { continue; }
            if (p2.y - p1.y).abs() <= 1e-9 {
                ivs.push((p1.x.min(p2.x), p1.x.max(p2.x)));
            } else {
                let t0 = ((y0 - p1.y) / (p2.y - p1.y)).clamp(0.0, 1.0);
                let t1 = ((y1 - p1.y) / (p2.y - p1.y)).clamp(0.0, 1.0);
                let (xa, xb) = (p1.x + t0 * (p2.x - p1.x), p1.x + t1 * (p2.x - p1.x));
                ivs.push((xa.min(xb), xa.max(xb)));
            }
        }
        if ivs.is_empty() { continue; }
        ivs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let mut merged: Vec<(f64, f64)> = Vec::new();
        for (a, b_x) in ivs {
            if let Some(last) = merged.last_mut() {
                if a <= last.1 + 6.0 { last.1 = last.1.max(b_x); continue; }
            }
            merged.push((a, b_x));
        }
        for (x0, x1) in merged {
            if x1 > x0 + 0.1 { strips.push([x0, y0, x1 - x0, y1 - y0]); }
        }
    }
    if strips.is_empty() { return vec![[0.0, 0.0, w, h]]; }
    let mut out: Vec<[f64; 4]> = Vec::with_capacity(strips.len());
    for s in strips {
        if let Some(prev) = out.iter_mut().rev().find(|p| {
            ((p[1] + p[3]) - s[1]).abs() < 1e-3 && (p[0] - s[0]).abs() < 4.0 && ((p[0] + p[2]) - (s[0] + s[2])).abs() < 4.0
        }) {
            let nx0 = prev[0].min(s[0]);
            let nx1 = (prev[0] + prev[2]).max(s[0] + s[2]);
            prev[0] = nx0; prev[2] = nx1 - nx0; prev[3] += s[3];
        } else {
            out.push(s);
        }
    }
    out
}

fn extract_hole_inscribed_rect(hole_pts: &[Pt], chunks: &[EdgeChunk], hb: &BBox, spacing: f64) -> Option<BBox> {
    let eff = spacing + 0.1;
    let (rw, rh) = (hb.w() - 2.0 * eff, hb.h() - 2.0 * eff);
    if rw < 15.0 || rh < 15.0 { return None; }
    let cand = BBox::new(hb.min_x + eff, hb.min_y + eff, hb.max_x - eff, hb.max_y - eff);
    let rpts = vec![Pt { x: 0.0, y: 0.0 }, Pt { x: rw, y: 0.0 }, Pt { x: rw, y: rh }, Pt { x: 0.0, y: rh }];
    let rpoly = OrientedPoly {
        rot_deg: 0.0, w: rw, h: rh, area: rw * rh, fill_ratio: 1.0, is_rect: true,
        shift_x: 0.0, shift_y: 0.0, chunks: build_chunks(&rpts), pts: rpts,
        centroid: Pt { x: rw * 0.5, y: rh * 0.5 }, centroid_inside: true,
        holes: Vec::new(), cover_strips: Vec::new(),
    };
    let tmp_hole = OrientedHole { pts: hole_pts.to_vec(), bbox: *hb, chunks: chunks.to_vec(), inscribed_rect: None };
    if poly_contained_in_hole_exact(&rpoly, cand.min_x, cand.min_y, &tmp_hole, 0.0, 0.0, spacing) {
        return Some(cand);
    }
    const G: usize = 12;
    let (cw, ch) = (hb.w() / G as f64, hb.h() / G as f64);
    let mut grid = [[false; G]; G];
    for (gy, grid_row) in grid.iter_mut().enumerate() {
        let (y0, y1) = (hb.min_y + gy as f64 * ch, hb.min_y + (gy + 1) as f64 * ch);
        for (gx, cell) in grid_row.iter_mut().enumerate() {
            let (x0, x1) = (hb.min_x + gx as f64 * cw, hb.min_x + (gx + 1) as f64 * cw);
            *cell = [Pt { x: x0, y: y0 }, Pt { x: x1, y: y0 }, Pt { x: x1, y: y1 }, Pt { x: x0, y: y1 }]
                .iter().all(|&c| point_in_poly_local(c, hole_pts, chunks, eff));
        }
    }
    largest_grid_rect(&grid, cw, ch).map(|(x1, y1, x2, y2, _)| {
        BBox::new(hb.min_x + x1 as f64 * cw, hb.min_y + y1 as f64 * ch, hb.min_x + x2 as f64 * cw, hb.min_y + y2 as f64 * ch)
    })
}

pub fn build_oriented_poly(
    raw_contour: &[[f64; 2]], raw_holes: Option<&Vec<Vec<[f64; 2]>>>,
    rot_deg: f64, spacing: f64, sheet_in_sheet: bool, area_override: Option<f64>,
) -> OrientedPoly {
    let rot_deg = norm_deg(rot_deg);
    let mut rot_pts: Vec<Pt> = raw_contour.iter().map(|c| rotate_pt(Pt { x: c[0], y: c[1] }, rot_deg)).collect();
    let b = BBox::from_pts(&rot_pts);
    let (shift_x, shift_y) = (-b.min_x, -b.min_y);
    for p in &mut rot_pts { p.x += shift_x; p.y += shift_y; }
    let (w, h) = (b.w(), b.h());
    let poly_area = poly_signed_area(&rot_pts).abs();
    let area = area_override.filter(|v| *v > 1e-4).unwrap_or(poly_area);
    let fill_ratio = (poly_area / (w * h).max(1e-6)).clamp(0.0, 1.0);

    let mut holes_out = Vec::new();
    if let Some(hs) = raw_holes {
        for h_raw in hs {
            if h_raw.len() < 3 { continue; }
            let hpts: Vec<Pt> = h_raw.iter().map(|c| {
                let rp = rotate_pt(Pt { x: c[0], y: c[1] }, rot_deg);
                Pt { x: rp.x + shift_x, y: rp.y + shift_y }
            }).collect();
            let hb = BBox::from_pts(&hpts);
            let hchunks = build_chunks(&hpts);
            let inscribed_rect = if sheet_in_sheet { extract_hole_inscribed_rect(&hpts, &hchunks, &hb, spacing) } else { None };
            holes_out.push(OrientedHole { pts: hpts, bbox: hb, chunks: hchunks, inscribed_rect });
        }
    }
    let is_rect = check_exact_rect(&rot_pts, w, h, poly_area, !holes_out.is_empty());
    let chunks = build_chunks(&rot_pts);
    let centroid = poly_centroid(&rot_pts);
    let centroid_inside = is_rect || point_in_poly_local(centroid, &rot_pts, &chunks, 0.0);
    let cover_strips = if !is_rect && holes_out.is_empty() && fill_ratio > 0.20 && fill_ratio < 0.76 && (w * h > 30_000.0) {
        let cs = compute_cover_strips(&rot_pts, w, h, 6);
        if cs.len() <= 5 { cs } else { vec![[0.0, 0.0, w, h]] }
    } else {
        vec![[0.0, 0.0, w, h]]
    };
    OrientedPoly { rot_deg, w, h, area, fill_ratio, is_rect, shift_x, shift_y, pts: rot_pts, chunks, centroid, centroid_inside, holes: holes_out, cover_strips }
}
