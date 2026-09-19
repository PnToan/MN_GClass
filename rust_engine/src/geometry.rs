// src/geometry.rs
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    #[inline]
    pub fn new(x: f64, y: f64) -> Self {
        Point { x, y }
    }

    #[inline]
    pub fn rotate(&self, angle_rad: f64, center: Point) -> Point {
        let cos = angle_rad.cos();
        let sin = angle_rad.sin();
        let dx = self.x - center.x;
        let dy = self.y - center.y;
        Point {
            x: center.x + dx * cos - dy * sin,
            y: center.y + dx * sin + dy * cos,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

impl Rect {
    #[inline]
    pub fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Self {
        Rect { min_x, min_y, max_x, max_y }
    }

    pub fn from_points(points: &[Point]) -> Self {
        if points.is_empty() {
            return Rect::new(0.0, 0.0, 0.0, 0.0);
        }
        let mut min_x = points[0].x;
        let mut max_x = points[0].x;
        let mut min_y = points[0].y;
        let mut max_y = points[0].y;
        for p in &points[1..] {
            if p.x < min_x { min_x = p.x; }
            if p.x > max_x { max_x = p.x; }
            if p.y < min_y { min_y = p.y; }
            if p.y > max_y { max_y = p.y; }
        }
        Rect { min_x, min_y, max_x, max_y }
    }

    #[inline]
    pub fn width(&self) -> f64 {
        (self.max_x - self.min_x).max(0.0)
    }

    #[inline]
    pub fn height(&self) -> f64 {
        (self.max_y - self.min_y).max(0.0)
    }

    #[inline]
    pub fn area(&self) -> f64 {
        self.width() * self.height()
    }

    #[inline]
    pub fn intersects(&self, other: &Rect, tolerance: f64) -> bool {
        self.min_x + tolerance < other.max_x
            && self.max_x - tolerance > other.min_x
            && self.min_y + tolerance < other.max_y
            && self.max_y - tolerance > other.min_y
    }

    #[inline]
    pub fn contains_rect(&self, other: &Rect, tolerance: f64) -> bool {
        other.min_x >= self.min_x - tolerance
            && other.max_x <= self.max_x + tolerance
            && other.min_y >= self.min_y - tolerance
            && other.max_y <= self.max_y + tolerance
    }

    #[inline]
    pub fn translate(&self, dx: f64, dy: f64) -> Rect {
        Rect {
            min_x: self.min_x + dx,
            min_y: self.min_y + dy,
            max_x: self.max_x + dx,
            max_y: self.max_y + dy,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Polygon {
    pub points: Vec<Point>,
}

impl Polygon {
    pub fn new(points: Vec<Point>) -> Self {
        Polygon { points }
    }

    pub fn from_raw(raw: &[[f64; 2]]) -> Self {
        let pts: Vec<Point> = raw.iter().map(|p| Point::new(p[0], p[1])).collect();
        Polygon { points: pts }
    }

    pub fn to_raw(&self) -> Vec<[f64; 2]> {
        self.points.iter().map(|p| [p.x, p.y]).collect()
    }

    pub fn bounding_box(&self) -> Rect {
        Rect::from_points(&self.points)
    }

    pub fn signed_area(&self) -> f64 {
        let n = self.points.len();
        if n < 3 {
            return 0.0;
        }
        let mut area = 0.0;
        for i in 0..n {
            let j = (i + 1) % n;
            area += self.points[i].x * self.points[j].y;
            area -= self.points[j].x * self.points[i].y;
        }
        area * 0.5
    }

    #[inline]
    pub fn area(&self) -> f64 {
        self.signed_area().abs()
    }

    pub fn perimeter(&self) -> f64 {
        let n = self.points.len();
        if n < 2 { return 0.0; }
        let mut p = 0.0;
        for i in 0..n {
            let p1 = self.points[i];
            let p2 = self.points[(i + 1) % n];
            p += ((p2.x - p1.x).powi(2) + (p2.y - p1.y).powi(2)).sqrt();
        }
        p
    }

    pub fn is_rectangular(&self) -> bool {
        let clean = if self.points.len() > 4 {
            simplify_collinear(&self.points, 0.5)
        } else {
            self.points.clone()
        };
        if clean.len() != 4 {
            return false;
        }
        let clean_poly = Polygon::new(clean);
        let bbox = clean_poly.bounding_box();
        let bbox_area = bbox.area();
        let poly_area = clean_poly.area();
        if bbox_area <= 0.0 {
            return false;
        }
        (bbox_area - poly_area).abs() / bbox_area < 0.01
    }

    pub fn is_circular(&self) -> bool {
        let n = self.points.len();
        if n < 8 { return false; }
        let bbox = self.bounding_box();
        let w = bbox.width();
        let h = bbox.height();
        if w <= 0.0 || (w - h).abs() / w.max(h) > 0.08 {
            return false;
        }
        let perim = self.perimeter();
        if perim <= 0.0 { return false; }
        let circularity = (4.0 * PI * self.area()) / (perim * perim);
        circularity >= 0.82
    }

    pub fn circle_radius(&self) -> f64 {
        let bbox = self.bounding_box();
        (bbox.width() + bbox.height()) * 0.25
    }

    pub fn centroid(&self) -> Point {
        let n = self.points.len();
        if n == 0 {
            return Point::new(0.0, 0.0);
        }
        let mut cx = 0.0;
        let mut cy = 0.0;
        let area = self.signed_area();
        if area.abs() < 1e-9 {
            for p in &self.points {
                cx += p.x;
                cy += p.y;
            }
            return Point::new(cx / n as f64, cy / n as f64);
        }
        for i in 0..n {
            let j = (i + 1) % n;
            let factor = self.points[i].x * self.points[j].y - self.points[j].x * self.points[i].y;
            cx += (self.points[i].x + self.points[j].x) * factor;
            cy += (self.points[i].y + self.points[j].y) * factor;
        }
        Point::new(cx / (6.0 * area), cy / (6.0 * area))
    }

    pub fn translate(&self, dx: f64, dy: f64) -> Polygon {
        Polygon {
            points: self.points.iter().map(|p| Point::new(p.x + dx, p.y + dy)).collect(),
        }
    }

    pub fn rotate_degrees(&self, degrees: f64, center: Point) -> Polygon {
        if degrees.abs() < 1e-6 {
            return self.clone();
        }
        let rad = degrees * PI / 180.0;
        Polygon {
            points: self.points.iter().map(|p| p.rotate(rad, center)).collect(),
        }
    }

    pub fn normalize_to_origin(&self) -> (Polygon, f64, f64) {
        let bbox = self.bounding_box();
        let poly = self.translate(-bbox.min_x, -bbox.min_y);
        (poly, -bbox.min_x, -bbox.min_y)
    }

    pub fn contains_point_strict(&self, p: Point, margin: f64) -> bool {
        let n = self.points.len();
        if n < 3 {
            return false;
        }
        let bbox = self.bounding_box();
        if p.x < bbox.min_x - 1e-4
            || p.x > bbox.max_x + 1e-4
            || p.y < bbox.min_y - 1e-4
            || p.y > bbox.max_y + 1e-4
        {
            return false;
        }
        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let pi = self.points[i];
            let pj = self.points[j];
            let dy = pj.y - pi.y;
            if dy.abs() > 1e-9 && ((pi.y > p.y) != (pj.y > p.y)) {
                let intersect_x = (pj.x - pi.x) * (p.y - pi.y) / dy + pi.x;
                if p.x < intersect_x {
                    inside = !inside;
                }
            }
            j = i;
        }
        if !inside {
            return false;
        }
        if margin > 1e-6 {
            for i in 0..n {
                let p1 = self.points[i];
                let p2 = self.points[(i + 1) % n];
                let d = point_to_segment_distance(p, p1, p2);
                if d < margin {
                    return false;
                }
            }
        }
        true
    }
}

#[inline]
pub fn point_to_segment_distance(p: Point, a: Point, b: Point) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let l2 = dx * dx + dy * dy;
    if l2 < 1e-12 {
        return ((p.x - a.x).powi(2) + (p.y - a.y).powi(2)).sqrt();
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / l2).clamp(0.0, 1.0);
    let proj_x = a.x + t * dx;
    let proj_y = a.y + t * dy;
    ((p.x - proj_x).powi(2) + (p.y - proj_y).powi(2)).sqrt()
}

#[inline]
pub fn segments_intersect_proper(p1: Point, p2: Point, p3: Point, p4: Point) -> bool {
    let d1 = ccw(p3, p4, p1);
    let d2 = ccw(p3, p4, p2);
    let d3 = ccw(p1, p2, p3);
    let d4 = ccw(p1, p2, p4);

    ((d1 > 1e-4 && d2 < -1e-4) || (d1 < -1e-4 && d2 > 1e-4))
        && ((d3 > 1e-4 && d4 < -1e-4) || (d3 < -1e-4 && d4 > 1e-4))
}

#[inline]
fn ccw(a: Point, b: Point, c: Point) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

#[inline]
pub fn segment_to_segment_distance(p1: Point, p2: Point, q1: Point, q2: Point) -> f64 {
    if segments_intersect_proper(p1, p2, q1, q2) {
        return 0.0;
    }
    let d1 = point_to_segment_distance(p1, q1, q2);
    let d2 = point_to_segment_distance(p2, q1, q2);
    let d3 = point_to_segment_distance(q1, p1, p2);
    let d4 = point_to_segment_distance(q2, p1, p2);
    d1.min(d2).min(d3).min(d4)
}

pub fn polygons_collide_with_spacing(poly1: &Polygon, poly2: &Polygon, spacing: f64) -> bool {
    let bbox1 = poly1.bounding_box();
    let bbox2 = poly2.bounding_box();

    // Fast bounding box rejection with spacing
    if bbox1.min_x >= bbox2.max_x + spacing - 1e-4
        || bbox1.max_x <= bbox2.min_x - spacing + 1e-4
        || bbox1.min_y >= bbox2.max_y + spacing - 1e-4
        || bbox1.max_y <= bbox2.min_y - spacing + 1e-4
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

    // 1. Check if any edge-edge distance is strictly less than spacing
    for i in 0..n1 {
        let p1 = poly1.points[i];
        let p2 = poly1.points[(i + 1) % n1];
        for j in 0..n2 {
            let q1 = poly2.points[j];
            let q2 = poly2.points[(j + 1) % n2];

            if spacing > 1e-4 {
                let d = segment_to_segment_distance(p1, p2, q1, q2);
                if d < spacing - 1e-4 {
                    return true;
                }
            } else {
                if segments_intersect_proper(p1, p2, q1, q2) {
                    return true;
                }
            }
        }
    }

    // 2. Comprehensive containment check (any vertex inside the other polygon)
    for pt in &poly1.points {
        if poly2.contains_point_strict(*pt, 0.0) {
            return true;
        }
    }
    for pt in &poly2.points {
        if poly1.contains_point_strict(*pt, 0.0) {
            return true;
        }
    }
    if poly1.contains_point_strict(poly2.centroid(), 0.0)
        || poly2.contains_point_strict(poly1.centroid(), 0.0)
    {
        return true;
    }

    false
}

#[allow(dead_code)]
#[inline]
pub fn polygons_intersect(poly1: &Polygon, poly2: &Polygon, _tolerance: f64) -> bool {
    polygons_collide_with_spacing(poly1, poly2, 0.0)
}

pub fn polygon_contained_in_hole(inner: &Polygon, hole: &Polygon, spacing: f64) -> bool {
    let inner_bbox = inner.bounding_box();
    let hole_bbox = hole.bounding_box();

    if inner_bbox.min_x < hole_bbox.min_x + spacing - 1e-4
        || inner_bbox.max_x > hole_bbox.max_x - spacing + 1e-4
        || inner_bbox.min_y < hole_bbox.min_y + spacing - 1e-4
        || inner_bbox.max_y > hole_bbox.max_y - spacing + 1e-4
    {
        return false;
    }

    for pt in &inner.points {
        if !hole.contains_point_strict(*pt, spacing * 0.5) {
            return false;
        }
    }

    let n1 = inner.points.len();
    let n2 = hole.points.len();
    for i in 0..n1 {
        let p1 = inner.points[i];
        let p2 = inner.points[(i + 1) % n1];
        for j in 0..n2 {
            let q1 = hole.points[j];
            let q2 = hole.points[(j + 1) % n2];
            let d1 = point_to_segment_distance(p1, q1, q2);
            let d2 = point_to_segment_distance(q1, p1, p2);
            if d1 < spacing - 1e-3 || d2 < spacing - 1e-3 {
                return false;
            }
        }
    }

    true
}

pub fn offset_polygon(poly: &Polygon, padding: f64) -> Polygon {
    if padding.abs() < 1e-6 || poly.points.len() < 3 {
        return poly.clone();
    }

    if poly.is_rectangular() {
        let bbox = poly.bounding_box();
        return Polygon::new(vec![
            Point::new(bbox.min_x - padding, bbox.min_y - padding),
            Point::new(bbox.max_x + padding, bbox.min_y - padding),
            Point::new(bbox.max_x + padding, bbox.max_y + padding),
            Point::new(bbox.min_x - padding, bbox.max_y + padding),
        ]);
    }

    let n = poly.points.len();
    let mut new_points = Vec::with_capacity(n);
    let is_ccw = poly.signed_area() > 0.0;

    for i in 0..n {
        let prev = poly.points[(i + n - 1) % n];
        let curr = poly.points[i];
        let next = poly.points[(i + 1) % n];

        let v1_x = curr.x - prev.x;
        let v1_y = curr.y - prev.y;
        let len1 = (v1_x * v1_x + v1_y * v1_y).sqrt().max(1e-9);
        let n1_x = if is_ccw { v1_y / len1 } else { -v1_y / len1 };
        let n1_y = if is_ccw { -v1_x / len1 } else { v1_x / len1 };

        let v2_x = next.x - curr.x;
        let v2_y = next.y - curr.y;
        let len2 = (v2_x * v2_x + v2_y * v2_y).sqrt().max(1e-9);
        let n2_x = if is_ccw { v2_y / len2 } else { -v2_y / len2 };
        let n2_y = if is_ccw { -v2_x / len2 } else { v2_x / len2 };

        let bisector_x = n1_x + n2_x;
        let bisector_y = n1_y + n2_y;
        let b_len = (bisector_x * bisector_x + bisector_y * bisector_y).sqrt().max(1e-9);
        let cos_half = ((n1_x * n2_x + n1_y * n2_y + 1.0) / 2.0).max(0.1).sqrt();
        let miter_dist = (padding / cos_half).min(padding * 2.5);

        let off_x = curr.x + (bisector_x / b_len) * miter_dist;
        let off_y = curr.y + (bisector_y / b_len) * miter_dist;
        new_points.push(Point::new(off_x, off_y));
    }

    Polygon::new(new_points)
}

pub fn simplify_collinear(points: &[Point], tolerance: f64) -> Vec<Point> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut clean: Vec<Point> = Vec::with_capacity(points.len());
    for &p in points {
        if let Some(&last) = clean.last() {
            let dx = p.x - last.x;
            let dy = p.y - last.y;
            if (dx * dx + dy * dy).sqrt() > 1e-4 {
                clean.push(p);
            }
        } else {
            clean.push(p);
        }
    }
    if clean.len() >= 2 {
        let first = clean[0];
        let last = *clean.last().unwrap();
        let dx = first.x - last.x;
        let dy = first.y - last.y;
        if (dx * dx + dy * dy).sqrt() <= 1e-4 {
            clean.pop();
        }
    }
    if clean.len() < 3 {
        return points.to_vec();
    }

    let n = clean.len();
    let mut result = Vec::with_capacity(n);
    for i in 0..n {
        let prev = clean[(i + n - 1) % n];
        let curr = clean[i];
        let next = clean[(i + 1) % n];

        let d = point_to_segment_distance(curr, prev, next);
        if d >= tolerance {
            result.push(curr);
        }
    }
    if result.len() < 3 {
        clean
    } else {
        result
    }
}

pub fn ensure_ccw(poly: &Polygon) -> Polygon {
    if poly.signed_area() < 0.0 {
        let mut rev = poly.points.clone();
        rev.reverse();
        Polygon::new(rev)
    } else {
        poly.clone()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Edge {
    pub p1: Point,
    pub p2: Point,
    pub length: f64,
    pub angle: f64,
    pub index: usize,
}

impl Edge {
    pub fn new(p1: Point, p2: Point, index: usize) -> Self {
        let dx = p2.x - p1.x;
        let dy = p2.y - p1.y;
        let length = (dx * dx + dy * dy).sqrt();
        let angle = dy.atan2(dx);
        Edge { p1, p2, length, angle, index }
    }
}

pub fn extract_edges(points: &[Point]) -> Vec<Edge> {
    let n = points.len();
    if n < 2 {
        return Vec::new();
    }
    let mut edges = Vec::with_capacity(n);
    for i in 0..n {
        let p1 = points[i];
        let p2 = points[(i + 1) % n];
        edges.push(Edge::new(p1, p2, i));
    }
    edges
}

pub fn is_truly_irregular(poly: &Polygon, bbox: &Rect, area: f64) -> bool {
    let bbox_area = bbox.area();
    if bbox_area <= 1.0 {
        return false;
    }

    if poly.is_circular() {
        return false;
    }

    let simplified = simplify_collinear(&poly.points, 0.5);
    if simplified.len() < 3 {
        return false;
    }

    if simplified.len() == 4 {
        let p = Polygon::new(simplified);
        return !p.is_rectangular();
    }

    if simplified.len() == 3 {
        return true;
    }

    let fill_ratio = area / bbox_area;
    if fill_ratio < 0.95 {
        return true;
    }

    let edges = extract_edges(&simplified);
    let has_slanted = edges.iter().any(|e| {
        if e.length < 10.0 {
            return false;
        }
        let deg = (e.angle * 180.0 / std::f64::consts::PI).abs() % 90.0;
        deg > 3.0 && deg < 87.0
    });

    if has_slanted {
        return true;
    }

    let p = Polygon::new(simplified);
    !p.is_rectangular()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_circle(radius: f64, segments: usize) -> Polygon {
        let mut pts = Vec::with_capacity(segments);
        for i in 0..segments {
            let angle = 2.0 * std::f64::consts::PI * (i as f64) / (segments as f64);
            pts.push(Point::new(radius * angle.cos(), radius * angle.sin()));
        }
        let poly = Polygon::new(pts);
        let (norm, _, _) = poly.normalize_to_origin();
        norm
    }

    #[test]
    fn test_circle_collision() {
        let c1 = make_circle(100.0, 24);
        let c2 = make_circle(100.0, 24);
        let c2_shifted = c2.translate(0.0, 150.0);
        assert!(polygons_collide_with_spacing(&c1, &c2_shifted, 6.0));
    }

    #[test]
    fn test_circle_sheet_context() {
        let c1 = make_circle(100.0, 24);
        let c2 = make_circle(100.0, 24);
        let mut ctx = crate::nfp::SheetContext::new(2440.0, 1220.0, 10.0, 6.0, 0.0, false, false, vec!["left".into(), "bottom".into()]);
        ctx.add_placed(c1.clone(), vec![], 10.0, 10.0, 0.0, false, 0.0);
        
        let dilated = offset_polygon(&c2, 3.0);
        let dilated_bbox = dilated.bounding_box();
        let can_place = ctx.can_place_precomputed(&c2, &dilated, &dilated_bbox, false, 10.0, 150.0, false, 0.0);
        assert!(!can_place);

        let anchor = ctx.find_best_anchor(&c2, &dilated, &dilated_bbox, false, false, 0.0, 1.0);
        assert!(anchor.is_some());
        let (x, y, _) = anchor.unwrap();
        assert!(!polygons_collide_with_spacing(&c1.translate(10.0, 10.0), &c2.translate(x, y), 6.0));
    }

    #[test]
    fn test_pack_circles() {
        let c = make_circle(100.0, 24);
        let mut parts = Vec::new();
        for i in 0..10 {
            parts.push(crate::types::PartInput {
                id: Some(format!("C{}", i)),
                entity_id: Some(format!("C{}", i)),
                name: Some(format!("Circle {}", i)),
                contour: c.to_raw(),
                holes: None,
                width: Some(200.0),
                height: Some(200.0),
                area: Some(std::f64::consts::PI * 10000.0),
                render_contour: None,
                render_holes: None,
                collision_contour: None,
                collision_holes: None,
                draw_layers: None,
                rotations: None,
                base_rotation_degrees: None,
                rotation_degrees: None,
                grain_locked: None,
                has_grain_label: None,
                grain_arrow_degrees: None,
                free_rotation: None,
                rotation_divisions: None,
                color: None,
                logical_part_count: None,
                manual_cluster_macro: None,
                manual_cluster_children: None,
                two_sided: None,
            });
        }
        let config = crate::types::ConfigurationInput {
            board_width: Some(1220.0),
            board_height: Some(2440.0),
            edge_margin: Some(10.0),
            part_spacing: Some(6.0),
            cut_gap: Some(6.0),
            rotation_divisions: Some(4),
            ..Default::default()
        };
        let mat_state = crate::types::MaterialStateInput {
            key: None,
            material: Some("MDF".to_string()),
            thickness: Some(17.5),
            group_index: None,
            configuration: Some(config),
            original_part_count: None,
            parts,
            learning_key: None,
        };
        let layout = crate::packing::pack_material(&mat_state, 0);
        for sheet in &layout.sheets {
            for i in 0..sheet.placements.len() {
                let p1 = &sheet.placements[i];
                let poly1 = Polygon::from_raw(&p1.contour).translate(p1.x, p1.y);
                for j in (i + 1)..sheet.placements.len() {
                    let p2 = &sheet.placements[j];
                    let poly2 = Polygon::from_raw(&p2.contour).translate(p2.x, p2.y);
                    let collides = polygons_collide_with_spacing(&poly1, &poly2, 6.0);
                    assert!(!collides, "Collision between {} and {} at ({}, {}) and ({}, {})", 
                        p1.name.as_deref().unwrap_or(""), p2.name.as_deref().unwrap_or(""), p1.x, p1.y, p2.x, p2.y);
                }
            }
        }

        let opt_layout = crate::epochs::optimize_material_layout(&mat_state, |_phase, _pct, _iter, _tot, _lay| {});
        for sheet in &opt_layout.sheets {
            for i in 0..sheet.placements.len() {
                let p1 = &sheet.placements[i];
                let poly1 = Polygon::from_raw(&p1.contour).translate(p1.x, p1.y);
                for j in (i + 1)..sheet.placements.len() {
                    let p2 = &sheet.placements[j];
                    let poly2 = Polygon::from_raw(&p2.contour).translate(p2.x, p2.y);
                    let collides = polygons_collide_with_spacing(&poly1, &poly2, 6.0);
                    assert!(!collides, "Collision in opt_layout between {} and {} at ({}, {}) and ({}, {})", 
                        p1.name.as_deref().unwrap_or(""), p2.name.as_deref().unwrap_or(""), p1.x, p1.y, p2.x, p2.y);
                }
            }
        }
    }

    #[test]
    fn test_circle_mating() {
        let c = make_circle(100.0, 24);
        let part = crate::types::PartInput {
            id: Some("C1".into()),
            entity_id: Some("C1".into()),
            name: Some("Circle 1".into()),
            contour: c.to_raw(),
            holes: None,
            width: Some(200.0),
            height: Some(200.0),
            area: Some(std::f64::consts::PI * 10000.0),
            render_contour: None,
            render_holes: None,
            collision_contour: None,
            collision_holes: None,
            draw_layers: None,
            rotations: None,
            base_rotation_degrees: None,
            rotation_degrees: None,
            grain_locked: None,
            has_grain_label: None,
            grain_arrow_degrees: None,
            free_rotation: None,
            rotation_divisions: None,
            color: None,
            logical_part_count: None,
            manual_cluster_macro: None,
            manual_cluster_children: None,
            two_sided: None,
        };
        let config = crate::types::ConfigurationInput {
            board_width: Some(1220.0),
            board_height: Some(2440.0),
            edge_margin: Some(10.0),
            part_spacing: Some(6.0),
            cut_gap: Some(6.0),
            rotation_divisions: Some(4),
            ..Default::default()
        };
        let mating = crate::macro_comb::find_best_mating(&part, &part, 6.0, Some(&config));
        assert!(mating.is_none(), "Circles must never mate into comb macros");
    }

    #[test]
    fn test_simulate_sheet13() {
        let c = make_circle(200.0, 24); // diameter 400
        let mut parts = Vec::new();
        // 5 circles
        for i in 0..5 {
            parts.push(crate::types::PartInput {
                id: Some(format!("Circle_{}", i)),
                entity_id: Some(format!("Circle_{}", i)),
                name: Some(format!("Circle {}", i)),
                contour: c.to_raw(),
                holes: None,
                width: Some(400.0),
                height: Some(400.0),
                area: Some(std::f64::consts::PI * 40000.0),
                render_contour: None,
                render_holes: None,
                collision_contour: None,
                collision_holes: None,
                draw_layers: None,
                rotations: None,
                base_rotation_degrees: None,
                rotation_degrees: None,
                grain_locked: None,
                has_grain_label: None,
                grain_arrow_degrees: None,
                free_rotation: None,
                rotation_divisions: Some(4),
                color: None,
                logical_part_count: None,
                manual_cluster_macro: None,
                manual_cluster_children: None,
                two_sided: None,
            });
        }
        // 10 right triangles (400 x 400)
        let tri_contour = vec![[0.0, 0.0], [400.0, 0.0], [0.0, 400.0]];
        for i in 0..10 {
            parts.push(crate::types::PartInput {
                id: Some(format!("Tri_{}", i)),
                entity_id: Some(format!("Tri_{}", i)),
                name: Some(format!("Tri {}", i)),
                contour: tri_contour.clone(),
                holes: None,
                width: Some(400.0),
                height: Some(400.0),
                area: Some(80000.0),
                render_contour: None,
                render_holes: None,
                collision_contour: None,
                collision_holes: None,
                draw_layers: None,
                rotations: None,
                base_rotation_degrees: None,
                rotation_degrees: None,
                grain_locked: None,
                has_grain_label: None,
                grain_arrow_degrees: None,
                free_rotation: None,
                rotation_divisions: Some(4),
                color: None,
                logical_part_count: None,
                manual_cluster_macro: None,
                manual_cluster_children: None,
                two_sided: None,
            });
        }

        let config = crate::types::ConfigurationInput {
            board_width: Some(1220.0),
            board_height: Some(2440.0),
            edge_margin: Some(10.0),
            part_spacing: Some(7.0),
            cut_gap: Some(7.0),
            rotation_divisions: Some(4),
            merge_cut_paths: Some(false),
            small_part_threshold: Some(150.0),
            small_part_clearance: Some(15.0),
            ..Default::default()
        };
        let mat_state = crate::types::MaterialStateInput {
            key: None,
            material: Some("MDF".to_string()),
            thickness: Some(17.5),
            group_index: None,
            configuration: Some(config),
            original_part_count: None,
            parts,
            learning_key: None,
        };

        let opt_layout = crate::epochs::optimize_material_layout(&mat_state, |_phase, _pct, _iter, _tot, _lay| {});
        for sheet in &opt_layout.sheets {
            for p in &sheet.placements {
                if p.name.as_deref().unwrap_or("").starts_with("Circle") {
                    assert_ne!(p.manual_cluster_macro, Some(true), "Circle must not be a macro");
                }
            }
            for i in 0..sheet.placements.len() {
                let p1 = &sheet.placements[i];
                let poly1 = Polygon::from_raw(&p1.contour).translate(p1.x, p1.y);
                for j in (i + 1)..sheet.placements.len() {
                    let p2 = &sheet.placements[j];
                    let poly2 = Polygon::from_raw(&p2.contour).translate(p2.x, p2.y);
                    let collides = polygons_collide_with_spacing(&poly1, &poly2, 7.0);
                    assert!(!collides, "Collision between {} and {} at ({}, {}) and ({}, {})",
                        p1.name.as_deref().unwrap_or(""), p2.name.as_deref().unwrap_or(""), p1.x, p1.y, p2.x, p2.y);
                }
            }
        }
    }
}

