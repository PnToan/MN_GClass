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
    pub fn from_raw(raw: &[[f64; 2]]) -> Self {
        let points = raw.iter().map(|p| Point::new(p[0], p[1])).collect();
        Polygon { points }
    }

    pub fn to_raw(&self) -> Vec<[f64; 2]> {
        self.points.iter().map(|p| [p.x, p.y]).collect()
    }

    pub fn area(&self) -> f64 {
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
        area.abs() * 0.5
    }

    pub fn bounding_box(&self) -> Rect {
        Rect::from_points(&self.points)
    }

    pub fn centroid(&self) -> Point {
        let n = self.points.len();
        if n == 0 {
            return Point::new(0.0, 0.0);
        }
        let mut cx = 0.0;
        let mut cy = 0.0;
        let mut signed_area = 0.0;
        for i in 0..n {
            let j = (i + 1) % n;
            let a = self.points[i].x * self.points[j].y - self.points[j].x * self.points[i].y;
            signed_area += a;
            cx += (self.points[i].x + self.points[j].x) * a;
            cy += (self.points[i].y + self.points[j].y) * a;
        }
        signed_area *= 0.5;
        if signed_area.abs() < 1e-6 {
            let sum_x: f64 = self.points.iter().map(|p| p.x).sum();
            let sum_y: f64 = self.points.iter().map(|p| p.y).sum();
            return Point::new(sum_x / n as f64, sum_y / n as f64);
        }
        cx /= 6.0 * signed_area;
        cy /= 6.0 * signed_area;
        Point::new(cx, cy)
    }

    pub fn translate(&self, dx: f64, dy: f64) -> Polygon {
        Polygon {
            points: self.points.iter().map(|p| Point::new(p.x + dx, p.y + dy)).collect(),
        }
    }

    pub fn normalize_to_origin(&self) -> Polygon {
        let bbox = self.bounding_box();
        self.translate(-bbox.min_x, -bbox.min_y)
    }

    pub fn rotate_deg(&self, deg: f64) -> Polygon {
        let rad = deg * PI / 180.0;
        let origin = Point::new(0.0, 0.0);
        let rotated_points: Vec<Point> = self.points.iter().map(|p| p.rotate(rad, origin)).collect();
        let rotated = Polygon { points: rotated_points };
        rotated.normalize_to_origin()
    }

    pub fn is_rectangular(&self) -> bool {
        let bbox = self.bounding_box();
        let area = self.area();
        let bbox_area = bbox.area();
        if bbox_area < 1e-4 {
            return false;
        }
        (area - bbox_area).abs() / bbox_area < 0.005
    }

    pub fn is_circular(&self) -> bool {
        if self.points.len() < 8 {
            return false;
        }
        let center = self.centroid();
        let radii: Vec<f64> = self.points.iter().map(|p| {
            ((p.x - center.x).powi(2) + (p.y - center.y).powi(2)).sqrt()
        }).collect();
        let avg_r: f64 = radii.iter().sum::<f64>() / radii.len() as f64;
        let max_diff: f64 = radii.iter().map(|r| (r - avg_r).abs()).fold(0.0, f64::max);
        max_diff / avg_r.max(1.0) < 0.05
    }

    pub fn circle_radius(&self) -> f64 {
        let center = self.centroid();
        let radii: Vec<f64> = self.points.iter().map(|p| {
            ((p.x - center.x).powi(2) + (p.y - center.y).powi(2)).sqrt()
        }).collect();
        radii.iter().sum::<f64>() / radii.len().max(1) as f64
    }

    pub fn contains_point(&self, pt: Point) -> bool {
        let mut inside = false;
        let n = self.points.len();
        if n < 3 {
            return false;
        }
        let mut j = n - 1;
        for i in 0..n {
            let pi = self.points[i];
            let pj = self.points[j];
            if ((pi.y > pt.y) != (pj.y > pt.y))
                && (pt.x < (pj.x - pi.x) * (pt.y - pi.y) / (pj.y - pi.y + 1e-12) + pi.x)
            {
                inside = !inside;
            }
            j = i;
        }
        inside
    }
}

pub fn rotate_raw_points(points: &[[f64; 2]], rad: f64) -> Vec<[f64; 2]> {
    let cos = rad.cos();
    let sin = rad.sin();
    points.iter().map(|p| {
        let rx = (p[0] * cos - p[1] * sin) * 1000.0;
        let ry = (p[0] * sin + p[1] * cos) * 1000.0;
        [rx.round() / 1000.0, ry.round() / 1000.0]
    }).collect()
}

pub fn rotate_shape_full(
    contour: &[[f64; 2]],
    holes: Option<&Vec<Vec<[f64; 2]>>>,
    draw_layers: Option<&Vec<serde_json::Value>>,
    deg: f64,
) -> (Polygon, Option<Vec<Vec<[f64; 2]>>>, Option<Vec<serde_json::Value>>) {
    let rad = deg * PI / 180.0;
    let rot_contour = rotate_raw_points(contour, rad);
    let min_x = rot_contour.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    let min_y = rot_contour.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);

    let norm_contour: Vec<[f64; 2]> = rot_contour.iter().map(|p| [
        ((p[0] - min_x) * 1000.0).round() / 1000.0,
        ((p[1] - min_y) * 1000.0).round() / 1000.0,
    ]).collect();

    let norm_holes = holes.map(|h_list| {
        h_list.iter().map(|h| {
            let rot_h = rotate_raw_points(h, rad);
            rot_h.iter().map(|p| [
                ((p[0] - min_x) * 1000.0).round() / 1000.0,
                ((p[1] - min_y) * 1000.0).round() / 1000.0,
            ]).collect()
        }).collect()
    });

    let norm_layers = draw_layers.map(|layers| {
        layers.iter().map(|layer| {
            let mut layer_obj = layer.clone();
            if let Some(paths) = layer_obj.get_mut("paths").and_then(|p| p.as_array_mut()) {
                for path in paths.iter_mut() {
                    if let Some(points) = path.get_mut("points").and_then(|pts| pts.as_array_mut()) {
                        let raw_pts: Vec<[f64; 2]> = points.iter().filter_map(|pt| {
                            if let Some(arr) = pt.as_array() {
                                if arr.len() >= 2 {
                                    return Some([arr[0].as_f64().unwrap_or(0.0), arr[1].as_f64().unwrap_or(0.0)]);
                                }
                            }
                            None
                        }).collect();

                        let rot_pts = rotate_raw_points(&raw_pts, rad);
                        let norm_pts: Vec<serde_json::Value> = rot_pts.iter().map(|p| {
                            serde_json::json!([
                                ((p[0] - min_x) * 1000.0).round() / 1000.0,
                                ((p[1] - min_y) * 1000.0).round() / 1000.0
                            ])
                        }).collect();
                        *points = norm_pts;
                    }
                }
            }
            layer_obj
        }).collect()
    });

    (Polygon::from_raw(&norm_contour), norm_holes, norm_layers)
}

pub fn offset_polygon(poly: &Polygon, delta: f64) -> Polygon {
    let bbox = poly.bounding_box();
    if poly.is_rectangular() {
        return Polygon {
            points: vec![
                Point::new(bbox.min_x - delta, bbox.min_y - delta),
                Point::new(bbox.max_x + delta, bbox.min_y - delta),
                Point::new(bbox.max_x + delta, bbox.max_y + delta),
                Point::new(bbox.min_x - delta, bbox.max_y + delta),
            ],
        };
    }
    let n = poly.points.len();
    if n < 3 {
        return poly.clone();
    }
    let mut new_points = Vec::with_capacity(n);
    for i in 0..n {
        let prev = poly.points[(i + n - 1) % n];
        let curr = poly.points[i];
        let next = poly.points[(i + 1) % n];

        let v1 = Point::new(curr.x - prev.x, curr.y - prev.y);
        let l1 = (v1.x * v1.x + v1.y * v1.y).sqrt().max(1e-6);
        let n1 = Point::new(-v1.y / l1, v1.x / l1);

        let v2 = Point::new(next.x - curr.x, next.y - curr.y);
        let l2 = (v2.x * v2.x + v2.y * v2.y).sqrt().max(1e-6);
        let n2 = Point::new(-v2.y / l2, v2.x / l2);

        let avg_n = Point::new((n1.x + n2.x) * 0.5, (n1.y + n2.y) * 0.5);
        let l_avg = (avg_n.x * avg_n.x + avg_n.y * avg_n.y).sqrt().max(1e-6);
        let scale = delta / l_avg;

        new_points.push(Point::new(curr.x + avg_n.x * scale, curr.y + avg_n.y * scale));
    }
    Polygon { points: new_points }
}

#[inline]
fn dist_to_segment(p: Point, a: Point, b: Point) -> f64 {
    let l2 = (b.x - a.x).powi(2) + (b.y - a.y).powi(2);
    if l2 < 1e-8 {
        return ((p.x - a.x).powi(2) + (p.y - a.y).powi(2)).sqrt();
    }
    let t = (((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2).clamp(0.0, 1.0);
    let proj = Point::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
    ((p.x - proj.x).powi(2) + (p.y - proj.y).powi(2)).sqrt()
}

fn segments_intersect(a1: Point, a2: Point, b1: Point, b2: Point) -> bool {
    fn ccw(a: Point, b: Point, c: Point) -> bool {
        (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x)
    }
    ccw(a1, b1, b2) != ccw(a2, b1, b2) && ccw(a1, a2, b1) != ccw(a1, a2, b2)
}

pub fn polygons_collide_with_spacing(poly1: &Polygon, poly2: &Polygon, spacing: f64) -> bool {
    let bbox1 = poly1.bounding_box();
    let bbox2 = poly2.bounding_box();
    if !bbox1.intersects(&bbox2.translate(-spacing * 0.5, -spacing * 0.5), spacing) {
        return false;
    }

    let n1 = poly1.points.len();
    let n2 = poly2.points.len();

    // 1. Direct edge intersections
    for i in 0..n1 {
        let p1 = poly1.points[i];
        let p2 = poly1.points[(i + 1) % n1];
        for j in 0..n2 {
            let q1 = poly2.points[j];
            let q2 = poly2.points[(j + 1) % n2];
            if segments_intersect(p1, p2, q1, q2) {
                return true;
            }
        }
    }

    // 2. Vertex inside check
    if poly1.contains_point(poly2.points[0]) || poly2.contains_point(poly1.points[0]) {
        return true;
    }

    // 3. Minimum distance between edges < spacing
    if spacing > 1e-4 {
        for pt in &poly1.points {
            for j in 0..n2 {
                let q1 = poly2.points[j];
                let q2 = poly2.points[(j + 1) % n2];
                if dist_to_segment(*pt, q1, q2) < spacing - 1e-4 {
                    return true;
                }
            }
        }
        for pt in &poly2.points {
            for i in 0..n1 {
                let p1 = poly1.points[i];
                let p2 = poly1.points[(i + 1) % n1];
                if dist_to_segment(*pt, p1, p2) < spacing - 1e-4 {
                    return true;
                }
            }
        }
    }

    false
}

pub fn polygon_contained_in_hole(inner: &Polygon, hole: &Polygon, margin: f64) -> bool {
    let inner_bbox = inner.bounding_box();
    let hole_bbox = hole.bounding_box();

    if !hole_bbox.contains_rect(&inner_bbox, -margin) {
        return false;
    }

    for pt in &inner.points {
        if !hole.contains_point(*pt) {
            return false;
        }
    }

    let n_hole = hole.points.len();
    for pt in &inner.points {
        for j in 0..n_hole {
            let q1 = hole.points[j];
            let q2 = hole.points[(j + 1) % n_hole];
            if dist_to_segment(*pt, q1, q2) < margin - 1e-4 {
                return false;
            }
        }
    }

    let n_inner = inner.points.len();
    for i in 0..n_inner {
        let p1 = inner.points[i];
        let p2 = inner.points[(i + 1) % n_inner];
        for j in 0..n_hole {
            let q1 = hole.points[j];
            let q2 = hole.points[(j + 1) % n_hole];
            if segments_intersect(p1, p2, q1, q2) {
                return false;
            }
        }
    }

    true
}
