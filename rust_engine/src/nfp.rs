// src/nfp.rs
use crate::geometry::{Point, Polygon, Rect, polygon_contained_in_hole, polygons_collide_with_spacing};
use std::f64::consts::PI;

#[derive(Debug, Clone)]
pub struct PlacedShape {
    pub poly: Polygon,
    pub bbox: Rect,
    pub holes: Vec<Polygon>,
    pub is_rect: bool,
    pub is_circular: bool,
    pub radius: f64,
    pub centroid: Point,
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
    pub rotation: f64,
}

impl PlacedShape {
    pub fn new(poly: Polygon, holes: Option<Vec<Vec<[f64; 2]>>>, x: f64, y: f64, rotation: f64) -> Self {
        let placed_poly = poly.translate(x, y);
        let bbox = placed_poly.bounding_box();
        let is_rect = placed_poly.is_rectangular();
        let is_circular = placed_poly.is_circular();
        let radius = if is_circular { placed_poly.circle_radius() } else { 0.0 };
        let centroid = placed_poly.centroid();
        let placed_holes = holes.unwrap_or_default().into_iter().map(|raw| {
            Polygon::from_raw(&raw).translate(x, y)
        }).collect();

        PlacedShape {
            poly: placed_poly,
            bbox,
            holes: placed_holes,
            is_rect,
            is_circular,
            radius,
            centroid,
            x,
            y,
            width: bbox.width(),
            height: bbox.height(),
            rotation,
        }
    }
}

pub struct SheetContext {
    pub width: f64,
    pub height: f64,
    pub margin: f64,
    pub spacing: f64,
    pub sheet_in_sheet: bool,
    pub compact_directions: Vec<String>,
    pub placed: Vec<PlacedShape>,
    pub free_rects: Vec<Rect>,
}

impl SheetContext {
    pub fn new(width: f64, height: f64, margin: f64, spacing: f64, sheet_in_sheet: bool, compact_directions: Vec<String>) -> Self {
        let initial_free = Rect::new(margin, margin, width - margin, height - margin);
        SheetContext {
            width,
            height,
            margin,
            spacing,
            sheet_in_sheet,
            compact_directions,
            placed: Vec::new(),
            free_rects: vec![initial_free],
        }
    }

    #[inline]
    pub fn valid_bounds(&self) -> Rect {
        Rect::new(
            self.margin,
            self.margin,
            self.width - self.margin,
            self.height - self.margin,
        )
    }

    pub fn can_place_precomputed(
        &self,
        candidate_poly: &Polygon,
        _candidate_dilated_poly: &Polygon,
        _candidate_dilated_bbox: &Rect,
        is_rect: bool,
        x: f64,
        y: f64,
    ) -> bool {
        let shifted_bbox = candidate_poly.bounding_box().translate(x, y);
        let bounds = self.valid_bounds();

        if !bounds.contains_rect(&shifted_bbox, 1e-4) {
            return false;
        }

        let shifted_poly = candidate_poly.translate(x, y);

        for shape in &self.placed {
            // Sheet-in-sheet: If candidate is inside a hole of this shape
            if self.sheet_in_sheet && !shape.holes.is_empty() {
                let inside_hole = shape.holes.iter().any(|h| {
                    polygon_contained_in_hole(&shifted_poly, h, self.spacing)
                });
                if inside_hole {
                    continue;
                }
            }

            if is_rect && shape.is_rect {
                if shifted_bbox.min_x < shape.bbox.max_x + self.spacing - 1e-4
                    && shifted_bbox.max_x > shape.bbox.min_x - self.spacing + 1e-4
                    && shifted_bbox.min_y < shape.bbox.max_y + self.spacing - 1e-4
                    && shifted_bbox.max_y > shape.bbox.min_y - self.spacing + 1e-4
                {
                    return false;
                }
                continue;
            }

            // Circular fast check
            if shape.is_circular {
                let c_center = Point::new(x + candidate_poly.centroid().x, y + candidate_poly.centroid().y);
                let dist_sq = (c_center.x - shape.centroid.x).powi(2) + (c_center.y - shape.centroid.y).powi(2);
                let min_dist = shape.radius + self.spacing;
                if dist_sq < min_dist * min_dist {
                    return false;
                }
            }

            // Bounding box filter with spacing
            let expanded_shape_bbox = Rect::new(
                shape.bbox.min_x - self.spacing,
                shape.bbox.min_y - self.spacing,
                shape.bbox.max_x + self.spacing,
                shape.bbox.max_y + self.spacing,
            );

            if expanded_shape_bbox.intersects(&shifted_bbox, 1e-4) {
                if polygons_collide_with_spacing(&shifted_poly, &shape.poly, self.spacing) {
                    return false;
                }
            }
        }

        true
    }

    pub fn find_best_anchor(
        &self,
        norm_poly: &Polygon,
        dilated_poly: &Polygon,
        dilated_bbox: &Rect,
        is_rect: bool,
        strategy_bl_weight: f64,
    ) -> Option<(f64, f64, f64)> {
        let bounds = self.valid_bounds();
        let poly_bbox = norm_poly.bounding_box();
        let pw = poly_bbox.width();
        let ph = poly_bbox.height();

        if pw > bounds.width() || ph > bounds.height() {
            return None;
        }

        let pull_right = self.compact_directions.iter().any(|d| d.eq_ignore_ascii_case("right"));
        let pull_top = self.compact_directions.iter().any(|d| d.eq_ignore_ascii_case("top"));

        let candidate_is_circular = norm_poly.is_circular();
        let candidate_radius = if candidate_is_circular { norm_poly.circle_radius() } else { 0.0 };

        let mut candidate_points = Vec::with_capacity((self.placed.len() + self.free_rects.len() + 1) * 8);

        // 1. MaxRects Free Rectangles Anchors - strictly placed from bottom-left corner
        for r in &self.free_rects {
            if pw <= r.width() + 1e-4 && ph <= r.height() + 1e-4 {
                let px = if pull_right { r.max_x - pw } else { r.min_x };
                let py = if pull_top { r.max_y - ph } else { r.min_y };
                candidate_points.push(Point::new(px, py));
            }
        }

        // 2. Touching anchors directly adjacent to already placed shapes
        for shape in &self.placed {
            let rx = shape.x + shape.width + self.spacing;
            let lx = shape.x - pw - self.spacing;
            let ty = shape.y + shape.height + self.spacing;
            let by = shape.y - ph - self.spacing;

            candidate_points.push(Point::new(rx, shape.y));
            candidate_points.push(Point::new(rx, bounds.min_y));
            candidate_points.push(Point::new(lx, shape.y));
            candidate_points.push(Point::new(lx, bounds.min_y));

            candidate_points.push(Point::new(shape.x, ty));
            candidate_points.push(Point::new(bounds.min_x, ty));
            candidate_points.push(Point::new(shape.x, by));
            candidate_points.push(Point::new(bounds.min_x, by));

            // 3. Staggered Honeycomb Anchors for Circular Shapes
            if candidate_is_circular && shape.is_circular {
                let dist = shape.radius + candidate_radius + self.spacing;
                let angles = [0.0, 30.0, 45.0, 60.0, 90.0, 120.0, 135.0, 150.0, 180.0];
                for &deg in &angles {
                    let rad = deg * PI / 180.0;
                    let dx = dist * rad.cos();
                    let dy = dist * rad.sin();

                    let cx1 = shape.centroid.x + dx;
                    let cy1 = shape.centroid.y + dy;
                    candidate_points.push(Point::new(cx1 - candidate_radius, cy1 - candidate_radius));

                    let cx2 = shape.centroid.x - dx;
                    let cy2 = shape.centroid.y + dy;
                    candidate_points.push(Point::new(cx2 - candidate_radius, cy2 - candidate_radius));
                }
            }

            // 4. Hole anchors for Sheet-in-Sheet
            if self.sheet_in_sheet {
                for hole in &shape.holes {
                    let h_bbox = hole.bounding_box();
                    candidate_points.push(Point::new(h_bbox.min_x + self.spacing * 0.5, h_bbox.min_y + self.spacing * 0.5));
                }
            }
        }

        // Deduplicate and filter within bounds
        candidate_points.retain(|p| {
            p.x >= bounds.min_x - 1e-4
                && p.y >= bounds.min_y - 1e-4
                && p.x + pw <= bounds.max_x + 1e-4
                && p.y + ph <= bounds.max_y + 1e-4
        });

        // Directional scoring function (Strict Bottom-Left sorting)
        let score_fn = |p: &Point| -> f64 {
            let dx = if pull_right { bounds.max_x - p.x - pw } else { p.x - bounds.min_x };
            let dy = if pull_top { bounds.max_y - p.y - ph } else { p.y - bounds.min_y };
            dy * 2.0 + dx * strategy_bl_weight
        };

        candidate_points.sort_by(|a, b| {
            score_fn(a).partial_cmp(&score_fn(b)).unwrap_or(std::cmp::Ordering::Equal)
        });

        for pt in candidate_points {
            if self.can_place_precomputed(norm_poly, dilated_poly, dilated_bbox, is_rect, pt.x, pt.y) {
                let score = score_fn(&pt);
                return Some((pt.x, pt.y, score));
            }
        }

        None
    }

    pub fn add_placed(&mut self, poly: Polygon, holes: Option<Vec<Vec<[f64; 2]>>>, x: f64, y: f64, rotation: f64) {
        let placed_shape = PlacedShape::new(poly, holes, x, y, rotation);
        let occupied = Rect::new(
            placed_shape.x - self.spacing,
            placed_shape.y - self.spacing,
            placed_shape.x + placed_shape.width + self.spacing,
            placed_shape.y + placed_shape.height + self.spacing,
        );

        // Split intersecting MaxRects free rectangles
        let mut new_free = Vec::new();
        for r in &self.free_rects {
            if !r.intersects(&occupied, 1e-4) {
                new_free.push(*r);
                continue;
            }

            if occupied.max_y < r.max_y {
                new_free.push(Rect::new(r.min_x, occupied.max_y, r.max_x, r.max_y));
            }
            if occupied.min_y > r.min_y {
                new_free.push(Rect::new(r.min_x, r.min_y, r.max_x, occupied.min_y));
            }
            if occupied.min_x > r.min_x {
                new_free.push(Rect::new(r.min_x, r.min_y, occupied.min_x, r.max_y));
            }
            if occupied.max_x < r.max_x {
                new_free.push(Rect::new(occupied.max_x, r.min_y, r.max_x, r.max_y));
            }
        }

        let mut pruned: Vec<Rect> = Vec::new();
        for i in 0..new_free.len() {
            let mut is_contained = false;
            for j in 0..new_free.len() {
                if i != j && new_free[j].contains_rect(&new_free[i], 1e-4) {
                    is_contained = true;
                    break;
                }
            }
            if !is_contained && new_free[i].area() > 1.0 {
                pruned.push(new_free[i]);
            }
        }

        if pruned.len() > 64 {
            pruned.sort_by(|a, b| b.area().partial_cmp(&a.area()).unwrap_or(std::cmp::Ordering::Equal));
            pruned.truncate(64);
        }

        self.free_rects = pruned;
        self.placed.push(placed_shape);
    }
}
