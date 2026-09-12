#![allow(dead_code)]
// src/nfp.rs
use crate::geometry::{Point, Polygon, Rect, polygon_contained_in_hole, polygons_collide_with_spacing};

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
    pub small_part: bool,
    pub small_part_clearance: f64,
}

impl PlacedShape {
    pub fn new(
        poly: Polygon,
        holes: Vec<Polygon>,
        x: f64,
        y: f64,
        rotation: f64,
        small_part: bool,
        small_part_clearance: f64,
    ) -> Self {
        let placed_poly = poly.translate(x, y);
        let bbox = placed_poly.bounding_box();
        let is_rect = placed_poly.is_rectangular() && holes.is_empty();
        let is_circular = placed_poly.is_circular() && holes.is_empty();
        let radius = if is_circular { placed_poly.circle_radius() } else { 0.0 };
        let centroid = placed_poly.centroid();
        let placed_holes = holes.into_iter().map(|h| h.translate(x, y)).collect();

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
            small_part,
            small_part_clearance,
        }
    }
}

pub struct SheetContext {
    pub width: f64,
    pub height: f64,
    pub margin: f64,
    pub spacing: f64,
    pub small_part_edge_zone: f64,
    pub merge_cut_paths: bool,
    pub sheet_in_sheet: bool,
    pub compact_directions: Vec<String>,
    pub placed: Vec<PlacedShape>,
    pub free_rects: Vec<Rect>,
}

impl SheetContext {
    pub fn new(
        width: f64,
        height: f64,
        margin: f64,
        spacing: f64,
        small_part_edge_zone: f64,
        merge_cut_paths: bool,
        sheet_in_sheet: bool,
        compact_directions: Vec<String>,
    ) -> Self {
        let initial_free = Rect::new(margin, margin, width - margin, height - margin);
        SheetContext {
            width,
            height,
            margin,
            spacing,
            small_part_edge_zone,
            merge_cut_paths,
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

    #[inline]
    fn small_part_uses_protected_clearance(
        &self,
        small_part: bool,
        x: f64,
        y: f64,
        width: f64,
        height: f64,
    ) -> bool {
        if !small_part {
            return false;
        }
        if !self.merge_cut_paths {
            return true;
        }
        let zone = self.small_part_edge_zone.max(0.0);
        let left = x.max(0.0);
        let right = (self.width - (x + width)).max(0.0);
        let top = (self.height - (y + height)).max(0.0);
        left.min(right).min(top) <= zone + 0.001
    }

    #[inline]
    pub fn small_part_edge_protected(
        &self,
        small_part: bool,
        x: f64,
        y: f64,
        width: f64,
        height: f64,
    ) -> bool {
        small_part && self.merge_cut_paths && self.small_part_uses_protected_clearance(small_part, x, y, width, height)
    }

    #[inline]
    fn candidate_required_clearance(
        &self,
        small_part: bool,
        small_part_clearance: f64,
        x: f64,
        y: f64,
        width: f64,
        height: f64,
    ) -> f64 {
        if self.small_part_uses_protected_clearance(small_part, x, y, width, height) {
            self.spacing.max(small_part_clearance.max(0.0))
        } else {
            self.spacing
        }
    }

    #[inline]
    fn placed_required_clearance(&self, shape: &PlacedShape) -> f64 {
        self.candidate_required_clearance(
            shape.small_part,
            shape.small_part_clearance,
            shape.x,
            shape.y,
            shape.width,
            shape.height,
        )
    }

    #[inline]
    fn pair_clearance(
        &self,
        fixed: &PlacedShape,
        moving_small_part: bool,
        moving_small_part_clearance: f64,
        moving_x: f64,
        moving_y: f64,
        moving_width: f64,
        moving_height: f64,
    ) -> f64 {
        self.spacing
            .max(self.placed_required_clearance(fixed))
            .max(self.candidate_required_clearance(
                moving_small_part,
                moving_small_part_clearance,
                moving_x,
                moving_y,
                moving_width,
                moving_height,
            ))
    }

    pub fn can_place_precomputed(
        &self,
        candidate_poly: &Polygon,
        _candidate_dilated_poly: &Polygon,
        _candidate_dilated_bbox: &Rect,
        is_rect: bool,
        x: f64,
        y: f64,
        candidate_small_part: bool,
        candidate_small_part_clearance: f64,
    ) -> bool {
        let shifted_bbox = candidate_poly.bounding_box().translate(x, y);
        let bounds = self.valid_bounds();

        if !bounds.contains_rect(&shifted_bbox, 1e-4) {
            return false;
        }

        let shifted_poly = candidate_poly.translate(x, y);

        let candidate_width = shifted_bbox.width();
        let candidate_height = shifted_bbox.height();

        for shape in &self.placed {
            let required_clearance = self.pair_clearance(
                shape,
                candidate_small_part,
                candidate_small_part_clearance,
                x,
                y,
                candidate_width,
                candidate_height,
            );

            // Sheet-in-sheet / Hole cavity nesting: If candidate is inside a hole of this shape
            if !shape.holes.is_empty() {
                let inside_hole = shape.holes.iter().any(|h| {
                    polygon_contained_in_hole(&shifted_poly, h, required_clearance)
                });
                if inside_hole {
                    continue;
                }
            }

            if is_rect && shape.is_rect {
                if shifted_bbox.min_x < shape.bbox.max_x + required_clearance - 1e-4
                    && shifted_bbox.max_x > shape.bbox.min_x - required_clearance + 1e-4
                    && shifted_bbox.min_y < shape.bbox.max_y + required_clearance - 1e-4
                    && shifted_bbox.max_y > shape.bbox.min_y - required_clearance + 1e-4
                {
                    return false;
                }
            } else if polygons_collide_with_spacing(&shifted_poly, &shape.poly, required_clearance) {
                return false;
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
        candidate_small_part: bool,
        candidate_small_part_clearance: f64,
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
        let _candidate_radius = if candidate_is_circular { norm_poly.circle_radius() } else { 0.0 };

        let mut candidate_points = Vec::with_capacity((self.placed.len() + self.free_rects.len() + 1) * 8);

        // 1. MaxRects Free Rectangles Anchors - strictly placed from bottom-left corner
        for r in &self.free_rects {
            if pw <= r.width() + 1e-4 && ph <= r.height() + 1e-4 {
                let px = if pull_right { r.max_x - pw } else { r.min_x };
                let py = if pull_top { r.max_y - ph } else { r.min_y };
                candidate_points.push(Point::new(px, py));
            }
        }

        // 2. Touching anchors directly adjacent to already placed shapes.
        // Pair-clearance rule: fixed protection is always considered;
        // a moving small part in merge mode also gets a protected-clearance candidate.
        for shape in &self.placed {
            let fixed_clearance = self.placed_required_clearance(shape);
            let mut clearances = vec![self.spacing.max(fixed_clearance)];
            if candidate_small_part {
                clearances.push(self.spacing.max(candidate_small_part_clearance.max(0.0)));
            }
            clearances.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            clearances.dedup_by(|a, b| (*a - *b).abs() <= 1e-6);

            for clearance in clearances {
                let rx = shape.x + shape.width + clearance;
                let lx = shape.x - pw - clearance;
                let ty = shape.y + shape.height + clearance;
                let by = shape.y - ph - clearance;

                candidate_points.push(Point::new(rx, shape.y));
                candidate_points.push(Point::new(rx, bounds.min_y));
                candidate_points.push(Point::new(lx, shape.y));
                candidate_points.push(Point::new(lx, bounds.min_y));

                candidate_points.push(Point::new(shape.x, ty));
                candidate_points.push(Point::new(bounds.min_x, ty));
                candidate_points.push(Point::new(shape.x, by));
                candidate_points.push(Point::new(bounds.min_x, by));
            }

            // 3. Hole cavity anchors for nesting inside hollow shapes.
            let mut hole_clearances = vec![self.spacing.max(fixed_clearance)];
            if candidate_small_part {
                hole_clearances.push(self.spacing.max(candidate_small_part_clearance.max(0.0)));
            }
            hole_clearances.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            hole_clearances.dedup_by(|a, b| (*a - *b).abs() <= 1e-6);
            for hole in &shape.holes {
                let h_bbox = hole.bounding_box();
                for hole_clearance in &hole_clearances {
                    let clearance = *hole_clearance;
                    if pw <= h_bbox.width() - clearance * 2.0 && ph <= h_bbox.height() - clearance * 2.0 {
                        candidate_points.push(Point::new(h_bbox.min_x + clearance, h_bbox.min_y + clearance));
                        candidate_points.push(Point::new(h_bbox.min_x + (h_bbox.width() - pw) * 0.5, h_bbox.min_y + (h_bbox.height() - ph) * 0.5));

                        let step_x = (h_bbox.width() - pw - clearance * 2.0).max(0.0) / 4.0;
                        let step_y = (h_bbox.height() - ph - clearance * 2.0).max(0.0) / 4.0;
                        if step_x > 0.0 && step_y > 0.0 {
                            for ix in 0..=4 {
                                for iy in 0..=4 {
                                    candidate_points.push(Point::new(
                                        h_bbox.min_x + clearance + ix as f64 * step_x,
                                        h_bbox.min_y + clearance + iy as f64 * step_y,
                                    ));
                                }
                            }
                        }
                    }
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
        candidate_points.dedup_by(|a, b| (a.x - b.x).abs() < 0.5 && (a.y - b.y).abs() < 0.5);

        for pt in candidate_points {
            if self.can_place_precomputed(
                norm_poly,
                dilated_poly,
                dilated_bbox,
                is_rect,
                pt.x,
                pt.y,
                candidate_small_part,
                candidate_small_part_clearance,
            ) {
                let score = score_fn(&pt);
                return Some((pt.x, pt.y, score));
            }
        }

        None
    }

    #[inline]
    pub fn has_holes(&self) -> bool {
        self.placed.iter().any(|s| !s.holes.is_empty())
    }

    pub fn find_best_hole_anchor(
        &self,
        norm_poly: &Polygon,
        dilated_poly: &Polygon,
        dilated_bbox: &Rect,
        is_rect: bool,
        candidate_small_part: bool,
        candidate_small_part_clearance: f64,
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

        let mut candidate_points = Vec::new();

        for shape in &self.placed {
            if shape.holes.is_empty() {
                continue;
            }
            let fixed_clearance = self.placed_required_clearance(shape);
            let mut hole_clearances = vec![self.spacing.max(fixed_clearance)];
            if candidate_small_part {
                hole_clearances.push(self.spacing.max(candidate_small_part_clearance.max(0.0)));
            }
            hole_clearances.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            hole_clearances.dedup_by(|a, b| (*a - *b).abs() <= 1e-6);

            for hole in &shape.holes {
                let h_bbox = hole.bounding_box();
                for hole_clearance in &hole_clearances {
                    let clearance = *hole_clearance;
                    if pw <= h_bbox.width() - clearance * 2.0 && ph <= h_bbox.height() - clearance * 2.0 {
                        candidate_points.push(Point::new(h_bbox.min_x + clearance, h_bbox.min_y + clearance));
                        candidate_points.push(Point::new(
                            h_bbox.min_x + (h_bbox.width() - pw) * 0.5,
                            h_bbox.min_y + (h_bbox.height() - ph) * 0.5,
                        ));

                        let step_x = (h_bbox.width() - pw - clearance * 2.0).max(0.0) / 4.0;
                        let step_y = (h_bbox.height() - ph - clearance * 2.0).max(0.0) / 4.0;
                        if step_x > 0.0 && step_y > 0.0 {
                            for ix in 0..=4 {
                                for iy in 0..=4 {
                                    candidate_points.push(Point::new(
                                        h_bbox.min_x + clearance + ix as f64 * step_x,
                                        h_bbox.min_y + clearance + iy as f64 * step_y,
                                    ));
                                }
                            }
                        }
                    }
                }
            }
        }

        if candidate_points.is_empty() {
            return None;
        }

        candidate_points.retain(|p| {
            p.x >= bounds.min_x - 1e-4
                && p.y >= bounds.min_y - 1e-4
                && p.x + pw <= bounds.max_x + 1e-4
                && p.y + ph <= bounds.max_y + 1e-4
        });

        let score_fn = |p: &Point| -> f64 {
            let dx = if pull_right { bounds.max_x - p.x - pw } else { p.x - bounds.min_x };
            let dy = if pull_top { bounds.max_y - p.y - ph } else { p.y - bounds.min_y };
            dy * 2.0 + dx * strategy_bl_weight
        };

        candidate_points.sort_by(|a, b| {
            score_fn(a).partial_cmp(&score_fn(b)).unwrap_or(std::cmp::Ordering::Equal)
        });
        candidate_points.dedup_by(|a, b| (a.x - b.x).abs() < 0.5 && (a.y - b.y).abs() < 0.5);

        for pt in candidate_points {
            if self.can_place_precomputed(
                norm_poly,
                dilated_poly,
                dilated_bbox,
                is_rect,
                pt.x,
                pt.y,
                candidate_small_part,
                candidate_small_part_clearance,
            ) {
                let score = score_fn(&pt);
                return Some((pt.x, pt.y, score));
            }
        }

        None
    }

    pub fn add_placed(
        &mut self,
        poly: Polygon,
        holes: Vec<Polygon>,
        x: f64,
        y: f64,
        rotation: f64,
        small_part: bool,
        small_part_clearance: f64,
    ) {
        let placed_shape = PlacedShape::new(
            poly,
            holes,
            x,
            y,
            rotation,
            small_part,
            small_part_clearance,
        );
        let required_clearance = self.placed_required_clearance(&placed_shape);
        let occupied = Rect::new(
            placed_shape.x - required_clearance,
            placed_shape.y - required_clearance,
            placed_shape.x + placed_shape.width + required_clearance,
            placed_shape.y + placed_shape.height + required_clearance,
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
