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

        let candidate_width = shifted_bbox.width();
        let candidate_height = shifted_bbox.height();
        let mut shifted_poly_opt: Option<Polygon> = None;

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

            if shifted_bbox.min_x >= shape.bbox.max_x + required_clearance - 1e-4
                || shifted_bbox.max_x <= shape.bbox.min_x - required_clearance + 1e-4
                || shifted_bbox.min_y >= shape.bbox.max_y + required_clearance - 1e-4
                || shifted_bbox.max_y <= shape.bbox.min_y - required_clearance + 1e-4
            {
                continue;
            }

            if is_rect && shape.is_rect {
                return false;
            }

            let shifted_poly = shifted_poly_opt.get_or_insert_with(|| {
                if candidate_poly.points.len() > 64 {
                    crate::macro_comb::decimate_curved_polygon(&candidate_poly.points, 16.0).translate(x, y)
                } else {
                    candidate_poly.translate(x, y)
                }
            });

            // Sheet-in-sheet / Hole cavity nesting: If candidate is inside a hole of this shape
            if !shape.holes.is_empty() {
                let inside_hole = shape.holes.iter().any(|h| {
                    polygon_contained_in_hole(shifted_poly, h, required_clearance)
                        || (shape.poly.is_rectangular()
                            && Self::polygon_in_open_mouth_hole(
                                shifted_poly,
                                &shifted_bbox,
                                is_rect,
                                h,
                                &shape.bbox,
                                required_clearance,
                            ))
                });
                if inside_hole {
                    continue;
                }
            }

            if shape.is_rect && is_rect {
                if polygons_collide_with_spacing(shifted_poly, &shape.poly, required_clearance) {
                    return false;
                }
            } else if crate::macro_comb::polygons_collide_concave_safe(shifted_poly, &shape.poly, required_clearance) {
                return false;
            }
        }

        true
    }

    #[inline]
    fn polygon_in_open_mouth_hole(
        _shifted_poly: &Polygon,
        shifted_bbox: &Rect,
        _is_rect: bool,
        hole: &Polygon,
        shape_bbox: &Rect,
        clearance: f64,
    ) -> bool {
        let h_bbox = hole.bounding_box();
        let open_top = (h_bbox.max_y - shape_bbox.max_y).abs() <= 2.0;
        let open_bottom = (h_bbox.min_y - shape_bbox.min_y).abs() <= 2.0;
        let open_left = (h_bbox.min_x - shape_bbox.min_x).abs() <= 2.0;
        let open_right = (h_bbox.max_x - shape_bbox.max_x).abs() <= 2.0;
        if !(open_top || open_bottom || open_left || open_right) {
            return false;
        }
        if shifted_bbox.min_x < shape_bbox.min_x - 1e-4
            || (!open_right && shifted_bbox.max_x > shape_bbox.max_x + 1e-4)
            || shifted_bbox.min_y < shape_bbox.min_y - 1e-4
            || shifted_bbox.max_y > shape_bbox.max_y + 1e-4
        {
            return false;
        }
        if !open_left && shifted_bbox.min_x < h_bbox.min_x + clearance - 1e-4 {
            return false;
        }
        if !open_right && shifted_bbox.max_x > h_bbox.max_x - clearance + 1e-4 {
            return false;
        }
        if !open_bottom && shifted_bbox.min_y < h_bbox.min_y + clearance - 1e-4 {
            return false;
        }
        if !open_top && shifted_bbox.max_y > h_bbox.max_y - clearance + 1e-4 {
            return false;
        }
        let inter_w = (shifted_bbox.max_x.min(h_bbox.max_x) - shifted_bbox.min_x.max(h_bbox.min_x)).max(0.0);
        let inter_h = (shifted_bbox.max_y.min(h_bbox.max_y) - shifted_bbox.min_y.max(h_bbox.min_y)).max(0.0);
        let pw = shifted_bbox.width().max(1e-6);
        let ph = shifted_bbox.height().max(1e-6);
        let req_inter_w = if open_right && (open_top || open_bottom) { 2.0 } else { pw * 0.42 };
        inter_w >= req_inter_w && inter_h >= ph * 0.42
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

        let mut candidate_points = Vec::with_capacity((self.placed.len() + self.free_rects.len() + 1) * 10);

        // 1. MaxRects Free Rectangles Anchors - strictly placed from bottom-left corner
        for r in &self.free_rects {
            if pw <= r.width() + 1e-4 && ph <= r.height() + 1e-4 {
                if !pull_right && !pull_top && r.min_x > bounds.min_x + 1.0 && ph > 10.0 {
                    let rows_r = ((r.height() + self.spacing) / (ph + self.spacing)).floor();
                    if rows_r >= 1.0 && self.free_rects.iter().any(|r2| {
                        r2.min_x < r.min_x - 1.0
                            && r2.max_x >= r.max_x - 1.0
                            && r2.min_y > r.min_y
                            && r2.min_y - r.min_y <= ph * 0.35
                            && ((r2.height() + self.spacing) / (ph + self.spacing)).floor() >= rows_r
                    }) {
                        continue;
                    }
                }
                let px = if pull_right { r.max_x - pw } else { r.min_x };
                let py = if pull_top { r.max_y - ph } else { r.min_y };
                candidate_points.push(Point::new(px, py));
            }
        }

        // 2. Touching anchors directly adjacent to already placed shapes.
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

                if shape.is_rect || shape.width.max(shape.height) >= 250.0 || (shape.x - bounds.min_x).abs() <= 1.0 {
                    candidate_points.push(Point::new(shape.x, ty));
                    candidate_points.push(Point::new(shape.x, by));
                }
                candidate_points.push(Point::new(bounds.min_x, ty));
                candidate_points.push(Point::new(bounds.min_x, by));

                if !shape.poly.is_rectangular() && shape.width.max(shape.height) >= 300.0 {
                    for &rx_ratio in &[0.0, 0.08, 0.16, 0.25, 0.35, 0.48, 0.62, 0.75, 0.88] {
                        let pen_rx = shape.x + shape.width * rx_ratio;
                        candidate_points.push(Point::new(pen_rx, shape.y));
                        candidate_points.push(Point::new(pen_rx, bounds.min_y));
                        for &ty_ratio in &[0.06, 0.15, 0.25, 0.36, 0.48, 0.60, 0.72, 0.82, 0.91] {
                            let pen_ty = shape.y + shape.height * ty_ratio;
                            candidate_points.push(Point::new(pen_rx, pen_ty));
                        }
                    }
                    for &ty_ratio in &[0.06, 0.15, 0.25, 0.36, 0.48, 0.60, 0.72, 0.82, 0.91] {
                        let pen_ty = shape.y + shape.height * ty_ratio;
                        candidate_points.push(Point::new(shape.x, pen_ty));
                        candidate_points.push(Point::new(bounds.min_x, pen_ty));
                        candidate_points.push(Point::new((bounds.max_x - pw).max(bounds.min_x), pen_ty));
                    }
                    let mut v_added = 0usize;
                    let v_step = (shape.poly.points.len() / 36).max(1);
                    for v in shape.poly.points.iter().step_by(v_step) {
                        let in_x = v.x > shape.bbox.min_x + 2.0 && v.x < shape.bbox.max_x - 2.0;
                        let in_y = v.y > shape.bbox.min_y + 2.0 && v.y < shape.bbox.max_y - 2.0;
                        if in_x || in_y {
                            candidate_points.push(Point::new(v.x + clearance, v.y));
                            candidate_points.push(Point::new(v.x + clearance, v.y + clearance));
                            candidate_points.push(Point::new(v.x + clearance, (v.y - ph - clearance).max(bounds.min_y)));
                            candidate_points.push(Point::new((v.x - pw - clearance).max(bounds.min_x), v.y));
                            candidate_points.push(Point::new((v.x - pw - clearance).max(bounds.min_x), v.y + clearance));
                            candidate_points.push(Point::new(bounds.min_x, v.y));
                            candidate_points.push(Point::new(bounds.min_x, v.y + clearance));
                            candidate_points.push(Point::new((bounds.max_x - pw).max(bounds.min_x), v.y));
                            candidate_points.push(Point::new((bounds.max_x - pw).max(bounds.min_x), v.y + clearance));
                            candidate_points.push(Point::new(v.x + clearance, shape.y));
                            candidate_points.push(Point::new(shape.x, v.y + clearance));
                            v_added += 1;
                            if v_added >= 36 {
                                break;
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
                && !self.placed.iter().any(|s| {
                    s.poly.is_rectangular()
                        && !s.holes.is_empty()
                        && p.x < s.bbox.max_x - 1e-4
                        && p.x + pw > s.bbox.min_x + 1e-4
                        && p.y < s.bbox.max_y - 1e-4
                        && p.y + ph > s.bbox.min_y + 1e-4
                })
        });

        // Directional scoring function (Strict Bottom-Left sorting)
        let score_fn = |p: &Point| -> f64 {
            let dx = if pull_right { bounds.max_x - p.x - pw } else { p.x - bounds.min_x };
            let dy = if pull_top { bounds.max_y - p.y - ph } else { p.y - bounds.min_y };
            dy * 2.0 + dx * strategy_bl_weight + ph * 0.15
        };

        candidate_points.sort_by(|a, b| {
            score_fn(a).partial_cmp(&score_fn(b)).unwrap_or(std::cmp::Ordering::Equal)
        });
        candidate_points.dedup_by(|a, b| (a.x - b.x).abs() < 0.5 && (a.y - b.y).abs() < 0.5);

        let has_irregular_placed = !is_rect && pw.max(ph) >= 250.0;

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
                let mut fx = pt.x;
                let mut fy = pt.y;
                if has_irregular_placed {
                    let mut step_x = if pull_right { 36.0 } else { -36.0 };
                    for _ in 0..5 {
                        let nx = (fx + step_x).clamp(bounds.min_x, bounds.max_x - pw);
                        if (nx - fx).abs() > 0.2 && self.can_place_precomputed(
                            norm_poly, dilated_poly, dilated_bbox, is_rect,
                            nx, fy, candidate_small_part, candidate_small_part_clearance,
                        ) {
                            fx = nx;
                        } else {
                            step_x *= 0.5;
                        }
                    }
                    let mut step_y = if pull_top { 36.0 } else { -36.0 };
                    for _ in 0..5 {
                        let ny = (fy + step_y).clamp(bounds.min_y, bounds.max_y - ph);
                        if (ny - fy).abs() > 0.2 && self.can_place_precomputed(
                            norm_poly, dilated_poly, dilated_bbox, is_rect,
                            fx, ny, candidate_small_part, candidate_small_part_clearance,
                        ) {
                            fy = ny;
                        } else {
                            step_y *= 0.5;
                        }
                    }
                } else if !is_rect {
                    let mut step_x = if pull_right { 2.0 } else { -2.0 };
                    for _ in 0..2 {
                        let nx = (fx + step_x).clamp(bounds.min_x, bounds.max_x - pw);
                        if (nx - fx).abs() > 0.15 && self.can_place_precomputed(
                            norm_poly, dilated_poly, dilated_bbox, is_rect,
                            nx, fy, candidate_small_part, candidate_small_part_clearance,
                        ) {
                            fx = nx;
                        } else {
                            step_x *= 0.5;
                        }
                    }
                }
                let final_pt = Point::new(fx, fy);
                let score = score_fn(&final_pt);
                return Some((fx, fy, score));
            }
        }

        None
    }

    #[inline]
    pub fn has_holes(&self) -> bool {
        self.placed.iter().any(|s| !s.holes.is_empty())
    }

    #[inline]
    pub fn has_holes_or_pockets(&self) -> bool {
        self.has_holes()
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
        self.find_best_hole_or_pocket_anchor(
            norm_poly,
            dilated_poly,
            dilated_bbox,
            is_rect,
            candidate_small_part,
            candidate_small_part_clearance,
            strategy_bl_weight,
            false,
        )
    }

    pub fn find_best_hole_or_pocket_anchor(
        &self,
        norm_poly: &Polygon,
        dilated_poly: &Polygon,
        dilated_bbox: &Rect,
        is_rect: bool,
        candidate_small_part: bool,
        candidate_small_part_clearance: f64,
        strategy_bl_weight: f64,
        _include_concave_pockets: bool,
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

            let mouth_tol = if shape.poly.is_rectangular() { 2.0 } else { 15.0 };

            for hole in &shape.holes {
                let h_bbox = hole.bounding_box();
                let open_top = (h_bbox.max_y - shape.bbox.max_y).abs() <= mouth_tol;
                let open_bottom = (h_bbox.min_y - shape.bbox.min_y).abs() <= mouth_tol;
                let open_left = (h_bbox.min_x - shape.bbox.min_x).abs() <= mouth_tol;
                let open_right = (h_bbox.max_x - shape.bbox.max_x).abs() <= mouth_tol;

                for hole_clearance in &hole_clearances {
                    let clearance = *hole_clearance;
                    let req_w = if open_left || open_right {
                        pw * 0.42 + clearance
                    } else {
                        pw + clearance * 2.0
                    };
                    let req_h = if open_top || open_bottom {
                        ph * 0.42 + clearance
                    } else {
                        ph + clearance * 2.0
                    };
                    if h_bbox.width() + 1e-4 < req_w || h_bbox.height() + 1e-4 < req_h {
                        continue;
                    }

                    let mut x_anchors = Vec::with_capacity(14);
                    let left_pad = if open_left { 0.0 } else { clearance };
                    let right_pad = if open_right { 0.0 } else { clearance };
                    if open_right && !open_left {
                        for col in 0..6 {
                            let cx = h_bbox.max_x - right_pad - pw - (col as f64) * (pw + clearance);
                            if cx >= h_bbox.min_x + left_pad - 1e-4 {
                                x_anchors.push(cx);
                            } else {
                                break;
                            }
                        }
                        if open_top || open_bottom {
                            for col in 0..7 {
                                let cx = h_bbox.min_x + left_pad + (col as f64) * (pw + clearance);
                                if cx + pw <= bounds.max_x + 1e-4 && cx < h_bbox.max_x - 2.0 {
                                    x_anchors.push(cx);
                                } else {
                                    break;
                                }
                            }
                        }
                    } else {
                        for col in 0..6 {
                            let cx = h_bbox.min_x + left_pad + (col as f64) * (pw + clearance);
                            if cx + pw <= h_bbox.max_x - right_pad + 1e-4 {
                                x_anchors.push(cx);
                            } else {
                                break;
                            }
                        }
                    }

                    let mut y_anchors = Vec::with_capacity(10);
                    let bot_pad = if open_bottom { 0.0 } else { clearance };
                    let top_pad = if open_top { 0.0 } else { clearance };
                    if open_top && !open_bottom {
                        for row in 0..6 {
                            let cy = h_bbox.max_y - top_pad - ph - (row as f64) * (ph + clearance);
                            if cy >= h_bbox.min_y + bot_pad - 1e-4 {
                                y_anchors.push(cy);
                            } else {
                                break;
                            }
                        }
                    } else {
                        for row in 0..6 {
                            let cy = h_bbox.min_y + bot_pad + (row as f64) * (ph + clearance);
                            if cy + ph <= h_bbox.max_y - top_pad + 1e-4 {
                                y_anchors.push(cy);
                            } else {
                                break;
                            }
                        }
                    }

                    for inner in &self.placed {
                        if std::ptr::eq(inner, shape) {
                            continue;
                        }
                        if inner.bbox.max_x > h_bbox.min_x + 1.0
                            && inner.bbox.min_x < h_bbox.max_x - 1.0
                            && inner.bbox.max_y > h_bbox.min_y + 1.0
                            && inner.bbox.min_y < h_bbox.max_y - 1.0
                        {
                            x_anchors.push(inner.x + inner.width + clearance);
                            x_anchors.push(inner.x - pw - clearance);
                            y_anchors.push(inner.y + inner.height + clearance);
                            y_anchors.push(inner.y - ph - clearance);
                            x_anchors.push(inner.x);
                            y_anchors.push(inner.y);
                        }
                    }

                    for &cx in &x_anchors {
                        for &cy in &y_anchors {
                            let cand_bbox = Rect {
                                min_x: cx,
                                min_y: cy,
                                max_x: cx + pw,
                                max_y: cy + ph,
                            };
                            let in_closed_bbox = cand_bbox.min_x >= h_bbox.min_x + clearance - 1e-4
                                && cand_bbox.max_x <= h_bbox.max_x - clearance + 1e-4
                                && cand_bbox.min_y >= h_bbox.min_y + clearance - 1e-4
                                && cand_bbox.max_y <= h_bbox.max_y - clearance + 1e-4;
                            let in_open_pocket = Self::polygon_in_open_mouth_hole(
                                norm_poly,
                                &cand_bbox,
                                is_rect,
                                hole,
                                &shape.bbox,
                                clearance,
                            );
                            if in_closed_bbox || in_open_pocket {
                                let h_score = if open_top && !open_bottom {
                                    10000.0 + (shape.bbox.max_y - (cy + ph)).max(0.0) * 3.0 + (cx - bounds.min_x) * strategy_bl_weight
                                } else if open_right && !open_left {
                                    20000.0 + (shape.bbox.max_x - (cx + pw)).max(0.0) * 3.0 + (cy - bounds.min_y) * strategy_bl_weight
                                } else {
                                    (cy - bounds.min_y) * 3.0 + (cx - bounds.min_x) * strategy_bl_weight
                                };
                                candidate_points.push((Point::new(cx, cy), h_score));
                            }
                        }
                    }
                }
            }
        }

        if candidate_points.is_empty() {
            return None;
        }

        candidate_points.retain(|(p, _)| {
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
            a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal)
        });
        candidate_points.dedup_by(|a, b| (a.0.x - b.0.x).abs() < 0.5 && (a.0.y - b.0.y).abs() < 0.5);

        for (pt, _) in candidate_points {
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
                let mut fx = pt.x;
                let mut fy = pt.y;
                if !is_rect && pw.max(ph) >= 250.0 {
                    let mut step_x = if pull_right { 16.0 } else { -16.0 };
                    for _ in 0..4 {
                        let nx = (fx + step_x).clamp(bounds.min_x, bounds.max_x - pw);
                        if (nx - fx).abs() > 0.2 && self.can_place_precomputed(
                            norm_poly, dilated_poly, dilated_bbox, is_rect,
                            nx, fy, candidate_small_part, candidate_small_part_clearance,
                        ) {
                            fx = nx;
                        } else {
                            step_x *= 0.5;
                        }
                    }
                    let mut step_y = if pull_top { 16.0 } else { -16.0 };
                    for _ in 0..4 {
                        let ny = (fy + step_y).clamp(bounds.min_y, bounds.max_y - ph);
                        if (ny - fy).abs() > 0.2 && self.can_place_precomputed(
                            norm_poly, dilated_poly, dilated_bbox, is_rect,
                            fx, ny, candidate_small_part, candidate_small_part_clearance,
                        ) {
                            fy = ny;
                        } else {
                            step_y *= 0.5;
                        }
                    }
                }
                let final_pt = Point::new(fx, fy);
                let score = score_fn(&final_pt);
                return Some((fx, fy, score));
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
