// src/macro_comb.rs
use crate::geometry::{Polygon, Rect, polygons_collide_with_spacing};
use crate::types::{ClusterChild, PartInput};

pub fn try_build_comb_pair(
    part_a: &PartInput,
    part_b: &PartInput,
    spacing: f64,
) -> Option<(PartInput, f64)> {
    let poly_a = Polygon::from_raw(&part_a.contour);
    let poly_b = Polygon::from_raw(&part_b.contour);

    if poly_a.is_rectangular() && poly_b.is_rectangular() {
        return None;
    }

    let bbox_a = poly_a.bounding_box();
    let bbox_b = poly_b.bounding_box();
    let separate_bbox_area = bbox_a.area() + bbox_b.area();
    if separate_bbox_area <= 0.0 {
        return None;
    }

    let test_rotations = [180.0, 0.0];
    let mut best_saving = 0.0;
    let mut best_offset_x = 0.0;
    let mut best_offset_y = 0.0;
    let mut best_rot_b = 180.0;
    let mut best_combined_bbox = Rect::new(0.0, 0.0, 0.0, 0.0);

    for &rot_b in &test_rotations {
        let rotated_b = poly_b.rotate_deg(rot_b);
        let norm_b = rotated_b.normalize_to_origin();
        let b_bbox = norm_b.bounding_box();

        // Dense alignment grid around edges and notches
        let mut candidate_offsets = Vec::with_capacity(36);
        let xs = [
            0.0,
            bbox_a.width() - b_bbox.width(),
            (bbox_a.width() - b_bbox.width()) * 0.5,
            bbox_a.width() + spacing,
            -b_bbox.width() - spacing,
        ];
        let ys = [
            0.0,
            bbox_a.height() - b_bbox.height(),
            (bbox_a.height() - b_bbox.height()) * 0.5,
            bbox_a.height() + spacing,
            -b_bbox.height() - spacing,
        ];

        for &ox in &xs {
            for &oy in &ys {
                candidate_offsets.push((ox, oy));
            }
        }

        // Notch sliding points
        for pa in &poly_a.points {
            for pb in &norm_b.points {
                candidate_offsets.push((pa.x - pb.x + spacing, pa.y - pb.y));
                candidate_offsets.push((pa.x - pb.x, pa.y - pb.y + spacing));
                candidate_offsets.push((pa.x - pb.x - spacing, pa.y - pb.y));
                candidate_offsets.push((pa.x - pb.x, pa.y - pb.y - spacing));
            }
        }

        for (ox, oy) in candidate_offsets {
            let shifted_b = norm_b.translate(ox, oy);
            if !polygons_collide_with_spacing(&poly_a, &shifted_b, spacing) {
                let shifted_b_bbox = b_bbox.translate(ox, oy);
                let c_min_x = bbox_a.min_x.min(shifted_b_bbox.min_x);
                let c_min_y = bbox_a.min_y.min(shifted_b_bbox.min_y);
                let c_max_x = bbox_a.max_x.max(shifted_b_bbox.max_x);
                let c_max_y = bbox_a.max_y.max(shifted_b_bbox.max_y);
                let c_bbox = Rect::new(c_min_x, c_min_y, c_max_x, c_max_y);
                let saving = (separate_bbox_area - c_bbox.area()) / separate_bbox_area;

                if saving > 0.10 && saving > best_saving {
                    best_saving = saving;
                    best_offset_x = ox - c_min_x;
                    best_offset_y = oy - c_min_y;
                    best_rot_b = rot_b;
                    best_combined_bbox = c_bbox;
                }
            }
        }
    }

    if best_saving > 0.10 {
        let child_a = ClusterChild {
            entity_id: part_a.entity_id.clone(),
            id: part_a.id.clone(),
            offset_x: Some(0.0),
            offset_y: Some(0.0),
            rotation_degrees: Some(0.0),
            name: part_a.name.clone(),
            width: part_a.width,
            height: part_a.height,
            area: part_a.area,
            contour: Some(part_a.contour.clone()),
            holes: part_a.holes.clone(),
            color: part_a.color.clone(),
        };
        let child_b = ClusterChild {
            entity_id: part_b.entity_id.clone(),
            id: part_b.id.clone(),
            offset_x: Some(best_offset_x),
            offset_y: Some(best_offset_y),
            rotation_degrees: Some(best_rot_b),
            name: part_b.name.clone(),
            width: part_b.width,
            height: part_b.height,
            area: part_b.area,
            contour: Some(part_b.contour.clone()),
            holes: part_b.holes.clone(),
            color: part_b.color.clone(),
        };

        let macro_w = best_combined_bbox.width();
        let macro_h = best_combined_bbox.height();
        let macro_contour = vec![
            [0.0, 0.0],
            [macro_w, 0.0],
            [macro_w, macro_h],
            [0.0, macro_h],
        ];

        let macro_part = PartInput {
            id: Some(format!("macro_{}_{}", part_a.id.clone().unwrap_or_default(), part_b.id.clone().unwrap_or_default())),
            entity_id: Some(format!("macro_{}_{}", part_a.entity_id.clone().unwrap_or_default(), part_b.entity_id.clone().unwrap_or_default())),
            name: Some(format!("Macro: {} + {}", part_a.name.clone().unwrap_or_default(), part_b.name.clone().unwrap_or_default())),
            width: Some(macro_w),
            height: Some(macro_h),
            area: Some(part_a.area.unwrap_or(0.0) + part_b.area.unwrap_or(0.0)),
            contour: macro_contour,
            holes: None,
            rotations: part_a.rotations.clone(),
            base_rotation_degrees: Some(0.0),
            rotation_degrees: Some(0.0),
            grain_locked: part_a.grain_locked,
            has_grain_label: part_a.has_grain_label,
            grain_arrow_degrees: part_a.grain_arrow_degrees,
            free_rotation: part_a.free_rotation,
            rotation_divisions: part_a.rotation_divisions,
            color: part_a.color.clone(),
            logical_part_count: Some(2),
            manual_cluster_macro: Some(true),
            manual_cluster_children: Some(vec![child_a, child_b]),
            two_sided: part_a.two_sided,
            special_shape: Some(false),
            draw_layers: None,
            source_entity_ids: None,
        };

        return Some((macro_part, best_saving));
    }

    None
}
