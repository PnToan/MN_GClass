// src/svg.rs
use crate::types::{PlacementOutput, ResultOutput};

fn escape_xml(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

fn svg_path(placement: &PlacementOutput) -> String {
    let mut polygons = vec![placement.contour.clone()];
    if let Some(ref holes) = placement.holes {
        polygons.extend(holes.clone());
    }

    let mut path_d = String::new();
    for poly in polygons {
        if poly.len() < 3 {
            continue;
        }
        let first = poly[0];
        path_d.push_str(&format!("M {} {} ", first[0], first[1]));
        for pt in &poly[1..] {
            path_d.push_str(&format!("L {} {} ", pt[0], pt[1]));
        }
        path_d.push_str("Z ");
    }

    path_d.trim().to_string()
}

pub fn generate_result_svg(result: &ResultOutput) -> String {
    let padding = 30.0;
    let mut cursor_x = padding;
    let mut cursor_y = padding;
    let mut row_height = 0.0f64;
    let mut maximum_width = 0.0f64;
    let mut blocks = Vec::new();

    for layout in &result.layouts {
        for sheet in &layout.sheets {
            if cursor_x > padding && cursor_x + layout.board_width + padding > 10000.0 {
                cursor_x = padding;
                cursor_y += row_height + padding;
                row_height = 0.0;
            }

            let mut shapes = String::new();
            for placement in &sheet.placements {
                let entity_id = escape_xml(&placement.entity_id);
                let part_id = escape_xml(&placement.id);
                let rotation = placement.rotation_degrees;
                let color = placement.color.as_deref().unwrap_or("#d6d6d6");
                let d = svg_path(placement);

                shapes.push_str(&format!(
                    r##"<path data-entity-id="{}" data-part-id="{}" data-rotation="{}" transform="translate({} {})" d="{}" fill="{}" stroke="#20262c" stroke-width="0.4" fill-rule="evenodd"/>"##,
                    entity_id, part_id, rotation, placement.x, placement.y, d, color
                ));
            }

            let material = escape_xml(&layout.material);
            let thickness = layout.thickness;
            let sheet_index = sheet.index;

            blocks.push(format!(
                r##"<g data-material="{}" data-thickness="{}" data-sheet="{}" transform="translate({} {})"><rect x="0" y="0" width="{}" height="{}" fill="#fff" stroke="#333"/>{}</g>"##,
                material, thickness, sheet_index, cursor_x, cursor_y, layout.board_width, layout.board_height, shapes
            ));

            cursor_x += layout.board_width + padding;
            row_height = row_height.max(layout.board_height);
            maximum_width = maximum_width.max(cursor_x);
        }
    }

    let height = cursor_y + row_height + padding;
    let final_w = if maximum_width > 0.0 { maximum_width } else { 100.0 };
    let final_h = if height > 0.0 { height } else { 100.0 };

    format!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}\" height=\"{}\" viewBox=\"0 0 {} {}\">{}</svg>\n",
        final_w, final_h, final_w, final_h, blocks.join("")
    )
}
