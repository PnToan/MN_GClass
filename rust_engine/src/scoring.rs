// src/scoring.rs
use crate::types::LayoutOutput;

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct LayoutScore {
    pub sheet_count: usize,
    pub underfilled_sheets: usize,
    pub target_deficit: f64,
    pub waste_area: f64,
    pub front_load_score: f64,
    pub compact_area: f64,
    pub last_sheet_compact_area: f64,
    pub macro_pair_count: isize, // negative so more pairs = better score
}

impl LayoutScore {
    pub fn compute(layout: &LayoutOutput) -> Self {
        let sheet_count = layout.sheets.len();
        let board_area = layout.board_width * layout.board_height;
        let target_util = layout.target_sheet_utilization.max(80.0);

        let mut underfilled = 0;
        let mut target_deficit = 0.0;
        let mut front_load = 0.0;
        let mut total_compact_area = 0.0;
        let mut total_used_area = 0.0;

        for (idx, sheet) in layout.sheets.iter().enumerate() {
            let used_area: f64 = sheet.placements.iter().map(|p| p.area).sum();
            total_used_area += used_area;
            let util_pct = if board_area > 0.0 { (used_area / board_area) * 100.0 } else { 0.0 };

            if idx < sheet_count.saturating_sub(1) {
                if util_pct < 80.0 {
                    underfilled += 1;
                }
                let deficit = (target_util - util_pct).max(0.0);
                target_deficit += deficit * (sheet_count - idx) as f64;
            }

            front_load += idx as f64 * used_area;

            if !sheet.placements.is_empty() {
                let max_x = sheet.placements.iter().map(|p| p.x + p.packed_width).fold(0.0, f64::max);
                let max_y = sheet.placements.iter().map(|p| p.y + p.packed_height).fold(0.0, f64::max);
                let c_area = (max_x - layout.edge_margin).max(0.0) * (max_y - layout.edge_margin).max(0.0);
                total_compact_area += c_area;
            }
        }

        let total_board_area = sheet_count as f64 * board_area;
        let waste_area = (total_board_area - total_used_area).max(0.0);

        let last_sheet_compact = if let Some(last) = layout.sheets.last() {
            if !last.placements.is_empty() {
                let max_x = last.placements.iter().map(|p| p.x + p.packed_width).fold(0.0, f64::max);
                let max_y = last.placements.iter().map(|p| p.y + p.packed_height).fold(0.0, f64::max);
                (max_x - layout.edge_margin).max(0.0) * (max_y - layout.edge_margin).max(0.0)
            } else {
                0.0
            }
        } else {
            0.0
        };

        LayoutScore {
            sheet_count,
            underfilled_sheets: underfilled,
            target_deficit,
            waste_area,
            front_load_score: front_load,
            compact_area: total_compact_area,
            last_sheet_compact_area: last_sheet_compact,
            macro_pair_count: -(layout.comb_pair_placements as isize),
        }
    }
}

