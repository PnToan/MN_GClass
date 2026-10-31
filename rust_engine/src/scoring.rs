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

pub fn compute_room_dispersion(layout: &LayoutOutput) -> (usize, usize, i64) {
    use std::collections::{BTreeMap, BTreeSet};

    let sheet_count = layout.sheets.len();
    let mut mixed_leading_penalty = 0usize;
    let mut sheets_per_room: BTreeMap<String, BTreeSet<usize>> = BTreeMap::new();
    let mut intra_sheet_span_mm: i64 = 0;

    for (s_idx, sheet) in layout.sheets.iter().enumerate() {
        let mut room_boxes: BTreeMap<String, (f64, f64, f64, f64, usize)> = BTreeMap::new();
        for p in &sheet.placements {
            let r = crate::types::get_placement_room_key(p);
            if r.is_empty() {
                continue;
            }
            sheets_per_room.entry(r.clone()).or_default().insert(s_idx);
            let cx = p.x + p.packed_width * 0.5;
            let cy = p.y + p.packed_height * 0.5;
            let entry = room_boxes.entry(r).or_insert((cx, cx, cy, cy, 0));
            entry.0 = entry.0.min(cx);
            entry.1 = entry.1.max(cx);
            entry.2 = entry.2.min(cy);
            entry.3 = entry.3.max(cy);
            entry.4 += 1;
        }
        if s_idx < sheet_count.saturating_sub(1) && room_boxes.len() > 1 {
            mixed_leading_penalty += (room_boxes.len() - 1) * 2;
        } else if room_boxes.len() > 1 {
            mixed_leading_penalty += room_boxes.len() - 1;
        }
        for (_r, (min_x, max_x, min_y, max_y, count)) in room_boxes {
            if count >= 2 {
                intra_sheet_span_mm += ((max_x - min_x) + (max_y - min_y)).round() as i64;
            }
        }
    }

    let room_sheet_splits: usize = sheets_per_room
        .values()
        .map(|set| set.len().saturating_sub(1))
        .sum();

    (room_sheet_splits * 2 + mixed_leading_penalty, room_sheet_splits, intra_sheet_span_mm)
}

pub fn effective_room_underfilled(layout: &LayoutOutput) -> usize {
    use std::collections::BTreeSet;
    let sheet_count = layout.sheets.len();
    let board_area = layout.board_width * layout.board_height;
    let mut underfilled = 0usize;

    for (idx, sheet) in layout.sheets.iter().enumerate() {
        if idx >= sheet_count.saturating_sub(1) {
            break;
        }
        let used_area: f64 = sheet.placements.iter().map(|p| p.area).sum();
        let util_pct = if board_area > 0.0 { (used_area / board_area) * 100.0 } else { 0.0 };
        if util_pct < 80.0 {
            let mut rooms = BTreeSet::new();
            let mut has_unassigned = false;
            for p in &sheet.placements {
                let r = crate::types::get_placement_room_key(p);
                if r.is_empty() {
                    has_unassigned = true;
                } else {
                    rooms.insert(r);
                }
            }
            let is_pure_single_room = rooms.len() == 1 && !has_unassigned;
            if !(is_pure_single_room && util_pct >= 62.0) {
                underfilled += 1;
            }
        }
    }
    underfilled
}

pub fn is_room_candidate_better(
    cand: &LayoutOutput,
    current_best: &LayoutOutput,
    baseline_sheets: usize,
    baseline_underfilled: usize,
) -> bool {
    if cand.sheets.is_empty() || cand.sheets.len() > baseline_sheets {
        return false;
    }
    let cand_score = LayoutScore::compute(cand);
    let cand_eff_uf = effective_room_underfilled(cand);
    if cand.sheets.len() == baseline_sheets && cand_eff_uf > baseline_underfilled {
        return false;
    }

    if cand.sheets.len() != current_best.sheets.len() {
        return cand.sheets.len() < current_best.sheets.len();
    }
    let best_score = LayoutScore::compute(current_best);
    let best_eff_uf = effective_room_underfilled(current_best);
    if cand_eff_uf != best_eff_uf {
        return cand_eff_uf < best_eff_uf;
    }

    let disp_cand = compute_room_dispersion(cand);
    let disp_best = compute_room_dispersion(current_best);
    if disp_cand != disp_best {
        return disp_cand < disp_best;
    }

    if cand_score.underfilled_sheets != best_score.underfilled_sheets {
        return cand_score.underfilled_sheets < best_score.underfilled_sheets;
    }

    cand_score < best_score
}


