// src/main.rs
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

mod geometry;
mod types;
mod nfp;
mod macro_comb;
mod scoring;
mod packing;
mod epochs;

use types::{JobInput, ProgressOutput, ResultOutput, LayoutOutput, PreviewOutput};

fn parse_arg(args: &[String], flag: &str) -> Option<String> {
    for i in 0..args.len() {
        if args[i] == flag && i + 1 < args.len() {
            return Some(args[i + 1].clone());
        }
    }
    None
}

fn is_cancelled(cancel_path: Option<&PathBuf>) -> bool {
    if let Some(path) = cancel_path {
        path.exists()
    } else {
        false
    }
}

fn write_json_atomic<T: serde::Serialize>(target_path: &Path, data: &T) {
    let tmp_path = target_path.with_extension("tmp");
    if let Ok(json_str) = serde_json::to_string(data) {
        if let Ok(mut f) = File::create(&tmp_path) {
            let _ = f.write_all(json_str.as_bytes());
            let _ = f.flush();
            drop(f);
            let _ = std::fs::rename(&tmp_path, target_path);
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();

    let job_path = parse_arg(&args, "--job").expect("Missing --job parameter");
    let result_path = PathBuf::from(parse_arg(&args, "--result").expect("Missing --result parameter"));
    let progress_path = parse_arg(&args, "--progress").map(PathBuf::from);
    let preview_path = parse_arg(&args, "--preview").map(PathBuf::from);
    let cancel_path = parse_arg(&args, "--cancel").map(PathBuf::from);

    let workers_count = parse_arg(&args, "--workers")
        .and_then(|w| w.parse::<usize>().ok())
        .unwrap_or_else(|| std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4));

    rayon::ThreadPoolBuilder::new()
        .num_threads(workers_count)
        .build_global()
        .unwrap_or_default();

    let raw_content = std::fs::read_to_string(&job_path).expect("Cannot read job file");
    let content = raw_content.trim_start_matches('\u{feff}');
    let job_input: JobInput = serde_json::from_str(content).expect("Cannot parse job JSON");

    let start_time = Instant::now();
    let total_materials = job_input.materials.len();
    let mut current_layouts: Vec<LayoutOutput> = Vec::with_capacity(total_materials);

    // 1. FAST INITIAL PROGRESS
    if let Some(ref p_path) = progress_path {
        write_json_atomic(
            p_path,
            &ProgressOutput {
                running: true,
                overall_percent: 15.0,
                optimization_iteration: 1,
                optimization_total: 10,
                message: "Đang nén và tối ưu hóa diện tích phôi...".to_string(),
            },
        );
    }

    // 2. MAX-SPEED MULTI-THREADED PARALLEL OPTIMIZATION
    for (mat_idx, material_state) in job_input.materials.iter().enumerate() {
        if is_cancelled(cancel_path.as_ref()) {
            let cancel_res = ResultOutput {
                ok: false,
                layouts: vec![],
                population_total: 0,
                message: "Đã dừng tiến trình Nesting.".to_string(),
                cancelled: true,
            };
            write_json_atomic(&result_path, &cancel_res);
            return;
        }

        let p_path_clone = progress_path.clone();
        let prev_path_clone = preview_path.clone();
        let prev_layouts_base = current_layouts.clone();

        let final_mat_layout = epochs::optimize_material_layout(material_state, |phase, local_pct, iter, total, layout_variant| {
            let base_pct = ((mat_idx as f64) / (total_materials as f64)) * 80.0 + 15.0;
            let mat_slice = 80.0 / (total_materials as f64);
            let pct = (base_pct + (local_pct / 100.0) * mat_slice).clamp(15.0, 95.0);

            if let Some(ref prev_path) = prev_path_clone {
                let mut prev_layouts = prev_layouts_base.clone();
                prev_layouts.push(layout_variant.clone());
                write_json_atomic(
                    prev_path,
                    &PreviewOutput {
                        ok: true,
                        partial: true,
                        layouts: prev_layouts,
                    },
                );
            }

            if let Some(ref p_path) = p_path_clone {
                let cur_elapsed = start_time.elapsed().as_secs_f64();
                let time_str = if cur_elapsed >= 60.0 {
                    let m = (cur_elapsed / 60.0).floor() as u64;
                    let rem = cur_elapsed % 60.0;
                    format!("{}p {:.1}s", m, rem)
                } else {
                    format!("{:.2}s", cur_elapsed)
                };
                let iter_str = if total > 0 {
                    format!("chiến lược {}/{}", iter, total)
                } else {
                    format!("tấm {}", iter)
                };
                write_json_atomic(
                    p_path,
                    &ProgressOutput {
                        running: true,
                        overall_percent: pct,
                        optimization_iteration: iter,
                        optimization_total: total,
                        message: format!(
                            "Đang {} ({}, {})...",
                            phase,
                            iter_str,
                            time_str
                        ),
                    },
                );
            }
        });

        current_layouts.push(final_mat_layout);
    }

    let elapsed = start_time.elapsed().as_secs_f64();
    let total_sheets: usize = current_layouts.iter().map(|l| l.sheets.len()).sum();
    let total_parts: usize = current_layouts.iter().map(|l| l.sheets.iter().map(|s| s.placements.len()).sum::<usize>()).sum();

    let time_str = if elapsed >= 60.0 {
        let m = (elapsed / 60.0).floor() as u64;
        let rem = elapsed % 60.0;
        format!("{}p {:.1}s", m, rem)
    } else {
        format!("{:.2} giây", elapsed)
    };

    let final_message = format!(
        "Nesting hoàn thành bằng Rust Engine: {} tấm, {} chi tiết trong {}.",
        total_sheets,
        total_parts,
        time_str
    );

    let result = ResultOutput {
        ok: true,
        layouts: current_layouts,
        population_total: 10,
        message: final_message,
        cancelled: false,
    };

    write_json_atomic(&result_path, &result);
}
