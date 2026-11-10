// src/main.rs
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, Instant};

mod geometry;
mod types;
mod nfp;
mod macro_comb;
mod scoring;
mod packing;
mod epochs;
mod svg;

use types::{JobInput, LayoutOutput, PreviewOutput, ProgressOutput, ResultOutput};

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

fn write_file_atomic(target_path: &Path, content: &str) {
    let tmp_path = target_path.with_extension("tmp");
    if let Ok(mut f) = File::create(&tmp_path) {
        let _ = f.write_all(content.as_bytes());
        let _ = f.flush();
        drop(f);
        let _ = std::fs::rename(&tmp_path, target_path);
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();

    let job_path = parse_arg(&args, "--job").expect("Missing --job parameter");
    let result_path = PathBuf::from(parse_arg(&args, "--result").expect("Missing --result parameter"));
    let progress_path = parse_arg(&args, "--progress").map(PathBuf::from);
    let preview_path = parse_arg(&args, "--preview").map(PathBuf::from);
    let svg_path = parse_arg(&args, "--svg").map(PathBuf::from);
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

    // 1. INSTANT ZERO-DELAY FIRST PREVIEW: Generate initial layout in < 20ms and push immediately to UI
    let total_materials = job_input.materials.len();
    let mut current_layouts: Vec<LayoutOutput> = Vec::with_capacity(total_materials);

    let total_parts_in_job: usize = job_input.materials.iter().map(|m| m.parts.len()).sum();

    for material_state in &job_input.materials {
        let initial_fast = packing::pack_material(material_state, 0);
        current_layouts.push(initial_fast);
    }

    let default_board_width = current_layouts.first().map(|l| l.board_width).unwrap_or(2440.0);
    let default_board_height = current_layouts.first().map(|l| l.board_height).unwrap_or(1220.0);

    if let Some(ref prev_path) = preview_path {
        write_json_atomic(
            prev_path,
            &PreviewOutput {
                ok: true,
                partial: true,
                layouts: current_layouts.clone(),
            },
        );
    }

    if let Some(ref p_path) = progress_path {
        write_json_atomic(
            p_path,
            &ProgressOutput {
                running: true,
                overall_percent: 10.0,
                percent: 10.0,
                optimization_iteration: 1,
                optimization_total: 10,
                stage_name: "Đang Xếp tấm lớn...".to_string(),
                message: "Đã xuất sơ đồ ban đầu; đang tối ưu và dồn chi tiết thời gian thực...".to_string(),
                completed_parts: total_parts_in_job / 4,
                total_parts: total_parts_in_job,
            },
        );
    }

    // Small yield so Ruby immediately captures frame 0 within 50ms
    thread::sleep(Duration::from_millis(20));

    // 2. REAL-TIME PROGRESSIVE STREAMING & OPTIMIZATION
    for (mat_idx, material_state) in job_input.materials.iter().enumerate() {
        if is_cancelled(cancel_path.as_ref()) {
            let cancel_res = ResultOutput {
                ok: false,
                layouts: vec![],
                population_total: 0,
                message: "Đã dừng tiến trình Nesting.".to_string(),
                cancelled: true,
                board_width: Some(default_board_width),
                board_height: Some(default_board_height),
            };
            write_json_atomic(&result_path, &cancel_res);
            return;
        }

        let p_path_clone = progress_path.clone();
        let prev_path_clone = preview_path.clone();
        let mut working_layouts = current_layouts.clone();

        let final_mat_layout = epochs::optimize_material_layout(material_state, |step, layout_variant| {
            working_layouts[mat_idx] = layout_variant.clone();

            let base_pct = ((mat_idx as f64) / (total_materials as f64)) * 90.0;
            let step_pct = (step as f64 / 10.0) * (90.0 / total_materials as f64);
            let pct = (base_pct + step_pct).min(95.0);

            if let Some(ref prev_path) = prev_path_clone {
                write_json_atomic(
                    prev_path,
                    &PreviewOutput {
                        ok: true,
                        partial: true,
                        layouts: working_layouts.clone(),
                    },
                );
            }

            let stage_name = if pct < 30.0 {
                "Đang Xếp tấm lớn..."
            } else if pct < 60.0 {
                "Đang Xếp tấm Dị hình..."
            } else if pct < 85.0 {
                "Đang Lồng Tấm & tối ưu vùng ưu tiên..."
            } else {
                "Đang Hậu Cần dồn ván..."
            };

            if let Some(ref p_path) = p_path_clone {
                write_json_atomic(
                    p_path,
                    &ProgressOutput {
                        running: true,
                        overall_percent: pct,
                        percent: pct,
                        optimization_iteration: step,
                        optimization_total: 10,
                        stage_name: stage_name.to_string(),
                        message: format!(
                            "Đang tối ưu & nén sơ đồ ván (bước {}/10, {:.2}s)...",
                            step,
                            start_time.elapsed().as_secs_f64()
                        ),
                        completed_parts: ((total_parts_in_job as f64) * (pct / 100.0)).round() as usize,
                        total_parts: total_parts_in_job,
                    },
                );
            }

            thread::sleep(Duration::from_millis(15));
        });

        current_layouts[mat_idx] = final_mat_layout;
    }

    let elapsed = start_time.elapsed().as_secs_f64();
    let total_sheets: usize = current_layouts.iter().map(|l| l.sheets.len()).sum();
    let total_parts: usize = current_layouts.iter().map(|l| l.sheets.iter().map(|s| s.placements.len()).sum::<usize>()).sum();

    let final_message = format!(
        "Nesting hoàn thành bằng Rust Engine: {} tấm, {} chi tiết trong {:.2} giây.",
        total_sheets,
        total_parts,
        elapsed
    );

    let result = ResultOutput {
        ok: true,
        layouts: current_layouts,
        population_total: 10,
        message: final_message,
        cancelled: false,
        board_width: Some(default_board_width),
        board_height: Some(default_board_height),
    };

    write_json_atomic(&result_path, &result);

    if let Some(ref s_path) = svg_path {
        let svg_content = svg::generate_result_svg(&result);
        write_file_atomic(s_path, &svg_content);
    }
}
