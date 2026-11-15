#!/usr/bin/env bash
set -euo pipefail

# Di chuyển về thư mục chứa script
cd "$(dirname "$0")"

echo "==> [1/5] Kiểm tra Cargo và Rustup..."
if ! command -v cargo >/dev/null 2>&1; then
  echo "LỖI: Chưa cài đặt Rust. Hãy cài đặt tại: https://rustup.rs"
  exit 1
fi

# Đảm bảo tương thích toàn bộ thế hệ macOS chạy Apple Silicon M1–M6 và Intel Mac
export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-11.0}"

echo "==> [2/5] Thêm target cho Apple Silicon M1–M6 (aarch64) và Intel Mac (x86_64)..."
rustup target add aarch64-apple-darwin x86_64-apple-darwin

echo "==> [3/5] Biên dịch bản Apple Silicon (M1–M6 / arm64)..."
cargo build --release --target aarch64-apple-darwin

echo "==> [4/5] Biên dịch bản Intel Mac (x86_64)..."
cargo build --release --target x86_64-apple-darwin

echo "==> [5/5] Đóng gói Universal Binary (M1–M6 + Intel) bằng lipo & ký ad-hoc..."
lipo -create \
  target/aarch64-apple-darwin/release/mn_nesting_engine \
  target/x86_64-apple-darwin/release/mn_nesting_engine \
  -output ../mn_nesting_engine

chmod +x ../mn_nesting_engine
xattr -cr ../mn_nesting_engine 2>/dev/null || true
codesign --force --sign - ../mn_nesting_engine 2>/dev/null || true

echo "==> HOÀN TẤT THÀNH CÔNG (Apple Silicon M1–M6 & Intel Mac): ../mn_nesting_engine"
file ../mn_nesting_engine
