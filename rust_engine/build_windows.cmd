@echo off
setlocal
cd /d "%~dp0"

where cargo >nul 2>nul
if errorlevel 1 (
  echo ERROR: cargo not found. Install Rust toolchain first.
  exit /b 1
)

where rustup >nul 2>nul
if not errorlevel 1 (
  rustup target add x86_64-pc-windows-gnu
  if errorlevel 1 exit /b 1
)

cargo build --release --target x86_64-pc-windows-gnu
if errorlevel 1 exit /b 1

copy /Y "target\x86_64-pc-windows-gnu\release\mn_nesting_engine.exe" "..\mn_nesting_engine.exe" >nul
if errorlevel 1 exit /b 1

echo OK: ..\mn_nesting_engine.exe
exit /b 0
