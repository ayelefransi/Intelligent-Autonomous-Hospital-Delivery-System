# ============================================================
# Hospital Robot Docker Launcher (Windows PowerShell → WSL2)
# ============================================================
# Runs the pre-built hospital-robot Docker image via WSL2 with
# WSLg GPU/display passthrough.
#
# Usage:
#   .\launch_wsl.ps1                           # SLAM mode (headless)
#   .\launch_wsl.ps1 -Mode slam -RViz          # SLAM + RViz2
#   .\launch_wsl.ps1 -Mode nav -Map ~/hospital_map.yaml
#   .\launch_wsl.ps1 -Mode multi               # 3-robot fleet
#   .\launch_wsl.ps1 -Build                    # rebuild image first
# ============================================================

param(
    [ValidateSet("slam","nav","multi")]
    [string]$Mode  = "slam",
    [switch]$RViz  = $false,
    [switch]$Build = $false,
    [string]$Map   = "",
    [string]$WslDistro = "Ubuntu"
)

$ErrorActionPreference = "Stop"

Write-Host ""
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "  Hospital Robot Docker Launcher (WSL2/WSLg)" -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host ""

# ── 1. Check WSL is running ─────────────────────────────────
Write-Host "[1/4] Checking WSL2..." -ForegroundColor Yellow
$wslCheck = wsl -d $WslDistro -- bash -c "echo OK" 2>&1
if ($wslCheck -notmatch "OK") {
    Write-Error "WSL distribution '$WslDistro' is not available. Run: wsl --install -d Ubuntu"
}
Write-Host "      WSL2 ($WslDistro) is running." -ForegroundColor Green

# ── 2. Check Docker image ──────────────────────────────────
Write-Host "[2/4] Checking hospital-robot Docker image..." -ForegroundColor Yellow
$imgCheck = wsl -d $WslDistro -- bash -c "docker images --format '{{.Repository}}:{{.Tag}}' 2>/dev/null | grep hospital-robot" 2>&1
if ($Build -or ($imgCheck -notmatch "hospital-robot")) {
    Write-Host "      Building hospital-robot image (this takes ~10 min first time)..." -ForegroundColor Magenta
    $winPath = "/mnt/c/Users/Hello/Music/hospital_robot_ws"
    wsl -d $WslDistro -- bash -c @"
cd $winPath && docker build -f Dockerfile -t hospital-robot ./hospital_ws/
"@
    if ($LASTEXITCODE -ne 0) { Write-Error "Docker build failed." }
    Write-Host "      Image built successfully." -ForegroundColor Green
} else {
    Write-Host "      Image 'hospital-robot:latest' found." -ForegroundColor Green
}

# ── 3. Build the launch command ─────────────────────────────
Write-Host "[3/4] Preparing launch command ($Mode mode)..." -ForegroundColor Yellow

$useRviz    = if ($RViz)  { "true" }  else { "false" }
$useExplore = if ($Mode -eq "slam") { "true" } else { "false" }

$rosCmd = switch ($Mode) {
    "slam"  { "ros2 launch hospital_robot hospital_slam.launch.py use_rviz:=$useRviz use_explore:=$useExplore" }
    "nav"   {
        if ($Map -eq "") { $Map = "\$HOME/hospital_map.yaml" }
        "ros2 launch hospital_robot hospital_nav.launch.py map:=$Map use_rviz:=$useRviz"
    }
    "multi" { "ros2 launch hospital_robot hospital_multi.launch.py" }
}

Write-Host "      ROS command: $rosCmd" -ForegroundColor White

# ── 4. Run Docker container via WSL ─────────────────────────
Write-Host "[4/4] Launching Docker container..." -ForegroundColor Yellow
Write-Host "      GUI will appear via WSLg. Press Ctrl+C to stop." -ForegroundColor White
Write-Host ""

$dockerRunCmd = @"
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
docker run -it --rm \
    --name hospital-robot-slam \
    --privileged \
    --network host \
    -e DISPLAY=\$DISPLAY \
    -e WAYLAND_DISPLAY=\$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=\$XDG_RUNTIME_DIR \
    -e LIBGL_ALWAYS_SOFTWARE=0 \
    -e TURTLEBOT3_MODEL=waffle \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /mnt/wslg:/mnt/wslg:ro \
    -v /usr/lib/wsl:/usr/lib/wsl:ro \
    -v /mnt/c/Users/Hello/Music/hospital_robot_ws/hospital_ws:/home/robot/hospital_ws_src:ro \
    --device=/dev/dri \
    hospital-robot \
    bash -c "source /opt/ros/jazzy/setup.bash && source ~/hospital_ws/install/setup.bash && $rosCmd"
"@

wsl -d $WslDistro -- bash -c $dockerRunCmd
