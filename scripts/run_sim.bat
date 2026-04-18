@echo off
rem Launch the full park_bot simulation on Windows.
setlocal

pushd "%~dp0\..\..\"
if not exist "install\setup.bat" (
  echo error: workspace not built. Run scripts\build.bat first. 1>&2
  popd & exit /b 1
)

call install\setup.bat
start "park_bot sim" ros2 launch park_bot sim.launch.py
rviz2 -d src\park_bot\config\main.rviz
popd
endlocal
