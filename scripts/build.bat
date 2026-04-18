@echo off
rem Build the park_bot colcon workspace.
setlocal

pushd "%~dp0\..\..\"
where colcon >nul 2>&1
if errorlevel 1 (
  echo error: colcon not found. Source ROS2 Humble and re-run. 1>&2
  popd & exit /b 1
)

colcon build --symlink-install %*
if errorlevel 1 (
  popd & exit /b %errorlevel%
)

echo.
echo [ok] build complete -- call install\setup.bat before running.
popd
endlocal
