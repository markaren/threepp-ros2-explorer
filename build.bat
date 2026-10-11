@echo off
rem Build the workspace with colcon.
rem
rem   pixi run -e jazzy build                                    everything
rem   pixi run -e jazzy build --packages-select explorer_sim     one package
rem
rem Why a script and not just "colcon build"? On Windows, colcon refuses to start unless
rem the Visual Studio compiler environment is loaded (cl.exe on PATH, VisualStudioVersion
rem set). A plain terminal does not have it. This script finds Visual Studio, loads its
rem environment, and then runs colcon, so the build works from any terminal. If you are
rem already inside a "x64 Native Tools Command Prompt", or CLion started you with its
rem Visual Studio toolchain, the first step is skipped.

setlocal

if not defined CONDA_PREFIX goto :not_in_pixi
if defined VisualStudioVersion goto :build

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto :no_vs
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%i"
if not defined VSROOT goto :no_vs

rem VsDevCmd itself looks for vswhere.exe on PATH and prints an error if it is missing.
set "PATH=%PATH%;%ProgramFiles(x86)%\Microsoft Visual Studio\Installer"
call "%VSROOT%\Common7\Tools\VsDevCmd.bat" -arch=amd64 -host_arch=amd64 -no_logo
rem Visual Studio ships its own cmake and ninja and has just put them on PATH. Put the
rem pixi environment back in front, so the build uses one set of tools: the environment's.
set "PATH=%CONDA_PREFIX%\Library\bin;%CONDA_PREFIX%\Scripts;%CONDA_PREFIX%;%PATH%"

:build
rem --base-paths src   look for packages in src/ only (not in .pixi/ or build/)
rem --merge-install    one shared install/ tree. Windows copes badly with the default,
rem                    which is one install folder per package and long PATHs.
rem Release matters: the ROS libraries are release builds, and MSVC cannot mix debug and
rem release C++ runtimes in one program.
colcon build --base-paths src --merge-install --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release %*
exit /b %ERRORLEVEL%

:not_in_pixi
echo This script must run inside the pixi environment:  pixi run -e jazzy build
exit /b 1

:no_vs
echo Visual Studio 2022 with the "Desktop development with C++" workload was not found.
exit /b 1
