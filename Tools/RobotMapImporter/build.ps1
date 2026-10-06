param(
    [string]$VcpkgRoot = 'D:\vcpkg',
    [switch]$RunTests
)
$ErrorActionPreference = 'Stop'
$sourceDir = $PSScriptRoot
$buildDir = Join-Path $sourceDir 'build\Release'
$vswherePath = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path -LiteralPath $vswherePath)) {
    throw 'Visual Studio C++ tools were not found. Install Desktop development with C++ and a Windows 10/11 SDK.'
}
$installation = (& $vswherePath -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -format json | ConvertFrom-Json)[0]
if (-not $installation) { throw 'Install the Visual Studio Desktop development with C++ workload.' }
$cmakeExe = Join-Path $installation.installationPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
$ctestExe = Join-Path $installation.installationPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ctest.exe'
$ninjaExe = Join-Path $installation.installationPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe'
$vsdevPath = Join-Path $installation.installationPath 'VC\Auxiliary\Build\vcvars64.bat'
$toolchain = Join-Path $VcpkgRoot 'scripts\buildsystems\vcpkg.cmake'
if (-not (Test-Path -LiteralPath $toolchain)) { throw "vcpkg was not found at $VcpkgRoot. Supply -VcpkgRoot with its installed location." }
if (-not (Test-Path -LiteralPath $cmakeExe)) { throw 'Install the Visual Studio C++ CMake tools component.' }
if (-not (Test-Path -LiteralPath $ninjaExe)) { throw 'Install the Visual Studio C++ CMake tools component (including Ninja).' }
$env:MAPIMPORT_SOURCE = $sourceDir
$env:MAPIMPORT_BUILD = $buildDir
$env:MAPIMPORT_CMAKE = $cmakeExe
$env:MAPIMPORT_NINJA = $ninjaExe
$env:MAPIMPORT_VSDEV = $vsdevPath
$env:MAPIMPORT_TOOLCHAIN = $toolchain
$env:MAPIMPORT_TESTING = if ($RunTests) { 'ON' } else { 'OFF' }
try {
    & $env:ComSpec /d /c (Join-Path $sourceDir 'build_driver.cmd')
    if ($LASTEXITCODE -ne 0) { throw 'CMake configuration or C++ build failed. See the output and dependency instructions in README.md.' }
    if ($RunTests) {
        & $ctestExe --test-dir $buildDir --output-on-failure
        if ($LASTEXITCODE -ne 0) { throw 'Geometry tests failed.' }
    }
} finally {
    'MAPIMPORT_SOURCE','MAPIMPORT_BUILD','MAPIMPORT_CMAKE','MAPIMPORT_NINJA','MAPIMPORT_VSDEV','MAPIMPORT_TOOLCHAIN','MAPIMPORT_TESTING' | ForEach-Object { Remove-Item -LiteralPath "Env:\$_" -ErrorAction SilentlyContinue }
}
Write-Host "Ready: $(Join-Path $buildDir 'robot_map_importer.exe')"
