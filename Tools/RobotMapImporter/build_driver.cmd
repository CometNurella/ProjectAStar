@echo off
call "%MAPIMPORT_VSDEV%"
if errorlevel 1 exit /b %errorlevel%
"%MAPIMPORT_CMAKE%" -S "%MAPIMPORT_SOURCE%" -B "%MAPIMPORT_BUILD%" -G Ninja "-DCMAKE_MAKE_PROGRAM=%MAPIMPORT_NINJA%" "-DCMAKE_TOOLCHAIN_FILE=%MAPIMPORT_TOOLCHAIN%" -DVCPKG_TARGET_TRIPLET=x64-windows -DCMAKE_BUILD_TYPE=Release "-DBUILD_TESTING=%MAPIMPORT_TESTING%"
if errorlevel 1 exit /b %errorlevel%
"%MAPIMPORT_CMAKE%" --build "%MAPIMPORT_BUILD%" --parallel
exit /b %errorlevel%
