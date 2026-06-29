@echo off
set DEVICE=%1
if "%DEVICE%"=="" set DEVICE=192.168.35.202:44861
echo [1/4] Building APK...
cmd /c flutter build apk --debug
if errorlevel 1 echo BUILD FAILED && pause && exit /b 1
echo [2/4] Copying APK...
if not exist C:\temp mkdir C:\temp
copy /y "build\app\outputs\flutter-apk\app-debug.apk" "C:\temp\app-debug.apk"
echo [3/4] Uninstalling old version...
adb -s %DEVICE% uninstall com.wemeettrip.arcore_streamer
echo [4/4] Installing new version...
adb -s %DEVICE% install --no-streaming "C:\temp\app-debug.apk"
if errorlevel 1 echo INSTALL FAILED && pause && exit /b 1
echo === SUCCESS. Tap app icon on S24. ===
pause
