@echo off
call "C:\Users\Alexs\esp\v5.4.1\esp-idf\export.bat"
cd /d "C:\Users\Alexs\Documents\GitHub\ESP32_Logger\component_test"
idf.py menuconfig
pause
