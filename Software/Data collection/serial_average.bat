@echo off
cd /d "%~dp0"
python serial_average.py
if errorlevel 1 pause
