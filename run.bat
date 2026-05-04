@echo off

echo Starting Python engine...
start cmd /k python main.py

echo Starting renderer...
cd renderer\build
start cmd /k renderer.exe