@echo off
setlocal

cd /d %~dp0\..

echo Building and starting services with Docker Compose...
docker compose up -d --build
if errorlevel 1 (
  echo Compose up failed.
  exit /b 1
)

docker compose ps

endlocal

