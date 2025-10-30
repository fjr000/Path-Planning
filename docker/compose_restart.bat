@echo off
setlocal

cd /d %~dp0\..

echo Restarting services with Docker Compose...
docker compose down
if errorlevel 1 (
  echo Compose down failed.
)

docker compose up -d
if errorlevel 1 (
  echo Compose up failed.
  exit /b 1
)

docker compose ps

endlocal

