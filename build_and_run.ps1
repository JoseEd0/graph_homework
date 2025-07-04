# build_and_run.ps1
Write-Host "=== Compilando proyecto ===" -ForegroundColor Green

# Crear directorio build si no existe
if (!(Test-Path "build")) {
    Write-Host "Creando directorio build..." -ForegroundColor Yellow
    New-Item -ItemType Directory -Name "build"
}

# Ir al directorio build
Set-Location "build"

# Ejecutar cmake
Write-Host "Ejecutando cmake..." -ForegroundColor Yellow
cmake ..

if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: cmake falló" -ForegroundColor Red
    exit 1
}

# Compilar
Write-Host "Compilando..." -ForegroundColor Yellow
make

if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: make falló" -ForegroundColor Red
    exit 1
}

# Verificar que el ejecutable existe
Write-Host "Verificando archivos generados..." -ForegroundColor Yellow
Get-ChildItem -Name

if (Test-Path "graph_homework.exe") {
    Write-Host "=== Ejecutable creado exitosamente ===" -ForegroundColor Green
    Write-Host "Ejecutando graph_homework.exe..." -ForegroundColor Cyan
    .\graph_homework.exe
} elseif (Test-Path "graph_homework") {
    Write-Host "=== Ejecutable creado exitosamente ===" -ForegroundColor Green
    Write-Host "Ejecutando graph_homework..." -ForegroundColor Cyan
    .\graph_homework
} else {
    Write-Host "ERROR: No se pudo crear el ejecutable" -ForegroundColor Red
    Write-Host "Archivos en el directorio build:" -ForegroundColor Yellow
    Get-ChildItem
}

# Volver al directorio anterior
Set-Location ".."