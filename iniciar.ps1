$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    if (-not (Test-Path '.\target\release\ztrak-ojo.exe')) {
        cargo build --release --locked
        if ($LASTEXITCODE -ne 0) { throw 'No se pudo compilar ZTrak Ojo' }
    }
    & '.\target\release\ztrak-ojo.exe' @args
} finally {
    Pop-Location
}
