# Separate NF4 overlay. Reuse stable dependencies without reinstalling/upgrading them.
$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$stablePython = Join-Path $taskRoot '.venv\Scripts\python.exe'
$overlayRoot = Join-Path $taskRoot '.venv4b'
if (-not (Test-Path -LiteralPath $stablePython)) { throw 'Stable .venv is required.' }
if (-not (Test-Path -LiteralPath (Join-Path $overlayRoot 'Scripts\python.exe'))) {
    & $stablePython -m venv $overlayRoot
    if ($LASTEXITCODE -ne 0) { throw 'venv creation failed' }
}
$stablePackages = (Resolve-Path -LiteralPath (Join-Path $taskRoot '.venv\Lib\site-packages')).Path
$overlayPackages = Join-Path $overlayRoot 'Lib\site-packages'
Set-Content -LiteralPath (Join-Path $overlayPackages 'stable_dependencies.pth') -Value $stablePackages -Encoding ascii
# Prevent imports from creating bytecode in the shared stable dependency directory.
Set-Content -LiteralPath (Join-Path $overlayPackages 'sitecustomize.py') -Value "import sys`nsys.dont_write_bytecode = True" -Encoding ascii
$overlayPython = Join-Path $overlayRoot 'Scripts\python.exe'
& $overlayPython -m pip install --no-deps bitsandbytes==0.49.2
if ($LASTEXITCODE -ne 0) { throw 'Optional bitsandbytes installation failed' }
& $overlayPython -m pip check
if ($LASTEXITCODE -ne 0) { throw 'Overlay dependency check failed' }
