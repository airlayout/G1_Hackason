$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$taskStablePython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
$taskOptionalPython = Join-Path $PSScriptRoot '.venv36\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $taskStablePython)) { throw 'Existing stable .venv is required.' }
if (-not (Test-Path -LiteralPath $taskOptionalPython)) {
    & $taskStablePython -m venv .venv36
    if ($LASTEXITCODE -ne 0) { throw 'Optional venv creation failed.' }
}
# Read-only reuse of existing torch/CUDA dependencies. Install only into .venv36.
$taskStableSite = Join-Path $PSScriptRoot '.venv\Lib\site-packages'
$taskPth = Join-Path $PSScriptRoot '.venv36\Lib\site-packages\stable_packages.pth'
[System.IO.File]::WriteAllText($taskPth, ($taskStableSite + [Environment]::NewLine), [Text.UTF8Encoding]::new($false))
& $taskOptionalPython -m pip install --no-deps -r requirements-hybrid-lock.txt
if ($LASTEXITCODE -ne 0) { throw 'Optional dependency installation failed.' }
& $taskOptionalPython -m pip check
if ($LASTEXITCODE -ne 0) { throw 'Optional dependency check failed.' }
& $taskOptionalPython -c "from pathlib import Path; import hashlib,urllib.request; p=Path('.cache/phase36/yolo11n.pt'); p.parent.mkdir(parents=True,exist_ok=True); expected='0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1'; urllib.request.urlretrieve('https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt',p) if not p.exists() else None; assert hashlib.sha256(p.read_bytes()).hexdigest()==expected, 'YOLO weight hash mismatch'"
if ($LASTEXITCODE -ne 0) { throw 'YOLO weight verification failed.' }
