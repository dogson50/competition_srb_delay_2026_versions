param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('hot_ptr', 'lazy_ady', 'lrintf')]
    [string]$Variant,
    [ValidateSet('linux', 'windows')]
    [string]$Target = 'linux',
    [string]$Zig
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
if (-not $Zig) {
    $bundledZig = Join-Path $repoRoot 'tools/zig-windows-x86_64-0.13.0/zig.exe'
    if (Test-Path -LiteralPath $bundledZig) {
        $Zig = $bundledZig
    } else {
        $Zig = (Get-Command zig -ErrorAction Stop).Source
    }
}
$Zig = (Get-Command $Zig -ErrorAction Stop).Source
$zigVersion = (& $Zig version | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or $zigVersion -ne '0.13.0') {
    throw "These snapshots require Zig 0.13.0; found '$zigVersion'."
}

$common = Join-Path $PSScriptRoot 'common'
$expectedModel = '7fca5445bfc09f27f7a07961f99db2f39f3dc5cc4710c8423f6246884d562605'
$actualModel = (Get-FileHash (Join-Path $common 'joint_model.bin.z') -Algorithm SHA256).Hash
if ($actualModel.ToLowerInvariant() -ne $expectedModel) { throw 'Joint model checksum mismatch.' }
$outputDir = Join-Path $repoRoot "build/$Variant/$Target"
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$triple = if ($Target -eq 'linux') { 'x86_64-linux-musl' } else { 'x86_64-windows-gnu' }
$cpu = 'x86_64'
if ($Variant -eq 'hot_ptr') {
    $cpu = if ($Target -eq 'linux') { 'baseline' } else { 'x86_64_v3' }
}
$cppFlags = @('-target', $triple, "-mcpu=$cpu", '-std=c++17', '-O3', '-DNDEBUG',
              '-fno-fast-math', '-ffp-contract=off', '-I', $common)
if ($Variant -eq 'hot_ptr') { $cppFlags += '-DHOT_TERMS' }
if ($Variant -eq 'hot_ptr' -and $Target -eq 'linux') { $cppFlags += '-static' }

# .incbin resolves the model path from the compiler's working directory.
Push-Location $common
try {
    $object = Join-Path $outputDir 'estimate.o'
    Write-Output "Building $Variant for $triple with Zig $zigVersion"
    & $Zig c++ @cppFlags -c (Join-Path $PSScriptRoot "$Variant/estimate.cpp") -o $object
    if ($LASTEXITCODE -ne 0) { throw 'C++ compile failed.' }
    $objects = @($object)
    foreach ($name in @('miniz', 'miniz_tinfl', 'miniz_tdef')) {
        $object = Join-Path $outputDir "$name.o"
        & $Zig cc -target $triple "-mcpu=$cpu" -O3 -DNDEBUG -c "$name.c" -o $object
        if ($LASTEXITCODE -ne 0) { throw "C compile failed: $name" }
        $objects += $object
    }
    $binary = Join-Path $outputDir $(if ($Target -eq 'linux') { 'estimate' } else { 'estimate.exe' })
    $linkFlags = @()
    if ($Target -eq 'linux') { $linkFlags += '-static' }
    & $Zig c++ @cppFlags @linkFlags @objects -o $binary
    if ($LASTEXITCODE -ne 0) { throw 'Link failed.' }
    Get-FileHash -LiteralPath $binary -Algorithm SHA256 | Format-List
} finally {
    Pop-Location
}
