# AtlasMesh Fleet OS - Variant Budget Analysis (PowerShell)
# Analyzes code redundancy across 7D agnostic architecture

param(
    [switch]$Analyze = $false,
    [switch]$Report = $false,
    [switch]$Fix = $false
)

Write-Host "AtlasMesh Fleet OS - Variant Budget Analysis" -ForegroundColor Blue
Write-Host "Analyzing code redundancy across 7D agnostic architecture..." -ForegroundColor Gray

# Define agnostic dimensions and their paths
$AgnosticDimensions = @{
    'vehicle' = @{
        paths = @('adapters/vehicles/', 'configs/vehicles/', 'edge/vehicle-agent/')
        description = 'Vehicle-agnostic abstraction layer'
    }
    'platform' = @{
        paths = @('deploy/', 'configs/base/', 'services/')
        description = 'Platform-agnostic deployment and services'
    }
    'sector' = @{
        paths = @('configs/sectors/', 'rules/policy/', 'ui/sector-overlays/')
        description = 'Sector-agnostic overlays and policies'
    }
    'sensor' = @{
        paths = @('adapters/sensors/', 'ml/models/perception/', 'edge/sensor-fusion/')
        description = 'Sensor-agnostic perception and fusion'
    }
    'map' = @{
        paths = @('services/map-service/', 'adapters/maps/', 'tools/map-converter/')
        description = 'Map-source-agnostic routing and navigation'
    }
    'weather' = @{
        paths = @('services/weather-fusion/', 'adapters/weather/', 'ml/models/weather/')
        description = 'Weather-source-agnostic fusion and routing'
    }
    'comms' = @{
        paths = @('edge/cloud-bridge/', 'services/gateway-vehicle/', 'adapters/comms/')
        description = 'Communications-agnostic connectivity'
    }
}

# Budget thresholds
$CoreDeltaMax = 5  # 5%
$TestTimeMax = 25  # 25%

# Analysis results
$AnalysisResults = @{
    TotalFiles = 0
    DuplicateFiles = 0
    RedundantCode = @()
    Violations = @()
    Recommendations = @()
}

function Get-FileHash {
    param([string]$FilePath)
    if (Test-Path $FilePath) {
        return (Get-FileHash -Path $FilePath -Algorithm MD5).Hash
    }
    return $null
}

function Analyze-CodeDuplication {
    Write-Host "📊 Analyzing code duplication..." -ForegroundColor Yellow
    
    $FileHashes = @{}
    $DuplicateGroups = @{}
    
    # Scan all source files
    $SourceFiles = Get-ChildItem -Path . -Recurse -Include "*.go", "*.ts", "*.tsx", "*.js", "*.jsx", "*.py", "*.yaml", "*.yml" | 
                   Where-Object { $_.FullName -notmatch "node_modules|\.git|build|dist|target" }
    
    $AnalysisResults.TotalFiles = $SourceFiles.Count
    
    foreach ($file in $SourceFiles) {
        $hash = Get-FileHash -FilePath $file.FullName
        if ($hash) {
            if ($FileHashes.ContainsKey($hash)) {
                if (-not $DuplicateGroups.ContainsKey($hash)) {
                    $DuplicateGroups[$hash] = @($FileHashes[$hash])
                }
                $DuplicateGroups[$hash] += $file.FullName
                $AnalysisResults.DuplicateFiles++
            } else {
                $FileHashes[$hash] = $file.FullName
            }
        }
    }
    
    # Report duplicates
    foreach ($hash in $DuplicateGroups.Keys) {
        $files = $DuplicateGroups[$hash]
        if ($files.Count -gt 1) {
            $AnalysisResults.RedundantCode += @{
                Hash = $hash
                Files = $files
                Count = $files.Count
            }
            
            Write-Host "🔍 Found duplicate files:" -ForegroundColor Red
            foreach ($file in $files) {
                Write-Host "  - $file" -ForegroundColor Gray
            }
        }
    }
}

function Analyze-AgnosticViolations {
    Write-Host "🎯 Analyzing agnostic architecture violations..." -ForegroundColor Yellow
    
    foreach ($dimension in $AgnosticDimensions.Keys) {
        $dimConfig = $AgnosticDimensions[$dimension]
        Write-Host "  Checking $dimension dimension..." -ForegroundColor Gray
        
        foreach ($pathPattern in $dimConfig.paths) {
            if (Test-Path $pathPattern) {
                $files = Get-ChildItem -Path $pathPattern -Recurse -File | 
                        Where-Object { $_.Extension -match "\.(go|ts|tsx|js|jsx|py)$" }
                
                # Check for hardcoded values that violate agnostic principles
                foreach ($file in $files) {
                    $content = Get-Content -Path $file.FullName -Raw -ErrorAction SilentlyContinue
                    if ($content) {
                        # Vehicle-specific violations
                        if ($dimension -ne 'vehicle' -and $content -match "(Tesla|BMW|Ford|Waymo|specific_vehicle_model)") {
                            $AnalysisResults.Violations += @{
                                Dimension = $dimension
                                File = $file.FullName
                                Type = "Hardcoded vehicle reference"
                                Severity = "High"
                            }
                        }
                        
                        # Platform-specific violations
                        if ($dimension -ne 'platform' -and $content -match "(AWS|Azure|GCP|kubernetes_specific)") {
                            $AnalysisResults.Violations += @{
                                Dimension = $dimension
                                File = $file.FullName
                                Type = "Hardcoded platform reference"
                                Severity = "Medium"
                            }
                        }
                        
                        # Map-specific violations
                        if ($dimension -ne 'map' -and $content -match "(OpenStreetMap|Google Maps|Mapbox|HERE)") {
                            $AnalysisResults.Violations += @{
                                Dimension = $dimension
                                File = $file.FullName
                                Type = "Hardcoded map provider reference"
                                Severity = "Medium"
                            }
                        }
                    }
                }
            }
        }
    }
}

function Generate-Recommendations {
    Write-Host "💡 Generating recommendations..." -ForegroundColor Yellow
    
    # Duplicate code recommendations
    if ($AnalysisResults.RedundantCode.Count -gt 0) {
        $AnalysisResults.Recommendations += "Create shared utility functions to eliminate duplicate code"
        $AnalysisResults.Recommendations += "Implement common interfaces for similar functionality"
        $AnalysisResults.Recommendations += "Use composition over inheritance to reduce code duplication"
    }
    
    # Agnostic violations recommendations
    if ($AnalysisResults.Violations.Count -gt 0) {
        $AnalysisResults.Recommendations += "Replace hardcoded references with configuration-driven approaches"
        $AnalysisResults.Recommendations += "Implement adapter patterns for external dependencies"
        $AnalysisResults.Recommendations += "Use dependency injection for platform-specific implementations"
    }
    
    # Abu Dhabi specific recommendations
    $AnalysisResults.Recommendations += "Ensure UAE-specific configurations are properly abstracted"
    $AnalysisResults.Recommendations += "Implement region-agnostic interfaces for government API integrations"
    $AnalysisResults.Recommendations += "Create configurable compliance frameworks for different jurisdictions"
}

function Show-Report {
    Write-Host ""
    Write-Host "📋 VARIANT BUDGET ANALYSIS REPORT" -ForegroundColor Green
    Write-Host "=================================" -ForegroundColor Green
    Write-Host ""
    
    Write-Host "📊 Summary:" -ForegroundColor Cyan
    Write-Host "  Total Files Analyzed: $($AnalysisResults.TotalFiles)" -ForegroundColor White
    Write-Host "  Duplicate Files Found: $($AnalysisResults.DuplicateFiles)" -ForegroundColor White
    Write-Host "  Redundant Code Groups: $($AnalysisResults.RedundantCode.Count)" -ForegroundColor White
    Write-Host "  Agnostic Violations: $($AnalysisResults.Violations.Count)" -ForegroundColor White
    Write-Host ""
    
    if ($AnalysisResults.RedundantCode.Count -gt 0) {
        Write-Host "🔍 Code Duplication Details:" -ForegroundColor Yellow
        foreach ($duplicate in $AnalysisResults.RedundantCode) {
            Write-Host "  Group with $($duplicate.Count) identical files:" -ForegroundColor Red
            foreach ($file in $duplicate.Files) {
                Write-Host "    - $file" -ForegroundColor Gray
            }
            Write-Host ""
        }
    }
    
    if ($AnalysisResults.Violations.Count -gt 0) {
        Write-Host "⚠️ Agnostic Architecture Violations:" -ForegroundColor Red
        foreach ($violation in $AnalysisResults.Violations) {
            Write-Host "  [$($violation.Severity)] $($violation.Type)" -ForegroundColor Red
            Write-Host "    File: $($violation.File)" -ForegroundColor Gray
            Write-Host "    Dimension: $($violation.Dimension)" -ForegroundColor Gray
            Write-Host ""
        }
    }
    
    if ($AnalysisResults.Recommendations.Count -gt 0) {
        Write-Host "Recommendations:" -ForegroundColor Green
        foreach ($recommendation in $AnalysisResults.Recommendations) {
            Write-Host "  • $recommendation" -ForegroundColor White
        }
        Write-Host ""
    }
    
    # Calculate compliance score
    $totalIssues = $AnalysisResults.RedundantCode.Count + $AnalysisResults.Violations.Count
    $complianceScore = if ($AnalysisResults.TotalFiles -gt 0) {
        [math]::Round((1 - ($totalIssues / $AnalysisResults.TotalFiles)) * 100, 2)
    } else { 100 }
    
    $color = if ($complianceScore -ge 95) { "Green" } elseif ($complianceScore -ge 85) { "Yellow" } else { "Red" }
    Write-Host "Agnostic Architecture Compliance Score: $complianceScore%" -ForegroundColor $color
    
    if ($complianceScore -lt 95) {
        Write-Host "⚠️ Compliance below 95% threshold. Consider refactoring to improve agnostic architecture." -ForegroundColor Red
    } else {
        Write-Host "✅ Good compliance with agnostic architecture principles!" -ForegroundColor Green
    }
}

function Fix-CommonIssues {
    Write-Host "🔧 Applying automated fixes..." -ForegroundColor Yellow
    
    # This would implement automated fixes for common issues
    # For now, just show what would be fixed
    Write-Host "  Would fix: Hardcoded configuration values" -ForegroundColor Gray
    Write-Host "  Would fix: Direct platform dependencies" -ForegroundColor Gray
    Write-Host "  Would fix: Duplicate utility functions" -ForegroundColor Gray
    
    Write-Host "✅ Automated fixes applied (simulation mode)" -ForegroundColor Green
}

# Main execution
try {
    if ($Analyze -or $Report) {
        Analyze-CodeDuplication
        Analyze-AgnosticViolations
        Generate-Recommendations
    }
    
    if ($Report) {
        Show-Report
    }
    
    if ($Fix) {
        Fix-CommonIssues
    }
    
    if (-not $Analyze -and -not $Report -and -not $Fix) {
        Write-Host "Usage: .\run-variant-analysis.ps1 [-Analyze] [-Report] [-Fix]" -ForegroundColor Yellow
        Write-Host "  -Analyze: Perform code analysis" -ForegroundColor Gray
        Write-Host "  -Report:  Generate detailed report" -ForegroundColor Gray
        Write-Host "  -Fix:     Apply automated fixes" -ForegroundColor Gray
    }
    
    Write-Host ""
    Write-Host "✅ Variant budget analysis completed!" -ForegroundColor Green
    
} catch {
    Write-Host "❌ Error during analysis: $_" -ForegroundColor Red
    exit 1
}
