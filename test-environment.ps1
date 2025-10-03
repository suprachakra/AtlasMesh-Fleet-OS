# Test Environment Script
# Check what development tools are available

Write-Host "🔍 AtlasMesh Fleet OS - Environment Check" -ForegroundColor Blue
Write-Host "=" * 50

# Check Node.js
Write-Host "`n📦 Checking Node.js..." -ForegroundColor Yellow
try {
    $nodeVersion = node --version 2>$null
    if ($nodeVersion) {
        Write-Host "✅ Node.js: $nodeVersion" -ForegroundColor Green
        
        # Check npm
        $npmVersion = npm --version 2>$null
        if ($npmVersion) {
            Write-Host "✅ npm: $npmVersion" -ForegroundColor Green
        }
    }
} catch {
    Write-Host "❌ Node.js not found" -ForegroundColor Red
}

# Check Go
Write-Host "`n🐹 Checking Go..." -ForegroundColor Yellow
try {
    $goVersion = go version 2>$null
    if ($goVersion) {
        Write-Host "✅ Go: $goVersion" -ForegroundColor Green
    }
} catch {
    Write-Host "❌ Go not found" -ForegroundColor Red
}

# Check Docker
Write-Host "`n🐳 Checking Docker..." -ForegroundColor Yellow
try {
    $dockerVersion = docker --version 2>$null
    if ($dockerVersion) {
        Write-Host "✅ Docker: $dockerVersion" -ForegroundColor Green
        
        # Check if Docker is running
        $dockerInfo = docker info 2>$null
        if ($dockerInfo) {
            Write-Host "✅ Docker daemon is running" -ForegroundColor Green
        } else {
            Write-Host "⚠️  Docker installed but daemon not running" -ForegroundColor Yellow
        }
    }
} catch {
    Write-Host "❌ Docker not found" -ForegroundColor Red
}

# Check Python
Write-Host "`n🐍 Checking Python..." -ForegroundColor Yellow
try {
    $pythonVersion = python --version 2>$null
    if ($pythonVersion) {
        Write-Host "✅ Python: $pythonVersion" -ForegroundColor Green
    } else {
        $python3Version = python3 --version 2>$null
        if ($python3Version) {
            Write-Host "✅ Python3: $python3Version" -ForegroundColor Green
        }
    }
} catch {
    Write-Host "❌ Python not found" -ForegroundColor Red
}

# Check Git
Write-Host "`n📚 Checking Git..." -ForegroundColor Yellow
try {
    $gitVersion = git --version 2>$null
    if ($gitVersion) {
        Write-Host "✅ Git: $gitVersion" -ForegroundColor Green
    }
} catch {
    Write-Host "❌ Git not found" -ForegroundColor Red
}

# Check project structure
Write-Host "`n📁 Checking Project Structure..." -ForegroundColor Yellow

$requiredDirs = @("services", "ui", "docs", "scripts", "database")
foreach ($dir in $requiredDirs) {
    if (Test-Path $dir) {
        $itemCount = (Get-ChildItem $dir).Count
        Write-Host "✅ $dir/ ($itemCount items)" -ForegroundColor Green
    } else {
        Write-Host "❌ $dir/ missing" -ForegroundColor Red
    }
}

# Check key files
Write-Host "`n📄 Checking Key Files..." -ForegroundColor Yellow
$keyFiles = @("docker-compose.yml", "Makefile", "README.md", "package.json")
foreach ($file in $keyFiles) {
    if (Test-Path $file) {
        Write-Host "✅ $file" -ForegroundColor Green
    } else {
        Write-Host "❌ $file missing" -ForegroundColor Red
    }
}

Write-Host "`n🎯 Next Steps:" -ForegroundColor Cyan
Write-Host "1. Install missing dependencies" -ForegroundColor White
Write-Host "2. Start with available tools" -ForegroundColor White
Write-Host "3. Use alternative deployment methods" -ForegroundColor White

Write-Host "`n" -ForegroundColor White
