# Step 1. 设置镜像标签
$tag = "agv-server"

Write-Host "==> Docker build image: $tag"

# Step 2. 构建镜像
docker build -t $tag .

if ($LASTEXITCODE -ne 0) {
    Write-Host "Docker build failed" -ForegroundColor Red
    exit 1
}

# Step 3. 导出镜像
$outputFile = "$tag.tar"

docker save -o $outputFile $tag

if ($LASTEXITCODE -eq 0) {
    Write-Host "Docker image exported: $outputFile" -ForegroundColor Green
} else {
    Write-Host "Export failed" -ForegroundColor Red
}
