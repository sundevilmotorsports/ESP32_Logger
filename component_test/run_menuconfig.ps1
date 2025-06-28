# ESP-IDF MenuConfig PowerShell Script
Write-Host "Setting up ESP-IDF environment..."

# Set ESP-IDF environment variables
$env:IDF_PATH = "C:\Users\Alexs\esp\v5.4.1\esp-idf"
$env:PATH = "C:\Users\Alexs\.espressif\python_env\idf5.4_py3.12_env\Scripts;" + $env:PATH
$env:PATH = "C:\Users\Alexs\.espressif\tools\xtensa-esp-elf\esp-14.2.0_20241119\xtensa-esp-elf\bin;" + $env:PATH
$env:PATH = "C:\Users\Alexs\.espressif\tools\riscv32-esp-elf\esp-14.2.0_20241119\riscv32-esp-elf\bin;" + $env:PATH
$env:PATH = "C:\Users\Alexs\.espressif\tools\cmake\3.30.2\bin;" + $env:PATH
$env:PATH = "C:\Users\Alexs\.espressif\tools\ninja\1.12.1;" + $env:PATH

Write-Host "ESP-IDF environment activated."
Write-Host "Running menuconfig..."

# Change to project directory
Set-Location "C:\Users\Alexs\Documents\GitHub\ESP32_Logger\component_test"

# Run menuconfig
idf.py menuconfig

Write-Host "Press any key to continue..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
