@echo off
set MSYSTEM=
set IDF_PATH=C:\esp\v5.4.4\esp-idf
set IDF_TOOLS_PATH=C:\Espressif\tools
set IDF_PYTHON_ENV_PATH=C:\Users\12039\.espressif\python_env\idf5.4_py3.13_env
set IDF_TARGET=esp32s3

set "PATH=C:\Espressif\tools\cmake\3.30.2\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin;C:\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20260121\riscv32-esp-elf\bin;C:\Espressif\tools\xtensa-esp-elf-gdb\16.3_20250913\xtensa-esp-elf-gdb\bin;C:\Espressif\tools\riscv32-esp-elf-gdb\16.3_20250913\riscv32-esp-elf-gdb\bin;C:\Espressif\tools\esp32ulp-elf\2.38_20240113\esp32ulp-elf\bin;C:\Espressif\tools\openocd-esp32\v0.12.0-esp32-20260304\openocd-esp32\bin;C:\Espressif\tools\idf-exe\1.0.3;C:\Espressif\tools\ccache\4.12.1;C:\Espressif\tools\dfu-util\0.11;%PATH%"

cd /d d:\PPPP\Line-follower-robot
"C:\Users\12039\.espressif\python_env\idf5.4_py3.13_env\Scripts\python.exe" "C:\esp\v5.4.4\esp-idf\tools\idf.py" build > _build.log 2>&1
echo EXIT_CODE=%ERRORLEVEL% >> _build.log
