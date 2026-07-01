@echo off
REM AD7606 UDP Receiver launcher for Windows 11
REM Edit these variables as needed

set PORT=5001
set OUTPUT=ad7606_data.bin
set DURATION=1

echo ============================================
echo  AD7606 Ethernet UDP Receiver
echo ============================================
echo.
echo Port: %PORT%
echo Duration: %DURATION%s
echo Output: %OUTPUT%
echo.
echo Make sure:
echo   1. PC Ethernet adapter is set to static IP: 192.168.1.100 / 255.255.255.0
echo   2. Zynq board is connected via Ethernet cable
echo   3. Zynq app is configured with DEST_IP=192.168.1.100
echo.
echo Press Ctrl+C to stop.
echo ============================================
echo.

python ad7606_receiver.py --port %PORT% --output %OUTPUT% --duration %DURATION%

pause
