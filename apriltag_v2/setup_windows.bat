@echo off
REM ============================================================
REM  AprilTag 도킹 - 윈도우 노트북 셋업 (더블클릭 또는 터미널에서 실행)
REM
REM  이 파일이 있는 폴더(apriltag\)에서 도는 것으로 가정한다.
REM  offline_wheels\ 폴더가 있으면 인터넷 없이 그걸로 설치한다
REM  (만드는 법: 개발 PC 에서 tools/make_offline_bundle.sh).
REM
REM  이걸로 안 되는 것 딱 하나 - Kvaser 드라이버는 pip 가 아니라
REM  kvaser.com 의 "Kvaser Drivers for Windows" 설치파일로 깔아야 한다.
REM ============================================================
cd /d "%~dp0"

where python >nul 2>nul
if errorlevel 1 (
    echo [!] python 이 없다. Python 3.11 을 설치하고 PATH 에 넣은 뒤 다시 실행할 것.
    pause & exit /b 1
)

echo [1/2] 파이썬 패키지 설치
if exist offline_wheels (
    echo       offline_wheels\ 발견 - 인터넷 없이 설치한다
    python -m pip install --no-index --find-links=offline_wheels -r requirements.txt
) else (
    python -m pip install -r requirements.txt
)
if errorlevel 1 (
    echo [!] 설치 실패. 위 오류를 볼 것.
    pause & exit /b 1
)

echo.
echo [2/2] 셋업 점검
python tools\check_setup.py
echo.
pause
