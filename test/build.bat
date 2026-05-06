@echo off
echo Building Test Runner...
g++ -o test_runner.exe test/testbenches/main.cpp test/mocks/mocks.cpp PS2_controller.cpp phoenix_driver_ssc32.cpp Leg.cpp -I test/mocks -I . -D ARDUINO=100 -D __AVR__ -D USEPS2 -D __BOTBOARD_ARDUINOPROMINI__ -D USE_SSC32
if %ERRORLEVEL% NEQ 0 (
    echo Build Failed!
    exit /b %ERRORLEVEL%
)
echo Build Successful. Running Tests...
.\test_runner.exe
echo Done.
