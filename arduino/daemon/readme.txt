1. arduino-cli 설치
  가. https://github.com/arduino/arduino-cli
  나. https://arduino.github.io/arduino-cli/0.21/installation/
  다. $ curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
    1) 상기 명령을 실행한 디렉토리에 bin 디렉토리가 생성되고 그 안에 arduino-cli 바이너리 파일이 생성됨
    2) arduino-cli 파일을 /usr/bin 디렉토리로 옮기는 것이 좋음
      가) 글로벌하게 실행하기 위함
  라. snap에서 설치할 수 있는 arduino-cli는 동작이 정상적으로 되지 않았음
    1) port 열기 에러가 남

2. arduino-cli 사용방법
  가. $ arduino-cli config init
    1) 초기 설정파일 생성(필수)
  나. $ arduino-cli core install arduino:avr
    1) mega2560을 사용할 예정이므로 arduino:avr을 설치할 것
  다. 이후부터는 Makefile을 사용할 것
    1) make, make upload, make upload USER_PORT=/dev/ttyUSBn