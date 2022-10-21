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

3. arduino-cli 다른 제조사 json을 설치할 경우
  가. arduino가 가능할 경우 arduino에서 작업을 수행
    1) 라즈베리파이 피코를 예시로 둠
      가) arduino를 실행하여 설정에서 라즈베리파이 피코용 json을 추가함
      나) 보드매니저에서 패키지를 업데이트하고 pico를 추가함
      다) 컴파일
        (1) $ arduino-cli compile --fqbn rp2040:rp2040:rpipico 아두이노코드명
      라) 업로드
        (1) $ arduino-cli upload --fqbn rp2040:rp2040:rpipico -p /dev/ttyACMn 아두이노코드명
  나. arduino-cli로만 수행할 경우
    1) 라즈베리파이 피코를 예시로 둠
      가) 패키지 인덱스 추가(json)
        (1) $ arduino-cli core update-index --additional-urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
        (2) 상기와 같은 방식으로 패키지 검색이 되면 다음은 할 필요없음, 하지만 라즈베리파이 피코의 경우 일부 패키지는 검색이 되지 않았음 추가적으로 다음과 처리
        (3) 홈 디렉토리의 .arduino15 디렉토리에 존재하는 arduino-cli.yaml을 편집
          (가) board_manager: additional_urls: [] 를
          (나) board_manager: additional_urls: ["https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json"] 으로 변경
          (다) 추가 URL이 더 있을 경우 콤마를 입력하여 추가할 것(마지막 URL은 콤마 제외)
          (라) $ arduino-cli core update-index
      나) 패키지 검색
        (1) $ arduino-cli core search pico
      다) 패키지 설치
        (1) $ arduino-cli core install rp2040:rp2040
      라) 컴파일
        (1) $ arduino-cli compile --fqbn rp2040:rp2040:rpipico 아두이노코드명
      마) 업로드
        (1) $ arduino-cli upload --fqbn rp2040:rp2040:rpipico -p /dev/ttyACMn 아두이노코드명