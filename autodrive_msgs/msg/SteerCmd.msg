# 스티어링 명령. direction: 'F'|'L'|'R'.
# pulse=false → ArduinoNode.steer()(같은 값 연속 호출 무시)
# pulse=true  → ArduinoNode.steer_pulse()(dedupe 없이 강제 펄스, 차선변경/주차 기동용)
string direction
bool pulse
