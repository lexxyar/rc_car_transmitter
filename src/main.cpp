#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>

/**
 * RF24 ->  NANO
 * -------------
 * VCC      VCC
 * GND      GND
 * CE       D7
 * CSN      D8
 * SCK      D13
 * MOSI     D11
 * MISO     D12
 * 
 * J1   ->  UNO
 * ------------
 * GND      GND
 * VCC      VCC
 * VRX      A1
 * VRY      A2
 * 
 * POT  ->  UNO
 * ------------
 * GND      GND
 * VCC      VCC
 * SIG      A0
 * 
 * BTN  ->  UNO
 * ------------
 * VCC      VCC
 * ON/OFF   D3  (via resistor 120 -> D3 and pull down resistor -> GND)
 * Green    D4
 * Red      D5
 * */

#define PIN_RF24_CE 7
#define PIN_RF24_CSN 8
#define PIN_J1_X A1
#define PIN_J1_Y A2
#define PIN_POT A0
#define PIN_BUTTON 3
#define PIN_LED_RED 5
#define PIN_LED_GREEN 4
#define LONG_PRESS_TIME 1500

// Tx/Rx common part
#define RF_CHANNEL 0x6f
// #define RF24_ADDRESS "00001"
const byte RF24_ADDRESS[6] = "00001";
struct RFPackage{
  uint8_t mode; // 0 - drive, 1 - config
  uint16_t j1x;
  uint16_t j1y;
  uint16_t pot;
  uint8_t btn;
} rfPackage;
// --------------

RF24 radio(PIN_RF24_CE, PIN_RF24_CSN);
// const uint16_t LONG_PRESS_TIME  = 1500; // milliseconds
// Variables will change:
uint8_t lastState = LOW;  // the previous state from the input pin
uint8_t currentState;     // the current reading from the input pin
uint32_t pressedTime  = 0;
uint32_t releasedTime = 0;

struct Joystic
{
  uint16_t x;
  uint16_t y;
} j1;



void readJoystick();
void rf24Setup();
void blink(uint8_t ledPin, uint8_t count);
void turnOnLed(uint8_t ledPin);
void turnOnOff(uint8_t ledPin);


void setup() {
  Serial.begin(115200);

  // Настройка радиомодуля
  rf24Setup();

  // Пин для кнопки в режиме ввода
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  turnOnOff(PIN_LED_GREEN);
  turnOnOff(PIN_LED_RED);

  // Моргнем 3 раза красным
  blink(PIN_LED_RED, 3);

  // Зажигаем зеленый
  turnOnLed(PIN_LED_GREEN);
}

void loop() {
  readJoystick();
  if(rfPackage.mode != 1){
    rfPackage.mode = 0;
  }
  rfPackage.j1x = j1.x;
  rfPackage.j1y = j1.y;
  rfPackage.pot = analogRead(PIN_POT);
  rfPackage.btn = digitalRead(PIN_BUTTON);

  currentState = rfPackage.btn;
  if(lastState == LOW && currentState == HIGH)        // button is pressed
    pressedTime = millis();
  else if(lastState == HIGH && currentState == LOW) { // button is released
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if( pressDuration > LONG_PRESS_TIME )
      Serial.println("A long press is detected");
      rfPackage.mode = 1;
  }

  // save the the last state
  lastState = currentState;

  radio.write(&rfPackage, sizeof(rfPackage));
}

void blink(uint8_t ledPin, uint8_t count){
  uint16_t d = 200;
  for(uint8_t i = 0; i < count; i++){
    // digitalWrite(ledPin, LOW);
    turnOnLed(ledPin);
    delay(d);
    // digitalWrite(ledPin, HIGH);
    turnOnOff(ledPin);
    delay(d);
  }
}

void turnOnLed(uint8_t ledPin){
  digitalWrite(ledPin, LOW);
}

void turnOnOff(uint8_t ledPin){
  digitalWrite(ledPin, HIGH);
}

void readJoystick(){
  j1.x = analogRead(PIN_J1_X);
  j1.y = analogRead(PIN_J1_Y);
}

void rf24Setup(){
  // Инициируем работу nRF24L01+
  radio.begin();

  // Указываем мощность передатчика 
  // (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm)
  radio.setPALevel(RF24_PA_LOW);

  // Указываем канал передачи данных (от 0 до 125), 
  radio.setChannel(RF_CHANNEL);

  // Указываем скорость передачи данных 
  // (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
  radio.setDataRate(RF24_250KBPS); 

  // Открываем 1 трубу с адресом 1 передатчика 0xAABBCCDD11, для передачи данных
  radio.openWritingPipe(RF24_ADDRESS);

  // Включаем питание
  radio.powerUp();

  // Не слушаем
  radio.stopListening();
}