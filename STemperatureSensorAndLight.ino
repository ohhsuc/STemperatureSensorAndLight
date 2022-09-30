#include <Arduino.h>
#include <arduino_homekit_server.h>
#include <YYZT_DHT.h>
#include "wifi_info.h"

//温湿度传感器
#define dht11Pin 14  // ESP8266的14号引脚（GPIO14）接DHT11的数据引脚
DHT dht11;  // 创建dht11对象
#define lightStrip 13 //灯
#define lightOnOff 12 //开关
#define direction1 4 //方向1
#define direction2 5 //方向2
int tempPinB = -1; //存储方向2的状态
long int_nu = 0; //中断计数
unsigned long time1 = 0; //临时时间变量，用于防抖
bool onOff = false; //开关
int brightness = 50; // 当前亮度
int old_brightness = -1; // 上次亮度

#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);

void setup() {
	Serial.begin(9600);
	wifi_connect(); // in wifi_info.h
  //homekit_storage_reset();
	my_homekit_setup();
}

void loop() {
	my_homekit_loop();
	delay(10);
}

//==============================
// Homekit setup and loop
//==============================

// access your homekit characteristics defined in my_accessory.c
extern homekit_server_config_t config;
extern "C" homekit_characteristic_t cha_temperature;
extern "C" homekit_characteristic_t cha_humidity;
extern "C" homekit_characteristic_t cha_light_bulb_on;
extern "C" homekit_characteristic_t cha_light_bulb_brightness;

static uint32_t next_heap_millis = 0; //统计堆内存和客户端数量延时使用
static uint32_t next_report_millis = 0; //上报温湿度延时使用

// 从APP下发开关控制后触发
void cha_switch_on_setter(const homekit_value_t value) {
  bool on_temp = value.bool_value;
  cha_light_bulb_on.value.bool_value = on_temp;  //sync the value
  LOG_D("Switch: %s FROM HOMEKIT APP", on_temp ? "ON" : "OFF");
  onOff = on_temp;
  updateBrightness(); //设置亮度
}

// 从APP设置亮度后触发
void cha_brightness_setter(const homekit_value_t value) {
  int bright_temp = value.int_value;
  cha_light_bulb_brightness.value.int_value = bright_temp;  //sync the value
  LOG_D("Brightness: %d FROM HOMEKIT APP", bright_temp);
  brightness = bright_temp;
  updateBrightness(); //设置亮度
}

//设置亮度
void updateBrightness() {
  if(onOff){
    analogWrite(lightStrip, brightness / 100.0 * 255);
  }else{
    analogWrite(lightStrip, 0);
  }
}

void my_homekit_setup() {
  delay(1000);  // 等待DHT11上电稳定
  dht11.setPin(dht11Pin);  // 设置DHT数据引脚
  //灯相关
  pinMode(direction1, INPUT);
  pinMode(direction2, INPUT);
  pinMode(lightOnOff, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(direction1), RotaryEncoderInterrupt, CHANGE); //亮度中断
  attachInterrupt(digitalPinToInterrupt(lightOnOff), LightSwitch, FALLING); //开关中断
  time1 = millis(); //时间初值，不赋也可以
  //绑定APP开关下发方法
  cha_light_bulb_on.setter = cha_switch_on_setter;
  //绑定APP设置亮度方法
  cha_light_bulb_brightness.setter = cha_brightness_setter;
  arduino_homekit_setup(&config);
}

void my_homekit_loop() {
	arduino_homekit_loop();
	const uint32_t t = millis();
	if (t > next_report_millis) {
		next_report_millis = t + 10 * 1000;
		my_sensor_report(); // 每10秒上报温湿度
	}
	if (t > next_heap_millis) {
		next_heap_millis = t + 5 * 1000;
		LOG_D("Free heap: %d, HomeKit clients: %d",
				ESP.getFreeHeap(), arduino_homekit_connected_clients_count()); // 每五秒统计一下未使用的堆和homekit客户端
	}
}

// 上报温湿度
void my_sensor_report() {
  if (dht11.read() == YYZT_DHTReadSuccess) {
      float temperature_value = dht11.getTemperature();                             // 获取摄氏度温度
      float humidity_value = dht11.getHumidity();                                   // 获取湿度值

      cha_temperature.value.float_value = temperature_value;
      cha_humidity.value.float_value = humidity_value;
  
      LOG_D("Current temperature: %.1f", temperature_value);
      LOG_D("Current humidity: %.1f", humidity_value);
  
      homekit_characteristic_notify(&cha_temperature, cha_temperature.value);
      homekit_characteristic_notify(&cha_humidity, cha_humidity.value);
  }
}

// 上报亮度调节
void my_brightness_report() {
  if(brightness != old_brightness){
    if(brightness > 100){
      brightness = 100;
    }
    if(brightness < 0){
      brightness = 0;
    }
    old_brightness = brightness;

    LOG_D("Light brightness: %d", brightness);
    // 亮度上报
    cha_light_bulb_brightness.value.int_value = brightness; //sync the value
    homekit_characteristic_notify(&cha_light_bulb_brightness, cha_light_bulb_brightness.value);
    updateBrightness(); //设置亮度
  }
}

ICACHE_RAM_ATTR void RotaryEncoderInterrupt()
{
  if(onOff){
    int interruptPinAVal = digitalRead(direction1);
    int interruptPinBVal = digitalRead(direction2);
    if(int_nu == 0 && interruptPinAVal == 0){
      tempPinB = interruptPinBVal;
      int_nu = 1;
    }
    if(int_nu == 1 && interruptPinAVal == 1){
      if(interruptPinBVal == 0 && tempPinB == 1){
        brightness -= 10;
        my_brightness_report();
      }
      if(interruptPinBVal == 1 && tempPinB == 0){
        brightness += 10;
        my_brightness_report();
      }
      int_nu = 0;
    }
  }
}

ICACHE_RAM_ATTR void LightSwitch()
{
   if ((millis() - time1) > 500){ //防抖
    onOff = !onOff; //取反
    // 开关上报
    cha_light_bulb_on.value.bool_value = onOff;
    homekit_characteristic_notify(&cha_light_bulb_on, cha_light_bulb_on.value);
    updateBrightness(); //设置亮度
   }
   time1 = millis();
}
