/*
 Name:		Controller.ino
 Created:	10/11/2020
 Author:	Greg Sixtel
*/

#include "ToolStrings.h"
#include "HeatingScheduler.h"
#include "SimpleTimer.h"
#include "BinArray.h"
#include "HistDateTime.h"
#include "DataContainer.h"

#include <SPI.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>  
#include <spiffs.h>

#ifdef AsyncMqtt
#include <AsyncMqttClient.h>  
#else
#include <WiFiClient.h>
#include <PubSubClient.h>
#endif // AsyncMqtt

#define BLUE_LED     05
#define RELAY_PIN    27
#define ONE_WIRE_BUS 15
#define NET_FILE_NAME "/net.txt"
#define SHEDULE_FILE_NAME "/schedule.txt"

String WifiSsid;
String WifiPswd;
String MqttDomain;
String MqttClient;
String MqttPort;
String MqttToken;
String MqttChannel;
String TelemetryTopic;
String HistoryTTopic;
String HistoryHTopic;
String ScheduleTopic;
String RxTopic;
int MqttPortNum = 1883;

extern void WiFiEvent(WiFiEvent_t event);

#ifdef Adfr_SSD1306
#include "DispAdfSSD1306.h"
DispAdfSSD1306 Dsp;
#endif // Adfr_SSD1306

#ifdef AsyncMqtt
AsyncMqttClient mqttAsyncClient;
extern void onMqttConnect(bool sessionPresent);
extern void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
extern void onMqttSubscribe(uint16_t packetId, uint8_t qos);
extern void onMqttUnsubscribe(uint16_t packetId);
extern void onMqttPublish(uint16_t packetId);

extern void onMqttMessage(
	char* topic,
	char* payload,
	AsyncMqttClientMessageProperties properties,
	size_t length,
	size_t index,
	size_t total);
#else
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
bool _wasConnected = false;
extern void onMqttMessage(
	char* topic,
	uint8_t* payload,
	unsigned int length);
#endif // AsyncMqtt

extern void PublishTelemetry(bool ledOn);

#pragma region NTPClient
// Define NTP Client to get time
// Set offset time in seconds to adjust for your timezone, for example:
// GMT +1 = 3600
// GMT +8 = 28800
// GMT -1 = -3600
// GMT 0 = 0
// GMT +1 = 3600
// GMT +2 = 7200
// GMT +3 = 10800
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, 2L * 3600L);
#pragma endregion

HistDateTime HEAT_HISTORY[HEAT_HIST_DEEP];
uint32_t TEMP_HISTORY[TEMP_HIST_DEEP];
uint16_t STAT_FLG = 0u;

#define CLEAR_T_HIST (memset(TEMP_HISTORY, 0, sizeof(TEMP_HISTORY)))
#define CLEAR_H_HIST (memset(HEAT_HISTORY, 0, sizeof(HEAT_HISTORY)))

SimpleTimer SmplTimer;
HeatingScheduler Scheduler;
bool DetectedSensorsError = false;
bool SetupDone = false;
uint8_t HeatDestTemp = 0U;
uint32_t LastTemperatureMeasurement = 0U;
auto bLTM = reinterpret_cast<byte*>(&LastTemperatureMeasurement);
#define TAVG AVERAGE(bLTM[SENSORS - 2], bLTM[SENSORS - 1])

#ifdef DSP
#include "ToolDisp.h"
ToolDisp Disp;
#endif // DSP

#ifdef FLOW
#include "PeakControl.h"
PeakControl FlowControl(6, 7); // Using- D06 as Input Flow, D07 as Output Flow  

#define FLOW_SENSOR_INP  0 // Pin D02
#define FLOW_SENSOR_OUT  1 // Pin D03

volatile int16_t InpFlowPulse = 0;
volatile int16_t OutFlowPulse = 0;
#endif // FLOW

#ifndef SIMULATION
#include <DallasTemperature.h>
#include <OneWire.h>

#define TEMPERATURE_CONVERSION_DELAY 500ul // 500ms
OneWire OW(ONE_WIRE_BUS);
DallasTemperature Sensors(&OW); // DS18B20
DeviceAddress Address[SENSORS];
extern void OnReadSensor();
#endif // !SIMULATION

#define UPDATE_SENSORS_DATA_DELAY 10000ul  // 10s
#define LOCAL_HIST_DEEP 4  //
extern void OnRequestSensor();

#pragma region Timer#01
#define UPDATE_SYSTEM_STATE_INTERVAL 30000UL // 30 sec
#define UPDATE_SYSTEM_STATE_INTERVAL_ONES 300UL // 300 msec
int UpdateSystemStateTimerID = -1;
extern void _onUpdateSystemState();
#pragma endregion

#pragma region Timer#02
int PauseHeatingTimerID = -1;
extern void OnPauseHeatingTimer();
#pragma endregion

#pragma region Timer#03
#ifdef DBG
#define MAX_MANUAL_HEATING_INTERVAL  60UL * 1000UL * 2UL // 2 Min
#else
#define MAX_MANUAL_HEATING_INTERVAL 60UL * 1000UL * 120UL // 120 Min (2Hours)
#endif
int ManualHeatingTimerID = -1;
extern void OnManualHeatingTimer();
#pragma endregion

#pragma region Timer#04
#define TEMPR_HISTORY_UPDATE_INTERVAL 60UL * 1000UL * 5UL // 5 Min
int TemprHistoryUpdateTimerID = -1;
extern void TemperatureHistoryUpdate();
#pragma endregion

#pragma region Timer#05
#define RTC_READ_INTERVAL 20000UL // 20 sec
int RtcReadTimerID = -1;
extern void OnRTCReadTimer();
#pragma endregion

#pragma region Timer#06
#ifdef FLOW
#define FLOW_CONTROL_INTERVAL 1000UL // 1 sec
int FlowControlTimerID = -1;
extern void OnFlowControl();
#endif // FLOW
#pragma endregion

#pragma region Timer#07
int _mqttReconnectTimer = -1;
#define  MQTT_RECONNECT_INTERVAL 2000UL // 2sec
extern void _connectToMqtt();
#pragma endregion

#pragma region Timer#08
int _wifiReconnectTimer = -1;
#define  WIFI_RECONNECT_INTERVAL 2000UL // 2sec
extern void _connectToWifi();
#pragma endregion

#pragma region Timer#09
int _timeClientForceUpdateTimer = -1;
#define  TIME_CLIENT_FORCE_UPDATE_INTERVAL 10000UL // 10sec
extern void _forceUpdateTimeClient();
#pragma endregion

extern void SetHeatingHistory(bool on, uint16_t flagPrev);
extern void UpdateSystemStateRequestOnes();
extern void CalculateSensorData(int16_t* rawSensTempr);
extern void _connectToTimeClient();
extern uint8_t PauseHeatingCmd(uint8_t interval);
extern bool ManualHeatingCmd(uint8_t temperature); // Destination temperature, or extremal time (2 Hours)
extern uint8_t AutoHeatingCmd(uint8_t on);

void setup()
{
	HeatDestTemp = 0U;
	LastTemperatureMeasurement = 0U;

	pinMode(RELAY_PIN, OUTPUT);
	digitalWrite(RELAY_PIN, HIGH);

	pinMode(BLUE_LED, OUTPUT);
	digitalWrite(BLUE_LED, LOW);

	delay(500UL); // power-up safety delay
	// Reset History values
	CLEAR_T_HIST;
	CLEAR_H_HIST;

#ifdef DBG
	//Dbg.begin(9600);
	Dbg.begin(115200);
	while (!Dbg);
	Dbg.println();
	Dbg.println(F("SETUP START"));

	Dbg.print(F("DateTime Size: "));
	Dbg.print(sizeof(DateTime));
	Dbg.println(F(" bytes"));

	Dbg.print(F("HistDateTime Size: "));
	Dbg.print(sizeof(HistDateTime));
	Dbg.println(F(" bytes"));

	Dbg.print(F("HEAT HISTORY Size: "));
	Dbg.print(sizeof(HEAT_HISTORY));
	Dbg.println(F(" bytes"));
	
	Dbg.print(F("TEMP HISTORY Size: "));
	Dbg.print(sizeof(TEMP_HISTORY));
	Dbg.println(F(" bytes"));
#endif // DBG

	if (!SPIFFS.begin(true)) {
#ifdef DBG
		Dbg.println(F("An Error has occurred while mounting SPIFFS"));
#endif // DBG
		return;
	}

	{
		File file = SPIFFS.open(NET_FILE_NAME);
		if (!file) {
#ifdef DBG
			Dbg.println(F("Failed to open file for reading"));
#endif // DBG
			return;
		}

#ifdef DBG
		Dbg.println();
		Dbg.print(F("Reading file: "));
		Dbg.println(NET_FILE_NAME);
#endif // DBG
		auto iData = 0;
		String data;
		data.reserve(50);
		char ch;
		bool read = false;
		while (file.available()) {
			ch = (char)file.read();
#ifdef DBG
			Dbg.write(ch);
#endif // DBG
			if (ch == '[') {
				read = true;
			}
			else if (ch == ']') {
				read = false;
			}
			else if (ch == '#') {
				switch (iData) {
				case 0:  WifiSsid		= data;	break;
				case 1:  WifiPswd		= data;	break;
				case 2:  MqttDomain		= data;	break;
				case 3:  MqttClient		= data;	break;
				case 4:  MqttPort		= data;	break;
				case 5:  MqttToken      = data;	break;
				case 6:  MqttChannel    = data; break;
				case 7:  TelemetryTopic = MqttChannel + "/" + data;	break; 
				case 8:  HistoryTTopic  = MqttChannel + "/" + data;	break;
				case 9:  HistoryHTopic  = MqttChannel + "/" + data;	break;
				case 10: ScheduleTopic  = MqttChannel + "/" + data;	break;
				case 11: RxTopic        = MqttChannel + "/" + data;	break;
				default: break;
				}
				++iData;
				data = "";
				read = false;
			}
			else  if (ch >= '#' && ch <= 'z' && read) {
				data += ch;
			}
		}

		MqttPortNum = MqttPort.toInt();
		file.close();

#ifdef DBG
		Dbg.print(F("Wifi SSID: "));		Dbg.println(WifiSsid);
		Dbg.print(F("Wifi Passw: "));		Dbg.println(WifiPswd);
		Dbg.print(F("Mqtt Domain: "));		Dbg.println(MqttDomain);
		Dbg.print(F("Mqtt Client: "));		Dbg.println(MqttClient);
		Dbg.print(F("Mqtt Port: "));		Dbg.println(MqttPort);
		Dbg.print(F("Mqtt Token: "));		Dbg.println(MqttToken);
		Dbg.print(F("Mqtt Channel: "));		Dbg.println(MqttChannel);
		Dbg.print(F("Telemetry Topic: "));	Dbg.println(TelemetryTopic);
		Dbg.print(F("HistoryT Topic: "));	Dbg.println(HistoryTTopic);
		Dbg.print(F("HistoryH Topic: "));	Dbg.println(HistoryHTopic);
		Dbg.print(F("Schedule Topic: "));	Dbg.println(ScheduleTopic);
		Dbg.print(F("Rx Topic: "));			Dbg.println(RxTopic);
#endif // DBG

		file = SPIFFS.open(SHEDULE_FILE_NAME);
		if (!file) {
#ifdef DBG
			Dbg.println(F("Failed to open file for reading"));
#endif // DBG
			return;
		}

#ifdef Adfr_SSD1306
		Dsp.Init();
#endif //Adfr_SSD1306

#ifdef DBG
		Dbg.println();
		Dbg.print(F("Reading file: "));
		Dbg.println(SHEDULE_FILE_NAME);
#endif // DBG
		iData = 0;
		data = "";
		ScheduleElm elm;
		byte elmIndex = 0;
		while (file.available()) {
			ch = (char)file.read();
#ifdef DBG
			Dbg.write(ch);
#endif // DBG

			if (',' == ch) {
				switch (iData) {
				case 0: elm.DOW = Scheduler.GetDow(data); break;
				case 1: elm.DOW |= (((byte)data.toInt()) << 7); break;
				case 2: elm.TimeFr = (byte)data.toInt(); break;
				case 3: elm.TimeTo = (byte)data.toInt(); break;
				case 4: elm.TemperatureAvg = (byte)data.toInt(); break;
				default: break;
				}
				++iData;
				data = "";
			}
			else  if ('#' == ch) {
				Scheduler.SetSchedule(elmIndex, elm);
				iData = 0;
				++elmIndex;
				data = "";
			} else if (ch >= '#' && ch <= 'z') {
				data += ch;
			}
		}
#ifdef DBG
		Dbg.println();
#endif // DBG
		file.close();
	}

	WiFi.onEvent(WiFiEvent);

#ifdef AsyncMqtt
	mqttAsyncClient.onConnect(onMqttConnect);
	mqttAsyncClient.onDisconnect(onMqttDisconnect);
	mqttAsyncClient.onSubscribe(onMqttSubscribe);
	mqttAsyncClient.onUnsubscribe(onMqttUnsubscribe);
	mqttAsyncClient.onMessage(onMqttMessage);
	mqttAsyncClient.onPublish(onMqttPublish);
	mqttAsyncClient.setServer(MqttDomain.c_str(), MqttPortNum);
	mqttAsyncClient.setClientId(MqttClient.c_str());
	mqttAsyncClient.setCredentials(MqttToken.c_str());
#else
	mqttClient.setServer(MqttDomain.c_str(), MqttPortNum);
	mqttClient.setCallback(onMqttMessage);
#endif // AsyncMqtt

	timeClient.setUpdateInterval(1000UL * 60UL * 60UL); // Ones per hour

	auto loaded = Scheduler.Init(SCHEDULER_CODE);
#ifdef DBG
	Dbg.println();
	Dbg.println(loaded ? F("Loaded Schedulers") : F("Default Schedulers"));
#endif

	UpdateSystemStateTimerID = SmplTimer.setInterval(UPDATE_SYSTEM_STATE_INTERVAL, _onUpdateSystemState);
	SmplTimer.disable(UpdateSystemStateTimerID);

	RtcReadTimerID = SmplTimer.setInterval(RTC_READ_INTERVAL, OnRTCReadTimer);
	SmplTimer.disable(RtcReadTimerID);

	TemprHistoryUpdateTimerID = SmplTimer.setInterval(TEMPR_HISTORY_UPDATE_INTERVAL, TemperatureHistoryUpdate);
	SmplTimer.disable(TemprHistoryUpdateTimerID);

#ifdef FLOW
	FlowControlTimerID = SmplTimer.setInterval(FLOW_CONTROL_INTERVAL, OnFlowControl);
	SmplTimer.disable(FlowControlTimerID);
#endif // FLOW


#ifndef SIMULATION // !SIMULATION
	Sensors.begin();
	DetectedSensorsError = (SENSORS != Sensors.getDeviceCount());
	memset(Address, 0, SENSORS * sizeof(DeviceAddress));

	for (uint8_t s = 0; s < SENSORS; ++s) {
		// Search the wire for address
		auto stt = Sensors.getAddress(Address[s], s);
		if (stt) {
			// set the resolution to 9 bit
			// (Each Dallas/Maxim device is capable of several different resolutions)
			Sensors.setResolution(Address[s], 9);
		}

#ifdef DBG_SENSORS
		Dbg.print(F("Sensors: "));
		Dbg.print(s + 1);

		if (stt) {
			Dbg.print(F("Address: "));
			for (byte a = 0; a < 8; ++a) {
				Dbg.print(Address[s][a]);
				Dbg.print(".");
			}
		}
		else {
			Dbg.print(F("Address NOT detected"));
		}
		Dbg.println();
#endif // DBG
	}

	Sensors.setWaitForConversion(false);

	//==attachInterrupt(FLOW_SENSOR_INP, InpFlowPulseCounter, RISING);
	//==attachInterrupt(FLOW_SENSOR_OUT, OutFlowPulseCounter, RISING);

#else // SIMULATION

#ifdef DBG
	Dbg.println();
	Dbg.println(F("SIMULATION"));
#endif // DBG
#endif // SIMULATION


	OnRequestSensor();
	RST_FLG_AUTOMATIC_MODE(STAT_FLG);
	SetupDone = true;
	SmplTimer.enable(RtcReadTimerID);
	SmplTimer.enable(UpdateSystemStateTimerID);
	SmplTimer.setTimeout(500UL, _connectToWifi);
	SmplTimer.enable(TemprHistoryUpdateTimerID);
	AutoHeatingCmd(1);
	SmplTimer.setTimeout(5000ul, TemperatureHistoryUpdate);
}//--------------------------------------------------------------------------

void loop()
{
	if (WiFi.isConnected() && _timeClientForceUpdateTimer < 0) {
		while (!timeClient.update()) {
#ifdef DBG
			Dbg.println(F("TimeClient ForceUpdate Update"));
#endif // DBG
			_timeClientForceUpdateTimer = 
				SmplTimer.setTimeout(TIME_CLIENT_FORCE_UPDATE_INTERVAL, _forceUpdateTimeClient);
		}
	}

#ifndef AsyncMqtt
	if (!mqttClient.loop() && _wasConnected && _mqttReconnectTimer < 0) {
#ifdef DBG
		Dbg.println();
		Dbg.println(F("MQTT is disconnected, try to reconnect..."));
#endif // DBG
		_mqttReconnectTimer = SmplTimer.setTimeout(MQTT_RECONNECT_INTERVAL, _connectToMqtt);
	}
#endif // !AsyncMqtt

	SmplTimer.run();
	
#ifdef FLOW
	FlowControl.Run();
#endif // FLOW

}//--------------------------------------------------------------------------

void PublishTelemetry(bool ledOn) 
{
	
#ifdef AsyncMqtt
	if (!mqttAsyncClient.connected()) {
		return;
	}
#else
	if (!mqttClient.connected()) {
		return;
	}
#endif // AsyncMqtt


	if (ledOn) {
		digitalWrite(BLUE_LED, HIGH);
		SmplTimer.setTimeout(150UL, LedOff);
	}

	auto left = Scheduler.LeftPauses();
	uint16_t LeftPauseMinutes = (-1 == PauseHeatingTimerID) ? 0 :
		(uint16_t)(SmplTimer.leftTime(PauseHeatingTimerID) / 60000ul); // Left time in minutes

	auto dt = RtcNow();

	auto send = BTelemetry(
		STAT_FLG,
		HeatDestTemp,
		LeftPauseMinutes,
		left,
		LastTemperatureMeasurement,
		dt);

#ifdef DBG
	Dbg.println();
	Dbg.println(F("Publish Telemetry:"));
	Dbg.print(" T: ");
	Dbg.print(bLTM[0]);
	Dbg.print(" ");
	Dbg.print(bLTM[1]);
	Dbg.print(" ");
	Dbg.print(bLTM[2]);
	Dbg.print(" ");
	Dbg.print(bLTM[3]);
	//Dbg.print(F(" RT: Full Size: "));
	//Dbg.println(send.Header.FullSize);
	//Dbg.print(F(" RT: Data Size: "));
	//Dbg.println(send.Header.DataSize);
#endif // DBG

	auto time = send.Time.timestamp();
#if defined Ucglib_SSD1351 || defined Adfr_SSD1306
	Dsp.Show(
		send.TemperatureMeasurement,
		time.c_str(),
		false,
		send.Flags,
		send.LeftPauseMinutes, // in min, 0 - is Off
		send.HeatDestTemp,
		0);
#endif //Ucglib_SSD1351

	PublishMqtt(&send.Header, TelemetryTopic.c_str());
#ifdef DBG
	Dbg.println();
	Dbg.print(F("Relay Pin Level: "));
	Dbg.println(digitalRead(RELAY_PIN) ? F("HIGH") : F("LOW"));
#endif // DBG
}


#pragma region Pause Heating
// Pause heating if heating on process. Interval value in Hours. 25 hours MAX
void OnPauseHeatingTimer()
{
	// Stop pause
	PauseHeatingCmd(0U);
}

// Interval in hours, max 25(HEAT_MAX_PAUSE)
uint8_t PauseHeatingCmd(uint8_t interval)
{
	auto isHeatingPaused = IS_FLG_HEATING_PAUSED_YES(STAT_FLG);
	uint8_t ret = 1;

	// If Pause Heating already activated, switch it OFF first
	SmplTimer.deleteTimer(PauseHeatingTimerID);
	PauseHeatingTimerID = -1;
	RST_FLG_HEATING_PAUSE(STAT_FLG);
	
	// Activate New/Correct Pause Interval
	if (0 != interval) {
		interval = MIN(interval, HEAT_MAX_PAUSE);
		SET_FLG_HEATING_PAUSE(STAT_FLG);
		PauseHeatingTimerID = SmplTimer.setTimeout(
			(unsigned long)interval * 1000ul * 3600ul,
			OnPauseHeatingTimer);
		if (-1 == PauseHeatingTimerID) { 
			ret = 0;
		}
	}	

	// State is changed, Update the system
	if (IS_FLG_HEATING_PAUSED_YES(STAT_FLG) != isHeatingPaused) { 
		UpdateSystemStateRequestOnes();
	}

	return ret;
}
#pragma endregion

#pragma region Manual Heating
void OnManualHeatingTimer()
{
	// Stop Manual Heating
	ManualHeatingCmd(0U);
}

bool ManualHeatingCmd(uint8_t temperature)
{
	auto heatingSttBefore = IS_FLG_MANUAL_HEATING_ON(STAT_FLG);
	auto ret = false;
	// If Manual Heating already activated, switch it OFF first
	SmplTimer.deleteTimer(ManualHeatingTimerID);
	ManualHeatingTimerID = -1;
	HeatDestTemp = 0;
	RST_FLG_MANUAL_HEATING(STAT_FLG);

	// Activate Manual Heating
	if (0 != temperature)  {
		// Not valid last temperature measurement
		// System temperature measurement problem
		// Heating activation is prohibited!
		if (IS_FLG_SENSORS_FAULT_NO(STAT_FLG) && IS_FLG_HEATING_OVER_NO(STAT_FLG)) {
			auto requestTemp = MIN(MAX_AUTOMATIC_TEMP, temperature);
			auto tempAvg = TAVG;

			if (tempAvg < requestTemp) {
				auto maxInterval = MAX_MANUAL_HEATING_INTERVAL;
				SET_FLG_MANUAL_HEATING(STAT_FLG);
				ManualHeatingTimerID = SmplTimer.setTimeout(maxInterval, OnManualHeatingTimer);
				HeatDestTemp = requestTemp;
				ret = true;
			}
		}
	}
	else {
		if (IS_FLG_IN_TIME_RANGE_IN(STAT_FLG)) {
			HeatDestTemp = Scheduler.GetActiveTemperature();
		}
		
		ret = true;
	}

	// State is changed, Update the system
	if (IS_FLG_MANUAL_HEATING_ON(STAT_FLG) != heatingSttBefore) {
		UpdateSystemStateRequestOnes(); 
	}

	return ret;
}
#pragma endregion

#pragma region Auto Heating
uint8_t AutoHeatingCmd(uint8_t on)
{
	if (on) {
		if (IS_FLG_AUTOMATIC_MODE_ON(STAT_FLG)) { 
			return on; 
		}

		SET_FLG_AUTOMATIC_MODE(STAT_FLG);
		// HeatingMode flag will be set at the OnRtcRead() function if needed.
		SmplTimer.enable(RtcReadTimerID);
		SmplTimer.restartTimer(RtcReadTimerID);
		
		// Immediately checking
		SmplTimer.setTimeout(100UL, OnRTCReadTimer);
	}
	else {
		if (IS_FLG_AUTOMATIC_MODE_OFF(STAT_FLG)) {
			return on;
		}

		RST_FLG_AUTOMATIC_MODE(STAT_FLG);
		RST_FLG_AUTO_HEATING(STAT_FLG);
		// If Auto mode is OFF needs to reset Auto Heating any case.
		SmplTimer.disable(RtcReadTimerID);

		// If heating is activated and not from the manual mode, deactivate it
		//if (IS_FLG_AUTO_HEATING_ON(STAT_FLG) && IS_FLG_MANUAL_HEATING_OFF(STAT_FLG))
		//	{ RST_FLG_AUTO_HEATING(STAT_FLG); }
	}

	UpdateSystemStateRequestOnes();
	return on;
}
#pragma endregion

int cmpfunc(const void* a, const void* b)
{
	//return ((int)(*(float*)a + 0.5f) - (int)(*(float*)b + 0.5f));
	return (*(uint8_t*)a < *(uint8_t*)b ? -1 : *(uint8_t*)a > * (uint8_t*)b ? 1 : 0);
	//return (*(byte*)a > *(byte*)b ? -1 : *(byte*)a < *(byte*)b ? 1 : 0);
}//--------------------------------------------------------------------------

#ifdef SIMULATION
void OnRequestSensor()
{
#define SUN_TIME_MAX 6u // 1 MIN
	static uint8_t sunTime = 0;
	const float sunBios = 1.4f;
	static bool Sun = true;

#define CONSUMER_TIME_ON 6u // 1 MIN
#define CONSUMER_TIME_OFF 12u // 2 MIN
	static uint8_t consumerTime = 0;
	const float consumerBios = -2.5f;
	static bool Consumer = false;

	static int16_t rawSensTempr[SENSORS] = {0};
		
	const float TempDeltaSim[SENSORS] = {0.8f, 0.9f, 1.1f, 1.2f}; // Init for simulation mode
	static float fSensorsSim[SENSORS] = {28.0f, 29.0f, 32.0f, 35.0f};

	auto heatingBios = (IS_HEATING(STAT_FLG)) ? (1.6f) : (-0.2f); // Heating simulation bios

	++consumerTime;
	++sunTime;
	 
	if (sunTime > SUN_TIME_MAX) {
		Sun = !Sun;
		sunTime = 0;
	}

	if (Consumer && consumerTime > CONSUMER_TIME_ON) {
		Consumer = !Consumer;
		consumerTime = 0;
	}
	else if (!Consumer && consumerTime > CONSUMER_TIME_OFF) {
		Consumer = !Consumer;
		consumerTime = 0;
	}

	auto finBios = heatingBios + (Sun ? sunBios : 0.0f) + (Consumer ? consumerBios : 0.0f);

#ifdef DBG
	Dbg.println();
	Dbg.println("SIMULATION");
	Dbg.print(F("Sun: "));
	Dbg.print(Sun ? F("ON") : F("OFF"));
	Dbg.print(F(" Consumer: "));
	Dbg.print(Consumer ? F("ON") : F("OFF"));
	Dbg.print(F(" Heating: "));
	Dbg.println(IS_HEATING(STAT_FLG) ? F("ON") : F("OFF"));
	Dbg.print(F(" T: "));
#endif //DBG

	for (uint8_t s = 0; s < SENSORS; ++s) {
		auto biosSens = (fSensorsSim[s] + finBios * TempDeltaSim[s]);
		fSensorsSim[s] = (MIN(80.0f, max(biosSens, 16.0f)));
		auto sens = (int16_t)round(fSensorsSim[s]);
		
#ifdef DBG
		Dbg.print(sens);
		if (s < SENSORS - 1) { 
			Dbg.print(F(", "));
		}
#endif //DBG

		rawSensTempr[s] = sens << 7;
	}

#ifdef DBG
	Dbg.println();
#endif //DBG
	CalculateSensorData(rawSensTempr);

	// reread temperature sensors after 
	SmplTimer.setTimeout(UPDATE_SENSORS_DATA_DELAY, OnRequestSensor);
}

#else // DALLAS

void OnRequestSensor()
{
	// Request all sensors to perform a temperature conversion		
	Sensors.requestTemperatures(); 
	// By the data datasheet, conversion for 9Bit resolution is 94ms, but I run timer to 500ms 
	SmplTimer.setTimeout(TEMPERATURE_CONVERSION_DELAY, OnReadSensor);
}

void OnReadSensor()
{
	static int16_t rawSensTempr[SENSORS] = {0};

#ifdef DBG_READ_SENS
	Dbg.println(F("On Read Sensors"));
	Dbg.print(F("T: "));
#endif //DBG

	for (uint8_t s = 0; s < SENSORS; ++s) {
		rawSensTempr[s] = DEVICE_DISCONNECTED_RAW;
		// Try max 10 times to read sensors 
		for (uint8_t i = 0; i < 10 && DEVICE_DISCONNECTED_RAW == rawSensTempr[s]; ++i) {
			rawSensTempr[s] = Sensors.getTemp(Address[s]);
#ifdef DBG_READ_SENS
			Dbg.print(F("Sens:"));
			Dbg.print(s + 1);
			Dbg.print(F(" Tmp:"));
			Dbg.print(rawSensTempr[s]);
			Dbg.println();
#endif //DBG
		}

		if (DEVICE_DISCONNECTED_RAW == rawSensTempr[s]) {
			rawSensTempr[s] = (UNKNOWN_TEMPERATURE << 7);
		}

#ifdef DBG_READ_SENS
		Dbg.print(rawSensTempr[s] >> 7);
		if (s < SENSORS - 1) {
			Dbg.print(F(", "));
		}
#endif //DBG
	}
#ifdef DBG_READ_SENS
	Dbg.println();
#endif //DBG
	CalculateSensorData(rawSensTempr);
	// reread temperature sensors after 
	SmplTimer.setTimeout(UPDATE_SENSORS_DATA_DELAY, OnRequestSensor);
}
#endif //SIMULATION // DALLAS

void CalculateSensorData(int16_t *rawSensTempr)
{
	static uint32_t sLocalHist[SENSORS] = {0};
	uint8_t validHd = 0;
	uint16_t sum = 0;
	int16_t rawTempr = 0;

#ifdef DBG_HIST
	Dbg.println();
	Dbg.println(F("Sensors local history"));
#endif //DBG

	for (uint8_t s = 0; s < SENSORS; ++s) {
		auto rawTempr = rawSensTempr[s];
		// Free first byte of local hist for new temperature data
		sLocalHist[s] <<= 8;
		auto bLocalHistSensorElements = (uint8_t*)(sLocalHist + s);

		// >> 7 - Convert from RAW to Celsius INT (RAW/128).
		bLocalHistSensorElements[0] = (uint8_t)abs((rawTempr >> 7));

		validHd = 0;
		sum = 0.0f;
#ifdef DBG_HIST
		Dbg.print(F("Sensor: "));
		Dbg.print(s + 1);
		Dbg.print(F("  T: "));
#endif //DBG
		// Calculate average local history temperature
		for (uint8_t d = 0; d < LOCAL_HIST_DEEP; ++d) {
			if (0 != bLocalHistSensorElements[d]) {
				sum += bLocalHistSensorElements[d];
				++validHd;
			}

#ifdef DBG_HIST
			Dbg.print(bLocalHistSensorElements[d]);
			if (d < LOCAL_HIST_DEEP - 1) {
				Dbg.print(F(", "));
			}
#endif //DBG
		}

#ifdef DBG_HIST
		Dbg.println();
#endif //DBG

		if (validHd > 0) {
			bLTM[s] = (uint8_t)round((float)sum / validHd);
		}
	}

	qsort(bLTM, SENSORS, sizeof(uint8_t), cmpfunc);
	auto tempAvg = TAVG;
	RS_FG_IF(tempAvg > TEMP_HI_LEVEL_DEGREE, Flags::OverHeating, STAT_FLG);
	RS_FG_IF(tempAvg < UNKNOWN_TEMPERATURE_THRSH, Flags::SensorsFault, STAT_FLG);

	if (tempAvg > TEMP_HI_LEVEL_DEGREE) {
		RST_FLG_MANUAL_HEATING(STAT_FLG);
		RST_FLG_AUTO_HEATING(STAT_FLG);
	}

	if (IS_FLG_MANUAL_HEATING_ON(STAT_FLG) && tempAvg > HeatDestTemp) {
		ManualHeatingCmd(0);
	}

	if (IS_FLG_AUTO_HEATING_ON(STAT_FLG) && tempAvg > HeatDestTemp) {
		SmplTimer.restartTimer(RtcReadTimerID);
		OnRTCReadTimer();
	}
}

void TemperatureHistoryUpdate()
{
	// Decided not check sensors data validity.
	// If sensor data does not present (0), history get it.
	// After few not valid iterations all of the history will be reset.
	for (uint8_t h = TEMP_HIST_DEEP - 1; h > 0; --h) {
		// Move row of the history right, last one will be missing
		TEMP_HISTORY[h] = TEMP_HISTORY[h - 1];
	}
	TEMP_HISTORY[0] = LastTemperatureMeasurement;
}

#pragma region OnRTCReadTimer
void OnRTCReadTimer()
{
#ifdef DBG_RTC
	Dbg.println();
	Dbg.println(F("RTC Callback"));
#endif // DBG_RTC

	if (!SetupDone || IS_FLG_AUTOMATIC_MODE_OFF(STAT_FLG)) {
		return;
	}

	if (IS_FLG_SENSORS_FAULT_YES(STAT_FLG)) {
		UpdateSystemStateRequestOnes();
		return;
	}

	auto dt = RtcNow();

	auto wday = dt.dayOfTheWeek();
	auto hour = dt.hour();
	auto min = dt.minute();

	auto day = dt.day();
	auto month = dt.month();
	auto year = dt.year();
	auto tempAvg = TAVG;

#ifdef DBG_RTC
	String time = F("Current Time: ");
	ToolStr::AddDateTimeToStr(time, &dt, true);
	Dbg.println(time);
#endif // DBG_RTC

	// !!!! UNKNOWN temperature !!!!
	if (tempAvg < UNKNOWN_TEMPERATURE_THRSH) { 
		return;
	}

	auto heatStt = Scheduler.HeatingStatus(wday, hour, tempAvg);
		
#ifdef DBG
	auto dowStr = ToolStr::MsgGet((MMSG)(wday + (uint8_t)MMSG::STR_WEEK));
	Dbg.println();
	Dbg.print(F("Time: "));
	Dbg.println(dt.timestamp());

	//Dbg.print(dt.hour());
	//Dbg.print(F(":"));
	//Dbg.print(dt.minute());
	//Dbg.print(F(" - "));
	//Dbg.print(dt.day());
	//Dbg.print(F("."));
	//Dbg.print(dt.month());
	//Dbg.print(F("."));
	//Dbg.println(dt.year());

	//Dbg.print(hour, DEC);
	Dbg.print(F(" DOW: "));
	Dbg.print(dowStr);
	Dbg.print(F("("));
	Dbg.print(wday);
	Dbg.println(F(")"));
	Dbg.print(F(" Average Temp: "));
	Dbg.println(tempAvg, DEC);
	Dbg.print(F(" Scheduled Heating: ")); 
	Dbg.println((heatStt & eSchFlag::Heating) != 0 ? F("ON") : F("OFF"));
	Dbg.print(F(" In Time: "));
	Dbg.println((heatStt & eSchFlag::OnTime) != 0 ? F("Y") : F("N"));
	Dbg.print(F(" Scheduler Paused: "));
	Dbg.println((heatStt & eSchFlag::Paused) != 0 ? F("Y") : F("N"));
	Dbg.println();
#endif // DBG

	auto currHeatingStt = IS_HEATING(STAT_FLG);
	
	// IN TIME RANGE
	RS_FG_IF(0 != (heatStt & eSchFlag::OnTime), 
		Flags::InTimeRange, STAT_FLG); 
	
	// SCHEDULER IS PAUSED
	RS_FG_IF(0 != (heatStt & eSchFlag::Paused), 
		Flags::SchedulePaused, STAT_FLG); 

	// HEATING FLAG
	RS_FG_IF(0 != (heatStt & eSchFlag::Heating) && 
			 IS_FLG_IN_TIME_RANGE_IN(STAT_FLG) &&
			 IS_FLG_HEATING_OVER_NO(STAT_FLG), 
			 Flags::AutoHeating, STAT_FLG);

	// If flag Manual heating is OFF, update Destination temperature from Scheduler
	if (IS_FLG_MANUAL_HEATING_OFF(STAT_FLG)) { 
		HeatDestTemp = Scheduler.GetActiveTemperature();
	}

	// Detected Heating status was changed
	// Call update system state function, restart regular UpdateSystemStateTimer before,
	// for preventing function double calling. 
	if (IS_HEATING(STAT_FLG) != currHeatingStt) { 
		UpdateSystemStateRequestOnes(); 
	}
}
#pragma endregion

#pragma region Update System State
void UpdateSystemStateRequestOnes()
{
	SmplTimer.restartTimer(UpdateSystemStateTimerID);
	SmplTimer.setTimeout(UPDATE_SYSTEM_STATE_INTERVAL_ONES, _onUpdateSystemState);
}

void _onUpdateSystemState()
{
	static uint16_t prevFlag = 0;
	auto heatingSttNow = IS_HEATING(STAT_FLG);

	if (IS_FLG_HEATING_OVER_NO(STAT_FLG) && IS_FLG_SENSORS_FAULT_NO(STAT_FLG)) { 
		digitalWrite(RELAY_PIN, heatingSttNow ? LOW : HIGH);
	}
	else if (LOW == digitalRead(RELAY_PIN)) {
		// Problem condition
		digitalWrite(RELAY_PIN, HIGH);
#ifdef DBG_SELF_CHECKING
		Dbg.println(F("Self Checking Control - Overheat or Sensor reading fault. Relay: OFF"));
#endif // DBG_SELF_CHECKING
	}

	if (heatingSttNow != IS_HEATING(prevFlag)) {
		SetHeatingHistory(heatingSttNow, prevFlag);
		prevFlag = STAT_FLG;

#ifdef DBG
		Dbg.println();
		Dbg.println(F("Heating Status Changed"));
#endif // DBG
	}

	PublishTelemetry(true);
}
#pragma endregion

void SetHeatingHistory(bool on, uint16_t flagPrev)
{
	auto dt = RtcNow();

	// Just update existing first history slot
	if (!on) {
		if (HEAT_HISTORY[0].IsValid()) {
			HEAT_HISTORY[0].Update(false, dt, STAT_FLG, flagPrev);
#ifdef DBG_HIST
			Dbg.println();
			Dbg.println(F("Set To History Slot#0: OFF"));
#endif // DBG_HIST
		}
	}
	// Move the history, empty first history slot and update it with new data
	else {
		HistDateTime htm(on, dt, STAT_FLG, flagPrev);

		for (int8_t h = HEAT_HIST_DEEP - 1; h > 0; --h) {
			// Move row of the history right, last one will be missing
			HEAT_HISTORY[h] = HEAT_HISTORY[h - 1];
		}
		HEAT_HISTORY[0] = htm;
#ifdef DBG_HIST
		Dbg.println();
		Dbg.println(F("Set To History Slot#0: ON"));
#endif // DBG_HIST
	}
}

//==================================== FLOW CONTROL =================================
#ifdef FLOW
void OnFlowControl()
{
	// 10% is inaccuracy
	const float inaccuracyPercents = 10.0f;

	detachInterrupt(FLOW_SENSOR_INP);
	detachInterrupt(FLOW_SENSOR_OUT);
	int16_t deltaFlow = (InpFlowPulse - OutFlowPulse);
	// Really flow is detected and out flow is more
	if (InpFlowPulse > 3 && deltaFlow > 1) {
		// allow inaccuracy delta inaccuracyPercents % from the input flow value
		auto deltaF = (InpFlowPulse * inaccuracyPercents) / 100.0f;

		// With weak flow allow 3 peaks difference to error or lag
		auto allowDeltaFlow = (int16_t)(MIN(3.0, deltaF) + 0.5f);

#ifdef DBG_
		Dbg.println();
		Dbg.print(F("Delta Flow Detected: "));
		Dbg.print(deltaFlow);
		Dbg.print(F(" Allow: "));
		Dbg.print(allowDeltaFlow);
		Dbg.print(F("  Status: "));
		Dbg.println(deltaFlow > allowDeltaFlow ? F("ALARM") : F("OK"));
#endif // DBG

		// Stop water supplying !!!!
		if (deltaFlow > allowDeltaFlow) {
		}
	}

	attachInterrupt(FLOW_SENSOR_INP, InpFlowPulseCounter, RISING);
	attachInterrupt(FLOW_SENSOR_OUT, OutFlowPulseCounter, RISING);
}

// Returns true if peaks counts are the same or not more than
// inaccuracyPercents Delta
bool ComparePulses(float inaccuracyPercents = 10.0f)
{
	if (0 == InpFlowPulse && 0 == OutFlowPulse) {
		return true;
	}

	auto maxPeaks = max(InpFlowPulse, OutFlowPulse);
	auto minPeaks = MIN(InpFlowPulse, OutFlowPulse);
	auto deltaF = (maxPeaks * inaccuracyPercents) / 100.0f;

	// With weak flow allow 3 peaks difference to error or lag
	auto delta = (uint16_t)(MIN(3.0, deltaF) + 0.5f);
	return ((maxPeaks - minPeaks) < delta);
}

// interrupt service routine
void InpFlowPulseCounter()
{
	// increment the pulse counter
	++InpFlowPulse;
}

// interrupt service routine
void OutFlowPulseCounter()
{
	// increment the pulse counter
	++OutFlowPulse;
}
#endif // FLOW

DateTime RtcNow()
{
	auto epochTime = timeClient.getEpochTime();
	auto dt = DateTime(epochTime, true);
	#ifdef DBG_RTC
		Dbg.print("RtcNow() => ");
		Dbg.println(dt.timestamp());
	#endif // dt

	return dt;
}

void PublishMqtt(BHeader *header, const char* topic)
{
	auto toSend = header->FullSize;
	auto arrPtr = (char*)header;

#ifdef AsyncMqtt
	if (!mqttAsyncClient.connected()) {
#else
	if (!mqttClient.connected()) {
#endif // AsyncMqtt

#ifdef DBG
		Dbg.println();
		Dbg.println(F("MQTT Client is NOT connected"));
#endif // DBG
		return;
	}

	// Now publish the char buffer to Beebotte
#ifdef AsyncMqtt
	auto rc = mqttAsyncClient.publish(topic, 0, true, (char*)header, toSend);
#else
	auto rc = mqttClient.publish_P(topic, (byte*)header, toSend, true);
#endif // AsyncMqtt

	
#ifdef DBG
	Dbg.println();
	Dbg.println(F("Publish:"));
	Dbg.print(F(" Topic: "));
	Dbg.println(topic);
	Dbg.print(F(" Opcode: "));
	Dbg.println(ToolStr::CmdGet(header->OpCode));
	Dbg.print(F(" Size: "));
	Dbg.println(toSend);

	if (rc) {
		Dbg.println(F("== Success =="));
	}
	else {
		Dbg.println();
		auto plannedSend = strlen(topic) + 4 + toSend;
		Dbg.println(F("!!! Something wrong !!! "));
		Dbg.print(F("Tried to Publish: "));
		Dbg.println(plannedSend);
	}
#endif // DBG
}

void LedOff()
{
	digitalWrite(BLUE_LED, LOW);
}

void WiFiEvent(WiFiEvent_t event) 
{
#ifdef DBG
	Dbg.printf("[WiFi-event] event: %d\n", event);
#endif // DBG
	switch (event) {
	case SYSTEM_EVENT_STA_GOT_IP:
#ifdef DBG
		Dbg.println("WiFi connected");
		Dbg.print(" IP address: ");
		Dbg.println(WiFi.localIP());
		Dbg.print(" SSID: ");
		Dbg.println(WiFi.SSID());
		Dbg.print(" Channel: ");
		Dbg.println(WiFi.channel());
#endif // DBG
		_connectToTimeClient();
		_connectToMqtt();	
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
#ifdef DBG
		Dbg.println("WiFi lost connection");
#endif // DBG
#ifdef Adfr_SSD1306
		auto wifi = "Wifi: " + WiFi.SSID();
		Dsp.ShowInfo(
			0, 20, wifi.c_str(),
			0, 50, "Connection Lost");
#endif //Adfr_SSD1306
		timeClient.end();
		SmplTimer.deleteTimer(_mqttReconnectTimer);
		_wifiReconnectTimer = SmplTimer.setTimeout(WIFI_RECONNECT_INTERVAL, _connectToWifi);
		break;
	}
}

void _connectToWifi() 
{
	_wifiReconnectTimer = -1;
#ifdef DBG
	Dbg.println("Connecting to Wi-Fi...");
#endif // DBG
	WiFi.begin(WifiSsid.c_str(), WifiPswd.c_str());
}

void _connectToTimeClient()
{
#ifdef DBG
	Dbg.println("Connecting to Time Client...");
#endif // DBG
	timeClient.begin();
}

void _forceUpdateTimeClient()
{
	_timeClientForceUpdateTimer = -1;
#ifdef DBG
	Dbg.println("Force updating Time Client...");
#endif // DBG
	timeClient.forceUpdate();
}

void _connectToMqtt() 
{
#ifdef DBG
	Dbg.println("Connecting to MQTT...");
#endif // DBG

#ifdef AsyncMqtt
	mqttAsyncClient.connect();
#else
	if (mqttClient.connect(MqttClient.c_str(), MqttToken.c_str(), "")) {
		mqttClient.subscribe(RxTopic.c_str());
		_wasConnected = true;
		_mqttReconnectTimer = -1;
#ifdef DBG
		Dbg.println(F("MQTT Successfully Connected"));
#endif // DBG
	}
	else {	
#ifdef DBG
		Dbg.println(F("MQTT NOT Connected. Reconnecting...."));
#endif // DBG
		_mqttReconnectTimer = SmplTimer.setTimeout(MQTT_RECONNECT_INTERVAL, _connectToMqtt);
	}
#endif // AsyncMqtt
}

#ifdef AsyncMqtt
void onMqttConnect(bool sessionPresent) 
{
#ifdef DBG
	Dbg.println("Connected to MQTT.");
	Dbg.print("Session present: ");
	Dbg.println(sessionPresent);
#endif // DBG
	auto packetIdSub = mqttAsyncClient.subscribe(RxTopic.c_str(), 0);
#ifdef DBG
	Dbg.print("Subscribing at QoS 0, packetId: ");
	Dbg.println(packetIdSub);
#endif // DBG
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) 
{
#ifdef DBG
	Dbg.println("Disconnected from MQTT.");
#endif // DBG
	if (WiFi.isConnected()) {
		_mqttReconnectTimer = SmplTimer.setTimeout(MQTT_RECONNECT_INTERVAL, _connectToMqtt);
	}
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) 
{
#ifdef DBG
	Dbg.println("Subscribe acknowledged.");
	Dbg.print("  packetId: ");
	Dbg.println(packetId);
	Dbg.print("  qos: ");
	Dbg.println(qos);
#endif // DBG
}

void onMqttUnsubscribe(uint16_t packetId) 
{
#ifdef DBG
	Dbg.println("Unsubscribe acknowledged.");
	Dbg.print("  packetId: ");
	Dbg.println(packetId);
#endif // DBG
}

void onMqttPublish(uint16_t packetId) 
{
#ifdef DBG
	Dbg.println("Publish acknowledged.");
	Dbg.print("  packetId: ");
	Dbg.println(packetId);
#endif // DBG
}

void onMqttMessage(
	char* topic,
	char* payload,
	AsyncMqttClientMessageProperties properties,
	size_t length,
	size_t index,
	size_t total)
#else
void onMqttMessage(
	char* topic,
	uint8_t* payload,
	unsigned int length)
#endif // AsyncMqtt
{
	digitalWrite(BLUE_LED, HIGH);
	SmplTimer.setTimeout(500UL, LedOff);
	auto header = (BHeader*)payload;

#ifdef DBG
	Dbg.println();
	Dbg.println(F("Message received:"));
	Dbg.print(F(" Topic: "));
	Dbg.println(topic);
	Dbg.print(F(" Length: "));
	Dbg.println(length);
#ifdef AsyncMqtt
	Dbg.print(F(" qos: "));
	Dbg.println(properties.qos);
	Dbg.print(F(" dup: "));
	Dbg.println(properties.dup);
	Dbg.print(F(" retain: "));
	Dbg.println(properties.retain);
	Dbg.print(F(" index: "));
	Dbg.println(index);
	Dbg.print(F(" total: "));
	Dbg.println(total);
#endif // AsyncMqtt
	Dbg.print(F(" App ID: "));
	Dbg.println(header->Id, HEX); // 0x3A505041 // 978,341,953
	Dbg.print(F(" Opcode: "));
	Dbg.println(ToolStr::CmdGet(header->OpCode));
	Dbg.print(F(" Data Size: "));
	Dbg.println(header->DataSize);
#endif // DBG

	switch (header->OpCode) {
	case Opcode::CMD_RT: // Read Telemetry. Key: [RT], Return Value: [Flags][Temperature Sensors]
	{
		PublishTelemetry(false);
		break;
	}
	case Opcode::CMD_HT: // Get Temperature History. Key: [HT]
	{
		uint8_t h = 0;
		for (; h < TEMP_HIST_DEEP && 0 != TEMP_HISTORY[h]; ++h);
		auto send = BHistTemp(TEMP_HISTORY, h);
		PublishMqtt(&send.Header, HistoryTTopic.c_str());
		break;
	}
	case Opcode::CMD_HH: // Get Heating History. Key: [HH]
	{
		uint8_t h = 0;
		for (; h < HEAT_HIST_DEEP && HEAT_HISTORY[h].IsValid(); ++h);
		auto send = BHistHeat(HEAT_HISTORY, h);
		PublishMqtt(&send.Header, HistoryHTopic.c_str());
		break;
	}
	case Opcode::CMD_AH: // Automatic(Scheduled) Heating ON/OFF. Key: [AH], Value: [1/0]
	{
		if (0 == header->DataSize) {
			return;
		}

		auto cmd = (BCommand*)header;
		AutoHeatingCmd(cmd->Command);
		PublishTelemetry(true);
		break;
	}
	case Opcode::CMD_MH: // Manual Heating ON/OFF. Key: [MH], Value: [Temperature] or MAX_MANUAL_HEATING_INTERVAL- 2 hours
	{
		if (0 == header->DataSize) {
			return;
		}

		auto cmd = (BCommand*)header;
		ManualHeatingCmd(cmd->Command);
		PublishTelemetry(true);
		break;
	}
	case Opcode::CMD_PH: // Pause Heating. Key: [PH], Value: [Interval(hours)], 0-OFF, max-25(HEAT_MAX_PAUSE)
	{
		if (0 == header->DataSize) {
			return;
		}

		auto cmd = (BCommand*)header;
		PauseHeatingCmd(cmd->Command);
		PublishTelemetry(true);
		break;
	}
	case Opcode::CMD_GS: // Get Schedule. Key: [GS], Return Value: All of the Schedules
	{
		auto send = BSchedule(Scheduler.GetJobs());
		PublishMqtt(&send.Header, ScheduleTopic.c_str());
		break;
	}
	case Opcode::CMD_US: // Update Schedule. Key: [US], Input Value: [ID,DOW,Fr,To,Tmp], Return Value: Schedule ID
	{
		if (0 == header->DataSize) {
			return;
		}

#ifdef DBG
		Dbg.println();
		Dbg.println(F("US: Updated Schedules are received"));
#endif // DBG

		auto cmd = (BSchedule*)header;
		for (size_t i = 0; i < NUM_SCHEDULES; ++i) {
			auto& sch = cmd->Schedulers[i];
			if (!Scheduler.IsEqual(i, sch)) {
				Scheduler.SetSchedule(i, sch);
#ifdef DBG
				Dbg.print(F("US: Updated Schedule Index: "));
				Dbg.println(i);
#endif // DBG
			}
		}

		auto send = BSchedule();
		PublishMqtt(&send.Header, ScheduleTopic.c_str());
		break;
	}
	//	case Opcode::CMD_DS: // Reset all of Schedules to a default(from EPROM). Key: [DS], Return Value: All of the Schedules
	//	{
	//		Scheduler.SetDefaultSchedules(SCHEDULER_CODE);
	//		dataSize = FillSchedulerToBinArr(arr);
	//#ifdef DBG
	//		Dbg.println();
	//		Dbg.print(F("DS: wrote to protocol bytes: "));
	//		Dbg.println(dataSize);
	//#endif // DBG
	//		break;
	//	}
	//	case Opcode::CMD_AS: // Activate Schedule. Key: [AS], Input Value: [ID,DOW], Return Value: Schedule ID
	//	{
	//		if (0 == dataSize) { 
	//			return; 
	//		}
	//
	//		auto val1 = arr.GetVal<uint8_t>(_DATA_START_INDEX);
	//		auto val2 = arr.NextVal<uint8_t>();
	//		auto resu = Scheduler.ActivateSchedule(val1, val2);
	//		dataSize = FillSchedulerToBinArr(arr);
	//#ifdef DBG
	//		Dbg.println();
	//		Dbg.print(F("AS: wrote to protocol bytes: "));
	//		Dbg.println(dataSize);
	//#endif // DBG
	//		break;
	//	}
	//	case Opcode::CMD_RS: // Remove Schedule. Key: [RS], Input Value: [ID]
	//	{
	//		if (0 == dataSize) { 
	//			return; 
	//		}
	//
	//		auto val = arr.GetVal<uint8_t>(_DATA_START_INDEX);
	//		Scheduler.RemoveSchedule(val);
	//		dataSize = FillSchedulerToBinArr(arr);
	//#ifdef DBG
	//		Dbg.println();
	//		Dbg.print(F("RS: wrote to protocol bytes: "));
	//		Dbg.println(dataSize);
	//#endif // DBG
	//		break;
	//	}
	//	case Opcode::CMD_PS: // Pause Schedule. Pause automatic heating (any case) until the current job ends. Key: [PH]
	//	{
	//		if (0 == dataSize) {
	//			return;
	//		}
	//
	//		auto pause = arr.GetVal<uint8_t>(_DATA_START_INDEX);
	//		Scheduler.Pause(pause);
	//		RS_FG_IF(pause, Flags::SchedulePaused, STAT_FLG); // SCHEDULER IS PAUSED
	//		arr.ClearData();
	//		arr.Add(STAT_FLG);
	//		pause = Scheduler.LeftPauses();
	//		arr.Add(pause);
	//		dataSize = arr.PinEndData();
	//#ifdef DBG
	//		Dbg.println();
	//		Dbg.print(F("PS: wrote to protocol bytes: "));
	//		Dbg.println(dataSize);
	//#endif // DBG
	//		break;
	//	}
	default:
		break;
	}
}

