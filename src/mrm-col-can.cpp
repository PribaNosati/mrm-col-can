#include "mrm-col-can.h"
#include <mrm-robot.h>

std::map<int, std::string>* Mrm_col_can::commandNamesSpecific = NULL;

/** Constructor
@param robot - robot containing this board
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_col_can::Mrm_col_can(uint8_t maxNumberOfBoards) : 
	SensorBoard(1, "Color", maxNumberOfBoards, ID_MRM_COL_CAN, MRM_COL_CAN_COLORS) {
	readings = new std::vector<uint16_t[MRM_COL_CAN_COLORS]>(maxNumberOfBoards);
	_hsv = new std::vector<bool>(maxNumberOfBoards);
	_hue = new std::vector<uint8_t>(maxNumberOfBoards);
	_saturation = new std::vector<uint8_t>(maxNumberOfBoards);
	_value = new std::vector<uint8_t>(maxNumberOfBoards);
	_patternByHSV = new std::vector<uint8_t>(maxNumberOfBoards);
	_patternBy6Colors = new std::vector<uint8_t>(maxNumberOfBoards);
	_patternRecognizedAtMs = new std::vector<uint32_t>(maxNumberOfBoards);


	if (commandNamesSpecific == NULL){
		commandNamesSpecific = new std::map<int, std::string>();
		commandNamesSpecific->insert({CAN_COL_SENDING_COLORS_1_TO_3, "Send 1-3"});
		commandNamesSpecific->insert({CAN_COL_SENDING_COLORS_4_TO_6, "Send 4-6"});
		commandNamesSpecific->insert({CAN_COL_ILLUMINATION_CURRENT, 	"Ill curre"});
		commandNamesSpecific->insert({CAN_COL_SWITCH_TO_HSV, 		"To HSV"});
		commandNamesSpecific->insert({CAN_COL_SWITCH_TO_6_COLORS, 	"To 6 colo"});
		commandNamesSpecific->insert({CAN_COL_SENDING_HSV, 			"Send HSV"});
		commandNamesSpecific->insert({CAN_COL_INTEGRATION_TIME, 		"Inte time"});
		commandNamesSpecific->insert({CAN_COL_GAIN, 					"Gain"});
		commandNamesSpecific->insert({CAN_COL_PATTERN_RECORD, 		"Patt reco"});
		commandNamesSpecific->insert({CAN_COL_PATTERN_SENDING, 		"Patt send"});
		commandNamesSpecific->insert({CAN_COL_PATTERN_REQUEST, 		"Patt requ"});
		commandNamesSpecific->insert({CAN_COL_PATTERN_ERASE, 		"Patt eras"});
	}
}

Mrm_col_can::~Mrm_col_can()
{
}

/** Add a mrm-col-can sensor
@param deviceName - device's name
*/
void Mrm_col_can::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_COL_CAN0_IN;
		canOut = CAN_ID_COL_CAN0_OUT;
		break;
	case 1:
		canIn = CAN_ID_COL_CAN1_IN;
		canOut = CAN_ID_COL_CAN1_OUT;
		break;
	case 2:
		canIn = CAN_ID_COL_CAN2_IN;
		canOut = CAN_ID_COL_CAN2_OUT;
		break;
	case 3:
		canIn = CAN_ID_COL_CAN3_IN;
		canOut = CAN_ID_COL_CAN3_OUT;
		break;
	case 4:
		canIn = CAN_ID_COL_CAN4_IN;
		canOut = CAN_ID_COL_CAN4_OUT;
		break;
	case 5:
		canIn = CAN_ID_COL_CAN5_IN;
		canOut = CAN_ID_COL_CAN5_OUT;
		break;
	case 6:
		canIn = CAN_ID_COL_CAN6_IN;
		canOut = CAN_ID_COL_CAN6_OUT;
		break;
	case 7:
		canIn = CAN_ID_COL_CAN7_IN;
		canOut = CAN_ID_COL_CAN7_OUT;
		break;
	default:
		sprintf(errorMessage, "Too many %s: %i.", _boardsName.c_str(), nextFree);
		return;
	}

	//for (uint8_t i = 0; i < MRM_NODE_SWITCHES_COUNT; i++)
	//	(*reading)[nextFree][i] = 0;

	SensorBoard::add(deviceName, canIn, canOut);
}

/** Blue
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_can::colorBlue(Device& device) {
	if (colorsStarted(device))
		return(*readings)[device.number][0];
	else
		return 0;
}

/** Green
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_can::colorGreen(Device& device) {
	if (colorsStarted(device))
		return(*readings)[device.number][1];
	else
		return 0;
}

/** Orange
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_can::colorOrange(Device& device) {
	if (colorsStarted(device))
		return(*readings)[device.number][2];
	else
		return 0;
}

/** Red
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_can::colorRed(Device& device) {
	if (colorsStarted(device))
		return(*readings)[device.number][3];
	else
		return 0;
}

/** If 6-colors mode not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_col_can::colorsStarted(Device& device) {
	if ((*_hsv)[device.number] || millis() - device.lastReadingsMs > MRM_COL_CAN_INACTIVITY_ALLOWED_MS || device.lastReadingsMs == 0) {
		//print("Switch to 6 col. %i %i \n\r", (*_hsv)[deviceNumber], (*_last6ColorsMs)[deviceNumber]);
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			switchTo6Colors(&device);
			// Wait for 1. message.
			uint64_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - device.lastReadingsMs < 100) {
					//print("6co confirmed\n\r");
					return true;
				}
				delay(1);
			}
		}
		sprintf(errorMessage, "%s %i dead.", _boardsName.c_str(), device.number);
		return false;
	}
	else
		return true;
}

/** Violet
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_can::colorViolet(Device& device) {
	if (colorsStarted(device))
		return(*readings)[device.number][4];
	else
		return 0;
}

/** Yellow
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - color intensity
*/
uint16_t Mrm_col_can::colorYellow(Device& device) {
	if (colorsStarted(device))
		return(*readings)[device.number][5];
	else
		return 0;
}


std::string Mrm_col_can::commandName(uint8_t byte){
	auto it = commandNamesSpecific->find(byte);
	if (it == commandNamesSpecific->end())
		return "Warning: no command found for key " + (int)byte;
	else
		return it->second;//commandNamesSpecific->at(byte);
}

/** Set gain
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param gainValue:
	0, 1x (default)
	1, 3.7x
	2, 16x
	3, 64x
*/
void Mrm_col_can::gain(Device * device, uint8_t gainValue) {
	if (device == nullptr)
		for (Device& dev : devices)
			gain(&dev, gainValue);
	else {
		canData[0] = CAN_COL_GAIN;
		canData[1] = gainValue;
		messageSend(canData, 2, device->number);
	}
}

/** If HSV not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_col_can::hsvStarted(Device& device) {
	if (!(*_hsv)[device.number] || millis() - device.lastReadingsMs > MRM_COL_CAN_INACTIVITY_ALLOWED_MS || device.lastReadingsMs == 0) {
		//print("Switch to HSV.\n\r");

		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			switchToHSV(&device);
			// Wait for 1. message.
			uint64_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - device.lastReadingsMs < 100) {
					//print("HSV confirmed\n\r");
					return true;
				}
				delay(1);
			}
		}
		sprintf(errorMessage, "%s %i dead.", _boardsName.c_str(), device.number);
		return false;
	}
	else
		return true;
}

/** Hue
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - Hue
*/
uint8_t Mrm_col_can::hue(Device * device) {
	if (hsvStarted(*device))
		return (*_hue)[device->number];
	else
		return 0;
}

/** Set illumination intensity
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param current - 0 - 3
*/
void Mrm_col_can::illumination(Device * device, uint8_t current) {
	if (device == nullptr)
		for(Device& dev : devices)
			illumination(&dev, current);
	else {
		canData[0] = CAN_COL_ILLUMINATION_CURRENT;
		canData[1] = current;
		messageSend(canData, 2, device->number);
	}
}

/** Set integration time
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - all sensors.
@param value - integration time will be value x 2.8 ms but double that in case of mode 2 (usual). value is between 0 and 255. value 18 is approx 10 FPS
*/
void Mrm_col_can::integrationTime(Device * device, uint8_t value) {
	if (device == nullptr)
		for (Device& dev : devices)
			integrationTime(&dev, value);
	else {
		canData[0] = CAN_COL_INTEGRATION_TIME;
		canData[1] = value;
		messageSend(canData, 2, device->number);
	}
}

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
*/
bool Mrm_col_can::messageDecode(CANMessage& message) {
for (Device& device : devices)
		if (isForMe(message.id, device)) {
			if (!messageDecodeCommon(message, device)) {
//				bool any = false;
//				uint8_t startIndex = 0;
				switch (message.data[0]) {
				case CAN_COL_PATTERN_SENDING:
					print("Sensor %i, pattern %i: %i/%i/%i (H/S/V)\n\r", device.number, message.data[1], message.data[2], message.data[3], message.data[4]);
					break;
				case COMMAND_SENSORS_MEASURE_SENDING:
//					startIndex = 0;
//					any = true;
					break;
				case CAN_COL_SENDING_COLORS_1_TO_3:
					(*readings)[device.number][0] = (message.data[1] << 8) | message.data[2]; // blue
					(*readings)[device.number][1] = (message.data[3] << 8) | message.data[4]; // green
					(*readings)[device.number][2] = (message.data[5] << 8) | message.data[6]; // orange
//					any = true;
					break;
				case CAN_COL_SENDING_COLORS_4_TO_6:
					(*readings)[device.number][3] = (message.data[1] << 8) | message.data[2]; // red
					(*readings)[device.number][4] = (message.data[3] << 8) | message.data[4]; // violet
					(*readings)[device.number][5] = (message.data[5] << 8) | message.data[6]; // yellow
					(*_patternByHSV)[device.number] = message.data[7] & 0xF;
					(*_patternBy6Colors)[device.number] = message.data[7] >> 4;
//					any = true;
					device.lastReadingsMs = millis();
					//print("RCV 6 col%i\n\r", (*_last6ColorsMs)[deviceNumber]);
					break;
				case CAN_COL_SENDING_HSV:
					(*_hue)[device.number] = (message.data[1] << 8) | message.data[2];
					(*_saturation)[device.number] = (message.data[3] << 8) | message.data[4];
					(*_value)[device.number] = (message.data[5] << 8) | message.data[6];
					(*_patternByHSV)[device.number] = message.data[7] & 0xF;
					(*_patternBy6Colors)[device.number] = message.data[7] >> 4;
					(*_patternRecognizedAtMs)[device.number] = millis();
					device.lastReadingsMs = millis();
					//print("RCV HSV%i\n\r", (*_lastHSVMs)[deviceNumber]);
					break;
				default:
					errorAdd(message, ERROR_COMMAND_UNKNOWN, false, true);
				}
			}
			return true;
		}
	return false;
}

/** Erase all patterns
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - in all sensors.
*/
void Mrm_col_can::patternErase(Device * device) {
	if (device == nullptr)
		for (Device& dev : devices)
			patternErase(&dev);
	else {
		canData[0] = CAN_COL_PATTERN_ERASE;
		messageSend(canData, 1, device->number);
	}
}

/** Print HSV patterns
*/
void Mrm_col_can::patternPrint() {
	for (Device& dev : devices) {
		canData[0] = CAN_COL_PATTERN_REQUEST;
		messageSend(canData, 1, dev.number);
	}
}


/** Choose a pattern closest to the current 6 colors
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - patternNumber
*/
uint8_t Mrm_col_can::patternRecognizedBy6Colors(Device& device) {
	if (hsvStarted(device))
		return (*_patternBy6Colors)[device.number];
	else
		return 0;
}

/** Choose a pattern closest to the current HSV values
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param includeValue - if true, HSV compared. If not, HS.
@raturn - patternNumber
*/
uint8_t Mrm_col_can::patternRecognizedByHSV(Device& device) {
	if (hsvStarted(device))
		return (*_patternByHSV)[device.number];
	else
		return 0;
}

/** Record a HSV pattern
@param patternNumber - 0 - MRM_COL_CAN_PATTERN_COUNT-1
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_col_can::patternRecord(uint8_t patternNumber, Device& device) {
	if (!_hsv || patternNumber >= MRM_COL_CAN_PATTERN_COUNT || device.number >= devices.size()) {
		strcpy(errorMessage, "Patt. err.");
		return;
	}
	canData[0] = CAN_COL_PATTERN_RECORD;
	canData[1] = patternNumber;
	messageSend(canData, 2, device.number);
}

/** Record patterns manually
*/
void Mrm_col_can::patternsRecord() {
	// Select device
	uint8_t sensorsAlive = count();
	print("Enter sensor id [0..%i]: ", sensorsAlive - 1);
	uint16_t deviceNumber = serialReadNumber(8000, 500, nextFree - 1 <= 9, sensorsAlive - 1, true);
	if (deviceNumber == 0xFFFF) {
		print("Exit\n\r");
		return;
	}
	print("%i\n\r", deviceNumber);
	// Select pattern
	print("Enter pattern id [0..%i]: ", MRM_COL_CAN_PATTERN_COUNT - 1);
	uint16_t patternNumber = serialReadNumber(8000, 500, MRM_COL_CAN_PATTERN_COUNT - 1 <= 9, MRM_COL_CAN_PATTERN_COUNT - 1, true);
	if (patternNumber == 0xFFFF) {
		print("Exit\n\r");
		return;
	}
	print("%i\n\r", patternNumber);
	patternRecord(patternNumber, devices[deviceNumber]);
}


/** Analog readings
@param colorId - one of 6 colors
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_col_can::reading(uint8_t colorId, Device * device) {
	if (device == nullptr || colorId >= MRM_COL_CAN_COLORS) {
		sprintf(errorMessage, "%s %i doesn't exist.", _boardsName.c_str(), device->number);
		return 0;
	}
	if (colorsStarted(*device))
		return (*readings)[device->number][colorId];
	else
		return 0;
}

/** Print all readings in a line
*/
void Mrm_col_can::readingsPrint() {
	print("Colors:");
	for (Device& device: devices)
		for (uint8_t colorId = 0; colorId < MRM_COL_CAN_COLORS; colorId++)
			print(" %3i", (*readings)[device.number][colorId]);
}


/** Saturation
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - saturation
*/
uint8_t Mrm_col_can::saturation(Device * device) {
	if (hsvStarted(*device))
		return (*_saturation)[device->number];
	else
		return 0;
}

/** Instruction to sensor to switch to converting R, G, and B on board and return hue, saturation and value
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0. 0xFF - all sensors.
*/
void Mrm_col_can::switchToHSV(Device * device) {
	if (device == nullptr)
		for(Device& dev : devices)
			switchToHSV(&dev);
	else {
		canData[0] = CAN_COL_SWITCH_TO_HSV;
		messageSend(canData, 1, device->number);
		(*_hsv)[device->number] = true;
	}
}


/** Instruction to sensor to start returning 6 raw colors
@param sensorNumber - Sensor's ordinal number. Each call of function add() assigns a increasing number to the sensor, starting with 0. 0xFF - all sensors.
*/
void Mrm_col_can::switchTo6Colors(Device * device) {
	if (device == nullptr)
		for(Device& dev : devices)
			switchTo6Colors(&dev);
	else {
		canData[0] = CAN_COL_SWITCH_TO_6_COLORS;
		messageSend(canData, 1, device->number);
		(*_hsv)[device->number] = false;
	}
}


/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
@param hsv - if not, then 6 colors
*/
void Mrm_col_can::test(bool hsvSelect)
{
	static uint64_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (Device& device : devices) {
			if (device.alive) {
				if (pass++)
					print(" | ");
				if (hsvSelect)
					print("HSV:%3i/%3i/%3i HSV/col:%i/%i", hue(&device), saturation(&device), value(device), patternRecognizedByHSV(device),
					patternRecognizedBy6Colors(device));
				else
					print("Bl:%3i Gr:%3i Or:%3i Re:%3i Vi:%3i Ye:%3i", colorBlue(device), colorGreen(device), colorOrange(device), colorRed(device),
						colorViolet(device), colorYellow(device));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}

/** Value
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - value
*/
uint8_t Mrm_col_can::value(Device& device) {
	if (hsvStarted(device))
		return (*_value)[device.number];
	else
		return 0;
}
