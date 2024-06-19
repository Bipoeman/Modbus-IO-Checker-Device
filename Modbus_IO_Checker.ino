#include <Wire.h>
#include <BluetoothSerial.h>
#include <LiquidCrystal_I2C.h>
#include <driver/uart.h>
#include <EEPROM.h>

#define EEPROM_SIZE 100
#define LED 2

#define MODE_SLAVE 0x00
#define MODE_MASTER 0x01

#define ON_COMMAND 0xFF00
#define OFF_COMMAND 0x0000

#define READ_OUTPUT_COIL 0x01
#define READ_INPUT_DIGITAL 0x02
#define READ_HOLDING_REGISTER 0x03
#define READ_INPUT_ANALOG 0x04
#define WRITE_OUTPUT_COIL 0x05

byte buttonBar[] = { B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111 };
byte buttonNoBar[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

int deviceMode = MODE_SLAVE;

BluetoothSerial BTSerial;
LiquidCrystal_I2C lcd(0x27, 16, 2);

String deviceName;

int connectionStatus;

static unsigned int MODBUS_CRC16(const unsigned char *buf, unsigned int len);
#define NUM_PINS 4
int inputPins[4] = { 33, 26, 27, 13 };
int outputPins[4] = { 5, 18, 19, 23 };
uint8_t prevInputData[NUM_PINS];
int slaveAddress = 0x01;

int slaveAddressEEPROMAddress = 0;
int deviceModeAddress = 1;

unsigned long masterUpdateTimeNow;
unsigned long masterUpdateInterval = 500;
int statusIndex = 0;

uint64_t displayOnTimeNow;
uint64_t displayOnTimeOut = 30000;


void setup() {

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MODBUS IO Check");  // Display before loop
  delay(1000);
  lcd.createChar(0, buttonBar);
  lcd.createChar(1, buttonNoBar);
  lcd.clear();
  EEPROM.begin(EEPROM_SIZE + 2);
  slaveAddress = EEPROM.read(slaveAddressEEPROMAddress);
  deviceMode = EEPROM.read(deviceModeAddress);

  for (int i = 2; i < EEPROM_SIZE && EEPROM.read(i) != '\0'; i++) {
    deviceName += (char)EEPROM.read(i);
  }

  BTSerial.begin(deviceName);
  BTSerial.register_callback(callback);
  for (int i = 0; i < NUM_PINS; i++) {
    pinMode(inputPins[i], INPUT_PULLUP);
    pinMode(outputPins[i], OUTPUT);
    // digitalWrite(outputPins[i], HIGH);
  }
  pinMode(LED, OUTPUT);
  // delay(100);
  Serial2.begin(9600);
  uart_set_pin(UART_NUM_2, 17, 16, 4, -1);
  uart_set_mode(UART_NUM_2, UART_MODE_RS485_HALF_DUPLEX);
  Serial.begin(115200);
  Serial.print("Address : ");
  Serial.println(slaveAddress);
  Serial.print("Name : ");
  Serial.println(deviceName);
  displayOnTimeNow = millis();

  // put your setup code here, to run once:
}
uint8_t data[15];
int dataLength;
unsigned long timeOut;

unsigned long ioBTTimeNow;
unsigned long ioBTInterval = 400;

int displayState = 0;
unsigned long displayTimeNow;
unsigned long displayScrollTimeNow;
unsigned long displaySwapInterval = 2000;
unsigned long displayScrollInterval = 1000;
uint8_t displayOffset;
String serialCommand;
String btCommand;
void loop() {

  if (millis() - displayOnTimeNow > displayOnTimeOut) {
    lcd.noDisplay();
    lcd.noBacklight();
    displayOnTimeNow = millis();
  }

  if (millis() - ioBTTimeNow > ioBTInterval && connectionStatus == 1) {
    String pinStatus = "PIN\t";
    int pinStatusInt = 0;
    for (int i = 0; i < NUM_PINS; i++) {
      pinStatusInt |= (!digitalRead(inputPins[i])) << i;
      Serial.print(!digitalRead(inputPins[i]));
      Serial.print(" ");
    }
    Serial.println();
    pinStatus += String(pinStatusInt);
    BTSerial.println(pinStatus);
    ioBTTimeNow = millis();
  }

  // if (millis() - displayTimeNow > displaySwapInterval) {
  //   if (deviceName.length() < 16) {
  //     lcd.setCursor((16 - (deviceName.length())) / 2, 0);
  //     lcd.print(deviceName);
  //   }
  //   // String addressString = "Address : ";

  //   displayTimeNow = millis();
  // }

  if (deviceName.length() > 16 && millis() - displayScrollTimeNow > displayScrollInterval) {
    lcd.setCursor(0, 0);
    lcd.print(deviceName.substring(displayOffset++));
    if (deviceName.length() - displayOffset < 8) {
      displayOffset = 0;
    }
    displayScrollTimeNow = millis();
  }

  // if (millis() - masterUpdateTimeNow > masterUpdateInterval) {  // Update master state every 1s
  //   uint8_t masterTransfer[8];
  //   masterTransfer[0] = slaveAddress;
  //   masterTransfer[1] = WRITE_OUTPUT_COIL;
  //   masterTransfer[2] = 0x00;
  //   masterTransfer[3] = statusIndex;
  //   masterTransfer[4] = !digitalRead(inputPins[statusIndex]) == 1 ? 0xFF : 0x00;
  //   masterTransfer[5] = 0x00;
  //   uint16_t crc_ckeck = MODBUS_CRC16(masterTransfer, 6);
  //   masterTransfer[6] = crc_ckeck & 255;
  //   masterTransfer[7] = (crc_ckeck >> 8) & 255;
  //   for (int j = 0; j < 8; j++) {
  //     Serial2.write(masterTransfer[j]);
  //   }
  //   masterUpdateTimeNow = millis();
  //   statusIndex++;

  //   if (statusIndex == 4) {
  //     statusIndex = 0;
  //   }
  //   masterUpdateInterval = random(500,3000);
  // }

  for (int i = 0; i < NUM_PINS; i++) {  // update self state and if state change, report to slave with the same id
    if (digitalRead(inputPins[i]) != prevInputData[i]) {
      digitalWrite(outputPins[i], !digitalRead(inputPins[i]));
      // if (deviceMode == MODE_MASTER) {
      uint8_t masterTransfer[8];
      masterTransfer[0] = slaveAddress;
      masterTransfer[1] = WRITE_OUTPUT_COIL;
      masterTransfer[2] = 0x00;
      masterTransfer[3] = i;
      masterTransfer[4] = !digitalRead(inputPins[i]) == 1 ? 0xFF : 0x00;
      masterTransfer[5] = 0x00;
      uint16_t crc_ckeck = MODBUS_CRC16(masterTransfer, 6);
      masterTransfer[6] = crc_ckeck & 255;
      masterTransfer[7] = (crc_ckeck >> 8) & 255;
      for (int j = 0; j < 8; j++) {
        Serial2.write(masterTransfer[j]);
      }
      masterUpdateTimeNow = millis();
      // }
      resetDisplayTimeout();
      lcd.setCursor(16 - (4 - i), 0);
      if (!digitalRead(inputPins[i]) == 1) {
        lcd.print(i + 1);
      } else {
        lcd.write(1);
      }
      prevInputData[i] = digitalRead(inputPins[i]);
    }
  }

  while (Serial2.available() > 0) {  // put serial read into buffer
    data[dataLength++] = Serial2.read();
    timeOut = millis();
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c != '\r' && c != '\n') {
      serialCommand += c;
    } else {
      Serial.println(serialCommand);
      String command = serialCommand.substring(0, serialCommand.indexOf('='));
      String params = serialCommand.substring(serialCommand.indexOf('=') + 1);
      resetDisplayTimeout();
      executeCommand(command, params);
      serialCommand = "";
    }
  }

  if (BTSerial.available()) {
    char c = BTSerial.read();
    if (c != '\r' && c != '\n') {
      btCommand += c;
      // Serial.write(c);
    } else {
      Serial.println(btCommand);
      String command = btCommand.substring(0, btCommand.indexOf('='));
      String params = btCommand.substring(btCommand.indexOf('=') + 1);
      resetDisplayTimeout();
      if (command.equals("address")) {
        int newID = params.toInt();
        if (params.equals("?")) {
          Serial.print("Address : ");
          Serial.println(slaveAddress);
          BTSerial.println(slaveAddress);
        } else {
          slaveAddress = newID;
          Serial.print("New Address : ");
          Serial.println(slaveAddress);
          EEPROM.write(slaveAddressEEPROMAddress, slaveAddress);
          EEPROM.commit();
          BTSerial.println("address done");
        }
      } else if (command.equals("name")) {
        if (params.equals("?")) {
          BTSerial.println(deviceName);
        } else {
          for (int i = 0; i < params.length(); i++) {
            EEPROM.write(i + 2, params.charAt(i));
          }
          EEPROM.write(params.length() + 2, '\0');
          EEPROM.commit();
          Serial.print("New Name : ");
          Serial.println(params);
          deviceName = params;
          BTSerial.println("name done");
        }

      } else if (command.equals("mode")) {
        if (params.equals("?")) {
          Serial.print("Mode : ");
          Serial.println(deviceMode == MODE_MASTER ? "MASTER" : "SLAVE");
          BTSerial.println(deviceMode == MODE_MASTER ? "MASTER" : "SLAVE");
        } else {
          deviceMode = params.toInt();
          EEPROM.write(deviceModeAddress, deviceMode);
          EEPROM.commit();
        }
      } else if (command.equals("all")) {
        if (params.equals("?")) {
          String BTTransmitt = "ALL\t";
          BTTransmitt += deviceName;
          BTTransmitt += "\t";
          BTTransmitt += String(slaveAddress);
          BTTransmitt += "\t";
          BTTransmitt += deviceMode == MODE_MASTER ? "MASTER" : "SLAVE";
          BTSerial.println(BTTransmitt);

          Serial.print(deviceName);
          Serial.print("\t");
          Serial.print(slaveAddress);
          Serial.print("\t");
          Serial.println(deviceMode == MODE_MASTER ? "MASTER" : "SLAVE");
        }
      }
      btCommand = "";
    }
  }

  if (millis() - timeOut > 5 && dataLength > 0) {
    for (int i = 0; i < dataLength; i++) {
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    uint16_t masterDestinationSlaveAddress = data[0];
    uint16_t functionCode = data[1];
    uint16_t dataAddress = (data[2] << 8) | data[3];
    uint16_t crc = (data[dataLength - 1] << 8) | data[dataLength - 2];
    Serial.println();
    if (MODBUS_CRC16(data, 6) == crc) {
      if (masterDestinationSlaveAddress == slaveAddress) {
        if (functionCode == WRITE_OUTPUT_COIL) {
          uint16_t commandValue = ((data[4] << 8) | data[5]);
          Serial.print("Command Address : ");
          Serial.print(dataAddress);
          Serial.print(" Command Value : ");
          Serial.println(commandValue);
          digitalWrite(outputPins[dataAddress], commandValue == ON_COMMAND);
          for (int i = 0; i < dataLength && deviceMode == MODE_SLAVE; i++) {
            Serial2.write(data[i]);
          }
          resetDisplayTimeout();
          for (int pin = 0; pin < NUM_PINS; pin++) {
            lcd.setCursor(16 - (4 - pin), 0);
            if (digitalRead(outputPins[pin]) == 1) {
              lcd.print(pin + 1);
            } else {
              lcd.write(1);
            }
          }
        } else if (functionCode == READ_INPUT_ANALOG || functionCode == READ_INPUT_DIGITAL) {
          uint16_t dataQuantity = ((data[4] << 8) | data[5]);
          uint8_t outputData[50];
          int outputLength = 0;
          outputData[outputLength++] = slaveAddress;  // Slave Address
          outputData[outputLength++] = functionCode;  // Function Code

          if (dataQuantity < 8 && functionCode == READ_INPUT_DIGITAL) {
            outputData[outputLength++] = 1;  // Output Byte Count = 1 (read discrete input)
          }

          if (functionCode == READ_INPUT_ANALOG) {
            outputData[outputLength++] = dataQuantity * 2;  // Output Byte Count = quantity * 2 (read input registers)
          }

          uint16_t output;
          for (int i = dataAddress; i < NUM_PINS; i++) {
            if (functionCode == READ_INPUT_ANALOG) {  // read input register
              outputData[outputLength++] = 0x00;
              outputData[outputLength++] = !digitalRead(inputPins[i]);
            } else if (functionCode == READ_INPUT_DIGITAL) {
              // output |= (!digitalRead(inputPins[i])) << (i - dataAddress);
              output |= (digitalRead(outputPins[i])) << (i - dataAddress);  // Send output status instead of input, to reflect the actual status
            }
          }

          if (functionCode == READ_INPUT_DIGITAL) {
            outputData[outputLength++] = output;
          }

          uint16_t crc_out = MODBUS_CRC16(outputData, outputLength);
          outputData[outputLength++] = crc_out & 255;
          outputData[outputLength++] = (crc_out >> 8) & 255;
          for (int i = 0; i < outputLength && deviceMode == MODE_MASTER; i++) {
            Serial2.write(outputData[i]);
          }
        }
        // Serial.println("CRC Valid");
      }
    }
    dataLength = 0;
  }
}

void resetDisplayTimeout() {
  lcd.backlight();
  lcd.display();
  lcd.setCursor(7, 1);
  lcd.print(slaveAddress);
  lcd.print(" ");
  lcd.setCursor(16 - 3, 1);
  lcd.print(deviceMode == MODE_MASTER ? "MAS" : "SLV");
  // lcd.setCursor((16 - (deviceName.length())) / 2, 0);
  lcd.setCursor(0, 0);
  lcd.print("            ");
  lcd.setCursor(0, 0);
  lcd.print(deviceName);
  String addressString = "Addr : ";
  lcd.setCursor(0, 1);
  lcd.print(addressString);
  lcd.print(slaveAddress);
  Serial.println("Update display timeout");
  displayOnTimeOut = millis();
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client Connected");
    digitalWrite(LED, HIGH);
    connectionStatus = 1;
  }

  if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("Client disconnected");
    digitalWrite(LED, LOW);
    connectionStatus = 0;
  }
}

void executeCommand(String command, String params) {  // {command}={param}
  if (command.equals("address")) {
    int newID = params.toInt();
    if (params.equals("?")) {
      Serial.print("Address : ");
      Serial.println(slaveAddress);
    } else {
      slaveAddress = newID;
      Serial.print("New Address : ");
      Serial.println(slaveAddress);
      EEPROM.write(slaveAddressEEPROMAddress, slaveAddress);
      EEPROM.commit();
    }
  } else if (command.equals("name")) {
    if (params.equals("?")) {
      Serial.println(deviceName);
    } else {
      for (int i = 0; i < params.length(); i++) {
        EEPROM.write(i + 2, params.charAt(i));
      }
      EEPROM.write(params.length() + 2, '\0');
      EEPROM.commit();
      Serial.print("New Name : ");
      Serial.println(params);
      deviceName = params;
    }
    lcd.setCursor(0, 0);
    lcd.print("               ");
  } else if (command.equals("mode")) {
    if (params.equals("?")) {
      Serial.print("Mode : ");
      Serial.println(deviceMode == MODE_MASTER ? "MASTER" : "SLAVE");
    } else {
      deviceMode = params.toInt();
      EEPROM.write(deviceModeAddress, deviceMode);
      EEPROM.commit();
    }
  }
}

unsigned int MODBUS_CRC16(const unsigned char *buf, unsigned int len) {
  static const unsigned int lut[2] = { 0x0000, 0xA001 };
  unsigned int crc = 0xFFFF;
  unsigned int i = 0;
  char bit = 0;
  unsigned int xorVal = 0;

  for (i = 0; i < len; i++) {
    crc ^= buf[i];

    for (bit = 0; bit < 8; bit++) {
      xorVal = crc & 0x01;
      crc >>= 1;
      crc ^= lut[xorVal];
    }
  }

  return crc;
}
