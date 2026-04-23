// -----------------------------------------------------------------------------
// Biodata 696 - WiFi first, BLE selectable from button menu
// Single-file sketch for ESP32 Feather
// Minimal fix: force a static MIDI channel and disable menu/EEPROM channel changes
// -----------------------------------------------------------------------------

#include <PinButton.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <AppleMIDI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>

// Wifi Credentials
char ssid[] = "cosmicarray";
char pass[] = "goodlife";

// MIDI channel / IP
byte channel = 1;   // STATIC MIDI CHANNEL: change this to 1, 2, 3, 4, or 5 as needed
IPAddress local_IP(192,168,8,50); //50// 51 // *** DONT FORGET TO CHANGE *** 
bool staticIP = true;

// MIDI note/control
const byte polyphony = 5;

// scales
int scaleMajor[]    = {7, 0, 2, 4, 5, 7, 9, 11};
int scaleDiaMinor[] = {7, 0, 2, 3, 5, 7, 8, 10};
int scaleIndian[]   = {7, 0, 1, 1, 4, 5, 8, 10};
int scaleMinor[]    = {5, 1, 3, 5, 8, 10};
int scaleChrom[]    = {13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
int *scaleSelect = scaleChrom;
int root = 0;

bool wifiWasConnected = false;
unsigned long lastWifiReconnectAttempt = 0;
const unsigned long wifiReconnectInterval = 3000;

// Debug and MIDI output settings
byte debugSerial = 1;
byte serialMIDI = 1;
byte wifiMIDI = 1;
byte bleMIDI = 0;
byte midiMode = 0;
byte wifiActive = 1;
byte bleActive = 0;
byte midiControl = channel;
int noteMin = 36;
int noteMax = 96;
byte controlNumber = 80;

#define EEPROM_SIZE 5 // scaleindex, midi channel, wifi, bluetooth, key

// pins
int buttonPin = 13;
int potPin = A2;
const byte interruptPin = 12;

// leds
byte leds[5] = { 26,25,4,5,18 };
byte ledBrightness[5] = {185,255,255,255,195};
byte maxBrightness = 60;
bool blinkToggle = 0;
unsigned long blinkTime = 0;

// Bluetooth
#define SERVICE_UUID        "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"

BLECharacteristic *pCharacteristic = nullptr;
BLEServer *pServer = nullptr;
BLEAdvertising *pAdvertising = nullptr;
bool bleInitialized = false;
bool deviceConnected = false;
bool isConnected = false;

uint8_t midiPacket[] = {
   0x80,
   0x80,
   0x00,
   0x3c,
   0x00
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      if (pAdvertising) {
        pAdvertising->start();
      }
    }
};

class samFader {
  public:
  byte pinNumber;
  byte espPWMchannel;
  int maxBright;
  int currentLevel = 0;
  int destinationLevel = 0;
  unsigned long startTime;
  int duration = 0;
  unsigned long stepSize;
  unsigned long stepTime;
  bool isRunning = 0;

  samFader(byte pin, byte pwmChannel, byte maxB) {
    pinNumber=pin;
    espPWMchannel=pwmChannel;
    maxBright=maxB;
  }

  void Set(int dest, int dur) {
    startTime = millis();
    destinationLevel = dest;
    duration = dur;
    stepTime = 0;
    int difference = abs(destinationLevel - currentLevel);
    stepSize = duration/(difference+1);
    isRunning = 1;
    if(dur == 0) {
      ledcWrite(espPWMchannel, dest);
      currentLevel = dest;
      isRunning = 0;
    }
  }

  void Update() {
    if(isRunning) {
      if(stepTime+stepSize<millis()) {
        if(currentLevel>destinationLevel) currentLevel--;
        if(currentLevel<destinationLevel) currentLevel++;
        stepTime = millis();
        ledcWrite(espPWMchannel, currentLevel);
      }
      if(startTime+duration<millis()) {
        ledcWrite(espPWMchannel, destinationLevel);
        currentLevel = destinationLevel;
        isRunning = 0;
      }
    }
  }

  void Setup(byte chan) {
    ledcSetup(chan,5000,13);
    ledcAttachPin(pinNumber,chan);
  }
};

samFader ledFaders[] = {
  samFader(leds[0],0,ledBrightness[0]),
  samFader(leds[1],1,ledBrightness[1]),
  samFader(leds[2],2,ledBrightness[2]),
  samFader(leds[3],3,ledBrightness[3]),
  samFader(leds[4],4,ledBrightness[4])
};

PinButton button(buttonPin);

unsigned long currentMillis = 0;

const byte samplesize = 10;
const byte analysize = samplesize - 1;

volatile unsigned long microseconds;
volatile byte sampleIndex = 0;
volatile unsigned long samples[samplesize];

float threshold = 1.71;
float threshMin = 1.61;
float threshMax = 4.01;
float prevThreshold = 0;

typedef struct _MIDImessage {
  unsigned int type;
  int value;
  int velocity;
  long duration;
  long period;
  int channel;
} MIDImessage;

MIDImessage noteArray[polyphony];
MIDImessage controlMessage;

#define SerialMon Serial
#define APPLEMIDI_DEBUG SerialMon
APPLEMIDI_CREATE_DEFAULTSESSION_INSTANCE();

void setupSerialMIDI() {
  if (debugSerial) Serial.println("MIDI set on Serial1 31250");
  Serial1.begin(31250);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void checkKnob() {
  threshold = analogRead(potPin);
  prevThreshold = threshold;
  threshold = mapfloat(threshold, 0, 4095, threshMin, threshMax);
}

void saveTransportState() {
  EEPROM.write(2, wifiMIDI ? 1 : 0);
  EEPROM.write(3, bleMIDI ? 1 : 0);
  EEPROM.commit();
}

void setMidiTransport(bool useWifi, bool useBle) {
  wifiMIDI = useWifi ? 1 : 0;
  wifiActive = useWifi ? 1 : 0;
  bleMIDI = useBle ? 1 : 0;
  bleActive = useBle ? 1 : 0;
}

void allNotesOff() {
  for (int ch = 1; ch <= 16; ch++) {
    if (serialMIDI) {
      byte statusbyte = (176 | ((ch - 1) & 0x0F));
      Serial1.write(statusbyte); Serial1.write(123); Serial1.write(0);
      Serial1.write(statusbyte); Serial1.write(121); Serial1.write(0);
    }
    if (wifiMIDI && wifiActive && WiFi.status() == WL_CONNECTED && isConnected) {
      MIDI.sendControlChange(123, 0, ch);
      MIDI.sendControlChange(121, 0, ch);
    }
  }
  for (int i = 0; i < polyphony; i++) noteArray[i].velocity = 0;
}

void stopWifiTransport() {
  isConnected = false;
  wifiWasConnected = false;
  allNotesOff();
  WiFi.disconnect(true, false);
  delay(50);
  WiFi.mode(WIFI_OFF);
  delay(50);
  if (debugSerial) Serial.println("WiFi transport off");
}

void setupWifi() {
  IPAddress gateway(192,168,8,1);
  IPAddress dns(192,168,8,1);
  IPAddress subnet(255,255,255,0);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  if (staticIP) {
    if (!WiFi.config(local_IP, gateway, subnet, dns)) {
      Serial.println("Static IP config Failure");
    }
  }

  Serial.print("Connecting to Wifi Network: ");
  Serial.print(ssid);
  Serial.print("  pass: ");
  Serial.println(pass);

  WiFi.begin(ssid, pass);

  bool ledToggle = true;
  unsigned long wifiMillis = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");

    if (ledToggle) ledFaders[0].Set(ledFaders[0].maxBright, 0);
    else ledFaders[0].Set(0, 0);

    ledToggle = !ledToggle;
    ledFaders[0].Update();

    if (millis() - wifiMillis > 15000) break;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    ledFaders[0].Set(0, 0);
    ledFaders[2].Set(ledFaders[2].maxBright, 0);
    delay(500);
    ledFaders[2].Set(0, 3000);
  } else {
    Serial.println("WiFi NOT connected");
    ledFaders[0].Set(ledFaders[0].maxBright, 0);
    wifiWasConnected = false;
    isConnected = false;
    return;
  }

  Serial.print("IP address is ");
  Serial.println(WiFi.localIP());

  delay(1000);
  AppleMIDI.begin();
  MIDI.begin(MIDI_CHANNEL_OMNI);
  Serial.println("AppleMIDI started");

  AppleMIDI.setHandleConnected([](const APPLEMIDI_NAMESPACE::ssrc_t &ssrc, const char* name) {
    isConnected = true;
    Serial.print("RTP MIDI Connected: ");
    Serial.println(name ? name : "unknown");
  });

  AppleMIDI.setHandleDisconnected([](const APPLEMIDI_NAMESPACE::ssrc_t &ssrc) {
    isConnected = false;
    Serial.println("RTP MIDI Disconnected!");
  });

  wifiWasConnected = true;
}

void serviceWifiMidi() {
  if (!(wifiMIDI && wifiActive)) return;

  wl_status_t s = WiFi.status();

  if (s != WL_CONNECTED) {
    isConnected = false;

    if (wifiWasConnected) {
      Serial.println("WiFi dropped");
      wifiWasConnected = false;
      allNotesOff();
    }

    if (millis() - lastWifiReconnectAttempt > wifiReconnectInterval) {
      lastWifiReconnectAttempt = millis();
      Serial.println("Trying WiFi reconnect...");
      WiFi.disconnect(false, false);
      delay(200);
      WiFi.begin(ssid, pass);
    }
    return;
  }

  if (!wifiWasConnected) {
    Serial.println("WiFi restored");
    wifiWasConnected = true;
    allNotesOff();
  }
}

void bleSetup() {
  if (bleInitialized) return;

  byte mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  int uniq = mac[0] + mac[1] + mac[2] + mac[3] + mac[4] + mac[5];
  String chipStr = "BIODATA ";
  chipStr.concat(uniq);
  char chipString[20];
  chipStr.toCharArray(chipString,20);

  BLEDevice::init(chipString);

  if(debugSerial) {
    Serial.print("BiodataBLE ");
    Serial.println(chipString);
  }

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));

  pCharacteristic = pService->createCharacteristic(
    BLEUUID(CHARACTERISTIC_UUID),
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_WRITE_NR
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->start();

  bleInitialized = true;
  ledFaders[3].Set(ledFaders[3].maxBright, 0);
  delay(300);
  ledFaders[3].Set(0, 1200);
  Serial.println("BLE started");
}

void stopBleTransport() {
  deviceConnected = false;
  if (bleInitialized) {
    BLEDevice::deinit(true);
    pAdvertising = nullptr;
    pServer = nullptr;
    pCharacteristic = nullptr;
    bleInitialized = false;
  }
  Serial.println("BLE transport off");
}

void switchToWifi() {
  if (bleMIDI || bleActive || bleInitialized) {
    stopBleTransport();
  }
  setMidiTransport(true, false);
  saveTransportState();
  setupWifi();
  Serial.println("Switched to WiFi MIDI");
}

void switchToBLE() {
  if (wifiMIDI || wifiActive) {
    stopWifiTransport();
  }
  setMidiTransport(false, true);
  saveTransportState();
  bleSetup();
  Serial.println("Switched to BLE MIDI");
}

// MIDI functions
void midiSerial(int type, int channelOut, int data1, int data2)
{
  if (serialMIDI) {
    data1 &= 0x7F;
    data2 &= 0x7F;
    byte statusbyte = (type | ((channelOut - 1) & 0x0F));
    Serial1.write(statusbyte);
    Serial1.write(data1);
    Serial1.write(data2);
  }
}

void setNote(int value, int velocity, long duration, int notechannel)
{
  for (int i = 0; i < polyphony; i++) {
    if (!noteArray[i].velocity) {
      noteArray[i].type = 0;
      noteArray[i].value = value;
      noteArray[i].velocity = velocity;
      noteArray[i].duration = currentMillis + duration;
      noteArray[i].channel = notechannel;

      if (serialMIDI) {
        midiSerial(144, notechannel, value, velocity);
      }

      if (wifiMIDI && wifiActive && WiFi.status() == WL_CONNECTED && isConnected) {
        MIDI.sendNoteOn(value, velocity, notechannel);
      }

      if (bleMIDI && bleActive && deviceConnected && pCharacteristic != nullptr) {
        midiPacket[2] = (144 | ((notechannel - 1) & 0x0F));
        midiPacket[3] = value;
        midiPacket[4] = velocity;
        pCharacteristic->setValue(midiPacket, 5);
        pCharacteristic->notify();
      }

      ledFaders[i].Set(ledFaders[i].maxBright, 350);
      break;
    }
  }
}

void setControl(int type, int value, int velocity, long duration)
{
  controlMessage.type = type;
  controlMessage.value = value;
  controlMessage.velocity = velocity;
  controlMessage.period = duration;
  controlMessage.duration = currentMillis + duration;
}

void checkControl()
{
  signed int distance = controlMessage.velocity - controlMessage.value;

  if (distance != 0) {
    if (currentMillis > controlMessage.duration) {
      controlMessage.duration = currentMillis + controlMessage.period;

      if (distance > 0) controlMessage.value += 1;
      else controlMessage.value -= 1;

      if (serialMIDI) {
        midiSerial(176, channel, controlMessage.type, controlMessage.value);
      }

      if (wifiMIDI && wifiActive && WiFi.status() == WL_CONNECTED && isConnected) {
        MIDI.sendControlChange(controlMessage.type, controlMessage.value, channel);
      }

      if (bleMIDI && bleActive && deviceConnected && pCharacteristic != nullptr) {
        midiPacket[2] = (176 | ((channel - 1) & 0x0F));
        midiPacket[3] = controlMessage.type;
        midiPacket[4] = controlMessage.value;
        pCharacteristic->setValue(midiPacket, 5);
        pCharacteristic->notify();
      }
    }
  }
}

void checkNote()
{
  for (int i = 0; i < polyphony; i++) {
    if (noteArray[i].velocity) {
      if (noteArray[i].duration <= currentMillis) {
        if (serialMIDI) {
          midiSerial(144, noteArray[i].channel, noteArray[i].value, 0);
        }

        if (wifiMIDI && wifiActive && WiFi.status() == WL_CONNECTED && isConnected) {
          MIDI.sendNoteOff(noteArray[i].value, 0, noteArray[i].channel);
        }

        if (bleMIDI && bleActive && deviceConnected && pCharacteristic != nullptr) {
          midiPacket[2] = (144 | ((noteArray[i].channel - 1) & 0x0F));
          midiPacket[3] = noteArray[i].value;
          midiPacket[4] = 0;
          pCharacteristic->setValue(midiPacket, 5);
          pCharacteristic->notify();
        }

        noteArray[i].velocity = 0;
        ledFaders[i].Set(0, 800);
      }
    }
  }
}

// Analysis functions
void sample(){
  if(sampleIndex < samplesize) {
    samples[sampleIndex] = micros() - microseconds;
    microseconds = samples[sampleIndex] + microseconds;
    sampleIndex += 1;
  }
}

int scaleSearch(int note, int scale[], int scalesize) {
  for (byte i = 1; i <= scalesize; i++) {
    if (note == scale[i]) return note;
    if (note < scale[i]) return scale[i];
  }
  return scale[scalesize];
}

int scaleNote(int note, int scale[], int rootNote) {
  int scaled = note%12;
  int octave = note/12;
  int scalesize = scale[0];
  scaled = scaleSearch(scaled, scale, scalesize);
  scaled = (scaled + (12 * octave)) + rootNote;
  return scaled;
}

void analyzeSample()
{
  unsigned long averg = 0;
  unsigned long maxim = 0;
  unsigned long minim = 100000;
  float stdevi = 0;
  unsigned long delta = 0;
  byte change = 0;

  if (sampleIndex >= samplesize) {
    unsigned long sampanalysis[analysize];
    for (byte i=0; i<analysize; i++){
      sampanalysis[i] = samples[i+1];
      if(sampanalysis[i] > maxim) { maxim = sampanalysis[i]; }
      if(sampanalysis[i] < minim) { minim = sampanalysis[i]; }
      averg += sampanalysis[i];
      stdevi += sampanalysis[i] * sampanalysis[i];
    }

    averg = averg/analysize;
    stdevi = sqrt(stdevi / analysize - averg * averg);
    if (stdevi < 1) { stdevi = 1.0; }
    delta = maxim - minim;

    if (delta > (stdevi * threshold)) {
      change = 1;
    }

    if(change){
      int dur = 250+(map(delta%127, 0, 127, 200, 6500));
      int ramp = 3 + (dur%100);
      byte vel = 100;
      int setnote = map(averg%127,0,127,noteMin,noteMax);
      setnote = scaleNote(setnote, scaleSelect, root);
      setNote(setnote, vel, dur, channel);
      setControl(controlNumber, controlMessage.value, delta%127, ramp);
    }

    sampleIndex = 0;
  }
}

void showMenuBlink(byte menuIndex) {
  for(byte i=0;i<5;i++) ledFaders[i].Set(0,0);
  if((blinkTime + 150) < millis()) {
    blinkToggle = !blinkToggle;
    blinkTime = millis();
  }
  if(blinkToggle) ledFaders[menuIndex].Set(ledFaders[menuIndex].maxBright,0);
}

void handleMenu() {
  float batteryLevel = float(analogRead(35))/float(4095)*2.0*3.3*1.1;

  int knobValue = analogRead(potPin);
  unsigned long menuTimer = millis();
  int menu = 0;

  while(menuTimer + 10000 > millis()) {
    delay(1);
    MIDI.read();
    serviceWifiMidi();
    button.update();

    knobValue = analogRead(potPin);
    menu = map(knobValue, 0, 4095, 0, 4);
    if (menu > 4) menu = 4;
    showMenuBlink(menu);

    if(button.isSingleClick()) {
      menuTimer = millis();

      while(menuTimer + 20000 > millis()) {
        delay(1);
        MIDI.read();
        serviceWifiMidi();
        button.update();

        knobValue = analogRead(potPin);
        int modeValue = 0;
        for(byte i=0;i<5;i++) ledFaders[i].Set(0,0);

        if(menu == 0) { // scale
          modeValue = map(knobValue, 0, 4095, 0, 4);
          if (modeValue > 4) modeValue = 4;
          showMenuBlink(modeValue);
          if(modeValue == 0) scaleSelect = scaleChrom;
          if(modeValue == 1) scaleSelect = scaleMinor;
          if(modeValue == 2) scaleSelect = scaleDiaMinor;
          if(modeValue == 3) scaleSelect = scaleMajor;
          if(modeValue == 4) scaleSelect = scaleIndian;
        }

        if(menu == 1) { // midi channel display only - fixed static channel
          modeValue = channel;
          int showChannel = modeValue;
          int binaryValue[5];
          for(int i=0;i<5;i++){
            binaryValue[i] = showChannel % 2;
            showChannel = showChannel/2;
          }
          for (byte i = 0; i < 5; i++) {
            if(binaryValue[i]) ledFaders[i].Set(ledFaders[i].maxBright, 0);
          }
        }

        if(menu == 2) { // WiFi transport
          modeValue = map(knobValue, 0, 4095, 0, 1);
          if (modeValue > 1) modeValue = 1;
          if((blinkTime + 150) < millis()) { blinkToggle = !blinkToggle; blinkTime = millis(); }
          if(blinkToggle) ledFaders[modeValue * 4].Set(ledFaders[modeValue].maxBright,0);
          ledFaders[menu].Set(ledFaders[menu].maxBright,0);
        }

        if(menu == 3) { // BLE transport
          modeValue = map(knobValue, 0, 4095, 0, 1);
          if (modeValue > 1) modeValue = 1;
          if((blinkTime + 150) < millis()) { blinkToggle = !blinkToggle; blinkTime = millis(); }
          if(blinkToggle) ledFaders[modeValue * 4].Set(ledFaders[modeValue].maxBright,0);
          ledFaders[menu].Set(ledFaders[menu].maxBright,0);
        }

        if(menu == 4) { // battery
          int batt = batteryLevel*10;
          modeValue = map(batt, 32, 44, 0, 4);
          if (modeValue < 0) modeValue = 0;
          if (modeValue > 4) modeValue = 4;
          showMenuBlink(modeValue);
          if(modeValue != menu) ledFaders[menu].Set(ledFaders[menu].maxBright,0);
        }

        if(button.isSingleClick()) {
          for(byte i=0;i<5;i++) ledFaders[i].Set(0,0);
          ledFaders[menu].Set(ledFaders[menu].maxBright,0);
          delay(75); ledFaders[menu].Set(0,0);
          delay(75); ledFaders[menu].Set(ledFaders[menu].maxBright,0);
          delay(75); ledFaders[menu].Set(0,0);

          if(menu == 0) {
            EEPROM.write(0, modeValue);
          }

          if(menu == 1) {
            // Fixed static MIDI channel - do not change or save channel from menu
          }

          if(menu == 2) { // WiFi menu
            if(modeValue == 0) {
              stopWifiTransport();
              setMidiTransport(false, false);
              saveTransportState();
            }
            if(modeValue == 1) {
              switchToWifi();
            }
          }

          if(menu == 3) { // BLE menu
            if(modeValue == 0) {
              stopBleTransport();
              setMidiTransport(false, false);
              saveTransportState();
            }
            if(modeValue == 1) {
              switchToBLE();
            }
          }

          EEPROM.commit();
          return;
        }
      }
      return;
    }
  }

  for(byte i=0;i<5;i++) ledFaders[i].Set(0,700);
}

void setup() {
  if (debugSerial) Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println("setup start");

  EEPROM.begin(EEPROM_SIZE);
  Serial.println("after EEPROM.begin");

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(interruptPin, INPUT_PULLUP);

  Serial.println(F("Welcome to Biodata Sonification .. now with BLE and Wifi!"));

  for(byte i=0;i<5;i++) {
    ledFaders[i].Setup(i);
    ledFaders[i].Set(ledFaders[i].maxBright,500*(i+1));
  }
  unsigned long prevMillisLocal = millis();
  while(prevMillisLocal+1000>millis()) {
    for(byte i=0;i<5;i++) ledFaders[i].Update();
    delay(1);
  }
  for(byte i=0;i<5;i++) {
    ledFaders[i].Set(0,500*(i+1));
  }
  prevMillisLocal = millis();
  while(prevMillisLocal+3000>millis()) {
    for(byte i=0;i<5;i++) ledFaders[i].Update();
    delay(1);
  }
  Serial.println("after LED startup");

  if(!digitalRead(buttonPin)) {
    if (debugSerial) Serial.println("Button Held at Bootup!");
    ledFaders[4].Set(255, 1000);
    while(ledFaders[4].isRunning) {
      ledFaders[4].Update();
      delay(1);
    }
    EEPROM.write(0, 0);
    EEPROM.write(1, channel);
    EEPROM.write(2, 1); // WiFi ON
    EEPROM.write(3, 0); // BLE OFF
    EEPROM.write(4, 1);
    EEPROM.commit();
    setMidiTransport(true, false);
    scaleSelect = scaleChrom;
    ledFaders[4].Set(0, 0);
  }

  byte scaleIndex = EEPROM.read(0);
  byte midiChannel = EEPROM.read(1);
  byte wifiPower = EEPROM.read(2);
  byte blePower = EEPROM.read(3);
  byte keybyte = EEPROM.read(4);

  if(keybyte != 1) {
    EEPROM.write(0, 0);
    EEPROM.write(1, channel);
    EEPROM.write(2, 1); // WiFi ON
    EEPROM.write(3, 0); // BLE OFF
    EEPROM.write(4, 1);
    EEPROM.commit();
    Serial.println("EEPROM Initialized - First time! WiFi ON, BLE OFF");
    scaleIndex = EEPROM.read(0);
    midiChannel = EEPROM.read(1);
    wifiPower = EEPROM.read(2);
    blePower = EEPROM.read(3);
  }
  Serial.println("after EEPROM read");

  // Force static channel every boot and store it back to EEPROM
 //   channel = 1; // 
  EEPROM.write(1, channel);
  EEPROM.commit();
  midiControl = channel;
  Serial.println("=================================");
Serial.print(">>> ACTIVE MIDI CHANNEL: ");
Serial.println(channel);
Serial.println("=================================");

  (void)midiChannel;

  if(scaleIndex == 0) scaleSelect = scaleChrom;
  if(scaleIndex == 1) scaleSelect = scaleMinor;
  if(scaleIndex == 2) scaleSelect = scaleDiaMinor;
  if(scaleIndex == 3) scaleSelect = scaleMajor;
  if(scaleIndex == 4) scaleSelect = scaleIndian;

  // boot WiFi-first, BLE off by default, but still selectable from menu later
  (void)wifiPower;
  (void)blePower;
  setMidiTransport(true, false);

  if (serialMIDI) {
    Serial.println("before setupSerialMIDI");
    setupSerialMIDI();
  }

  if (wifiMIDI && wifiActive) {
    Serial.println("before setupWifi");
    setupWifi();
    Serial.println("after setupWifi");
  }

  attachInterrupt(interruptPin, sample, RISING);
  Serial.println("setup done");

  for(byte i=0;i<5;i++) {
    ledFaders[i].Update();
    ledFaders[i].Set(0,2000);
  }
}

void loop() {
  currentMillis = millis();
  serviceWifiMidi();
  MIDI.read();

  if (sampleIndex >= samplesize) {
    analyzeSample();
  }

  checkNote();
  checkControl();

  for(byte i=0;i<5;i++) ledFaders[i].Update();
  checkKnob();

  if(wifiMIDI && WiFi.status() != WL_CONNECTED) {
    ledFaders[0].Set(ledFaders[0].maxBright,0);
  }

  button.update();

  if(button.isSingleClick()) {
    handleMenu();
  }
}
