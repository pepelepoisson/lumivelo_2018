// Arduino Built-in Library
#include<Wire.h>

// External Library
#include "painlessMesh.h"
#define FASTLED_ALLOW_INTERRUPTS 0  // To fix flickering issue https://github.com/FastLED/FastLED/issues/306
#include "FastLED.h"
FASTLED_USING_NAMESPACE

// Configuration
#define DEBUG_SERIAL 0  // Set to 1 to output detailed data to serial
#define MINIMAL_SERIAL 1 // Set to 1 to output minimal status data to serial
#define USE_PHOTOCELL 1 // Set to 1 to include photocell readings in state switching
#define LOW_BRIGHTNESS          30
#define HIGH_BRIGHTNESS          100
#define FRAMES_PER_SECOND   60 // 125 frames/sec <=> 8 milli/frame
#define NUM_LEDS  150
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define MIN_WAIT_SECOND 45  // Broadcast interval
#define MAX_WAIT_SECOND 90
#define DATA_PIN    14
#define MOSFET_GATE  16
#define RED_LED     15
#define GREEN_LED   12
#define BLUE_LED    13
#define PUSH_BUTTON_PIN 0
#define PhotocellPin 0 // the cell and 10K pulldown are connected to a0
#define MESH_PREFIX     "eqbe"
#define MESH_PASSWORD   "PapasInventeurs"
#define MESH_PORT       5858
#define MAX_FRAME_COUNT 160 // 160 * 8 milli = 1280 milli; Duration of transition effect is 1.3 second
#define ACCELEROMETER_ORIENTATION 4     // 0, 1, 2, 3 or 4 to set the orientation of the accerometer module
#define idle_test_min_count 30
#define fallen_test_min_count 5
#define strong_braking_test_min_count 5
#define long_braking_test_min_count 40
#define SET_IDLE_MILLISECONDS 40000 // how many seconds at idle before moving to sleep?
#define SET_PANIC_MILLISECONDS 40000 // how many seconds before moving away from Panic_mode?
#define SET_BRAKING_MILLISECONDS 1000 // how many seconds before moving away from Braking_mode?


enum patternStateEnum { NORMAL, TRANSITION, NEW_CONNECTION, CHANGED_CONNECTIONS, PATTERN_STATE_COUNT };
uint8_t gPatternState = NORMAL;

enum patternEnum { SOLID, RAINBOW, CONFETTI, SINELON, BPM, JUGGLE, PATTERN_COUNT };
const char *patternName[PATTERN_COUNT] = { "solid", "rainbow", "confetti", "sinelon", "bpm", "juggle"};
uint8_t gNextPatternIdx = PATTERN_COUNT;

enum riderEnum { SEBASTIEN, PASCAL, LUMIVELO_PP6, UNKNOWN_RIDER, RIDER_COUNT };
const char *rider[RIDER_COUNT] = { "Sebastien", "Pascal", "Lumivelo_PP6", "Unknown" };

CRGB leds[NUM_LEDS];

uint8_t gFrameCount = 0; // Inc by 1 for each Frame of Trasition, New/Changed connection(s) pattern
uint8_t gHue = 0;
void (*gActivePattern)();
void (*gSelectedPattern)();
bool gIsOurSentPattern = true;
bool wasPressed;

// Variables used in CheckLight() routine
String light_status=String("unknown");
int photocellReading=0; // the analog reading from the analog resistor divider
int day_limit=250, night_limit=300; // analog reading levels corresponding to switching from day to night - difference used to avoid toggling between two levels when light level is borderline

// Variables used in CheckAccel() routine
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
String accel_status=String("unknown");
int a_forward=0,a_sideway=0,a_vertical=0;
int a_forward_offset=0,a_sideway_offset=0,a_vertical_offset=0;
float a_forward_long_lag=0.0, a_sideway_long_lag=0.0, a_vertical_long_lag=0.0, a_ratio_long_lag=0.0;
float a_forward_lag=0.0, a_sideway_lag=0.0, a_vertical_lag=0.0, a_ratio_lag=0.0;
float a_forward_change=0.0, a_sideway_change=0.0, a_vertical_change=0.0, a_ratio_change=0.0;  
float long_lag_coef=0.005, lag_coef=0.1;
float a_forward_threshold=10.0, a_sideway_threshold=10.0, a_vertical_threshold=15.0;
float a_ratio_multiplier=200.0;
float idle_test_threshold=2.0,fallen_test_multiplier=0.75, strong_braking_test_threshold=50.0, long_braking_test_threshold=20.0;
//int idle_test_min_count=30, fallen_test_min_count=5, strong_braking_test_min_count=5, long_braking_test_min_count=40;
int idle_test_count=0, fallen_test_count=0, strong_braking_test_count=0, long_braking_test_count=0;
unsigned long start_time=0, current_time=0, elapsed_milliseconds=0, transition_timer=0;

enum {Sleep_mode,Idle_mode,Cruising_mode,Braking_mode,Post_Braking_mode,Stopped_mode,Panic_mode} condition=Sleep_mode;

void LED(String pattern){
  if (pattern=="idle"){
    digitalWrite(MOSFET_GATE,HIGH);
    digitalWrite(RED_LED,LOW);
    digitalWrite(GREEN_LED,LOW);
    digitalWrite(BLUE_LED,HIGH);
    FastLED.setBrightness(LOW_BRIGHTNESS); 
    //for (int i = NUM_LEDS; i >=0; i--){leds[i]=CRGB::Blue;}
    gActivePattern();
  }
  
  if (pattern=="cruising"){
    digitalWrite(MOSFET_GATE,HIGH);
    digitalWrite(RED_LED,LOW);
    digitalWrite(GREEN_LED,HIGH);
    digitalWrite(BLUE_LED,LOW);
    FastLED.setBrightness(LOW_BRIGHTNESS); 
    gActivePattern();
    //solid();
  }
  
  if (pattern=="braking"|| pattern=="panic"){
    digitalWrite(MOSFET_GATE,HIGH);
    digitalWrite(RED_LED,HIGH);
    digitalWrite(GREEN_LED,LOW);
    digitalWrite(BLUE_LED,LOW);
    FastLED.setBrightness(HIGH_BRIGHTNESS); 
    for (int i = NUM_LEDS; i >=0; i--){leds[i]=CRGB::Red;}
  }
  
  if (pattern=="off"){
    for (int i = NUM_LEDS; i >=0; i--) {leds[i].nscale8(230);}
    digitalWrite(MOSFET_GATE,LOW);    
    digitalWrite(RED_LED,LOW);
    digitalWrite(GREEN_LED,LOW);
    digitalWrite(BLUE_LED,LOW);
  }

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND);       
  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
}

void solid()
{
  // FastLED's built-in rainbow generator
  fill_solid( leds, NUM_LEDS, CHSV(gHue, 255, 255));
}

void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void confetti()
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS);
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for ( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for ( int i = 0; i < 8; i++) {
    leds[beatsin16(i + 7, 0, NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void transitionPulse() {
  gFrameCount += 1;

  uint8_t hue = gIsOurSentPattern ? HUE_AQUA : HUE_ORANGE;

  uint8_t value = 0;
  if (gFrameCount < MAX_FRAME_COUNT - 45) { // Tiny black interval between the pulse and next pattern
    value = quadwave8(gFrameCount * 2);
  }

  fill_solid(leds, NUM_LEDS, CHSV(hue, 255, value));
}

void glitter() {
  gFrameCount += 1;

  if (gFrameCount > MAX_FRAME_COUNT - 45) { // Tiny black interval between the pulse and next pattern
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  } else {
    if (gFrameCount % 4 == 1) { // Slow down frame rate
      for ( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CHSV(0, 0, random8() < 60 ? random8() : random8(64));
      }
    }
  }
}

void redGlitter() {
  gFrameCount += 1;

  if (gFrameCount > MAX_FRAME_COUNT - 45) { // Tiny black interval between the pulse and next pattern
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  } else {
    if (gFrameCount % 4 == 1) { // Slow down frame rate
      for ( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CHSV(HUE_RED, 0, random8() < 60 ? random8() : random8(64));
      }
    }
  }
}

painlessMesh  mesh;

int8_t nodeId2riderIdx(uint32_t nodeId) {
  switch (nodeId) {
    case 0xa6182c9a:
      return SEBASTIEN;
    case 0x7f0fb8c3:
      return PASCAL;
    case 0xa6168502:
      return LUMIVELO_PP6;
    default:
      return UNKNOWN_RIDER;
  }
}

void setSelectedPattern(uint8_t patternIdx) {
  // BUGGY: patternIdx sometimes out of expected range - Falling ont default case
  switch (patternIdx) {
    case SOLID:
      gSelectedPattern = solid;
      break;
    case RAINBOW:
      gSelectedPattern = rainbow;
      break;
    case CONFETTI:
      gSelectedPattern = confetti;
      break;
    case SINELON:
      gSelectedPattern = sinelon;
      break;
    case BPM:
      gSelectedPattern = bpm;
      break;
    case JUGGLE:
      gSelectedPattern = juggle;
      break;
      Serial.printf("ERROR! Unknown Pattern index, %d\n", patternIdx);
  }
}

Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, []() {
  DynamicJsonBuffer jsonBuffer;
  JsonObject& msg = jsonBuffer.createObject();

  uint8_t patternIdx = gNextPatternIdx < PATTERN_COUNT ? gNextPatternIdx : random(0, PATTERN_COUNT);   // Possible origin of bug in patternIdx????

  setSelectedPattern(patternIdx);
  msg["pattern"] = patternIdx;
  Serial.printf("  Setting my pattern to %s (index: %d)\n", patternName[patternIdx], patternIdx);

  gHue = random(0, 255);
  msg["hue"] = gHue;
  Serial.printf("  Setting my hue to %d\n", gHue);

  gIsOurSentPattern = true;

  String str;
  msg.printTo(str);
  mesh.sendBroadcast(str);
  taskSendMessage.setInterval( random( TASK_SECOND * MIN_WAIT_SECOND, TASK_SECOND * MAX_WAIT_SECOND ));

  gFrameCount = 0;
  gPatternState = TRANSITION;
});

void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("Message %s\n", msg.c_str());

  if (mesh.getNodeId() == from) {
    Serial.printf("  Sent from ourself! Going to ignore this message\n");
    return;
  }

  int8_t riderIdx = nodeId2riderIdx(from);
  Serial.printf("  Sent from rider %s (nodeId: 0x%08x)\n", rider[riderIdx], from);

  DynamicJsonBuffer jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(msg);
  if (root.containsKey("pattern")) {
    uint8_t patternIdx = root["pattern"];
    Serial.printf("  Set my pattern to %s (index: %d)\n", patternName[patternIdx], patternIdx);
    setSelectedPattern(patternIdx);

    gFrameCount = 0;
    gPatternState = TRANSITION;
    gIsOurSentPattern = false;

    // Reset the wait after message reception
    taskSendMessage.setInterval( random( TASK_SECOND * MIN_WAIT_SECOND, TASK_SECOND * MAX_WAIT_SECOND ));
  }

  if (root.containsKey("hue")) {
    gHue = root["hue"];
    Serial.printf("  Set my hue to %d\n", gHue);
  }
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New Connection\n");
  Serial.printf("  Remote module ==> NodeId: 0x%08x, Rider: %s\n", nodeId, rider[nodeId2riderIdx(nodeId)]);

  gFrameCount = 0;
  gPatternState = NEW_CONNECTION;
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
  Serial.printf("  %s\n", mesh.subConnectionJson().c_str());

  gFrameCount = 0;
  gPatternState = CHANGED_CONNECTIONS;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time\n");
  Serial.printf("  Node time: %u, Offset: %d\n", mesh.getNodeTime(), offset);
}

void updatePattern () {
  void (*currentPattern)() = gActivePattern;

  // Check if transitive pattern cycle is completed
  if (gPatternState != NORMAL && gFrameCount > MAX_FRAME_COUNT) {
    gPatternState = NORMAL;
  }

  switch (gPatternState) {
    case NORMAL:
      gActivePattern = gSelectedPattern;
      break;
    case TRANSITION:
      gActivePattern = transitionPulse;
      break;
    case NEW_CONNECTION:
      gActivePattern = glitter;
      break;
    case CHANGED_CONNECTIONS:
      gActivePattern = redGlitter;
      break;
    default:
      Serial.printf("Unknow pattern state idx %d\n", gPatternState);
  }

  // Force an update for newly active pattern
  if (currentPattern != gActivePattern) {
    gActivePattern();
    FastLED.show();
  }
}

void doButton() {
  bool isPressed = (digitalRead(PUSH_BUTTON_PIN) == LOW);

  if (isPressed && !wasPressed) { // Press transition
    wasPressed = true;
    Serial.println("[x] Push button pressed"); // Transition
    taskSendMessage.forceNextIteration();
  } else if (!isPressed && wasPressed) { // Unpress transition
    wasPressed = false;
    // Serial.println("[-] Push button released transition !!!");
  } else if (isPressed) {
    // Button is press (PRESS-STATE)
    // Serial.println("[_] Push button is pressed.");
  } else {
    // Serial.println("[^] Push button is released.");
  }
}

void CalibrateAccel(){
  // Reads acceleration from MPU6050 to evaluate installation offsets.
  // Tunables: 
  // Output values: a_forward_offset, a_sideway_offset, a_vertical_offset 

  for (int i=0; i<100;i++){
    // Get accelerometer readings
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 
    // Convert to expected orientation - includes unit conversion to "cents of g" for MPU range set to 2g
    a_forward = (ACCELEROMETER_ORIENTATION == 0?-AcX:(ACCELEROMETER_ORIENTATION == 1?-AcX:(ACCELEROMETER_ORIENTATION == 2?-AcX:(ACCELEROMETER_ORIENTATION == 3?-AcY:AcY))))/164.0;
    a_sideway = (ACCELEROMETER_ORIENTATION == 0?AcY:(ACCELEROMETER_ORIENTATION == 1?AcZ:(ACCELEROMETER_ORIENTATION == 2?-AcZ:(ACCELEROMETER_ORIENTATION == 3?AcZ:-AcZ))))/164.0;
    a_vertical = (ACCELEROMETER_ORIENTATION == 0?AcZ:(ACCELEROMETER_ORIENTATION == 1?-AcY:(ACCELEROMETER_ORIENTATION == 2?AcY:(ACCELEROMETER_ORIENTATION == 3?AcX:AcX))))/164.0;
    
    a_forward_offset=a_forward_offset+a_forward;
    a_sideway_offset=a_sideway_offset+a_sideway;
    a_vertical_offset=a_vertical_offset+a_vertical;
  }
  a_forward_offset=a_forward_offset/100;
  a_sideway_offset=a_sideway_offset/100;
  a_vertical_offset=a_vertical_offset/100;
}

void CheckAccel() {
  // Reads acceleration from MPU6050 to evaluate current condition.
  // Tunables:
  // Output values: still, cruising, braking, fallen, unknown
  // Get accelerometer readings
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // Convert to expected orientation - includes unit conversion to "cents of g" for MPU range set to 2g
  a_forward = (ACCELEROMETER_ORIENTATION == 0 ? -AcX : (ACCELEROMETER_ORIENTATION == 1 ? -AcX : (ACCELEROMETER_ORIENTATION == 2 ? -AcX : (ACCELEROMETER_ORIENTATION == 3 ? -AcY : AcY)))) / 164.0;
  a_sideway = (ACCELEROMETER_ORIENTATION == 0 ? AcY : (ACCELEROMETER_ORIENTATION == 1 ? AcZ : (ACCELEROMETER_ORIENTATION == 2 ? -AcZ : (ACCELEROMETER_ORIENTATION == 3 ? AcZ : -AcZ)))) / 164.0;
  a_vertical = (ACCELEROMETER_ORIENTATION == 0 ? AcZ : (ACCELEROMETER_ORIENTATION == 1 ? -AcY : (ACCELEROMETER_ORIENTATION == 2 ? AcY : (ACCELEROMETER_ORIENTATION == 3 ? AcX : AcX)))) / 164.0;

    //Serial.print("AcX: "); Serial.print(AcX);Serial.print(" AcY: "); Serial.print(AcY);Serial.print(" AcZ: "); Serial.print(AcZ);
  //Serial.print("a_forward:");Serial.print(a_forward);Serial.print(" a_sideway:");Serial.print(a_sideway);Serial.print(" a_vertical:");Serial.println(a_vertical);

  // Update long_lag references
  if (abs(a_forward-a_forward_long_lag)<a_forward_threshold){a_forward_long_lag=a_forward*long_lag_coef+(1-long_lag_coef)*a_forward_long_lag;}
  if (abs(a_sideway-a_sideway_long_lag)<a_sideway_threshold){a_sideway_long_lag=a_sideway*long_lag_coef+(1-long_lag_coef)*a_sideway_long_lag;}
  if (abs(a_vertical-a_vertical_long_lag)<a_vertical_threshold){a_vertical_long_lag=a_vertical*long_lag_coef+(1-long_lag_coef)*a_vertical_long_lag;}
  a_ratio_long_lag=a_ratio_multiplier*a_forward_long_lag/a_vertical_long_lag;
  
  // Update lag values
  a_forward_lag=a_forward*lag_coef+(1-lag_coef)*a_forward_lag;
  a_sideway_lag=a_sideway*lag_coef+(1-lag_coef)*a_sideway_lag;
  a_vertical_lag=a_vertical*lag_coef+(1-lag_coef)*a_vertical_lag;
  a_ratio_lag=a_ratio_multiplier*a_forward_lag/a_vertical_lag;

  // Update change values
  a_forward_change=a_forward_lag-a_forward_long_lag;
  a_sideway_change=a_sideway_lag-a_sideway_long_lag;
  a_vertical_change=a_vertical_lag-a_vertical_long_lag;
  a_ratio_change=a_ratio_lag-a_ratio_long_lag;

  // Evaluate current condition based on smoothed accelarations
  accel_status="cruising";

  // Test idle
  if(a_vertical!=0 && abs(a_vertical_change)<idle_test_threshold){idle_test_count++;}  // Prevent idle mode if accelerometer is not working
  else{idle_test_count=0;}
  if(idle_test_count>=idle_test_min_count){
    accel_status="idle";
    }

  // Test fallen
  if(abs(a_sideway_lag)>fallen_test_multiplier*abs(a_vertical_lag)){fallen_test_count++;}
  else{fallen_test_count=0;}
  if(fallen_test_count>=fallen_test_min_count){
    accel_status="fallen";
    }

  // Test strong braking
  if(a_ratio_change>=strong_braking_test_threshold){strong_braking_test_count++;}
  else{strong_braking_test_count=0;}
  if(strong_braking_test_count>=strong_braking_test_min_count){
    accel_status="strong_braking";
    }

  // Test long braking
  if(a_ratio_change>=long_braking_test_threshold){long_braking_test_count++;}
  else{long_braking_test_count=0;}
  if(long_braking_test_count>=long_braking_test_min_count){
    accel_status="long_braking";
    }

  
#if DEBUG_SERIAL
  Serial.print(" a_forward_change:");Serial.print(a_forward_change);
  Serial.print(" a_sideway_change:");Serial.print(a_sideway_change);
  Serial.print(" a_vertical_change:");Serial.print(a_vertical_change);
  Serial.print(" a_vertical:");Serial.print(a_vertical);
  Serial.print(" a_vertical_lag:");Serial.print(a_vertical_lag);
  Serial.print(" a_vertical_long_lag:");Serial.print(a_vertical_long_lag);
  Serial.print(" a_ratio_change:");Serial.print(a_ratio_change);
  Serial.print(" accel_status:");Serial.println(accel_status);
#endif
}

void CheckLight(){
  // Reads light level using photocell and writes status in light_status String.
  // Tunables: night_limit, day_limit
  // Output values: night, day, unknown
  photocellReading = analogRead(PhotocellPin);
  
  #if DEBUG_SERIAL
     Serial.print(" photocellReading: ");
     Serial.print(photocellReading);
  #endif
  
  if (photocellReading<night_limit){light_status="night";}
  if (photocellReading>day_limit){light_status="day";}
  // No changes in light_status for levels in between two limits to avoid constant toggling
  #if !USE_PHOTOCELL
     light_status="night";
  #endif
  
}

void setup() {
  // Initialize serial connection
  Serial.begin(115200);
  delay(500);
  Serial.printf("\nBegin Setup.\n");
  
  // Initialize board flash button
  wasPressed = false;
  pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);

  // Initialize MOSFET_GATE used to turn LED strip power on/off
  pinMode(MOSFET_GATE, OUTPUT); digitalWrite(MOSFET_GATE, HIGH);
  
  // Initialize built-in indicator LEDs
  pinMode(RED_LED, OUTPUT); digitalWrite(RED_LED, LOW);
  pinMode(BLUE_LED, OUTPUT); digitalWrite(BLUE_LED, LOW); 
  pinMode(GREEN_LED, OUTPUT); digitalWrite(GREEN_LED, LOW); 
   
  // Initialize LED strip and set startup pattern
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(LOW_BRIGHTNESS);
  fill_solid(leds, NUM_LEDS, CHSV(HUE_GREEN, 255, 100));
  FastLED.show();
  gPatternState = NORMAL;
  gActivePattern = solid;
  gSelectedPattern = solid;
  randomSeed( analogRead( A0 ) );

  // Set up MPU 6050 accelerometer
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  delay (500);

 // Run acceleometer calibration routine to set initial orientation
  Serial.println("Run acceleometer calibration routine to set initial orientation... ");
  CalibrateAccel();
  Serial.print("a_forward_offset = "); Serial.print(a_forward_offset);  
  Serial.print(" a_sideway_offset = "); Serial.print(a_sideway_offset);  
  Serial.print(" a_vertical_offset = "); Serial.println(a_vertical_offset); 

 // Initialize lagged variables used in CheckAccel() routine
  a_forward_long_lag=a_forward_offset;
  a_sideway_long_lag=a_sideway_offset;
  a_vertical_long_lag=a_vertical_offset;
  a_forward_lag=a_forward_offset;
  a_sideway_lag=a_sideway_offset;
  a_vertical_lag=a_vertical_offset;  
  idle_test_count=idle_test_min_count+1;

  // Initialize meshed network
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  mesh.scheduler.addTask( taskSendMessage );
  taskSendMessage.enable();

  uint32_t nodeId = mesh.getNodeId();
  Serial.printf("This module ==> NodeId: 0x%08x, Rider: %s\n", nodeId, rider[nodeId2riderIdx(nodeId)]);

  Serial.printf("Setup completed. %d\n", NUM_LEDS); 
}

void loop() {
  mesh.update();
  doButton();
  updatePattern();
  CheckAccel();
  
  EVERY_N_MILLISECONDS( 500 ) {
    CheckLight();
   }

#if MINIMAL_SERIAL
EVERY_N_MILLISECONDS( 500 ) {
  Serial.print("accel_status: "); Serial.print(accel_status);
  Serial.print(" light_status: "); Serial.println(light_status);
  }
#endif

switch (condition) {
    
    case Sleep_mode:
      LED("off");
      if (accel_status!="idle" & light_status=="night"){condition=Cruising_mode;}
      break;
      
    case Idle_mode:
      LED("idle");
      current_time=millis();
      elapsed_milliseconds=current_time-start_time;
      if (elapsed_milliseconds>SET_IDLE_MILLISECONDS){condition=Sleep_mode;}
      if (accel_status!="idle" & light_status=="night"){condition=Cruising_mode;}
      break;
      
    case Cruising_mode:
      if (light_status=="night"){LED("cruising");}else{LED("off");}
      if (accel_status=="strong_braking"||accel_status=="long_braking"){condition=Braking_mode;}
      if (accel_status=="fallen"){start_time=millis();condition=Panic_mode;}
      if (accel_status=="idle" & light_status=="night"){start_time=millis();condition=Idle_mode;}
      break;
      
    case Braking_mode:
      LED("braking");
      if (accel_status!="strong_braking" && accel_status!="long_braking"){condition=Post_Braking_mode;transition_timer=millis();}
      //if (accel_status!="strong_braking" && accel_status!="long_braking"){condition=Cruising_mode;}
      if (accel_status=="fallen"){condition=Panic_mode;}
      break;

    case Post_Braking_mode:
      if (millis()-transition_timer<SET_BRAKING_MILLISECONDS){LED("braking");}
      else {condition=Cruising_mode;}
      break;
      
    case Panic_mode:
      LED("panic");
      current_time=millis();
      elapsed_milliseconds=current_time-start_time;
      if (elapsed_milliseconds>SET_PANIC_MILLISECONDS){condition=Sleep_mode;}
      if (accel_status!="fallen"){condition=Cruising_mode;}
      break;
   }  
}


