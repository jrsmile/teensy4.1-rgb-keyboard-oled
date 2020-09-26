#include <U8g2lib.h>
#include <SPI.h>
#include <TeensyThreads.h>
#include "USBHost_t36.h"
#include <Smoothed.h>
#include <OctoWS2811.h>
#include <Encoder.h>
/*
 * Programming by Jan Reiss, 2020
 * Released to public domain
 */
 
#define ROW_COUNT 2        //Number of rows in the keyboard matrix
#define COL_COUNT 4       //Number of columns in the keyboard matrix

#define DEBOUNCE 5         //Adjust as needed: increase if bouncing problem, decrease if not all keypresses register; not less than 2
#define SCAN_DELAY 5        //Delay between scan cycles in ms

#define KEY_UNDEFINED -1    //For keyboard matrix positions not in use

#define TIME_CAL_1 2000 // joystick calibration time

int id1, id2, id3, id4; // threads

// OLED Stuff
IntervalTimer idleTimer; // Global keyboard idle timer
IntervalTimer PowerSaveTimer; // Global keyboard PowerSave timer
U8G2_SH1122_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);        // Enable U8G2_16BIT in u8g2.h

// Onboard LED Stuff disabled
const int LED = 19; // Teensy 4 LED PIN 13
volatile float smoothness_pts = 500;//larger=slower change in brightness  
volatile float gammab = 0.14; // affects the width of peak (more or less darkness)
volatile float beta = 0.5; // shifts the gaussian to be symmetric


//Keyboard Stuff
int rowPins[] = {2, 3};   //Teensy pins attached to matrix rows
int colPins[] = {7, 6, 5, 4};  //Teensy pins attached to matrix columns

//Key codes for layer 0 (standard layer)
int layer_0[] = {KEY_Q, KEY_W, KEY_E, MODIFIERKEY_ALT, 
                 KEY_A, KEY_S, KEY_D, KEY_SPACE};

volatile int currentRow = 0;
volatile int keyStatus[ROW_COUNT*COL_COUNT];

// LED Stuff 6x21 Matrix
const int numPins = 6;
byte pinList[numPins] = {24, 25, 26, 27, 28, 29};
const int ledsPerStrip = 27;
DMAMEM int displayMemory[ledsPerStrip * numPins * 3 / 4];
int drawingMemory[ledsPerStrip * numPins * 3 / 4];
const int config = WS2811_GRB | WS2811_800kHz;
OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, config, numPins, pinList);

// Encoder Stuff
const int knobCLK = 0;
const int knobDT = 1;
const int knobSW = 30;
int knobSWVal;
int knobSWlast;
int knobVal;
boolean bCW;
Encoder knob(knobCLK, knobDT);

// USB Host Stuff
USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
USBHIDParser hid4(myusb);
#define COUNT_JOYSTICKS 4
JoystickController joysticks[COUNT_JOYSTICKS](myusb);
int user_axis[64];
uint32_t buttons_prev = 0;

USBDriver *drivers[] = {&hub1, &joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3], &hid1, &hid2, &hid3, &hid4};
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1", "joystick[0D]", "joystick[1D]", "joystick[2D]", "joystick[3D]",  "HID1", "HID2", "HID3", "HID4"};
bool driver_active[CNT_DEVICES] = {false, false, false, false};

// Lets also look at HID Input devices
USBHIDInput *hiddrivers[] = {&joysticks[0], &joysticks[1], &joysticks[2], &joysticks[3]};
#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_DEVICES] = {"joystick[0H]", "joystick[1H]", "joystick[2H]", "joystick[3H]"};
bool hid_driver_active[CNT_DEVICES] = {false};
bool show_changed_only = false;

uint8_t joystick_left_trigger_value[COUNT_JOYSTICKS] = {0};
uint8_t joystick_right_trigger_value[COUNT_JOYSTICKS] = {0};
uint8_t joystick_left_stick_x_value[COUNT_JOYSTICKS] = {128};
uint8_t joystick_left_stick_y_value[COUNT_JOYSTICKS] = {128};
uint8_t joystick_right_stick_x_value[COUNT_JOYSTICKS] = {128};
uint8_t joystick_right_stick_y_value[COUNT_JOYSTICKS] = {128};
#define JOY_S 0
#define JOY_X 1
#define JOY_O 2
#define JOY_T 3
#define JOY_L1 4
#define JOY_R1 5
#define JOY_SHARE 8
#define JOY_OPTIONS 9
#define JOY_LS 10
#define JOY_RS 11
#define JOY_START 12
#define SCREEN_WIDTH 3840
#define SCREEN_HIGHT 2160
uint64_t joystick_full_notify_mask = (uint64_t) - 1;
Smoothed <uint8_t> rsvx_smoothed;
Smoothed <uint8_t> rsvy_smoothed;
int16_t  mouse_x = 0;
int16_t  mouse_y = 0;
bool right_down = false;
bool right_down_now = false;


void blinkthread() {
  while(1) {
    for (int ii=0;ii<smoothness_pts;ii++){
      float pwm_val = 255.0*(exp(-(pow(((ii/smoothness_pts)-beta)/gammab,2.0))/2.0));
      if (pwm_val <= 20) {
          pwm_val = 20;
      }
      analogWrite(LED,int(pwm_val));
    threads.delay(5);
    }
    threads.yield();
  }
}


void setup() {
  delay(1000);
  // Teensy OnBoard LED
  pinMode(LED, OUTPUT);
  id1 = threads.addThread(blinkthread);
  threads.setTimeSlice(id1, 10);
  //Row pins
  pinMode(rowPins[0], OUTPUT);
  digitalWrite(rowPins[0], LOW);
  
  int i;
  for (i=1;i<ROW_COUNT;i++){
    pinMode(rowPins[i], INPUT);
  }

  //Column pins
  for (i=0;i<COL_COUNT;i++){
    pinMode(colPins[i], INPUT_PULLUP);
  }

  //Clear keyStatus
  for (i=0;i<ROW_COUNT*COL_COUNT;i++){
    keyStatus[i]=0;
  }

  // Encoder Setup
  pinMode(knobCLK,INPUT_PULLUP);
  pinMode(knobDT,INPUT_PULLUP);
  pinMode(knobSW,INPUT_PULLUP);
  knobCLKlast = digitalRead(knobCLK);
  knobSWlast = digitalRead(knobSW);
  
  // OLED Setup
  u8g2.setBusClock(12000000); // ca. 100 FPS
  u8g2.enableUTF8Print();
  u8g2.begin(); // Init Display
  u8g2.setContrast(0);
  
  id2 = threads.addThread(scanKeys);
  id3 = threads.addThread(scanEncoder);
  Mouse.screenSize(SCREEN_WIDTH, SCREEN_HIGHT);  // configure screen size
  rsvx_smoothed.begin(SMOOTHED_EXPONENTIAL, 10);
  rsvy_smoothed.begin(SMOOTHED_EXPONENTIAL, 10);
  myusb.begin();
  leds.begin();
  leds.show();
  Serial.println("initialization done.");
  welcome();
#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0xFF1088
#define ORANGE 0xE05800
#define WHITE  0xFFFFFF
}


void loop() 
{
    
    // int microsec = 2000000 / leds.numPixels();  // change them all in 2 seconds

    // uncomment for voltage controlled speed
    // millisec = analogRead(A9) / 40;

    // colorWipe(RED, microsec);
    // colorWipe(GREEN, microsec);
    // colorWipe(BLUE, microsec);
    // colorWipe(YELLOW, microsec);
    // colorWipe(PINK, microsec);
    // colorWipe(ORANGE, microsec);
    // colorWipe(WHITE, microsec);
  
    myusb.Task();
    PrintDeviceListChanges();
    if (CheckUSBConnected()) {
      threads.suspend(id1);
      threads.suspend(id2);
    } else {
      threads.restart(id1);
      threads.restart(id2);
    }
    if (Serial.available()) {
    int ch = Serial.read(); // get the first char.
    while (Serial.read() != -1) ;
    if ((ch == 'b') || (ch == 'B')) {
      Serial.println("Only notify on Basic Axis changes");
      for (int joystick_index = 0; joystick_index < COUNT_JOYSTICKS; joystick_index++)
        joysticks[joystick_index].axisChangeNotifyMask(0x3ff);
    } else if ((ch == 'f') || (ch == 'F')) {
      Serial.println("Only notify on Full Axis changes");
      for (int joystick_index = 0; joystick_index < COUNT_JOYSTICKS; joystick_index++)
        joysticks[joystick_index].axisChangeNotifyMask(joystick_full_notify_mask);

    } else {
      if (show_changed_only) {
        show_changed_only = false;
        Serial.println("\n*** Show All fields mode ***");
      } else {
        show_changed_only = true;
        Serial.println("\n*** Show only changed fields mode ***");
      }
    }
  }

  for (int joystick_index = 0; joystick_index < COUNT_JOYSTICKS; joystick_index++) {
    if (joysticks[joystick_index].available()) {
      uint64_t axis_mask = joysticks[joystick_index].axisMask();
      uint64_t axis_changed_mask = joysticks[joystick_index].axisChangedMask();
      uint32_t buttons = joysticks[joystick_index].getButtons();
      Serial.printf("Joystick(%d): buttons = %x", joystick_index, buttons);
      Serial.println();
      //Serial.printf(" AMasks: %x %x:%x", axis_mask, (uint32_t)(user_axis_mask >> 32), (uint32_t)(user_axis_mask & 0xffffffff));
      //Serial.printf(" M: %lx %lx", axis_mask, joysticks[joystick_index].axisChangedMask());
      if (show_changed_only) {
        for (uint8_t i = 0; axis_changed_mask != 0; i++, axis_changed_mask >>= 1) {
          if (axis_changed_mask & 1) {
            //Serial.printf(" %d:%d", i, joysticks[joystick_index].getAxis(i));
          }
        }

      } else {
        for (uint8_t i = 0; axis_mask != 0; i++, axis_mask >>= 1) {
          if (axis_mask & 1) {
            // Serial.printf(" %d:%d", i, joysticks[joystick_index].getAxis(i));
          }
        }
      }
      uint8_t ltv;
      uint8_t rtv;
      uint8_t lsvx;
      uint8_t lsvy;
      uint8_t rsvx;
      uint8_t rsvy;
 
      switch (joysticks[joystick_index].joystickType()) {
        default:
          break;
        case JoystickController::PS4:
          lsvy = joysticks[joystick_index].getAxis(0);
          lsvx = joysticks[joystick_index].getAxis(1);
          rsvx = joysticks[joystick_index].getAxis(2);
          rsvy = joysticks[joystick_index].getAxis(5);
          ltv = joysticks[joystick_index].getAxis(3);
          rtv = joysticks[joystick_index].getAxis(4);
          if ((ltv != joystick_left_trigger_value[joystick_index]) || (rtv != joystick_right_trigger_value[joystick_index])) {
            joystick_left_trigger_value[joystick_index] = ltv;
            joystick_right_trigger_value[joystick_index] = rtv;
            joysticks[joystick_index].setRumble(ltv, rtv);
          }
          if ((lsvx != joystick_left_stick_x_value[joystick_index]) || ( lsvy != joystick_left_stick_y_value[joystick_index])) {
            joystick_left_stick_x_value[joystick_index] = lsvx;
            joystick_left_stick_y_value[joystick_index] = lsvy;
            if (lsvx >= 140) {
              Keyboard.press(KEY_S);
            } else if (lsvx <= 100) {
              Keyboard.press(KEY_W);
            } else if (lsvy >= 140) {
              Keyboard.press(KEY_D);
            } else if (lsvy <= 100) {
              Keyboard.press(KEY_A);
            }
            else {
              Keyboard.release(KEY_W);
              Keyboard.release(KEY_A);
              Keyboard.release(KEY_S);
              Keyboard.release(KEY_D);
            }

            
          }
          if (bitRead(buttons,JOY_RS)) {
              if (right_down && right_down_now) {
                Mouse.set_buttons(0, 0, 0);
                right_down_now = false;
                right_down = false;
              } else if (!right_down && right_down_now){
                Mouse.set_buttons(0, 0, 1);
                right_down_now = false;
                right_down = true;
              }
            } else {
              right_down_now = true;
            }
          if (bitRead(buttons,JOY_LS)) {
              Mouse.click();
            }
          if (bitRead(buttons,JOY_L1)) {
              Keyboard.press(KEY_TAB);
          } else {
            Keyboard.release(KEY_TAB);
            }
          if (bitRead(buttons,JOY_R1)) {
              Keyboard.press(KEY_SPACE);
          } else {
            Keyboard.release(KEY_SPACE);
            }
          if (bitRead(buttons,JOY_S)) {
              Keyboard.press(KEY_1);
          } else {
            Keyboard.release(KEY_1);
            }  
          if (bitRead(buttons,JOY_X)) {
              Keyboard.press(KEY_2);
          } else {
            Keyboard.release(KEY_2);
            }  
          if (bitRead(buttons,JOY_O)) {
              Keyboard.press(KEY_3);
          } else {
            Keyboard.release(KEY_3);
            }          
          if (bitRead(buttons,JOY_T)) {
              Keyboard.press(KEY_4);
          } else {
            Keyboard.release(KEY_4);
            }
            if (bitRead(buttons,JOY_START)) {
              Keyboard.press(KEY_ESC);
          } else {
            Keyboard.release(KEY_ESC);
            }
            if (bitRead(buttons,JOY_OPTIONS)) {
              Keyboard.press(KEY_C);
          } else {
            Keyboard.release(KEY_C);
            }
           if (bitRead(buttons,JOY_SHARE)) {
              Keyboard.press(KEY_B);
          } else {
            Keyboard.release(KEY_B);
            }    
          if ((rsvx != joystick_right_stick_x_value[joystick_index]) || ( rsvy != joystick_right_stick_y_value[joystick_index])) {
            rsvx_smoothed.add(rsvx);
            rsvy_smoothed.add(rsvy);
            if ((rsvx_smoothed.get() >= 140) || ( rsvx_smoothed.get() <= 100 ) || ( rsvy_smoothed.get() >= 140 ) || ( rsvy_smoothed.get() <= 100 )) {
              mouse_x = map(rsvx_smoothed.get(),0,255,-10,10);
              mouse_y = map(rsvy_smoothed.get(),0,255,-10,10);
            } else {
              mouse_x = 0;
              mouse_y = 0;
            }
            
/*
            if ((rsvx_smoothed.get() >= 140) || ( rsvx_smoothed.get() <= 100 ) || ( rsvy_smoothed.get() >= 140 ) || ( rsvy_smoothed.get() <= 100 )) {
              Mouse.set_buttons(0, 0, 1);
            } else {
              Mouse.set_buttons(0, 0, 0);
            }
*/
            Serial.printf("rsvx %d rsvy %d rsvx_smoothed %d rsvy_smoothed %d rsvx_old %d rsvy_old %d",rsvx,rsvy,rsvx_smoothed.get(),rsvy_smoothed.get(),joystick_right_stick_x_value[joystick_index], joystick_right_stick_y_value[joystick_index]);
            //Mouse.moveTo(map(rsvx_smoothed.get(),0,255,0,3840), map(rsvy_smoothed.get(),0,255,0,2160));
            Mouse.move(mouse_x, mouse_y);
            Serial.println();
            
            joystick_right_stick_x_value[joystick_index] = rsvx;
            joystick_right_stick_y_value[joystick_index] = rsvy;
          }
          break;

        case JoystickController::PS3:
          ltv = joysticks[joystick_index].getAxis(18);
          rtv = joysticks[joystick_index].getAxis(19);
          if ((ltv != joystick_left_trigger_value[joystick_index]) || (rtv != joystick_right_trigger_value[joystick_index])) {
            joystick_left_trigger_value[joystick_index] = ltv;
            joystick_right_trigger_value[joystick_index] = rtv;
            joysticks[joystick_index].setRumble(ltv, rtv, 50);
          }
          break;

        case JoystickController::XBOXONE:
        case JoystickController::XBOX360:
          ltv = joysticks[joystick_index].getAxis(4);
          rtv = joysticks[joystick_index].getAxis(5);
          if ((ltv != joystick_left_trigger_value[joystick_index]) || (rtv != joystick_right_trigger_value[joystick_index])) {
            joystick_left_trigger_value[joystick_index] = ltv;
            joystick_right_trigger_value[joystick_index] = rtv;
            joysticks[joystick_index].setRumble(ltv, rtv);
            // Serial.printf(" Set Rumble %d %d", ltv, rtv);
          }
          break;
      }
      if (buttons != buttons_prev) {
        if (joysticks[joystick_index].joystickType() == JoystickController::PS3) {
          joysticks[joystick_index].setLEDs((buttons >> 12) & 0xf); //  try to get to TRI/CIR/X/SQuare
        } else {
          uint8_t lr = (buttons & 1) ? 0xff : 0;
          uint8_t lg = (buttons & 2) ? 0xff : 0;
          uint8_t lb = (buttons & 4) ? 0xff : 0;
          joysticks[joystick_index].setLEDs(lr, lg, lb);
        }
        buttons_prev = buttons;
      }

      // Serial.println();
      joysticks[joystick_index].joystickDataClear();
    }
  }
}

void scanEncoder(){
    while(1) {
    knobVal = knob.read();
    if (knobVal != 0) {
      if (knobVal >= 1) {
        Keyboard.press(KEY_MEDIA_VOLUME_INC);
        Keyboard.release(KEY_MEDIA_VOLUME_INC);
      } else {
        Keyboard.press(KEY_MEDIA_VOLUME_DEC);
        Keyboard.release(KEY_MEDIA_VOLUME_DEC);
      }
      knob.write(0);
    }
    knobSWVal = digitalRead(knobSW);
    if (knobSWVal != knobSWlast) {
      if (knobSWVal){
        
      } else {
        Keyboard.press(KEY_MEDIA_MUTE);
        Keyboard.release(KEY_MEDIA_MUTE);
      }
      knobSWlast = knobSWVal;
    }
    threads.yield();
    }
}

/*
 * Scan keyboard matrix, results stored in keyStatus array
 * 
 * Key status values:
 * DEBOUNCE*2                   = key press event
 * DEBOUNCE+1 to DEBOUNCE*2-1   = key down
 * DEBOUNCE                     = key release event
 * 0 to DEBOUNCE-1              = key up
*/
void scanKeys(){
  int i;
  while(1) {
  for (i=0;i<ROW_COUNT*COL_COUNT;i++){
    int pin = getKeyPin(i);
    
    if (keyStatus[i]==0 && digitalRead(pin)==LOW){
      //Key press event
      keyStatus[i] = DEBOUNCE*2;
      keyPress(i);
    }
    else if (keyStatus[i]>DEBOUNCE+1){
      keyStatus[i]--;
    }
    else if (keyStatus[i]==DEBOUNCE+1 && digitalRead(pin)==HIGH){
      //Key release event
      keyStatus[i]--;
      keyRelease(i);
    }
    else if (keyStatus[i]>0 && keyStatus[i]<=DEBOUNCE){
      keyStatus[i]--;
    }
  }
  threads.yield();
  }
}

/*
 * Returns input pin to be read by keyScan method
 * 
 * Param key is the keyboard matrix scan code (row * COL_COUNT + col)
 */
int getKeyPin(int key){
  int p = key/COL_COUNT;
  if (p != currentRow){
    pinMode(rowPins[currentRow], INPUT);
    pinMode(rowPins[p], OUTPUT);
    digitalWrite(rowPins[p], LOW);
    currentRow=p;
  }
  delayMicroseconds(100);
  return colPins[key % COL_COUNT];
}

/*
 * Sends key press event
 * 
 * Param keyCode is the keyboard matrix scan code (row * COL_COUNT + col)
 */

void keyPress(int keyCode){
  if (layer_0[keyCode]!=KEY_UNDEFINED){
    Keyboard.press(layer_0[keyCode]);
    u8g2.setPowerSave(0);
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_unifont_t_latin); // choose a suitable font
    u8g2.drawStr(0,16,"pressed.");  // write something to the internal memory
    u8g2.setFont(u8g2_font_open_iconic_all_4x_t);
    u8g2.drawStr(200,32,"t");  // write something to the internal memory
    u8g2.setFont(u8g2_font_unifont_t_latin); // choose a suitable font
    u8g2.sendBuffer();          // transfer internal memory to the display
    idleTimer.begin(welcome,1000 * 1000 * 1);
  }
}

/*
 * Sends key release event
 * 
 * Param keyCode is the keyboard matrix scan code (row * COL_COUNT + col)
 */
 
void keyRelease(int keyCode){
  if (layer_0[keyCode]!=KEY_UNDEFINED){
    Keyboard.release(layer_0[keyCode]);
    u8g2.setPowerSave(0);
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_unifont_t_latin); // choose a suitable font
    u8g2.drawStr(0,16,"released.");  // write something to the internal memory
    u8g2.setFont(u8g2_font_open_iconic_all_4x_t);
    u8g2.drawStr(200,32,"w");  // write something to the internal memory
    u8g2.setFont(u8g2_font_unifont_t_latin); // choose a suitable font
    u8g2.sendBuffer();          // transfer internal memory to the display
    idleTimer.begin(welcome,1000 * 1000 * 1);
  }
}

/*
 * Relases all keys in the layer; called upon change of layer, i.e. press or release of FN key
 * 
 * Param layer[] array of key codes for the layer to be released
 */
void releaseLayer(int layer[]){
  int i;
  for (i=0;i<ROW_COUNT*COL_COUNT;i++){
    if (isKeyDown(i)){
      keyStatus[i] = DEBOUNCE;
      Keyboard.release(layer[i]);
    }
  }
}

/*
 * Returns 0 if the specified key is pressed, otherwise a value not equal to 0
 * 
 * Param keyCode is the keyboard matrix scan code (row * COL_COUNT * col)
 */
 int isKeyDown(int keyCode){
 if (keyStatus[keyCode]>DEBOUNCE) return 1; else return 0; 
}

//=============================================================================
// Show when devices are added or removed
//=============================================================================
void PrintDeviceListChanges() {
  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }

  for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
    if (*hiddrivers[i] != hid_driver_active[i]) {
      if (hid_driver_active[i]) {
        Serial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
        hid_driver_active[i] = false;
      } else {
        Serial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        hid_driver_active[i] = true;

        const uint8_t *psz = hiddrivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = hiddrivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = hiddrivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }
}

bool CheckUSBConnected() {
  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (driver_active[i]) {
      return true;
    } 
  }
  for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
    if (hid_driver_active[i]) {
      return true;
    }
  }
  return false;
}

void welcome()  {
  idleTimer.end();
  u8g2.setPowerSave(0);
  u8g2.clearBuffer();         // clear the internal memory
  //u8g2.setFont(u8g2_font_profont15_tf); // choose a suitable font 
  u8g2.setFont(u8g2_font_unifont_t_latin); // choose a suitable font
  u8g2.drawStr(0,16,"Taddie-Keyboard V1.0");  // write something to the internal memory
  u8g2.setFont(u8g2_font_open_iconic_all_4x_t);
  u8g2.drawStr(200,32,"s");  // write something to the internal memory
  u8g2.setFont(u8g2_font_unifont_t_latin); // choose a suitable font
  u8g2.drawStr(0,32,"Activating Keys...");  // write something to the internal memory
  u8g2.drawStr(0,48,"Activating LEDs...");  // write something to the internal memory
  u8g2.drawStr(0,64,"initialization done.");  // write something to the internal memory
  u8g2.sendBuffer();          // transfer internal memory to the display
  PowerSaveTimer.begin(poweroff,1000 * 1000 * 10);
}

void poweroff()  {
  PowerSaveTimer.end();
  u8g2.setPowerSave(1);
}

void colorWipe(int color, int wait)
{
  for (int i=0; i < leds.numPixels(); i++) {
    leds.setPixel(i, color);
    leds.show();
    delayMicroseconds(wait);
  }
}
