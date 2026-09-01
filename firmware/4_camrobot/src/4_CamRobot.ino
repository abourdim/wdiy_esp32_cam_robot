/*
  WDIY ESP32-CAM Plugin Robot
  Ported from the keyestudio ESP32-CAM Video Smart Car (see git history).

  Define Network SSID & Password
  Set ap to 1 to use ESP32-CAM as Standalone Access Point with default IP 192.168.4.1
  Set ap to 0 to connect to a router using DHCP with hostname espressif

  WHAT CHANGED FROM THE CAR, IN ONE PLACE
  ---------------------------------------
  The board underneath this sketch is esp32_cam_plugin_robot: an AI-Thinker
  ESP32-CAM module plugged into a carrier that gives it two motors, one RGB
  pixel and a socket for a motion sensor. Three of its differences from the
  keyestudio car reach all the way up here, and every one of them is a pin:

    GPIO12/13  Two low-side MOSFET gates, one per motor. The car had an I2C
               motor controller on 14/13 with a direction bit per side; this
               has a single switch per side and therefore NO REVERSE. See
               SetMotor.h -- it is the difference that shaped everything else.

    GPIO4      Where the car's light moved to. The car drove its LED on
               GPIO12, which is a motor here, so every analogWrite(12, ...)
               had to move. Nothing is fitted at D3 on this build, so this
               just drives the module's own onboard flash LED.
*/
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "esp_wifi.h"
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "SetMotor.h"
#include "app_server.h"

// bool ap = 0;  //When it is 1, esp32 turns on wifi, the mobile phone is connected, and the IP is 192.168.4.1; when it is 0, it is connected to wifi, and the IP needs to be obtained through serial port printing.
// const char* ssid = "ChinaNet_2.4G";        //AP Name or Router SSID
// const char* password = "ChinaNet@233";  //Password. Leave blank for open network.

bool ap = 1;
const char* ssid = "wdiy1";         //AP Name or Router SSID
const char* password = "88888888";  //Password. Leave blank for open network.


//AP Settings
int channel = 11;       // Channel for AP Mode
int hidden = 0;         // Probably leave at zero
int maxconnection = 4;  // The car shipped this as 1, which meant a phone that
                        // auto-joined once held the only slot and every other
                        // device associated but got nothing above the radio.

// Station mode only: how long to wait for the router before giving up and
// falling back to standalone AP mode. See the connect block in setup().
#define STA_CONNECT_TIMEOUT_MS 20000

// Set when a WiFi network configured via the web-triggered WiFiManager setup
// portal (see enterWifiSetupPortal() in app_server.h) is what's actually in
// use, so loop()'s reconnect logic knows to retry that saved network instead
// of the hardcoded ssid/password above.
bool g_usingSavedWifi = false;

// Camera Pin Definitions - Don't heckin' touch.
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// The car had this as 12 and drove its lamp with analogWrite. GPIO12 is motor
// 2 on this board -- writing a brightness to it spins a wheel -- so the pin
// moves and nothing else about it changes.
//
// D3 (an addressable RGB pixel) is not populated on this build, so what this
// actually lights is the ESP32-CAM module's own flash LED, which is on the
// same pin. If you ever do fit D3, this has to become a NeoPixel write --
// PWM on a WS2812 data line produces garbage, not brightness.
#define LED_GPIO_NUM 4

// Webserver / Controls Function
void startCameraServer();

void setup() {
  // Disable the brownout detector. Camera init draws a current spike that can
  // sag voltage enough to false-trigger it on marginal power supplies -- this
  // was shipped commented out in the original sketch, which is almost never
  // what you want on an ESP32-CAM. This masks voltage *symptoms*, though; if
  // you're still getting camera init failures, the real fix is a better 5V
  // supply (see the troubleshooting notes above esp_camera_init below).
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Motors first, and before anything that can take a while. Until
  // ledcAttachPin() runs, GPIO12 and GPIO13 are inputs held down by R1/R2 and
  // the wheels are safely still -- but the first thing this does is give both
  // channels a duty of zero, so the robot is committed to standing still
  // rather than merely defaulting to it while the camera and WiFi take their
  // seconds.
  motor_init();
  pinMode(LED_GPIO_NUM, OUTPUT);
  analogWrite(LED_GPIO_NUM, 0);

  Serial.begin(115200);
  Serial.println();
  Serial.printf("CamRobot v%s  |  built %s  |  %s\n", FW_VERSION, FW_BUILD, GIT_REV);
  Serial.setDebugOutput(true);
  Serial.println();

  // Camera Configuration - Again, don't touch.
  // Zero-initialised on purpose: fb_location and grab_mode are set below but
  // every other field the driver reads must be deterministic. Left as a bare
  // local, they held stack garbage -- which is how a no-PSRAM board ended up
  // asking for a PSRAM frame buffer despite the else branch below.
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    // ESP32-WROOM-class boards (e.g. Acebott's QA011 camera board) have no
    // PSRAM die at all, so the frame buffer has to live in internal DRAM.
    // Without fb_location set the driver defaults to PSRAM and cam_dma_config()
    // fails with "frame buffer malloc failed" -- this branch's QVGA/fb_count=1
    // settings were correct but never got the chance to matter. A QVGA JPEG is
    // roughly 15KB, which DRAM carries comfortably next to WiFi and the web
    // server; there is no headroom above QVGA, so don't raise it on these boards.
    // With a single buffer there is nothing to pick between, so GRAB_WHEN_EMPTY.
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  // Camera init, with one retry at a lower XCLK. Some OV2640 clones don't
  // reliably lock at 20MHz; dropping to 10MHz on the first failure resolves
  // it without giving up entirely (a common real-world cause of the
  // ESP_ERR_NOT_SUPPORTED / 0x106 error alongside a loose ribbon cable).
  config.xclk_freq_hz = 20000000;
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed at 20MHz (0x%x), retrying at 10MHz...\n", err);
    config.xclk_freq_hz = 10000000;
    err = esp_camera_init(&config);
  }
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    // A missing camera and a missing PSRAM fail in completely different places,
    // so don't send people to check a ribbon cable that was never the problem.
    // If the sensor had not answered we'd have failed in camera_probe() well
    // before here, so by this point the cable and the pin map are known good.
    if (!psramFound()) {
      Serial.println("No PSRAM on this board, so the frame buffer must be in DRAM at");
      Serial.println("QVGA or smaller. If the saved resolution is above QVGA, that is");
      Serial.println("the cause -- otherwise DRAM is exhausted, not the camera at fault.");
    } else {
      Serial.println("Check: ribbon cable fully seated (contacts toward the PCB), a");
      Serial.println("solid 5V/2A supply (not just the FTDI adapter's regulator), and");
      Serial.println("that this is really an AI-Thinker-pinout board.");
    }
  }

  // --- A DEAD CAMERA IS NOT A DEAD ROBOT ------------------------------------
  // This used to be an infinite blink loop, inherited from the car. It sat
  // ABOVE the WiFi block and startCameraServer(), so a camera that failed to
  // initialise took the access point and the web server down with it: no
  // network, no page, no way to ask the robot what was wrong -- and on a board
  // whose only other output is one RGB pixel, no way to find out at all
  // without a serial cable.
  //
  // Now the failure is recorded and setup() carries on. WiFi comes up, the
  // server starts, and /status reports "camera":0 with the error code, so the
  // robot can be asked what happened from a phone. loop() blinks the fault
  // pattern on the pixel instead of this blocking here.
  g_cameraOk  = (err == ESP_OK);
  g_cameraErr = (int)err;

  //drop down frame size for higher initial frame rate
  sensor_t* s = g_cameraOk ? esp_camera_sensor_get() : NULL;

  // Load settings persisted to flash (NVS) by previous /control calls, so
  // Speed/Trim/Lights/Quality/Resolution/Flip/Mirror survive reboots and
  // reflashes instead of resetting to hardcoded defaults every time.
  // Defaults here (framesize QVGA, vflip/hmirror on) match what this sketch
  // always hardcoded before, so a fresh board with nothing saved yet still
  // boots exactly as it always did.
  prefs.begin("camrobot", false);
  vflipState = prefs.getInt("vflip", 1);
  hmirrorState = prefs.getInt("hmirror", 1);
  // Not applied to the sensor -- it cannot rotate. Loaded so /status can
  // hand it to the page, which does the rotation in CSS.
  rotateState = prefs.getInt("rotate", 0);
  // Guarded: s is NULL when the camera did not come up, and the settings that
  // are not the sensor's still have to be restored so the robot drives.
  if (s) {
    s->set_framesize(s, (framesize_t)prefs.getInt("framesize", FRAMESIZE_QVGA));
    // Quality 20, not 10. On this driver a *lower* number means a better,
    // bigger JPEG -- 10 is a photography setting, and for teleoperation the
    // bytes cost more than the detail is worth: every extra kilobyte per frame
    // is airtime the control requests don't get. 20 sits in the 20-24 band
    // that measured best for driving at QVGA and is still perfectly legible.
    //
    // IMPORTANT: only the *default* moves. A board whose Quality slider has
    // ever been touched keeps its saved value -- reflashing does NOT clear
    // NVS -- so a robot previously set to 10 stays at 10 until it is changed
    // back from the page. Check the live value at /status before drawing any
    // conclusion from a latency test.
    s->set_quality(s, prefs.getInt("quality", 20));
    s->set_vflip(s, vflipState);
    s->set_hmirror(s, hmirrorState);
  }
  speed = prefs.getInt("speed", speed);
  trim = prefs.getInt("trim", trim);
  flashLevel = prefs.getInt("flash", flashLevel);
  // AP channel, persisted like everything else. 2.4GHz is crowded and a busy
  // channel shows up as retransmissions and latency spikes rather than as a
  // dropped link, so being able to try 1 / 6 / 11 without a reflash each time
  // is worth the two lines. Read before softAP() below, which is the only
  // place it is used. Changed from the page with
  // /control?var=wifichannel&val=N, which saves and reboots -- an AP's channel
  // cannot be moved while it is up.
  int savedChan = prefs.getInt("wifichan", channel);
  // Validated, not trusted. /control clamps before saving, but NVS can also
  // hold a value from an older build or a corrupted entry, and an out-of-range
  // channel means softAP() fails and the robot vanishes -- on a board with no
  // screen, that is a serial cable to diagnose. Fall back to the compiled
  // default rather than to something clever.
  if (savedChan < 1 || savedChan > 13) {
    Serial.printf("Saved WiFi channel %d is out of range, using %d\n", savedChan, channel);
  } else {
    channel = savedChan;
  }

  analogWrite(LED_GPIO_NUM, flashLevel);

  // A WiFi network configured through the web's "WiFi setup" button (see
  // enterWifiSetupPortal() in app_server.h) is only ever recorded here on
  // success, so a board that's never used that flow boots exactly as it
  // always did -- straight into the block below unchanged.
  g_usingSavedWifi = prefs.getInt("wifi_mode", 0) == 1;

  if (g_usingSavedWifi || !ap) {
    if (g_usingSavedWifi) {
      Serial.println("WiFi is Client Scout32 (network saved via setup portal)");
      WiFi.mode(WIFI_STA);
      WiFi.setAutoReconnect(true);
      WiFi.begin();  // no args: reconnects to whatever the ESP32 itself last saved
    } else {
      // Connect to Router
      Serial.println("ssid: " + (String)ssid);
      Serial.println("password: " + (String)password);
      Serial.println("WiFi is Client Scout32");
      WiFi.mode(WIFI_STA);
      WiFi.setAutoReconnect(true);
      WiFi.begin(ssid, password);
    }
    // Bounded wait. This loop used to be unbounded, so a wrong password, a
    // router that's out of range, or a 5GHz-only SSID meant setup() never
    // returned: no camera server, no AP, no way to reach the car at all
    // without pulling it apart and attaching a serial cable. Time out
    // instead and fall through to AP mode, which always works.
    unsigned long staStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - staStart) < STA_CONNECT_TIMEOUT_MS) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Camera Ready! Use 'http://");
      Serial.print(WiFi.localIP());
      Serial.println("' to connect");
      ap = 0;
    } else {
      Serial.println("Could not join the WiFi network within STA_CONNECT_TIMEOUT_MS.");
      Serial.println("Falling back to standalone AP mode so the car stays reachable.");
      WiFi.disconnect(true);
      ap = 1;  // also stops loop()'s station-mode reconnect logic from running
    }
  }

  // Not an 'else': if the station-mode attempt above timed out it sets ap = 1
  // and falls through to here, so the car always ends up serving something.
  if (ap) {
    // Setup Access Point
    Serial.println("ssid: " + (String)ssid);
    Serial.println("password: " + (String)password);
    Serial.println("WiFi is Standalone Scout32");
    WiFi.mode(WIFI_AP);
    // softAP() returns false on failure and this used to ignore it, so a
    // radio that never came up printed "Camera Ready!" and an IP anyway --
    // the one line in the log you would trust, lying. Everything downstream
    // (the servers, OTA) is meaningless if this failed, so say which it was.
    bool apOk = WiFi.softAP(ssid, password, channel, hidden, maxconnection);
    if (!apOk) {
      Serial.println("ERROR: WiFi.softAP() FAILED -- no network will exist.");
      Serial.printf("  ssid='%s' channel=%d maxconn=%d\n", ssid, channel, maxconnection);
    } else {
      Serial.printf("AP started on channel %d. ", channel);
      Serial.print("Camera Ready! Use 'http://");
      Serial.print(WiFi.softAPIP());
      Serial.println("' to connect");
    }
  }

  // --- No modem sleep --------------------------------------------------
  // The Arduino core turns on WIFI_PS_MIN_MODEM by default, which parks the
  // radio between beacons. It saves power that a car driving four motors is
  // never going to notice, and it costs up to a beacon interval of latency on
  // every request -- exactly the jitter you feel as the steering answering
  // late. Set after the interface is up, since mode()/begin() reapply the
  // default. This matters most in station mode; in AP mode it is close to a
  // no-op, but it is correct in both and there is nothing to gain by making
  // it conditional.
  WiFi.setSleep(false);

  // ArduinoOTA works over whichever interface just came up above -- station
  // or the car's own AP -- since espota.py can target an IP directly with
  // -i and doesn't need mDNS discovery to do it. See firmware/4_camrobot/
  // ota_flash.sh and README.html's OTA section.
  ArduinoOTA.setHostname("camrobot");
  ArduinoOTA.begin();

  // --- Start the webserver BEFORE the boot animation --------------------
  // This used to come last, after two seconds of LED flashing and, once the
  // screen was added, several more of splash. All of that is blocking delay()
  // in setup(), so every second of it was a second the car was on the network
  // but refusing connections -- and lengthening the splash made the car worse.
  //
  // Starting the server here decouples the two. The handlers only need things
  // that are already up by this point (camera, prefs, the I2C bus, the
  // screen), the server runs in its own task, and the boot animation below now
  // plays to an audience that can already reach the car. Which means the
  // splash can be as long as it deserves to be and costs nothing.
  startCameraServer();

  //Flash LED as ready indicator
  for (int i = 0; i < 5; i++) {
    analogWrite(LED_GPIO_NUM, 0);
    delay(200);
    analogWrite(LED_GPIO_NUM, 255);
    delay(200);
  }
  // The car's version of this loop ended on 255 and left the lamp at full
  // brightness, discarding the Lights value it had restored from NVS moments
  // earlier. Put it back.
  analogWrite(LED_GPIO_NUM, flashLevel);

  // paint over it the instant setup() returns.
}

int i = 0;

// If no movement command has been received for this long, assume the
// connection was lost (WiFi drop, phone locked, browser tab closed, etc.)
// and stop the car rather than let it keep driving unattended.
#define CONTROL_TIMEOUT_MS 500

// How often loop() retries a web server that did not come up at boot. Long
// enough that a board with a genuine, permanent shortage is not spending its
// life in httpd_start(), short enough that a transient one recovers before
// anybody has finished walking over to check on the robot.
#define SERVER_RETRY_INTERVAL_MS 5000

// How often (ms) to retry WiFi.begin() when in station mode and disconnected.
#define WIFI_RECONNECT_INTERVAL_MS 5000

void loop() {
  // --- Safety failsafe: stop the car if the client has gone quiet ---
  if (actstate != stp && (millis() - lastCommandMillis > CONTROL_TIMEOUT_MS)) {
    Car_stop();
    actstate = stp;
    Serial.println("Failsafe: no command received recently, stopping car");
  }

  // --- Station mode: watch the WiFi link and reconnect if it drops ---
  if (!ap && WiFi.status() != WL_CONNECTED) {
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > WIFI_RECONNECT_INTERVAL_MS) {
      Serial.println("WiFi disconnected, attempting to reconnect...");
      WiFi.disconnect();
      if (g_usingSavedWifi) WiFi.begin();
      else WiFi.begin(ssid, password);
      // WiFi.begin() reapplies the core's default power save, so the
      // setSleep(false) from setup() has to be reasserted here. Without this
      // the robot silently goes back to modem sleep -- and its command jitter
      // with it -- after the first dropped link, with nothing in the log to
      // explain why it got laggy mid-session.
      WiFi.setSleep(false);
      lastReconnectAttempt = millis();
    }
  }

  // --- Web server retry ------------------------------------------------------
  // A server that failed to start at boot used to stay dead until someone power
  // cycled the robot -- and since the control page is the only way to ask this
  // board anything, "dead server" and "dead robot" look identical from outside.
  //
  // Retrying is worth it because the failure is usually transient in a specific
  // way: internal DRAM is at its tightest during setup(), with camera init and
  // WiFi association having just run, and a few seconds later there is often
  // room for what did not fit before. startCameraServer() skips whichever
  // server is already up, so this only ever starts what is actually missing --
  // including the stream server alone, if port 80 came up and port 81 did not.
  if (!camera_httpd || !stream_httpd) {
    static uint32_t lastServerRetry = 0;
    if (millis() - lastServerRetry > SERVER_RETRY_INTERVAL_MS) {
      lastServerRetry = millis();
      Serial.printf("Web server incomplete (control=%s stream=%s) -- retrying. free=%u largest=%u\n",
                    camera_httpd ? "up" : "DOWN", stream_httpd ? "up" : "DOWN",
                    ESP.getFreeHeap(), ESP.getMaxAllocHeap());
      startCameraServer();
    }
  }

  // --- Camera fault indicator ----------------------------------------------
  // The blocking version of this used to live in setup() and never returned.
  // Same rhythm -- three fast blinks, then a pause -- but driven from here, so
  // the web server, OTA and the failsafe all keep running underneath it.
  //
  if (!g_cameraOk) {
    static const uint16_t faultMs[7] = { 100, 100, 100, 100, 100, 100, 800 };
    static uint8_t phase = 0;
    static uint32_t lastPhaseAt = 0;
    if (millis() - lastPhaseAt >= faultMs[phase]) {
      lastPhaseAt = millis();
      phase = (phase + 1) % 7;
      digitalWrite(LED_GPIO_NUM, (phase < 6 && (phase % 2) == 0) ? HIGH : LOW);
    }
  }

  ArduinoOTA.handle();


  delay(50);
  // Serial.printf("RSSi: %ld dBm\n",WiFi.RSSI());
}
