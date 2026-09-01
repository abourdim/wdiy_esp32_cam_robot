#ifndef APP_SERVER_H
#define APP_SERVER_H

#include <Preferences.h>
#include <Update.h>
#include <WiFiManager.h>

// Define Servo PWM Values & Stopped Variable
int speed = 248;  // matches the Speed slider's default UI position (8 * 31)
int trim = 0;

// Mirror of currently-applied settings that don't have their own dedicated
// global elsewhere, kept so /status can report the true device state (and
// so it round-trips exactly with what's persisted to flash below).
int flashLevel = 10;
int vflipState = 1;
int hmirrorState = 1;
// Display-only: the OV2640 has no rotate, so a sideways-mounted camera is
// corrected by a CSS transform in the control page. Stored here anyway --
// it describes how the camera is bolted to this chassis, so it belongs to
// the robot, not to whichever browser happens to be looking at it.
int rotateState = 0;   // 0, 90, 180 or 270 degrees

// Settings persist to the ESP32's own flash (NVS) via this, namespaced
// "videocar", so Speed/Trim/Lights/Quality/Resolution/Flip/Mirror survive
// reboots and reflashes and are the same for whichever device connects --
// see the load in setup() and the prefs.putInt() calls in cmd_handler below.
Preferences prefs;

// Turn rate, as a percentage of the Speed slider. Turns used to be hardcoded
// at PWM 150 and ignored the slider entirely. At the default slider position
// (speed = 248) this comes out at 148, i.e. the value the stock sketch
// hardcoded.
//
// It matters more here than it did on the car it came from. There, a turn ran
// the wheels in opposite directions and the robot spun inside its own
// footprint. With no reverse (see SetMotor.h) a turn is one wheel driven and
// one wheel coasting, so the robot swings through a wide arc around the
// stationary wheel -- and the faster the driven wheel goes, the wider that arc
// gets before anyone can react to it.
#define TURN_SPEED_PERCENT 60

// Apply the trim offset to one wheel's PWM magnitude and clamp to range.
// Trim compensates for one motor being physically stronger than the other:
// M1 gets -trim, M2 gets +trim. It earns its keep on this chassis more than it
// did on the geared one, because two bare MOSFETs driving two unmatched motors
// off the same battery rail is about as open-loop as a robot gets, and a
// straight line is entirely this slider's job.
// A wheel that isn't being driven stays stopped, so trim can't make the car
// creep when the throttle is at zero.
static inline int trimmed(int base, int offset) {
  if (base <= 0) return 0;
  int v = base + offset;
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return v;
}

// Whether the camera came up, and the error if it did not. setup() no longer
// stops on a camera failure (see 4_CamRobot.ino), so the web server can now be
// reached with no sensor behind it and every handler that touches one has to
// cope. Reported by /status so a phone can ask what went wrong -- which on a
// board with no serial and one RGB pixel is otherwise unanswerable.
volatile bool g_cameraOk = false;
volatile int  g_cameraErr = 0;

// Timestamp (millis) of the last movement command received from a client.
// loop() in the .ino watches this and force-stops the car if it goes stale,
// so the car doesn't keep driving after WiFi drops or the browser disconnects.
unsigned long lastCommandMillis = 0;


// Libraries. If you get errors compiling, please downgrade ESP32 by Espressif.
// Use version 1.0.2 (Tools, Manage Libraries).
#include <esp32-hal-ledc.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"

// --- Build identification ---------------------------------------------------
// There was previously no way to tell what was actually running on a car short
// of reflashing it, which made "are you sure that's the current firmware?" an
// unanswerable question -- and the troubleshooting notes lean on that question
// more than once. All three of these surface in the footer of the control page,
// in /status, and on the serial console at boot.
//
// FW_VERSION is bumped by hand; there's no release process behind it, it just
// gives the build a name a human can say out loud.
// Restarted at 1.0.0 for this board rather than carrying on from the VideoCar
// 1.6.1 this was ported from: same web app, different robot underneath, and a
// version that went backwards in capability while going forwards in number
// would be the wrong story to tell. The lineage is in the git history.
#define FW_VERSION "1.0.0"

// Set by the compiler, so it is always right and needs no build system support.
#define FW_BUILD __DATE__ " " __TIME__

// Injected by PlatformIO's pre-build hook (see firmware/4_camrobot/git_rev.py),
// which appends "-dirty" when the tree has uncommitted changes. The Arduino IDE
// has no pre-build hook, so builds from there fall back to "nogit" -- an honest
// "unknown", rather than a stale hash that would be worse than no hash at all.
#ifndef GIT_REV
#define GIT_REV "nogit"
#endif
#include "SetMotor.h"


// Stream Encoding
typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

// Ceiling on the MJPEG frame rate (50ms => 20fps). See the pacing block at
// the bottom of stream_handler for why an uncapped stream is a control-latency
// problem rather than a bandwidth one.
//
// This is also the A/B knob for "is video still stealing airtime from the
// controls?". Set it to 100 (10fps) and reflash: if steering tightens up
// noticeably, WiFi/video contention is still a major contributor and the
// answer is a lower frame rate or a smaller JPEG, not more control-path work.
// If it makes no difference, the remaining latency is elsewhere -- power,
// RF noise, or channel congestion.
#define STREAM_MIN_FRAME_MS 50

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;
static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  int64_t fr_start = esp_timer_get_time();

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

  // The camera is configured for PIXFORMAT_JPEG, so this is a straight
  // pass-through of the frame buffer. (The stock sketch carried an unreachable
  // RGB888 round-trip below this point -- dl_matrix3du_alloc / fmt2rgb888 /
  // fmt2jpg_cb -- which was the only thing that needed lib/dl_lib*.h. Both are
  // gone; if you ever set a non-JPEG pixel format, frame2jpg_cb here still
  // handles the conversion.)
  size_t fb_len = 0;
  if (fb->format == PIXFORMAT_JPEG) {
    fb_len = fb->len;
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    jpg_chunking_t jchunk = { req, 0 };
    res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);
    fb_len = jchunk.len;
  }
  esp_camera_fb_return(fb);
  int64_t fr_end = esp_timer_get_time();
  Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  // Let the control page read frames out of this stream with a canvas. The
  // page is served from port 80 and this stream from port 81, which browsers
  // treat as separate origins -- without this header, drawImage() from the
  // <img> taints the canvas and tf.browser.fromPixels() throws a SecurityError.
  // That's why the vision loops historically polled /capture instead, which
  // costs an extra full frame grab from the camera. With CORS the follow-me
  // loop can reuse frames the stream is already sending.
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  // Someone has opened the video. That is the most honest definition of
  // "connected" this car has, and it is what flips the screen from showing
  // its address to showing its face.

  while (true) {
    int64_t frame_start = esp_timer_get_time();
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }

    // --- Frame pacing ---------------------------------------------------
    // Uncapped, this loop hands out frames as fast as the sensor and the
    // radio can manage, and the video takes first call on all the spare
    // airtime -- control packets then have to fight their way through it.
    // 20fps is more than enough to drive by and leaves the /control and
    // /joystick requests room to get in and out promptly.
    //
    // Note this is a *ceiling*, not a target: a board with no PSRAM runs
    // fb_count=1 / GRAB_WHEN_EMPTY at QVGA and never gets near 20fps, so the
    // delay below simply never fires there. It's the PSRAM boards, which do
    // run flat out, that this reins in.
    //
    // The per-frame Serial.printf("MJPG: ...") that used to sit here is gone:
    // with setDebugOutput(true) it shared the UART with everything else and
    // bought nothing in normal use.
    int elapsed_ms = (int)((esp_timer_get_time() - frame_start) / 1000);
    if (elapsed_ms < STREAM_MIN_FRAME_MS) {
      vTaskDelay(pdMS_TO_TICKS(STREAM_MIN_FRAME_MS - elapsed_ms));
    }
  }

  // The loop above only ever leaves by the client going away, so this is
  // where "somebody is watching" stops being true.
  return res;
}



// Control Handling from Server
// Setup states of motion for Scout.
// 'trn' (turning in place) exists so that a turn counts as "the car is
// moving" for loop()'s failsafe check. Previously the Left/Right commands
// set the motors but left actstate untouched, so a turn started from a
// standstill left actstate == stp and the failsafe never fired -- lose WiFi
// mid-turn and the car span in place indefinitely. Nothing branches on the
// specific value, only on "!= stp", so adding a state here is safe.
enum state { fwd,
             rev,
             trn,
             stp };
state actstate = stp;

static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf;
  size_t buf_len;
  char variable[32] = {
    0,
  };
  char value[32] = {
    0,
  };

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK && httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  int val = atoi(value);
  // NULL when the camera failed to initialise. Only the four sensor settings
  // below need it; driving, Lights, Speed and Trim all work without a camera,
  // so this returns 500 for those four rather than refusing the whole endpoint.
  sensor_t *s = esp_camera_sensor_get();
  int res = 0;
  const bool needsSensor =
      !strcmp(variable, "framesize") || !strcmp(variable, "quality")
      || !strcmp(variable, "vflip") || !strcmp(variable, "hmirror");
  if (needsSensor && !s) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_send(req, "No camera on this robot right now -- see /status", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
  }

  if (!strcmp(variable, "framesize")) {
    Serial.println("framesize");
    if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
    prefs.putInt("framesize", val);
  } else if (!strcmp(variable, "quality")) {
    Serial.println("quality");
    res = s->set_quality(s, val);
    prefs.putInt("quality", val);
  } else if (!strcmp(variable, "vflip")) {
    Serial.println("vflip");
    res = s->set_vflip(s, val);
    vflipState = val;
    prefs.putInt("vflip", val);
  } else if (!strcmp(variable, "hmirror")) {
    Serial.println("hmirror");
    res = s->set_hmirror(s, val);
    hmirrorState = val;
    prefs.putInt("hmirror", val);
  } else if (!strcmp(variable, "rotate")) {
    // Deliberately NOT in needsSensor above: this never touches the sensor,
    // so it still works on a robot whose camera failed to come up.
    if (val != 90 && val != 180 && val != 270) val = 0;
    rotateState = val;
    prefs.putInt("rotate", val);
  }
  //Remote Control
  else if (!strcmp(variable, "flash"))  //Headlight brightness
  {
    // Was analogWrite(12, val). GPIO12 is motor 2 on this board and writing a
    // brightness to it would drive a wheel, so the pin moves to 4 -- the
    // module's own flash LED. Same 0..255 range, same NVS key.
    analogWrite(4, val);
    flashLevel = val;
    prefs.putInt("flash", val);
  } else if (!strcmp(variable, "speed"))  //Speed settings
  {
    if (val > 8) val = 8;
    else if (val < 0) val = 0;
    speed = val * 31;
    prefs.putInt("speed", speed);
  } else if (!strcmp(variable, "trim"))  //trim
  {
    if (val > 32) val = 32;
    else if (val < -32) val = -32;
    trim = val;
    prefs.putInt("trim", val);
  } else if (!strcmp(variable, "wifichannel"))  //AP channel, for congestion testing
  {
    // A busy 2.4GHz channel shows up as retransmissions and latency spikes,
    // not as a dropped link, so the only way to rule it out is to try a few.
    // 1, 6 and 11 are the non-overlapping ones worth comparing. Clamped to
    // the range every regulatory domain allows -- 12/13 are not legal
    // everywhere and a bad value would leave the AP unreachable, which on a
    // board with no screen means a serial cable to recover.
    if (val < 1) val = 1;
    else if (val > 11) val = 11;
    prefs.putInt("wifichan", val);
    // Answer before restarting, or the browser sees a dropped connection
    // rather than a confirmation. The AP's channel is fixed at softAP() time,
    // so there is no way to apply this without coming back up.
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, NULL, 0);
    Serial.printf("WiFi channel set to %d, restarting to apply...\n", val);
    delay(200);
    ESP.restart();
    return ESP_OK;
  } else if (!strcmp(variable, "car")) {
    lastCommandMillis = millis();
    if (val == 1)  //Forward
    {
      int speed1 = trimmed(speed, -trim);
      int speed2 = trimmed(speed, +trim);
      actstate = fwd;  // Set state to modify left & right behavior while moving.
      motorMirror(speed1, speed2, 0);
      motor_write(M1_CH, speed1);
      motor_write(M2_CH, speed2);
    } else if (val == 2)  //Backward -- not on this chassis
    {
      // The car this came from ran both motors the other way through its
      // H-bridge here. There is no H-bridge: each motor has one low-side
      // MOSFET and can only be driven or not driven (see SetMotor.h). So the
      // honest answer to "go backwards" is to stop.
      //
      // It still answers 200 rather than an error, and still stamps
      // lastCommandMillis, because this path is reachable from a browser
      // holding a cached copy of an older control page -- and a stale page
      // pressing Back should get a robot that sits still, not one that starts
      // logging failures at 10Hz. The current page has no Back button at all.
      actstate = stp;
      Car_stop();
    } else if (val == 3)  //Left
    {
      actstate = trn;  // must be non-stp or the failsafe in loop() ignores us
      int turnBase = (speed * TURN_SPEED_PERCENT) / 100;
      int speed1 = trimmed(turnBase, -trim);
      // A forward pivot, not a spin: drive the right wheel, let the left one
      // coast, and the robot swings left around it. Trim is applied to the
      // wheel that is actually turning and the other is simply zero -- there
      // is no second wheel to balance against here, which is why only one
      // trimmed() call survives from the four-line version this replaces.
      motorMirror(speed1, 0, -1);
      motor_write(M1_CH, speed1);
      motor_write(M2_CH, 0);
    } else if (val == 4)  //Right
    {
      actstate = trn;  // must be non-stp or the failsafe in loop() ignores us
      int turnBase = (speed * TURN_SPEED_PERCENT) / 100;
      int speed2 = trimmed(turnBase, +trim);
      motorMirror(0, speed2, +1);
      motor_write(M1_CH, 0);
      motor_write(M2_CH, speed2);
    }
     else if(val == 5) //Stop
    {
      Car_stop();
      actstate = stp;
    }
  } else {
    Serial.println("variable");
    res = -1;
  }

  if (res) { return httpd_resp_send_500(req); }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// Joystick control: GET /joystick?x=<-100..100>&y=<0..100>
// x: negative = left, positive = right. y: forward only -- see below.
// Uses arcade-drive mixing (throttle +/- steer) and honors the existing
// 'speed' slider value as the maximum PWM cap.
//
// The y range is the one thing about this endpoint that changed in the port.
// Negative y meant reverse and this chassis has none (SetMotor.h), so it is
// clamped to zero on arrival rather than rejected: the control page already
// stops sending it, but an older cached page, a saved bookmark or somebody
// poking at the API by hand should all get the same well-defined "stop
// pushing back, then" rather than a 404 or, worse, a wheel that lurches.
static esp_err_t joystick_handler(httpd_req_t *req) {
  char *buf;
  size_t buf_len;
  char xval[16] = { 0 };
  char yval[16] = { 0 };

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len <= 1) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  buf = (char *)malloc(buf_len);
  if (!buf) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  if (httpd_req_get_url_query_str(req, buf, buf_len) != ESP_OK
      || httpd_query_key_value(buf, "x", xval, sizeof(xval)) != ESP_OK
      || httpd_query_key_value(buf, "y", yval, sizeof(yval)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  lastCommandMillis = millis();

  int jx = atoi(xval);
  int jy = atoi(yval);
  if (jx > 100) jx = 100;
  else if (jx < -100) jx = -100;
  if (jy > 100) jy = 100;
  else if (jy < 0) jy = 0;  // no reverse on this chassis -- see the note above

  // Arcade mixing, unchanged from the car: M1 = throttle-steer, M2 =
  // throttle+steer, matching the Forward/Left/Right cases above.
  //
  // Only the lower clamp moved, from -100 to 0, and that one edit is the whole
  // of forward-only steering. A wheel whose mixed value goes negative simply
  // stops instead of reversing, so pushing the stick hard over at zero
  // throttle drives one wheel and coasts the other -- the same forward pivot
  // the D-pad's Left and Right do, arrived at by arithmetic rather than by a
  // special case. Push forward as well and the pivot opens out into a curve.
  int s1 = jy - jx;
  int s2 = jy + jx;
  if (s1 > 100) s1 = 100;
  else if (s1 < 0) s1 = 0;
  if (s2 > 100) s2 = 100;
  else if (s2 < 0) s2 = 0;

  // Reuse the existing Speed slider (0..248) as the max PWM. Note there is
  // deliberately no "if (cap <= 0) cap = 248" fallback here any more: that
  // made a slider sitting at 0 mean *full speed* on the joystick while
  // meaning stopped on the D-pad. Zero now means zero for both.
  int cap = speed;
  if (cap < 0) cap = 0;
  else if (cap > 255) cap = 255;
  // Trim applies here too, so the joystick drifts the same way the D-pad does
  // once you've dialled it in -- previously the slider silently did nothing
  // in joystick mode. abs() is kept even though s1/s2 can no longer be
  // negative: it costs nothing and it is one fewer thing to have to re-check
  // if the clamps above are ever loosened again.
  int pwm1 = trimmed((abs(s1) * cap) / 100, -trim);
  int pwm2 = trimmed((abs(s2) * cap) / 100, +trim);

  // jy is already clamped at zero, so a stick pushed straight back reads as
  // centred. Testing both against zero, rather than jy alone, is what makes
  // that a stop rather than a robot that keeps its last command until the
  // 500ms failsafe notices.
  if (jx == 0 && jy == 0) {
    Car_stop();
    actstate = stp;
  } else {
    // Intent for the face. Both wheel values are non-negative now, so this is
    // just the pair of PWMs; jx still carries the steer directly, which is why
    // it never had to be recovered from the wheels.
    motorMirror(pwm1, pwm2, jx > 0 ? 1 : (jx < 0 ? -1 : 0));
    motor_write(M1_CH, pwm1);
    motor_write(M2_CH, pwm2);
    // Never rev: with jy clamped at zero the only two states the robot has are
    // moving forward (possibly while pivoting) and stopped.
    actstate = fwd;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// --- WiFi setup portal ------------------------------------------------------
// The robot normally boots straight into its own "wdiy1" AP (or the hardcoded
// router credentials, if ap=0 was set at compile time) -- that default never
// changes. This adds an *opt-in* way to join a different WiFi network without
// reflashing: a button on the control page hits /wifi-setup, which tears down
// the camera server and hands control to WiFiManager's captive portal. The
// portal is its own separate WiFi network ("CamRobot-Setup"); join it from a
// phone or laptop, pick the target WiFi from the list it shows, and enter its
// password. On success the credentials are saved to the ESP32's own NVS (the
// same store WiFi.begin() with no arguments reconnects from) and a flag is
// set so setup() knows to try that network before falling back to today's
// default. The car reboots either way -- success, failure, or timeout -- so
// it always comes back up through the same, well-tested boot path.
#define WIFI_SETUP_PORTAL_TIMEOUT_S 180

void enterWifiSetupPortal() {
  Serial.println("Entering WiFi setup portal (CamRobot-Setup)...");
  httpd_stop(camera_httpd);
  httpd_stop(stream_httpd);

  WiFiManager wm;
  wm.setConfigPortalTimeout(WIFI_SETUP_PORTAL_TIMEOUT_S);
  bool connected = wm.startConfigPortal("CamRobot-Setup");

  if (connected) {
    Serial.println("WiFi setup portal succeeded, saving and rebooting...");
    prefs.putInt("wifi_mode", 1);
  } else {
    Serial.println("WiFi setup portal timed out or failed, rebooting to previous mode...");
  }
  delay(200);
  ESP.restart();
}

static esp_err_t wifisetup_handler(httpd_req_t *req) {
  const char *msg =
    "Rebooting into WiFi setup mode.\n\n"
    "Connect to the \"CamRobot-Setup\" WiFi network within the next 3 minutes, "
    "then follow the page that appears to pick your home WiFi and enter its "
    "password.\n\n"
    "If nothing happens within 3 minutes, the car reboots back into its usual "
    "mode on its own.";
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, msg, strlen(msg));
  // Give the response time to actually reach the client before the servers
  // backing this connection get torn down.
  delay(300);
  enterWifiSetupPortal();  // never returns -- reboots at the end
  return ESP_OK;
}

// --- OTA firmware upload (browser) ------------------------------------------
// POST /update with the raw .bin as the request body (no multipart wrapper --
// a plain `fetch('/update', {method:'POST', body: file})` sends exactly
// that, with a correct Content-Length, which is all this needs). Companion to
// ArduinoOTA (network push via PlatformIO's espota, set up in the .ino) for
// anyone who doesn't have PlatformIO installed. Same lack of authentication as
// every other endpoint on this car -- see report.html.
static esp_err_t update_handler(httpd_req_t *req) {
  int remaining = req->content_len;
  if (remaining <= 0) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  if (!Update.begin(remaining)) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_send(req, "Update.begin() failed -- image too big for the OTA partition?", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
  }

  char buf[1024];
  while (remaining > 0) {
    int to_read = remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf);
    int r = httpd_req_recv(req, buf, to_read);
    if (r <= 0) {
      Update.abort();
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (Update.write((uint8_t *)buf, r) != (size_t)r) {
      Update.abort();
      httpd_resp_set_status(req, "500 Internal Server Error");
      httpd_resp_send(req, Update.errorString(), HTTPD_RESP_USE_STRLEN);
      return ESP_FAIL;
    }
    remaining -= r;
  }

  if (!Update.end(true)) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_send(req, Update.errorString(), HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
  }

  httpd_resp_send(req, "OK, rebooting...", HTTPD_RESP_USE_STRLEN);
  delay(300);
  ESP.restart();
  return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];  // Tell Jason his name is spelled wrong.


  sensor_t *s = esp_camera_sensor_get();
  char *p = json_response;
  *p++ = '{';

  // Reported first and unconditionally: when this is 0 it is the only field
  // that matters, and it is the answer to "why is there no picture" for
  // somebody with no serial cable.
  p += sprintf(p, "\"camera\":%d,", g_cameraOk ? 1 : 0);
  p += sprintf(p, "\"camerr\":%d,", g_cameraErr);

  // s is NULL with no camera; fall back to the saved values so the page still
  // paints its sliders instead of failing to load.
  p += sprintf(p, "\"framesize\":%u,", s ? s->status.framesize : (uint8_t)FRAMESIZE_QVGA);
  p += sprintf(p, "\"quality\":%u,", s ? s->status.quality : 10);
  p += sprintf(p, "\"vflip\":%d,", vflipState);
  p += sprintf(p, "\"hmirror\":%d,", hmirrorState);
  p += sprintf(p, "\"rotate\":%d,", rotateState);
  p += sprintf(p, "\"speed\":%d,", speed / 31);
  p += sprintf(p, "\"trim\":%d,", trim);
  p += sprintf(p, "\"flash\":%d,", flashLevel);
  p += sprintf(p, "\"version\":\"%s\",", FW_VERSION);
  p += sprintf(p, "\"build\":\"%s\",", FW_BUILD);
  p += sprintf(p, "\"gitrev\":\"%s\",", GIT_REV);
  // Constant for this firmware, but the page has no other way to know it and
  // it decides whether a reverse control should be drawn at all.
  p += sprintf(p, "\"reverse\":0");
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

// Front End / GUI Webpage
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!doctype html>
<html>
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
        <title>WDIY ESP32-CAM Plugin Robot</title>
    <link href="data:image/png;base64,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" rel="icon" />
        <style>
          :root{
            --bg:#0B0F14; --panel:#121822; --panel-2:#0E141C; --border:#1F2B38;
            --cyan:#4CE0D2; --amber:#FFB454; --danger:#FF5C5C;
            --text:#E7F1F2; --muted:#7C93A0;
            --mono:ui-monospace,SFMono-Regular,Consolas,"Liberation Mono",Menlo,monospace;
          }
          *{box-sizing:border-box;-webkit-tap-highlight-color:transparent;}
          /* overflow-x lives on <html>, not <body>: an overflow value on an
             ancestor of a position:sticky element turns that ancestor into a
             scroll container and the stickiness silently stops working. On
             <html> it is the viewport scroller anyway, so sticky still resolves
             against the viewport. */
          html{overflow-x:hidden;}
          /* The viewfinder pins to the top of the screen while the panels scroll
             underneath. The drive controls sit directly below it now, but the
             sliders and vision panels are still a scroll away -- and reaching
             them used to mean scrolling the video off-screen and steering a
             moving vehicle blind. Full-bleed via negative margins so panels
             don't show through the body's side padding as they pass behind. */
          #video-dock{position:sticky;top:0;z-index:30;background:var(--bg);
            margin:0 -12px;padding:4px 12px 8px;}
          /* Collapsible panels. Eight panels is about five phone-screens; with
             Drive and Manual pinned under the video, everything else is
             reference you open when you want it rather than something you need
             in view while driving. Collapsed by default; the open set is
             remembered per browser. Drive and Manual are deliberately not
             collapsible -- they are the point of the page. */
          .panel.collapsible > .panel-label{cursor:pointer;}
          .panel.collapsible > .panel-label::before{content:"\25BE";font-size:8px;
            color:var(--cyan);margin-right:7px;transition:transform .15s ease;}
          .panel.collapsed > .panel-label::before{transform:rotate(-90deg);}
          .panel.collapsed > .panel-label{margin-bottom:0;}
          .panel.collapsed > *:not(.panel-label){display:none !important;}
          .panel.collapsed{padding-bottom:14px;}
          body{width:100%;max-width:420px;margin:0 auto;padding:0 12px 24px;
            font-family:var(--mono);background:var(--bg);color:var(--text);font-size:14px;}
          header{display:flex;align-items:center;justify-content:space-between;padding:14px 2px 10px;}
          header h1{font-size:15px;letter-spacing:.12em;margin:0;font-weight:700;text-transform:uppercase;}
          header h1 span{color:var(--amber);}
          #lang-switch{display:flex;gap:4px;}
          .lang-btn{background:var(--panel);border:1px solid var(--border);
            display:flex;align-items:center;padding:3px;border-radius:5px;cursor:pointer;
            opacity:.55;line-height:0;}
          .lang-btn.active{border-color:var(--amber);opacity:1;box-shadow:0 0 0 1px var(--amber);}
          .lang-btn svg{display:block;border-radius:2px;}
          .panel{background:var(--panel);border:1px solid var(--border);border-radius:10px;
            margin:0 0 12px;padding:12px;}
          .panel-label{font-size:10px;letter-spacing:.18em;text-transform:uppercase;color:var(--muted);
            margin:0 0 10px;display:flex;align-items:center;gap:6px;}
          .panel-label::after{content:"";flex:1;height:1px;background:var(--border);}

          #conn-status{display:flex;align-items:center;gap:6px;font-size:11px;letter-spacing:.06em;color:var(--muted);}
          #conn-dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--muted);
            box-shadow:0 0 6px var(--muted);transition:background .3s,box-shadow .3s;}

          #viewfinder{position:relative;border-radius:8px;overflow:hidden;background:var(--panel-2);
            border:1px solid var(--border);aspect-ratio:4/3;}
          #stream{display:block;width:100%;height:100%;object-fit:cover;}
          #detect-overlay,#follow-overlay,#cv-overlay{position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none;}

          /* --- Camera rotation -------------------------------------------
             The OV2640 can flip and mirror but it cannot rotate: there is no
             combination of set_vflip/set_hmirror that produces a quarter
             turn, so a camera mounted sideways on the chassis has to be
             corrected here, in the page. Applied to the video and the three
             overlay canvases together -- they are all the same box and the
             overlays draw in image coordinates, so rotating them as one keeps
             the detection boxes sitting on the thing they detected. The HUD
             (corner brackets, LIVE, CAM-01) is deliberately left out so it
             stays upright whichever way the camera is bolted on.

             At 90/270 the frame itself turns portrait, so the viewfinder's
             aspect-ratio is swapped to 3/4 and the media is sized to the
             parent's *other* axis before rotating: width 133.333% is the
             parent's height, height 75% is the parent's width, so the
             quarter-turn lands exactly on the box with nothing cropped and
             no letterboxing. */
          #viewfinder.rot90,#viewfinder.rot270{aspect-ratio:3/4;}
          #viewfinder.rot180 #stream,#viewfinder.rot180 canvas{
            transform:rotate(180deg);}
          #viewfinder.rot90 #stream,#viewfinder.rot90 canvas,
          #viewfinder.rot270 #stream,#viewfinder.rot270 canvas{
            position:absolute;top:50%;left:50%;width:133.3333%;height:75%;}
          #viewfinder.rot90 #stream,#viewfinder.rot90 canvas{
            transform:translate(-50%,-50%) rotate(90deg);}
          #viewfinder.rot270 #stream,#viewfinder.rot270 canvas{
            transform:translate(-50%,-50%) rotate(270deg);}
          .vf-corner{position:absolute;width:22px;height:22px;border:2px solid var(--cyan);opacity:.85;}
          .vf-tl{top:8px;left:8px;border-right:0;border-bottom:0;}
          .vf-tr{top:8px;right:8px;border-left:0;border-bottom:0;}
          .vf-bl{bottom:8px;left:8px;border-right:0;border-top:0;}
          .vf-br{bottom:8px;right:8px;border-left:0;border-top:0;}
          #rec-indicator{position:absolute;top:10px;left:34px;display:flex;align-items:center;gap:5px;
            font-size:10px;letter-spacing:.1em;color:var(--danger);text-shadow:0 1px 2px rgba(0,0,0,.6);}
          #rec-dot{width:7px;height:7px;border-radius:50%;background:var(--danger);animation:blink 1.2s infinite;}
          @keyframes blink{0%,45%{opacity:1;}50%,95%{opacity:.15;}100%{opacity:1;}}
          #vf-tag{position:absolute;bottom:10px;right:34px;font-size:10px;letter-spacing:.06em;
            color:var(--cyan);text-shadow:0 1px 2px rgba(0,0,0,.6);}

          #joystick-container{display:flex;justify-content:center;align-items:center;padding:6px 0 4px;}
          #joystick-base{position:relative;width:150px;height:150px;border-radius:50%;
            background:radial-gradient(circle at 50% 50%,var(--panel-2) 0%,var(--panel-2) 60%,var(--border) 100%);
            border:1px solid var(--border);touch-action:none;}
          #joystick-base::before,#joystick-base::after{content:"";position:absolute;background:var(--border);}
          #joystick-base::before{left:50%;top:6px;bottom:6px;width:1px;margin-left:-.5px;}
          #joystick-base::after{top:50%;left:6px;right:6px;height:1px;margin-top:-.5px;}
          #joystick-thumb{position:absolute;left:50%;top:50%;width:56px;height:56px;margin:-28px 0 0 -28px;
            border-radius:50%;background:radial-gradient(circle at 35% 30%,#6FE9DD,var(--cyan) 60%,#2FA79B 100%);
            box-shadow:0 2px 8px rgba(76,224,210,.45);}

          .btn-row{display:grid;grid-template-columns:1fr 1fr;gap:8px;}
          .btn-row button{height:44px;}
          #vision-btn{grid-column:1/-1;}
          #plates-btn{grid-column:1/-1;}
          #plates-status{margin-top:8px;font-size:11px;color:var(--muted);letter-spacing:.02em;min-height:14px;line-height:1.4;}
          #plates-btn.active{background:#E85DBF;border-color:#E85DBF;color:var(--bg);}
          #codes-btn{grid-column:1/-1;}
          #codes-status{margin-top:8px;font-size:11px;color:var(--muted);letter-spacing:.02em;min-height:14px;line-height:1.4;}
          #codes-btn.active{background:#D4E157;border-color:#D4E157;color:var(--bg);}
          #pose-btn{grid-column:1/-1;}
          #pose-status{margin-top:8px;font-size:11px;color:var(--muted);letter-spacing:.02em;min-height:14px;line-height:1.4;}
          #pose-btn.active{background:#7DD3FC;border-color:#7DD3FC;color:var(--bg);}
          #hands-btn{grid-column:1/-1;}
          #hands-status{margin-top:8px;font-size:11px;color:var(--muted);letter-spacing:.02em;min-height:14px;line-height:1.4;}
          #hands-btn.active{background:#FB923C;border-color:#FB923C;color:var(--bg);}
          #chase-btn{grid-column:1/-1;}
          #cv-status{margin-top:8px;font-size:11px;color:var(--muted);letter-spacing:.02em;min-height:14px;line-height:1.4;}
          #motion-btn.active{background:#F472B6;border-color:#F472B6;color:var(--bg);}
          #line-btn.active{background:#FBBF24;border-color:#FBBF24;color:var(--bg);}
          #chase-btn.active{background:#22D3EE;border-color:#22D3EE;color:var(--bg);}
          .cv-row{display:flex;gap:8px;align-items:center;margin-top:8px;font-size:11px;color:var(--muted);}
          .cv-row label{display:flex;align-items:center;gap:4px;}
          #cv-color{width:44px;height:32px;padding:2px;border:1px solid var(--border);border-radius:6px;
            background:var(--panel-2);cursor:pointer;}
          #cv-sample{height:32px;font-size:11px;padding:0 10px;}
          #offline-btn{grid-column:1/-1;}
          #offline-status{margin-top:8px;font-size:11px;color:var(--muted);letter-spacing:.02em;min-height:14px;line-height:1.4;}
          #offline-stats{margin-top:6px;font-size:11px;color:var(--cyan);letter-spacing:.02em;min-height:14px;}
          #follow-btn{grid-column:1/-1;}
          #follow-status{margin-top:8px;font-size:11px;color:var(--muted);letter-spacing:.02em;min-height:14px;line-height:1.4;}
          #follow-btn.active{background:#4ADE80;border-color:#4ADE80;color:var(--bg);}
          #follow-target{width:100%;margin-top:8px;height:38px;background:var(--panel-2);color:var(--fg);
            border:1px solid var(--border);border-radius:6px;padding:0 10px;font:inherit;font-size:13px;}
          #expr-btn{grid-column:1/-1;}
          #expr-status{margin-top:8px;font-size:11px;color:var(--muted);letter-spacing:.02em;min-height:14px;line-height:1.4;}
          #expr-btn.active{background:#FB7185;border-color:#FB7185;color:var(--bg);}
          #record-btn.active{background:var(--danger);border-color:var(--danger);color:var(--bg);}
          #record-status{margin-top:8px;font-size:11px;color:var(--muted);letter-spacing:.04em;min-height:14px;}
          #record-status.active{color:var(--danger);}
          #vision-status{margin-top:8px;font-size:11px;color:var(--muted);letter-spacing:.02em;min-height:14px;line-height:1.4;}
          #vision-btn.active{background:var(--cyan);border-color:var(--cyan);color:var(--bg);}
          #flip-row{display:grid;grid-template-columns:1fr 1fr;gap:8px;}
          #flip-row button{height:36px;font-size:10px;}
          #flip-row button.active{background:var(--cyan);border-color:var(--cyan);color:var(--bg);}

          #dpad{display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(2,56px);gap:6px;}
          #dpad button{grid-row:span 1;}
          button{display:block;width:100%;height:100%;border:1px solid var(--border);color:var(--text);
            background:var(--panel-2);border-radius:8px;outline:0;font-family:var(--mono);
            font-size:11px;letter-spacing:.08em;text-transform:uppercase;
            -webkit-touch-callout:none;-webkit-user-select:none;user-select:none;transition:background .15s,border-color .15s;}
          button:active{background:#1A2530;border-color:var(--cyan);color:var(--cyan);}
          #forward{grid-column:2;grid-row:1;}
          /* The car's Back button lived here. This chassis has no reverse
             (SetMotor.h), so rather than leave a hole in the middle of the
             D-pad the slot goes to Stop -- which the keyboard-driven loop
             below was already sending on key release, but which nothing on
             the page could ask for outright. */
          #stopbtn{grid-column:2;grid-row:2;}
          #stopbtn:active{background:#2A1520;border-color:var(--amber);color:var(--amber);}
          #turnleft{grid-column:1;grid-row:1/span 2;}
          #turnright{grid-column:3;grid-row:1/span 2;}

          .slider-row{display:grid;grid-template-columns:66px 1fr 34px;align-items:center;gap:8px;margin:10px 0;}
          .slider-row label{font-size:10px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);}
          .slider-row .val{font-size:11px;color:var(--amber);text-align:right;}
          input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:24px;background:transparent;margin:0;}
          input[type=range]:focus{outline:0;}
          input[type=range]::-webkit-slider-runnable-track{width:100%;height:4px;cursor:pointer;background:var(--border);border-radius:2px;}
          input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;border:2px solid var(--panel);height:20px;width:20px;
            border-radius:50%;background:var(--cyan);cursor:pointer;margin-top:-8px;box-shadow:0 0 6px rgba(76,224,210,.5);}
          input[type=range]:focus::-webkit-slider-runnable-track{background:var(--border);}
          input[type=range]::-moz-range-track{width:100%;height:4px;background:var(--border);border-radius:2px;}
          input[type=range]::-moz-range-thumb{border:2px solid var(--panel);height:20px;width:20px;border-radius:50%;
            background:var(--cyan);cursor:pointer;}
          #app-footer{width:100%;text-align:center;padding:16px 0 4px;}
          #fw-build{display:block;margin-top:8px;color:var(--muted);font-size:10px;
            letter-spacing:.06em;font-family:var(--mono,monospace);opacity:.75;}
          #app-footer a{color:var(--muted);font-size:11px;letter-spacing:.04em;text-decoration:none;
            border-bottom:1px dotted var(--border);}
          #app-footer a:active,#app-footer a:hover{color:var(--cyan);border-bottom-color:var(--cyan);}
        </style>
    </head>
    <body>
      <header>
        <h1>&#129302; CAM<span>ROBOT</span></h1>
        <div id="lang-switch">
          <button class="lang-btn active" data-lang="en" title="English" aria-label="English" onclick="setLang('en')"><svg width="22" height="15" viewBox="0 0 60 40"><rect width="60" height="40" fill="#012169"/><path d="M0 0L60 40M60 0L0 40" stroke="#fff" stroke-width="6.5"/><path d="M0 0L60 40M60 0L0 40" stroke="#C8102E" stroke-width="3.5"/><path d="M30 0V40M0 20H60" stroke="#fff" stroke-width="11"/><path d="M30 0V40M0 20H60" stroke="#C8102E" stroke-width="6.5"/></svg></button>
          <button class="lang-btn" data-lang="fr" title="Français" aria-label="Français" onclick="setLang('fr')"><svg width="22" height="15" viewBox="0 0 60 40"><rect width="20" height="40" fill="#002395"/><rect x="20" width="20" height="40" fill="#fff"/><rect x="40" width="20" height="40" fill="#ED2939"/></svg></button>
          <button class="lang-btn" data-lang="ar" title="العربية" aria-label="العربية" onclick="setLang('ar')"><svg width="22" height="15" viewBox="0 0 900 600"><defs><clipPath id="glVC1"><rect width="450" height="600"/></clipPath></defs><rect width="450" height="600" fill="#006233"/><rect x="450" width="450" height="600" fill="#FFF"/><g fill="#D21034" transform="translate(450,300)"><circle r="125"/><circle cx="25" r="100" fill="#FFF"/><g clip-path="url(#glVC1)" transform="translate(-450,-300)"><circle cx="475" cy="300" r="100" fill="#006233"/></g><path d="M85-12l22 67h71l-57 42 22 67-58-42-57 42 21-67-57-42h71z"/></g></svg></button>
        </div>
        <div id="conn-status"><span id="conn-dot"></span><span id="conn-text" data-i18n="conn_connecting">Connecting&hellip;</span></div>
      </header>

      <div id="video-dock">
      <div class="panel" style="padding:8px;">
        <div id="viewfinder">
          <img id="stream" src="">
          <canvas id="detect-overlay"></canvas>
          <canvas id="follow-overlay"></canvas>
          <canvas id="cv-overlay"></canvas>
          <div class="vf-corner vf-tl"></div>
          <div class="vf-corner vf-tr"></div>
          <div class="vf-corner vf-bl"></div>
          <div class="vf-corner vf-br"></div>
          <div id="rec-indicator"><span id="rec-dot"></span><span data-i18n="live">LIVE</span></div>
          <div id="vf-tag">CAM-01</div>
        </div>
      </div>

      </div>

      <section class="panel">
        <div class="panel-label" data-i18n="panel_drive">Drive</div>
        <div id="joystick-container">
          <div id="joystick-base">
            <div id="joystick-thumb"></div>
          </div>
        </div>
      </section>
      <section class="panel">
        <div class="panel-label" data-i18n="panel_manual">Manual</div>
        <div id="dpad">
          <button id="forward" onpointerdown="document.dispatchEvent(fwdpress);" onpointerup="document.dispatchEvent(fwdrelease);" onpointerleave="document.dispatchEvent(fwdrelease);">&#9650; <span data-i18n="btn_fwd">Fwd</span></button>
          <button id="turnleft" onpointerdown="document.dispatchEvent(leftpress);" onpointerup="document.dispatchEvent(leftrelease);" onpointerleave="document.dispatchEvent(leftrelease);">&#9664; <span data-i18n="btn_left">Left</span></button>
          <button id="turnright" onpointerdown="document.dispatchEvent(rightpress);" onpointerup="document.dispatchEvent(rightrelease);" onpointerleave="document.dispatchEvent(rightrelease);"><span data-i18n="btn_right">Right</span> &#9654;</button>
          <button id="stopbtn" onclick="stopNow();">&#9632; <span data-i18n="btn_stop">Stop</span></button>
        </div>
      </section>
      <section class="panel collapsible" data-panel="autonomy">
        <div class="panel-label" data-i18n="panel_autonomy">Autonomy</div>
        <div class="btn-row">
          <button id="follow-btn" onclick="toggleFollow()">&#127919; <span data-i18n="btn_follow">Follow Me</span></button>
        </div>
        <select id="follow-target">
          <option value="person" data-i18n="follow_person">Follow: person</option>
          <option value="sports ball" data-i18n="follow_ball">Follow: ball</option>
          <option value="dog" data-i18n="follow_dog">Follow: dog</option>
          <option value="cat" data-i18n="follow_cat">Follow: cat</option>
          <option value="bottle" data-i18n="follow_bottle">Follow: bottle</option>
          <option value="chair" data-i18n="follow_chair">Follow: chair</option>
        </select>
        <div id="follow-status" data-i18n="follow_status">Drives the robot on its own to keep the target centred and at a fixed distance. Any manual input disarms it. Give it clear floor space before arming.</div>
      </section>
      <section class="panel collapsible" data-panel="classiccv">
        <div class="panel-label" data-i18n="panel_classiccv">Classic CV</div>
        <div class="btn-row">
          <button id="motion-btn" onclick="toggleCV('motion')">&#128064; <span data-i18n="btn_motion">Motion</span></button>
          <button id="line-btn" onclick="toggleCV('line')">&#12336; <span data-i18n="btn_line">Line</span></button>
        </div>
        <div class="btn-row" style="margin-top:8px;">
          <button id="chase-btn" onclick="toggleCV('chase')">&#127912; <span data-i18n="btn_chase">Colour chase</span></button>
        </div>
        <div class="cv-row">
          <input type="color" id="cv-color" value="#dc2828">
          <button id="cv-sample" onclick="sampleColour()" data-i18n="btn_sample">Sample centre</button>
          <label><input type="checkbox" id="cv-invert"> <span data-i18n="label_light_line">light line</span></label>
        </div>
        <div id="cv-status" data-i18n="cv_status">No models and no internet &mdash; pure pixel maths, so these keep up with the video stream on the robot's own WiFi. Line and Colour chase drive the robot; give it floor space, and leave the Speed slider high since these cap themselves well below it.</div>
      </section>
      <section class="panel collapsible" data-panel="vision">
        <div class="panel-label" data-i18n="panel_vision">Vision</div>
        <div class="btn-row">
          <button id="vision-btn" onclick="toggleVision()">&#129302; <span data-i18n="btn_vision">AI Vision</span></button>
        </div>
        <div id="vision-status" data-i18n="vision_status">Runs in your browser (TensorFlow.js) &mdash; detects objects (cyan) and faces (amber). Needs internet on this network to load the models once.</div>
        <div class="btn-row" style="margin-top:8px;">
          <button id="plates-btn" onclick="togglePlates()">&#128290; <span data-i18n="btn_plates">Plates</span></button>
        </div>
        <div id="plates-status" data-i18n="plates_status">Reads text off detected vehicles (magenta) via OCR. Heuristic, low-res (320&times;240) camera &mdash; works best close, well-lit, and square-on to the plate.</div>
        <div class="btn-row" style="margin-top:8px;">
          <button id="codes-btn" onclick="toggleCodes()">&#9638; <span data-i18n="btn_codes">QR / Barcode</span></button>
        </div>
        <div id="codes-status" data-i18n="codes_status">Uses your browser's built-in scanner when available &mdash; no internet needed for this one. Falls back to a QR-only library (needs internet once) on browsers without native support.</div>
        <div class="btn-row" style="margin-top:8px;">
          <button id="pose-btn" onclick="togglePose()">&#128694; <span data-i18n="btn_pose">Pose</span></button>
        </div>
        <div id="pose-status" data-i18n="pose_status">Skeleton tracking (MoveNet) &mdash; needs internet once to load.</div>
        <div class="btn-row" style="margin-top:8px;">
          <button id="hands-btn" onclick="toggleHands()">&#9995; <span data-i18n="btn_hands">Hands</span></button>
        </div>
        <div id="hands-status" data-i18n="hands_status">Hand/finger tracking &mdash; needs internet once to load.</div>
        <div class="btn-row" style="margin-top:8px;">
          <button id="expr-btn" onclick="toggleExpr()">&#128512; <span data-i18n="btn_expr">Expression</span></button>
        </div>
        <div id="expr-status" data-i18n="expr_status">Reads facial expressions (happy/sad/surprised/...) &mdash; needs internet once to load.</div>
      </section>
      <section class="panel collapsible" data-panel="capture">
        <div class="panel-label" data-i18n="panel_capture">Capture</div>
        <div class="btn-row">
          <button id="snapshot-btn" onclick="takeSnapshot()">&#128247; <span data-i18n="btn_snapshot">Snapshot</span></button>
          <button id="record-btn" onclick="toggleRecording()">&#9210; <span data-i18n="btn_record">Record</span></button>
        </div>
        <div id="record-status"></div>
      </section>
      <section class="panel collapsible" data-panel="systems">
        <div class="panel-label" data-i18n="panel_systems">Systems</div>
        <div class="slider-row">
          <label data-i18n="label_speed">Speed</label>
          <input type="range" id="speed" min="0" max="8" value="8" oninput="document.getElementById('speed-val').textContent=this.value" onchange="sendControl('speed', this.value)">
          <span class="val" id="speed-val">8</span>
        </div>
        <div class="slider-row">
          <label data-i18n="label_trim">Trim</label>
          <input type="range" id="trim" min="-32" max="32" value="0" oninput="document.getElementById('trim-val').textContent=this.value" onchange="sendControl('trim', this.value)">
          <span class="val" id="trim-val">0</span>
        </div>
        <div class="slider-row">
          <label data-i18n="label_lights">Lights</label>
          <input type="range" id="flash" min="0" max="255" value="10" oninput="document.getElementById('flash-val').textContent=this.value" onchange="sendControl('flash', this.value)">
          <span class="val" id="flash-val">10</span>
        </div>
        <div class="slider-row">
          <label data-i18n="label_quality">Quality</label>
          <input type="range" id="quality" min="10" max="63" value="10" oninput="document.getElementById('quality-val').textContent=this.value" onchange="sendControl('quality', this.value)">
          <span class="val" id="quality-val">10</span>
        </div>
        <div class="slider-row">
          <label data-i18n="label_resolution">Resolution</label>
          <input type="range" id="framesize" min="0" max="6" value="5" oninput="document.getElementById('framesize-val').textContent=this.value" onchange="sendControl('framesize', this.value)">
          <span class="val" id="framesize-val">5</span>
        </div>
        <div class="slider-row" style="grid-template-columns:66px 1fr;">
          <label data-i18n="label_orientation">Orientation</label>
          <div id="flip-row">
            <button id="vflip-btn" class="active" onclick="toggleFlip('vflip', this)">&#8597; <span data-i18n="btn_flip">Flip</span></button>
            <button id="hmirror-btn" class="active" onclick="toggleFlip('hmirror', this)">&#8596; <span data-i18n="btn_mirror">Mirror</span></button>
            <button id="rotate-btn" onclick="cycleRotate()">&#8635; <span data-i18n="btn_rotate">Rotate</span> <span id="rotate-val">0&deg;</span></button>
          </div>
        </div>
      </section>
      <section class="panel collapsible" data-panel="offline">
        <div class="panel-label" data-i18n="panel_offline">Offline</div>
        <div id="offline-stats" data-i18n="offline_checking">Checking cache&hellip;</div>
        <div id="offline-status" data-i18n="offline_status">Model files are saved to this browser as they download. Turn the Vision features on once somewhere with internet and they'll work afterwards on the robot's own WiFi, which has none.</div>
        <div class="btn-row" style="margin-top:8px;">
          <button id="offline-btn" onclick="clearOfflineCache()">&#128465; <span data-i18n="btn_clear_cache">Clear cached models</span></button>
        </div>
      </section>
      <section class="panel collapsible" data-panel="firmware">
        <div class="panel-label" data-i18n="panel_firmware">Firmware / Network</div>
        <div id="ota-version" data-i18n="ota_version_loading">Checking current version&hellip;</div>
        <div class="btn-row" style="margin-top:8px;">
          <input type="file" id="ota-file" accept=".bin" style="display:none" onchange="otaFileChosen()">
          <button id="ota-pick-btn" onclick="document.getElementById('ota-file').click()">&#128190; <span data-i18n="btn_ota_pick">Choose .bin</span></button>
          <button id="ota-upload-btn" onclick="otaUpload()" disabled>&#11014; <span data-i18n="btn_ota_upload">Upload &amp; flash</span></button>
        </div>
        <div id="ota-progress-row" style="display:none;margin-top:8px;">
          <progress id="ota-progress" value="0" max="100" style="width:100%;"></progress>
        </div>
        <div id="ota-status" data-i18n="ota_status_idle">Pick a firmware.bin built for this board, then Upload &amp; flash. The robot reboots automatically when done.</div>
        <div class="btn-row" style="margin-top:14px;">
          <button id="wifi-setup-btn" onclick="wifiSetup()">&#128246; <span data-i18n="btn_wifi_setup">WiFi setup&hellip;</span></button>
        </div>
        <div id="wifi-setup-status" data-i18n="wifi_setup_status">Join a home WiFi network instead of the robot's own AP. Opens a separate "CamRobot-Setup" network to pick it from &mdash; the robot reboots and is briefly unreachable during setup.</div>
      </section>
      <footer id="app-footer">
        <a href="https://github.com/abourdim/wdiy_esp32_cam_robot/blob/main/README.md" target="_blank" rel="noopener">&#128214; <span data-i18n="footer_tutorial">Build guide</span></a>
        &nbsp;|&nbsp;
        <a href="https://github.com/abourdim/wdiy_esp32_cam_robot" target="_blank" rel="noopener">&#128187; <span data-i18n="footer_source">Source on GitHub</span></a>
        <span id="fw-build">firmware &hellip;</span>
      </footer>
    <script>
   // --- i18n (EN/FR/AR), same data-i18n + dictionary pattern as bit-bot ---
   var RTL_LANGS = { ar: true };
   var I18N = {
     en: {
       conn_connecting: 'Connecting…', live: 'LIVE',
       panel_drive: 'Drive', panel_manual: 'Manual',
       btn_fwd: 'Fwd', btn_left: 'Left', btn_right: 'Right', btn_stop: 'Stop',
       panel_autonomy: 'Autonomy', btn_follow: 'Follow Me',
       follow_person: 'Follow: person', follow_ball: 'Follow: ball', follow_dog: 'Follow: dog',
       follow_cat: 'Follow: cat', follow_bottle: 'Follow: bottle', follow_chair: 'Follow: chair',
       follow_status: 'Drives the robot on its own to keep the target centred and at a fixed distance. Any manual input disarms it. Give it clear floor space before arming.',
       panel_classiccv: 'Classic CV', btn_motion: 'Motion', btn_line: 'Line', btn_chase: 'Colour chase',
       btn_sample: 'Sample centre', label_light_line: 'light line',
       cv_status: 'No models and no internet — pure pixel maths, so these keep up with the video stream on the robot’s own WiFi. Line and Colour chase drive the robot; give it floor space, and leave the Speed slider high since these cap themselves well below it.',
       panel_vision: 'Vision', btn_vision: 'AI Vision',
       vision_status: 'Runs in your browser (TensorFlow.js) — detects objects (cyan) and faces (amber). Needs internet on this network to load the models once.',
       btn_plates: 'Plates',
       plates_status: 'Reads text off detected vehicles (magenta) via OCR. Heuristic, low-res (320×240) camera — works best close, well-lit, and square-on to the plate.',
       btn_codes: 'QR / Barcode',
       codes_status: 'Uses your browser’s built-in scanner when available — no internet needed for this one. Falls back to a QR-only library (needs internet once) on browsers without native support.',
       btn_pose: 'Pose', pose_status: 'Skeleton tracking (MoveNet) — needs internet once to load.',
       btn_hands: 'Hands', hands_status: 'Hand/finger tracking — needs internet once to load.',
       btn_expr: 'Expression', expr_status: 'Reads facial expressions (happy/sad/surprised/...) — needs internet once to load.',
       panel_capture: 'Capture', btn_snapshot: 'Snapshot', btn_record: 'Record',
       panel_systems: 'Systems', label_speed: 'Speed', label_trim: 'Trim', label_lights: 'Lights',
       label_quality: 'Quality', label_resolution: 'Resolution', label_orientation: 'Orientation',
       btn_flip: 'Flip', btn_mirror: 'Mirror', btn_rotate: 'Rotate',
       panel_offline: 'Offline', offline_checking: 'Checking cache…',
       offline_status: 'Model files are saved to this browser as they download. Turn the Vision features on once somewhere with internet and they’ll work afterwards on the robot’s own WiFi, which has none.',
       btn_clear_cache: 'Clear cached models',
       panel_firmware: 'Firmware / Network', ota_version_loading: 'Checking current version…',
       btn_ota_pick: 'Choose .bin', btn_ota_upload: 'Upload & flash',
       ota_status_idle: 'Pick a firmware.bin built for this board, then Upload & flash. The robot reboots automatically when done.',
       ota_status_ready: 'Ready to upload — this will reboot the robot when it finishes.',
       ota_status_uploading: 'Uploading…', ota_status_done: 'Done — rebooting…',
       ota_status_error: 'Upload failed: ',
       btn_wifi_setup: 'WiFi setup…',
       wifi_setup_status: 'Join a home WiFi network instead of the robot’s own AP. Opens a separate "CamRobot-Setup" network to pick it from — the robot reboots and is briefly unreachable during setup.',
       wifi_setup_confirm: 'This reboots the robot into WiFi setup mode. You’ll need to join the "CamRobot-Setup" network to finish. Continue?',
       wifi_setup_started: 'Rebooting into WiFi setup mode — join "CamRobot-Setup" from your WiFi list within 3 minutes.',
       footer_tutorial: 'Build guide', footer_source: 'Source on GitHub'
     },
     fr: {
       conn_connecting: 'Connexion…', live: 'DIRECT',
       panel_drive: 'Conduite', panel_manual: 'Manuel',
       btn_fwd: 'Avant', btn_left: 'Gauche', btn_right: 'Droite', btn_stop: 'Stop',
       panel_autonomy: 'Autonomie', btn_follow: 'Suivre',
       follow_person: 'Suivre : personne', follow_ball: 'Suivre : balle', follow_dog: 'Suivre : chien',
       follow_cat: 'Suivre : chat', follow_bottle: 'Suivre : bouteille', follow_chair: 'Suivre : chaise',
       follow_status: 'Conduit le robot tout seul pour garder la cible centrée et à distance fixe. Toute entrée manuelle la désarme. Prévoyez de l’espace au sol avant d’armer.',
       panel_classiccv: 'Vision classique', btn_motion: 'Mouvement', btn_line: 'Ligne', btn_chase: 'Poursuite couleur',
       btn_sample: 'Prélever au centre', label_light_line: 'ligne claire',
       cv_status: 'Aucun modèle ni internet — mathematiques de pixels pures, donc ça suit le flux vidéo sur le WiFi propre au robot. Ligne et Poursuite couleur pilotent le robot ; laissez de l’espace au sol, et gardez le curseur Vitesse haut car ils se plafonnent bien en dessous.',
       panel_vision: 'Vision', btn_vision: 'Vision IA',
       vision_status: 'S’exécute dans votre navigateur (TensorFlow.js) — détecte objets (cyan) et visages (ambre). Besoin d’internet sur ce réseau pour charger les modèles une fois.',
       btn_plates: 'Plaques',
       plates_status: 'Lit le texte sur les véhicules détectés (magenta) via OCR. Heuristique, caméra basse résolution (320×240) — fonctionne mieux de près, bien éclairé, et de face.',
       btn_codes: 'QR / Code-barres',
       codes_status: 'Utilise le scanner intégré du navigateur quand disponible — aucun internet nécessaire pour celui-ci. Se rabat sur une bibliothèque QR seule (internet nécessaire une fois) sur les navigateurs sans support natif.',
       btn_pose: 'Posture', pose_status: 'Suivi du squelette (MoveNet) — internet nécessaire une fois pour charger.',
       btn_hands: 'Mains', hands_status: 'Suivi des mains/doigts — internet nécessaire une fois pour charger.',
       btn_expr: 'Expression', expr_status: 'Lit les expressions faciales (heureux/triste/surpris/...) — internet nécessaire une fois pour charger.',
       panel_capture: 'Capture', btn_snapshot: 'Photo', btn_record: 'Enregistrer',
       panel_systems: 'Systèmes', label_speed: 'Vitesse', label_trim: 'Trim', label_lights: 'Lumières',
       label_quality: 'Qualité', label_resolution: 'Résolution', label_orientation: 'Orientation',
       btn_flip: 'Basculer', btn_mirror: 'Miroir', btn_rotate: 'Pivoter',
       panel_offline: 'Hors ligne', offline_checking: 'Vérification du cache…',
       offline_status: 'Les fichiers de modèles sont enregistrés dans ce navigateur au fur et à mesure du téléchargement. Activez une fois les fonctionnalités Vision quelque part avec internet et elles fonctionneront ensuite sur le WiFi propre au robot, qui n’en a pas.',
       btn_clear_cache: 'Vider les modèles en cache',
       panel_firmware: 'Firmware / Réseau', ota_version_loading: 'Vérification de la version actuelle…',
       btn_ota_pick: 'Choisir .bin', btn_ota_upload: 'Envoyer et flasher',
       ota_status_idle: 'Choisissez un firmware.bin compilé pour cette carte, puis Envoyer et flasher. Le robot redémarre automatiquement une fois terminé.',
       ota_status_ready: 'Prêt à envoyer — cela redémarrera le robot une fois terminé.',
       ota_status_uploading: 'Envoi en cours…', ota_status_done: 'Terminé — redémarrage…',
       ota_status_error: 'Échec de l’envoi : ',
       btn_wifi_setup: 'Configuration WiFi…',
       wifi_setup_status: 'Rejoignez un réseau WiFi domestique au lieu du point d’accès propre du robot. Ouvre un réseau séparé « CamRobot-Setup » pour le choisir — le robot redémarre et est brièvement injoignable pendant la configuration.',
       wifi_setup_confirm: 'Ceci redémarre le robot en mode configuration WiFi. Vous devrez rejoindre le réseau « CamRobot-Setup » pour terminer. Continuer ?',
       wifi_setup_started: 'Redémarrage en mode configuration WiFi — rejoignez « CamRobot-Setup » depuis votre liste WiFi dans les 3 minutes.',
       footer_tutorial: 'Guide de montage', footer_source: 'Source sur GitHub'
     },
     ar: {
       conn_connecting: 'جاري الاتصال…', live: 'مباشر',
       panel_drive: 'القيادة', panel_manual: 'يدوي',
       btn_fwd: 'أمام', btn_left: 'يسار', btn_right: 'يمين', btn_stop: 'قف',
       panel_autonomy: 'القيادة الذاتية', btn_follow: 'تتبع',
       follow_person: 'تتبع: شخص', follow_ball: 'تتبع: كرة', follow_dog: 'تتبع: كلب',
       follow_cat: 'تتبع: قط', follow_bottle: 'تتبع: زجاجة', follow_chair: 'تتبع: كرسي',
       follow_status: 'يقود الروبوت ذاتيًا لإبقاء الهدف في المركز وعلى مسافة ثابتة. أي إدخال يدوي يلغيه. وفّر مساحة أرضية واضحة قبل التفعيل.',
       panel_classiccv: 'رؤية حاسوبية كلاسيكية', btn_motion: 'حركة', btn_line: 'خط', btn_chase: 'مطاردة اللون',
       btn_sample: 'أخذ عينة من المركز', label_light_line: 'خط فاتح',
       cv_status: 'لا نماذج ولا إنترنت — حسابات بكسلات خالصة، لذا تواكب بث الفيديو على شبكة WiFi الخاصة بالروبوت. الخط ومطاردة اللون يقودان الروبوت؛ وفّر مساحة أرضية، وابق شريط السرعة مرتفعًا لأنهما يحدان أنفسهما تحته.',
       panel_vision: 'الرؤية', btn_vision: 'رؤية الذكاء الاصطناعي',
       vision_status: 'يعمل في متصفحك (TensorFlow.js) — يكتشف الأجسام (سيان) والوجوه (كهرماني). يحتاج إنترنت على هذه الشبكة لتحميل النماذج مرة واحدة.',
       btn_plates: 'اللوحات',
       plates_status: 'يقرأ النص من المركبات المكتشفة (وردي) عبر OCR. تقريبي، كاميرا منخفضة الدقة (320×240) — يعمل بشكل أفضل عن قرب، بإضاءة جيدة، ومواجهة اللوحة مباشرة.',
       btn_codes: 'QR / باركود',
       codes_status: 'يستخدم الماسح المدمج في المتصفح عند توفره — لا حاجة للإنترنت لهذا. يعود إلى مكتبة QR فقط (تحتاج إنترنت مرة واحدة) على المتصفحات بدون دعم أصلي.',
       btn_pose: 'الوضعية', pose_status: 'تتبع الهيكل العظمي (MoveNet) — يحتاج إنترنت مرة واحدة للتحميل.',
       btn_hands: 'اليدان', hands_status: 'تتبع اليدين/الأصابع — يحتاج إنترنت مرة واحدة للتحميل.',
       btn_expr: 'التعبير', expr_status: 'يقرأ تعابير الوجه (سعيد/حزين/مفاجئ/...) — يحتاج إنترنت مرة واحدة للتحميل.',
       panel_capture: 'التقاط', btn_snapshot: 'لقطة', btn_record: 'تسجيل',
       panel_systems: 'الأنظمة', label_speed: 'السرعة', label_trim: 'الضبط', label_lights: 'الأضواء',
       label_quality: 'الجودة', label_resolution: 'الدقة', label_orientation: 'الاتجاه',
       btn_flip: 'قلب', btn_mirror: 'مرآة', btn_rotate: 'تدوير',
       panel_offline: 'دون اتصال', offline_checking: 'جاري فحص الذاكرة المؤقتة…',
       offline_status: 'تُحفظ ملفات النماذج في هذا المتصفح أثناء تنزيلها. فعّل ميزات الرؤية مرة واحدة في مكان به إنترنت وستعمل بعد ذلك على شبكة WiFi الخاصة بالروبوت، التي لا تملك واحدة.',
       btn_clear_cache: 'مسح النماذج المخزّنة',
       panel_firmware: 'الفيرموير / الشبكة', ota_version_loading: 'جاري التحقق من الإصدار الحالي…',
       btn_ota_pick: 'اختر ملف .bin', btn_ota_upload: 'رفع وفلاش',
       ota_status_idle: 'اختر ملف firmware.bin مبنيًا لهذه اللوحة، ثم اضغط رفع وفلاش. يعيد الروبوت التشغيل تلقائيًا عند الانتهاء.',
       ota_status_ready: 'جاهز للرفع — سيؤدي هذا إلى إعادة تشغيل الروبوت عند الانتهاء.',
       ota_status_uploading: 'جاري الرفع…', ota_status_done: 'تم — جاري إعادة التشغيل…',
       ota_status_error: 'فشل الرفع: ',
       btn_wifi_setup: 'إعداد WiFi…',
       wifi_setup_status: 'انضم إلى شبكة WiFi منزلية بدلًا من نقطة وصول الروبوت الخاصة. يفتح شبكة منفصلة باسم "CamRobot-Setup" للاختيار منها — يعيد الروبوت التشغيل ويصبح غير متاح لفترة وجيزة أثناء الإعداد.',
       wifi_setup_confirm: 'سيؤدي هذا إلى إعادة تشغيل الروبوت في وضع إعداد WiFi. ستحتاج إلى الانضمام إلى شبكة "CamRobot-Setup" لإنهاء الإعداد. المتابعة؟',
       wifi_setup_started: 'جاري إعادة التشغيل إلى وضع إعداد WiFi — انضم إلى "CamRobot-Setup" من قائمة WiFi خلال 3 دقائق.',
       footer_tutorial: 'دليل التركيب', footer_source: 'المصدر على GitHub'
     }
   };
   function tr(key) {
     var lang = document.documentElement.lang || 'en';
     if (I18N[lang] && I18N[lang][key] !== undefined) return I18N[lang][key];
     return I18N.en[key] !== undefined ? I18N.en[key] : key;
   }
   function applyLang(lang) {
     if (!I18N[lang]) lang = 'en';
     document.documentElement.lang = lang;
     document.documentElement.dir = RTL_LANGS[lang] ? 'rtl' : 'ltr';
     document.querySelectorAll('[data-i18n]').forEach(function(el) { el.textContent = tr(el.getAttribute('data-i18n')); });
     document.querySelectorAll('.lang-btn').forEach(function(b) { b.classList.toggle('active', b.dataset.lang === lang); });
     try { localStorage.setItem('camrobot-lang', lang); } catch (e) {}
   }
   function setLang(lang) { applyLang(lang); }
   (function() {
     var saved = 'en';
     try { saved = localStorage.getItem('camrobot-lang') || 'en'; } catch (e) {}
     applyLang(saved);
   })();

   // --- Settings persistence ---
   // Every control (Speed/Trim/Lights/Quality/Resolution/Flip/Mirror) is
   // persisted two ways: on the car itself (NVS flash, via prefs.putInt in
   // cmd_handler -- survives reboots/reflashes, same for any device that
   // connects) and in this browser's localStorage (instant restore on
   // reload without waiting on a request). On load: paint immediately from
   // whatever's cached locally, then fetch /status -- the device's own
   // current values -- and let that win if it differs (e.g. a different
   // phone changed something since this browser last connected).
   var SETTINGS_KEY = 'camrobot-settings-v1';
   var currentSettings = {};

   function loadLocalSettings() {
     try {
       var raw = localStorage.getItem(SETTINGS_KEY);
       return raw ? JSON.parse(raw) : null;
     } catch (e) { return null; }
   }
   function saveLocalSettings() {
     try { localStorage.setItem(SETTINGS_KEY, JSON.stringify(currentSettings)); } catch (e) {}
   }
   function applySettings(vals) {
     if (!vals) return;
     for (var k in vals) { currentSettings[k] = vals[k]; }
     [['speed', 'speed-val'], ['trim', 'trim-val'], ['flash', 'flash-val'],
      ['quality', 'quality-val'], ['framesize', 'framesize-val']].forEach(function (pair) {
       var v = vals[pair[0]];
       if (v === undefined || v === null) return;
       var el = document.getElementById(pair[0]);
       var vEl = document.getElementById(pair[1]);
       if (el) el.value = v;
       if (vEl) vEl.textContent = v;
     });
     if (vals.vflip !== undefined) {
       var vb = document.getElementById('vflip-btn');
       if (vb) vb.classList.toggle('active', !!vals.vflip);
     }
     if (vals.hmirror !== undefined) {
       var hb = document.getElementById('hmirror-btn');
       if (hb) hb.classList.toggle('active', !!vals.hmirror);
     }
     if (vals.rotate !== undefined) applyRotation(vals.rotate);
   }

   // Camera rotation. Display-only -- the sensor cannot rotate, so this is a
   // CSS transform on the viewfinder -- but it is stored on the robot rather
   // than in this browser, because it describes how the camera is bolted to
   // the chassis. That is a property of the robot, not of whoever happens to
   // be looking at it, and every viewer should get it right without setting
   // it themselves.
   window.applyRotation = function (deg) {
     deg = ((parseInt(deg, 10) || 0) % 360 + 360) % 360;
     if (deg !== 90 && deg !== 180 && deg !== 270) deg = 0;
     var vf = document.getElementById('viewfinder');
     if (vf) {
       vf.classList.remove('rot90', 'rot180', 'rot270');
       if (deg) vf.classList.add('rot' + deg);
     }
     var lbl = document.getElementById('rotate-val');
     if (lbl) lbl.textContent = deg + '°';
     var btn = document.getElementById('rotate-btn');
     if (btn) btn.classList.toggle('active', deg !== 0);
     currentSettings.rotate = deg;
     return deg;
   };

   window.cycleRotate = function () {
     var next = ((parseInt(currentSettings.rotate, 10) || 0) + 90) % 360;
     applyRotation(next);
     sendControl('rotate', next);
   };
   function sendControl(varName, val) {
     currentSettings[varName] = val;
     saveLocalSettings();
     fetch(document.location.origin + '/control?var=' + varName + '&val=' + val).catch(function () {});
   }

   applySettings(loadLocalSettings());  // instant paint, may be stale/absent
   fetch(document.location.origin + '/status')
     .then(function (r) { return r.json(); })
     .then(function (vals) {
       applySettings(vals);
       saveLocalSettings();
       // Stamp the footer from the device itself rather than from anything
       // baked into this page, so the number shown is the firmware actually
       // answering, not whatever the browser happens to have cached.
       var el = document.getElementById('fw-build');
       if (el && vals.version) {
         el.textContent = 'firmware v' + vals.version + '  \u00b7  ' + vals.build
                        + '  \u00b7  ' + vals.gitrev;
         if (/-dirty$/.test(vals.gitrev || '')) el.style.color = 'var(--amber)';
       }
       var otaVer = document.getElementById('ota-version');
       if (otaVer && vals.version) {
         otaVer.textContent = 'v' + vals.version + '  \u00b7  ' + vals.build + '  \u00b7  ' + vals.gitrev;
       }
     })
     .catch(function () { /* offline on load -- local cache (if any) stands */ });

  // --- OTA firmware upload + WiFi setup ------------------------------------
  // Companion to app_server.h's /update (raw-body POST, no multipart needed)
  // and /wifi-setup (hands off to WiFiManager's captive portal) endpoints.
  var otaChosenFile = null;
  window.otaFileChosen = function () {
    var input = document.getElementById('ota-file');
    otaChosenFile = (input.files && input.files[0]) || null;
    document.getElementById('ota-upload-btn').disabled = !otaChosenFile;
    document.getElementById('ota-status').textContent = otaChosenFile
      ? (tr('ota_status_ready') + ' (' + otaChosenFile.name + ', ' + Math.round(otaChosenFile.size / 1024) + ' KB)')
      : tr('ota_status_idle');
  };
  window.otaUpload = function () {
    if (!otaChosenFile) return;
    var statusEl = document.getElementById('ota-status');
    var rowEl = document.getElementById('ota-progress-row');
    var barEl = document.getElementById('ota-progress');
    document.getElementById('ota-upload-btn').disabled = true;
    document.getElementById('ota-pick-btn').disabled = true;
    rowEl.style.display = '';
    statusEl.textContent = tr('ota_status_uploading');
    var xhr = new XMLHttpRequest();
    xhr.open('POST', document.location.origin + '/update');
    xhr.upload.onprogress = function (e) {
      if (e.lengthComputable) barEl.value = Math.round((e.loaded / e.total) * 100);
    };
    xhr.onload = function () {
      if (xhr.status >= 200 && xhr.status < 300) {
        barEl.value = 100;
        statusEl.textContent = tr('ota_status_done');
      } else {
        statusEl.textContent = tr('ota_status_error') + xhr.responseText;
        document.getElementById('ota-upload-btn').disabled = false;
        document.getElementById('ota-pick-btn').disabled = false;
      }
    };
    xhr.onerror = function () {
      statusEl.textContent = tr('ota_status_error') + 'network error';
      document.getElementById('ota-upload-btn').disabled = false;
      document.getElementById('ota-pick-btn').disabled = false;
    };
    xhr.send(otaChosenFile);
  };
  window.wifiSetup = function () {
    if (!window.confirm(tr('wifi_setup_confirm'))) return;
    document.getElementById('wifi-setup-status').textContent = tr('wifi_setup_started');
    fetch(document.location.origin + '/wifi-setup').catch(function () {
      // Expected: the car tears its server down mid-response once the portal starts.
    });
  };

  // --- Collapsible panels --------------------------------------------------
  // Collapsed by default so the page opens at roughly one and a half screens
  // instead of five. Which panels are open is remembered in localStorage, per
  // browser, so the set you actually use stays open across reloads.
  //
  // Collapsing only hides; nothing inside is stopped. An armed autonomy mode
  // or a running CV loop keeps going with its panel shut -- the arbiter and
  // the failsafe do not care what the page is displaying.
  (function () {
    var KEY = 'vc.panels.open';
    var open = {};
    try { open = JSON.parse(localStorage.getItem(KEY) || '{}') || {}; } catch (e) { open = {}; }

    function wire(sec) {
      var id = sec.getAttribute('data-panel');
      var label = sec.querySelector('.panel-label');
      if (!id || !label) return;
      if (!open[id]) sec.classList.add('collapsed');
      label.setAttribute('role', 'button');
      label.setAttribute('tabindex', '0');
      function toggle() {
        open[id] = !sec.classList.toggle('collapsed');
        try { localStorage.setItem(KEY, JSON.stringify(open)); } catch (e) {}
      }
      label.addEventListener('click', toggle);
      label.addEventListener('keydown', function (e) {
        // Enter/Space only. Arrow keys are the drive controls and must reach
        // the document handler untouched.
        if (e.key === 'Enter' || e.key === ' ' || e.keyCode === 13 || e.keyCode === 32) {
          e.preventDefault();
          toggle();
        }
      });
    }

    var secs = document.querySelectorAll('.panel.collapsible');
    for (var i = 0; i < secs.length; i++) wire(secs[i]);
  })();

  // --- Offline asset cache ------------------------------------------------
  // The whole Vision suite loads scripts from jsdelivr and model weights from
  // Google's model hosting, and the car's own AP has no route to either. The
  // obvious fix -- a service worker -- is unavailable: service workers and the
  // Cache API are both restricted to secure contexts, and this page is served
  // over plain http from 192.168.4.1. IndexedDB has no such restriction, so
  // that's the store, and window.fetch is wrapped to read through it.
  //
  // It caches on read rather than needing a separate "download" step: use a
  // feature once while you have internet and its files are kept, so the next
  // time you're on the car's AP the same fetches are served locally. The
  // script loaders below were switched from <script src> to fetch() for the
  // same reason -- a src attribute never reaches this shim.
  //
  // Not covered: Tesseract.js (Plates) pulls its worker and language data from
  // inside a Web Worker, which has its own fetch this patch can't see.
  var VCOffline = (function () {
    var DB_NAME = 'camrobot-offline', STORE = 'assets';
    var HOSTS = ['cdn.jsdelivr.net', 'storage.googleapis.com', 'tfhub.dev', 'www.kaggle.com'];
    var ready = null;

    function open() {
      if (ready) return ready;
      ready = new Promise(function (resolve) {
        if (!window.indexedDB) return resolve(null);
        var rq;
        try { rq = indexedDB.open(DB_NAME, 1); } catch (e) { return resolve(null); }
        rq.onupgradeneeded = function () {
          if (!rq.result.objectStoreNames.contains(STORE)) rq.result.createObjectStore(STORE);
        };
        rq.onsuccess = function () { resolve(rq.result); };
        rq.onerror = function () { resolve(null); };
      });
      return ready;
    }

    function store(mode) {
      return open().then(function (db) {
        if (!db) return null;
        try { return db.transaction(STORE, mode).objectStore(STORE); } catch (e) { return null; }
      });
    }

    function get(key) {
      return store('readonly').then(function (s) {
        if (!s) return null;
        return new Promise(function (resolve) {
          var r = s.get(key);
          r.onsuccess = function () { resolve(r.result || null); };
          r.onerror = function () { resolve(null); };
        });
      }).catch(function () { return null; });
    }

    function put(key, rec) {
      return store('readwrite').then(function (s) {
        if (!s) return;
        return new Promise(function (resolve) {
          var r = s.put(rec, key);
          r.onsuccess = resolve;
          // Quota exhaustion is expected on a phone with a full disk; a failed
          // cache write must never break the feature that triggered it.
          r.onerror = function () { resolve(); };
        });
      }).catch(function () {});
    }

    // Exact hostname match, not a substring search: a plain indexOf would also
    // match a URL that merely mentions one of these hosts in its query string.
    function cacheable(url) {
      var h;
      try { h = new URL(url, document.location.href).hostname; } catch (e) { return false; }
      for (var i = 0; i < HOSTS.length; i++) if (h === HOSTS[i]) return true;
      return false;
    }

    function stats() {
      return store('readonly').then(function (s) {
        if (!s) return { n: 0, bytes: 0, ok: false };
        return new Promise(function (resolve) {
          var n = 0, bytes = 0;
          var cur = s.openCursor();
          cur.onsuccess = function () {
            var c = cur.result;
            if (!c) return resolve({ n: n, bytes: bytes, ok: true });
            n++;
            if (c.value && c.value.blob) bytes += c.value.blob.size;
            c.continue();
          };
          cur.onerror = function () { resolve({ n: n, bytes: bytes, ok: true }); };
        });
      }).catch(function () { return { n: 0, bytes: 0, ok: false }; });
    }

    function clear() {
      return store('readwrite').then(function (s) {
        if (!s) return;
        return new Promise(function (resolve) {
          var r = s.clear();
          r.onsuccess = resolve;
          r.onerror = resolve;
        });
      }).catch(function () {});
    }

    var nativeFetch = window.fetch ? window.fetch.bind(window) : null;
    if (nativeFetch && window.indexedDB) {
      window.fetch = function (input, init) {
        var url = (typeof input === 'string') ? input : (input && input.url);
        if (!url || !cacheable(url)) return nativeFetch(input, init);
        return get(url).then(function (rec) {
          if (rec && rec.blob) {
            return new Response(rec.blob, {
              status: 200,
              headers: { 'Content-Type': rec.type || 'application/octet-stream' }
            });
          }
          return nativeFetch(input, init).then(function (r) {
            if (r && r.ok) {
              // Cache a clone so the caller still gets an unread body.
              try {
                r.clone().blob().then(function (b) {
                  put(url, { blob: b, type: r.headers.get('Content-Type') || '', ts: Date.now() });
                }).catch(function () {});
              } catch (e) {}
            }
            return r;
          });
        });
      };
    }

    return { stats: stats, clear: clear, cacheable: cacheable };
  })();

  (function () {
    var statsEl = document.getElementById('offline-stats');
    function human(b) {
      if (b < 1024) return b + ' B';
      if (b < 1048576) return (b / 1024).toFixed(0) + ' KB';
      return (b / 1048576).toFixed(1) + ' MB';
    }
    window.refreshOfflineStats = function () {
      VCOffline.stats().then(function (s) {
        if (!s.ok) { statsEl.textContent = 'Offline cache unavailable in this browser.'; return; }
        statsEl.textContent = s.n
          ? (s.n + ' file' + (s.n === 1 ? '' : 's') + ' cached, ' + human(s.bytes) + ' \u2014 these features now work without internet.')
          : 'Nothing cached yet.';
      });
    };
    window.clearOfflineCache = function () {
      VCOffline.clear().then(function () {
        statsEl.textContent = 'Cache cleared.';
        window.setTimeout(window.refreshOfflineStats, 800);
      });
    };
    window.refreshOfflineStats();
    // Re-read after a model load has had time to populate it.
    window.setInterval(window.refreshOfflineStats, 10000);
  })();

   // Functions to control streaming
  // Stream auto-recovery: an MJPEG <img> stream that drops (WiFi blip, phone
  // sleep, etc.) just goes silent with no event -- it doesn't reliably fire
  // 'error'. So we watch it and force a reconnect if it's ever broken, and
  // also periodically nudge it if the browser marked it errored.
  var source = document.getElementById('stream');
  var streamFails = 0;
  var streamCors = true;
  function startStream() {
    // crossorigin lets the follow-me loop grab frames straight off this <img>
    // with drawImage() instead of polling /capture for a second copy of a
    // frame the camera is already sending. The firmware sets a matching
    // Access-Control-Allow-Origin on the stream.
    //
    // The failure mode matters here: if that header is ever missing, the
    // browser refuses the image outright rather than merely tainting the
    // canvas -- which would kill the video feed, not just follow-me. So after
    // two consecutive load failures we drop crossorigin and reconnect plain,
    // and follow-me falls back to its /capture path. Video always wins.
    if (streamCors) source.setAttribute('crossorigin', 'anonymous');
    else source.removeAttribute('crossorigin');
    source.src = document.location.origin + ':81/stream?_=' + Date.now();
  }
  source.addEventListener('load', function () { streamFails = 0; });
  source.addEventListener('error', function () {
    streamFails++;
    if (streamCors && streamFails >= 2) {
      streamCors = false;
      streamFails = 0;
      window.__followNoCors = true;
    }
    setTimeout(startStream, 1000);
  });
  startStream();

  // --- Shared frame grabber ------------------------------------------------
  // One place that knows how to get a still out of the camera, used by
  // Follow-me and the Classic CV modes. Preferred path is drawImage() straight
  // off the live MJPEG <img>, because that frame is already on the wire and
  // costs the ESP32 nothing extra. It needs the stream's CORS header to avoid
  // tainting the canvas, which is verified once with a 1-pixel read rather than
  // assumed; failing that it polls /capture, which is same-origin and always
  // safe but makes the camera produce a second frame while the stream is
  // already running. That second frame is most of why the older vision loops
  // only manage 0.4-2.5Hz.
  var VCFrame = (function () {
    var useCapture = false, probed = false;
    function grabInto(cv, ctx) {
      if (!useCapture && !window.__followNoCors) {
        var w = source.naturalWidth || 0, h = source.naturalHeight || 0;
        if (!w || !h) return Promise.resolve(null);
        // Only when the size actually changed. Assigning to canvas.width resets
        // the drawing surface even when the value is identical, and Record
        // passes in the canvas that MediaRecorder is capturing from -- resizing
        // it on every grab tore the capture source down about seven times a
        // second for the whole recording.
        if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
        try {
          ctx.drawImage(source, 0, 0, w, h);
          if (!probed) { ctx.getImageData(0, 0, 1, 1); probed = true; }
          return Promise.resolve({ w: w, h: h });
        } catch (e) { useCapture = true; probed = true; }
      } else if (!useCapture) { useCapture = true; }
      return fetch(document.location.origin + '/capture?_=' + Date.now())
        .then(function (r) { return r.blob(); })
        .then(function (b) {
          return new Promise(function (resolve, reject) {
            var img = new Image(), url = URL.createObjectURL(b);
            img.onload = function () {
              if (cv.width !== img.naturalWidth || cv.height !== img.naturalHeight) {
                cv.width = img.naturalWidth; cv.height = img.naturalHeight;
              }
              ctx.drawImage(img, 0, 0);
              URL.revokeObjectURL(url);
              resolve({ w: cv.width, h: cv.height });
            };
            img.onerror = function () { URL.revokeObjectURL(url); reject(new Error('frame decode failed')); };
            img.src = url;
          });
        });
    }
    return { grabInto: grabInto, mode: function () { return useCapture ? 'capture' : 'stream'; } };
  })();

  // Vertical flip / horizontal mirror toggles. Both default to ON (matching
  // the s->set_vflip(s,1) / s->set_hmirror(s,1) the firmware sets at boot,
  // since the camera is mounted upside-down/mirrored on this chassis) --
  // toggle either off here if your image looks wrong without needing to
  // reflash.
  window.toggleFlip = function (varName, btn) {
    var turningOn = !btn.classList.contains('active');
    btn.classList.toggle('active', turningOn);
    sendControl(varName, turningOn ? 1 : 0);
  };

  // --- Snapshot & video recording ---
  // Snapshot: pulls a single fresh JPEG from the existing /capture endpoint and
  // downloads it. Video: since the ESP32 has neither the CPU headroom to encode
  // video nor an SD card wired up in this sketch, recording happens entirely in
  // the browser -- we repeatedly pull frames from /capture, draw them onto a
  // hidden canvas, and use the Canvas+MediaRecorder APIs to encode a .webm file
  // that gets saved locally when you stop. Quality/frame-rate is limited by how
  // fast the ESP32 can serve JPEGs over WiFi, not by the recording code.
  //
  // Both outputs carry the Workshop-DIY mark. It is stamped into the PIXELS
  // rather than overlaid on the page, because the point is that the file still
  // says where it came from once it has left the car -- a CSS overlay would
  // look identical on screen and survive nothing.
  //
  // The artwork is a data: URI. The car's own AP has no route to anywhere the
  // logo could be fetched from, and a watermark that only appears when the
  // workshop happens to have internet is not a watermark. It is stored as
  // grayscale+alpha rather than RGBA: the logo is pure white, so the colour
  // channels carry nothing the alpha channel does not already have, and
  // dropping them saves a third of the bytes.
  var WDIY_MARK = new Image();
  WDIY_MARK.src = 'data:image/png;base64,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';

  // Bottom-right, sized as a FRACTION of the frame rather than in pixels, so
  // it lands the same way whether the camera is set to QVGA or UXGA.
  function stampWatermark(ctx, w, h) {
    if (!WDIY_MARK.complete || !WDIY_MARK.naturalWidth) return;
    var mw = Math.round(w * 0.26);
    var mh = Math.round(mw * WDIY_MARK.naturalHeight / WDIY_MARK.naturalWidth);
    var pad = Math.max(4, Math.round(w * 0.025));
    ctx.save();
    // A soft dark halo behind it. The mark is white, and a workshop camera
    // spends most of its life pointed at a white wall or a bright floor --
    // without this it disappears exactly when it is most needed.
    ctx.shadowColor = 'rgba(0,0,0,0.7)';
    ctx.shadowBlur = Math.max(2, Math.round(w * 0.014));
    // 50%. globalAlpha multiplies the halo set above as well as the mark itself,
    // so the two fade together and the outline stays proportionate.
    ctx.globalAlpha = 0.5;
    ctx.drawImage(WDIY_MARK, w - mw - pad, h - mh - pad, mw, mh);
    ctx.restore();
  }

  (function () {
    var snapshotBtn = document.getElementById('snapshot-btn');
    var recordBtn = document.getElementById('record-btn');
    var statusEl = document.getElementById('record-status');
    var canvas = document.createElement('canvas');
    var ctx = canvas.getContext('2d');
    var recorder = null;
    var chunks = [];
    var pullTimer = null;
    var startTime = 0;
    var tickTimer = null;
    var recording = false;

    function timestamp() {
      var d = new Date();
      function p(n) { return (n < 10 ? '0' : '') + n; }
      return d.getFullYear() + p(d.getMonth() + 1) + p(d.getDate()) + '_' + p(d.getHours()) + p(d.getMinutes()) + p(d.getSeconds());
    }

    function downloadBlob(blob, filename) {
      var url = URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.href = url;
      a.download = filename;
      document.body.appendChild(a);
      a.click();
      a.remove();
      setTimeout(function () { URL.revokeObjectURL(url); }, 2000);
    }

    // Decode to a bitmap however this browser can. createImageBitmap is the
    // fast path and is what the recorder uses; the <img> route is here for
    // Safari versions that lack it, so a snapshot never depends on it.
    function decodeBlob(blob) {
      if (window.createImageBitmap) return createImageBitmap(blob);
      return new Promise(function (resolve, reject) {
        var url = URL.createObjectURL(blob);
        var img = new Image();
        img.onload = function () { URL.revokeObjectURL(url); resolve(img); };
        img.onerror = function () { URL.revokeObjectURL(url); reject(); };
        img.src = url;
      });
    }

    window.takeSnapshot = function () {
      fetch(document.location.origin + '/capture?_=' + Date.now())
        .then(function (r) { return r.blob(); })
        .then(function (blob) {
          return decodeBlob(blob).then(function (bmp) {
            var w = bmp.width || bmp.naturalWidth;
            var h = bmp.height || bmp.naturalHeight;
            var out = document.createElement('canvas');
            out.width = w;
            out.height = h;
            var octx = out.getContext('2d');
            octx.drawImage(bmp, 0, 0);
            stampWatermark(octx, w, h);
            return new Promise(function (resolve) {
              out.toBlob(function (stamped) {
                // toBlob can hand back null under memory pressure. Saving the
                // unmarked original beats saving nothing.
                downloadBlob(stamped || blob, 'camrobot_' + timestamp() + '.jpg');
                resolve();
              }, 'image/jpeg', 0.92);
            });
          }).catch(function () {
            // Could not decode it -- still give them the picture.
            downloadBlob(blob, 'camrobot_' + timestamp() + '.jpg');
          });
        })
        .catch(function () { statusEl.textContent = 'Snapshot failed -- check connection.'; });
    };

    function pickMimeType() {
      var options = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm', 'video/mp4'];
      for (var i = 0; i < options.length; i++) {
        if (window.MediaRecorder && MediaRecorder.isTypeSupported(options[i])) return options[i];
      }
      return null;
    }

    function pullFrame() {
      // Reads the frame off the live MJPEG <img> rather than polling /capture.
      // This runs every 150ms, so on the old path recording alone made the
      // camera produce nearly seven extra frames a second on top of the stream
      // -- the heaviest of the /capture pollers by a wide margin. (Snapshot
      // still uses /capture: a one-shot, full-quality grab is worth the frame.)
      VCFrame.grabInto(canvas, ctx)
        .then(function (dim) {
          if (!dim) return;
          // Every frame, not just the first: captureStream samples this canvas
          // continuously, so a mark drawn once would be overwritten by the
          // next frame that arrived.
          stampWatermark(ctx, canvas.width, canvas.height);
        })
        .catch(function () {});
    }

    window.toggleRecording = function () {
      if (!recording) {
        if (!window.MediaRecorder) {
          statusEl.textContent = "This browser doesn't support video recording. Try Snapshot instead.";
          return;
        }
        var mimeType = pickMimeType();
        if (!mimeType) {
          statusEl.textContent = "No supported video format on this browser. Try Snapshot instead.";
          return;
        }
        chunks = [];
        pullFrame();
        var stream = canvas.captureStream(8);
        try {
          recorder = new MediaRecorder(stream, { mimeType: mimeType });
        } catch (e) {
          statusEl.textContent = 'Could not start recorder: ' + e.message;
          return;
        }
        recorder.ondataavailable = function (e) { if (e.data && e.data.size > 0) chunks.push(e.data); };
        recorder.onstop = function () {
          var blob = new Blob(chunks, { type: mimeType.split(';')[0] });
          var ext = mimeType.indexOf('mp4') >= 0 ? '.mp4' : '.webm';
          downloadBlob(blob, 'camrobot_' + timestamp() + ext);
        };
        recorder.start();
        pullTimer = window.setInterval(pullFrame, 150); // ~6-7 fps, matched to what the ESP32 can realistically serve
        startTime = Date.now();
        tickTimer = window.setInterval(function () {
          var secs = Math.floor((Date.now() - startTime) / 1000);
          statusEl.textContent = 'Recording... ' + secs + 's';
        }, 500);
        recording = true;
        recordBtn.textContent = '\u23F9 Stop';
        recordBtn.classList.add('active');
        statusEl.classList.add('active');
      } else {
        window.clearInterval(pullTimer);
        window.clearInterval(tickTimer);
        if (recorder && recorder.state !== 'inactive') recorder.stop();
        recording = false;
        recordBtn.textContent = '\u23FA Record';
        recordBtn.classList.remove('active');
        statusEl.classList.remove('active');
        statusEl.textContent = 'Saved.';
      }
    };
  })();

  // --- AI Vision: client-side object + face detection ---
  // Two models: COCO-SSD for whole-object categories (person, dog, chair,
  // ...) and BlazeFace specifically for faces/heads, since COCO-SSD has no
  // "head" or "face" class -- its closest category is "person", trained
  // mostly on full-body shots, which handles close-up faces poorly.
  // Runs entirely in the browser via TensorFlow.js + the COCO-SSD model,
  // since the ESP32 itself has nowhere near enough headroom to run a neural
  // net alongside the camera/WiFi/webserver. The model (~a few MB) loads
  // from a CDN the first time you turn this on, so it needs internet access
  // on whatever network the phone/laptop is using -- if the device is only
  // joined to the car's own isolated AP (no internet), this will fail to
  // load, and the status line below says so rather than failing silently.
  // Detection reads frames off the live MJPEG stream via VCFrame (see
  // grabFrame below). That "extra CORS plumbing" the stream's <img> once
  // needed is in place -- Access-Control-Allow-Origin on stream_handler,
  // crossorigin="anonymous" on the <img> -- so the canvas is no longer
  // tainted and there is no reason left to make the camera shoot a second
  // frame for every scan. /capture remains the automatic fallback.
  (function () {
    var visionBtn = document.getElementById('vision-btn');
    var statusEl = document.getElementById('vision-status');
    var platesBtn = document.getElementById('plates-btn');
    var platesStatusEl = document.getElementById('plates-status');
    var codesBtn = document.getElementById('codes-btn');
    var codesStatusEl = document.getElementById('codes-status');
    var poseBtn = document.getElementById('pose-btn');
    var poseStatusEl = document.getElementById('pose-status');
    var handsBtn = document.getElementById('hands-btn');
    var handsStatusEl = document.getElementById('hands-status');
    var exprBtn = document.getElementById('expr-btn');
    var exprStatusEl = document.getElementById('expr-status');
    var overlay = document.getElementById('detect-overlay');
    var octx = overlay.getContext('2d');
    var workCanvas = document.createElement('canvas');
    var wctx = workCanvas.getContext('2d');
    var plateCanvas = document.createElement('canvas');
    var pctx = plateCanvas.getContext('2d');
    var objectModel = null;   // coco-ssd: whole-object categories (person, dog, chair, ...),
                               // also the only signal Plates has for "where's a vehicle"
    var faceModel = null;     // blazeface: purpose-built for faces/heads, which coco-ssd
                               // has no class for and handles poorly at close range
    var running = false;
    var loopTimer = null;
    var scriptsLoaded = false;

    var platesRunning = false;
    var plateLoopTimer = null;
    var ocrWorker = null;
    var ocrBusy = false;

    // Last-known detections from each (possibly independent, possibly
    // differently-timed) source, redrawn together so AI Vision and Plates
    // can run simultaneously without erasing each other's boxes.
    var lastObjects = [], lastFaces = [], lastPlates = [], lastCodes = [];
    var lastFrameW = 320, lastFrameH = 240;

    var codesRunning = false;
    var codeLoopTimer = null;
    var nativeBarcodeSupported = 'BarcodeDetector' in window;
    var barcodeDetector = null;
    var jsQRLoaded = false;

    var poseModel = null, poseScriptsLoaded = false, posesRunning = false, poseLoopTimer = null;
    var handModel = null, handScriptsLoaded = false, handsRunning = false, handLoopTimer = null;
    var faceApiLoaded = false, exprRunning = false, exprLoopTimer = null;
    var lastPoses = [], lastHands = [], lastExpressions = [];

    // Standard COCO/MoveNet 17-keypoint skeleton edges, by keypoint name.
    var POSE_EDGES = [
      ['left_shoulder', 'right_shoulder'], ['left_shoulder', 'left_elbow'], ['left_elbow', 'left_wrist'],
      ['right_shoulder', 'right_elbow'], ['right_elbow', 'right_wrist'],
      ['left_shoulder', 'left_hip'], ['right_shoulder', 'right_hip'], ['left_hip', 'right_hip'],
      ['left_hip', 'left_knee'], ['left_knee', 'left_ankle'], ['right_hip', 'right_knee'], ['right_knee', 'right_ankle']
    ];

    // Fetched rather than appended as <script src>, so it goes through the
    // offline cache shim above -- a <script src> would bypass window.fetch
    // entirely and always hit the network. The code is then injected inline,
    // with a sourceURL so devtools still attributes it to the real file.
    function loadScript(src) {
      return fetch(src)
        .then(function (r) {
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.text();
        })
        .then(function (code) {
          var s = document.createElement('script');
          s.text = code + '\n//# sourceURL=' + src;
          document.head.appendChild(s);
        })
        .catch(function (e) { throw new Error('Failed to load ' + src + ' (' + e.message + ')'); });
    }

    function ensureScripts() {
      if (scriptsLoaded) return Promise.resolve();
      statusEl.textContent = 'Loading AI models (needs internet on this network)...';
      return loadScript('https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@4.20.0/dist/tf.min.js')
        .then(function () {
          return Promise.all([
            loadScript('https://cdn.jsdelivr.net/npm/@tensorflow-models/coco-ssd@2.2.3/dist/coco-ssd.min.js'),
            // NOTE: this package's own package.json "jsdelivr"/"unpkg" field
            // points at "dist/blazeface.min.js", which does NOT exist in the
            // published tarball for 0.1.0 (verified against the actual npm
            // package contents) -- that's why both the bare URL and that
            // path 404. The real browser-global (UMD) build is this file:
            loadScript('https://cdn.jsdelivr.net/npm/@tensorflow-models/blazeface@0.1.0/dist/blazeface.min.umd.js')
          ]);
        })
        .then(function () { scriptsLoaded = true; });
    }

    function ensureObjectModel() {
      if (objectModel) return Promise.resolve();
      return ensureScripts().then(function () { return cocoSsd.load(); }).then(function (m) { objectModel = m; });
    }

    // Tesseract.js is only loaded when Plates is actually turned on, since
    // it's a separate multi-MB download (engine + language data) that most
    // people trying object/face detection don't need. Verified against the
    // real published npm tarball (registry.npmjs.org), not just its
    // package.json metadata -- see the blazeface comment above for why that
    // distinction matters.
    var tesseractLoaded = false;
    function ensureTesseract() {
      if (tesseractLoaded) return Promise.resolve();
      return loadScript('https://cdn.jsdelivr.net/npm/tesseract.js@7.0.0/dist/tesseract.min.js')
        .then(function () { tesseractLoaded = true; });
    }

    function resizeOverlay() {
      var vf = document.getElementById('viewfinder');
      overlay.width = vf.clientWidth;
      overlay.height = vf.clientHeight;
    }

    function drawBox(x, y, w, h, color, label) {
      octx.strokeStyle = color;
      octx.strokeRect(x, y, w, h);
      var textW = octx.measureText(label).width + 8;
      octx.fillStyle = color;
      octx.fillRect(x, Math.max(0, y - 16), textW, 16);
      octx.fillStyle = '#0B0F14';
      octx.fillText(label, x + 4, Math.max(0, y - 15));
    }

    function redrawOverlay() {
      resizeOverlay();
      var scaleX = overlay.width / lastFrameW;
      var scaleY = overlay.height / lastFrameH;
      octx.clearRect(0, 0, overlay.width, overlay.height);
      octx.lineWidth = 2;
      octx.font = '12px ui-monospace, monospace';
      octx.textBaseline = 'top';

      lastObjects.forEach(function (p) {
        var x = p.bbox[0] * scaleX, y = p.bbox[1] * scaleY;
        var w = p.bbox[2] * scaleX, h = p.bbox[3] * scaleY;
        drawBox(x, y, w, h, '#4CE0D2', p.class + ' ' + Math.round(p.score * 100) + '%');
      });

      lastFaces.forEach(function (f) {
        var x = f.topLeft[0] * scaleX, y = f.topLeft[1] * scaleY;
        var w = (f.bottomRight[0] - f.topLeft[0]) * scaleX;
        var h = (f.bottomRight[1] - f.topLeft[1]) * scaleY;
        var prob = Array.isArray(f.probability) ? f.probability[0] : f.probability;
        drawBox(x, y, w, h, '#FFB454', 'face ' + Math.round(prob * 100) + '%');
      });

      lastPlates.forEach(function (pl) {
        drawBox(pl.x * scaleX, pl.y * scaleY, pl.w * scaleX, pl.h * scaleY, '#E85DBF', pl.text);
      });

      lastCodes.forEach(function (c) {
        var label = c.text.length > 28 ? (c.text.slice(0, 28) + '\u2026') : c.text;
        drawBox(c.x * scaleX, c.y * scaleY, c.w * scaleX, c.h * scaleY, '#D4E157', label);
      });

      octx.fillStyle = '#7DD3FC';
      octx.strokeStyle = '#7DD3FC';
      lastPoses.forEach(function (pose) {
        var byName = {};
        pose.keypoints.forEach(function (kp) { byName[kp.name] = kp; });
        POSE_EDGES.forEach(function (edge) {
          var a = byName[edge[0]], b = byName[edge[1]];
          if (a && b && a.score > 0.3 && b.score > 0.3) {
            octx.beginPath();
            octx.moveTo(a.x * scaleX, a.y * scaleY);
            octx.lineTo(b.x * scaleX, b.y * scaleY);
            octx.stroke();
          }
        });
        pose.keypoints.forEach(function (kp) {
          if (kp.score > 0.3) {
            octx.beginPath();
            octx.arc(kp.x * scaleX, kp.y * scaleY, 3, 0, 2 * Math.PI);
            octx.fill();
          }
        });
      });

      lastHands.forEach(function (hand) {
        octx.fillStyle = '#FB923C';
        hand.keypoints.forEach(function (kp) {
          octx.beginPath();
          octx.arc(kp.x * scaleX, kp.y * scaleY, 3, 0, 2 * Math.PI);
          octx.fill();
        });
        var xs = hand.keypoints.map(function (kp) { return kp.x; });
        var ys = hand.keypoints.map(function (kp) { return kp.y; });
        var minX = Math.min.apply(null, xs), maxX = Math.max.apply(null, xs);
        var minY = Math.min.apply(null, ys), maxY = Math.max.apply(null, ys);
        drawBox(minX * scaleX, minY * scaleY, (maxX - minX) * scaleX, (maxY - minY) * scaleY, '#FB923C', hand.handedness);
      });

      lastExpressions.forEach(function (e) {
        var b = e.box;
        drawBox(b.x * scaleX, b.y * scaleY, b.width * scaleX, b.height * scaleY, '#FB7185', e.label);
      });
    }

    // --- One frame, into workCanvas ------------------------------------------
    // Every loop below used to do its own fetch('/capture') on a 400-500ms
    // timer. That made the camera produce a *second* frame and pushed a second
    // JPEG over the same WiFi as the live video -- two extra sensor grabs a
    // second per active feature, and on a no-PSRAM board (fb_count=1,
    // GRAB_WHEN_EMPTY) the two HTTP tasks then serialise on the single frame
    // buffer, so each poll stalls the stream for a whole frame period. That is
    // most of why the video stuttered whenever anything in this panel was on.
    //
    // VCFrame reads the frame straight off the live MJPEG <img> with
    // drawImage(), which costs the ESP32 nothing at all because that frame is
    // already on the wire. It verifies the CORS read once and falls back to
    // /capture by itself if the browser ever refuses, so the old path is still
    // there -- it just isn't the default any more.
    //
    // Resolves false when the stream hasn't produced a frame yet (naturalWidth
    // is still 0), which callers treat as "skip this tick".
    function grabFrame() {
      return VCFrame.grabInto(workCanvas, wctx).then(function (dim) {
        if (!dim) return false;
        lastFrameW = workCanvas.width;
        lastFrameH = workCanvas.height;
        return true;
      });
    }

    function detectOnce() {
      grabFrame()
        .then(function (ok) {
          if (!ok) return null;
          return Promise.all([
            objectModel ? objectModel.detect(workCanvas) : Promise.resolve([]),
            faceModel ? faceModel.estimateFaces(workCanvas, false) : Promise.resolve([])
          ]);
        })
        .then(function (results) {
          if (!results || !running) return;
          lastObjects = results[0];
          lastFaces = results[1];
          redrawOverlay();
          var seen = lastObjects.map(function (p) { return p.class; });
          if (lastFaces.length) seen.push(lastFaces.length === 1 ? 'a face' : (lastFaces.length + ' faces'));
          statusEl.textContent = seen.length ? ('Sees: ' + seen.join(', ')) : 'Watching... nothing recognized yet.';
        })
        .catch(function () { /* transient network hiccup -- next tick retries */ });
    }

    // --- Plates: heuristic crop of a detected vehicle + real OCR ---
    // There's no off-the-shelf, CDN-loadable "license plate detector" model
    // like there is for objects (COCO-SSD) or faces (BlazeFace) -- so this
    // is deliberately a simpler pipeline: reuse the object detector's
    // car/truck/bus boxes (already computed for AI Vision, or computed here
    // on demand if AI Vision isn't also running), crop the lower-center
    // portion of the largest vehicle where a plate typically sits, upscale
    // it (OCR accuracy improves a lot with more pixels), and run real OCR
    // (Tesseract.js) restricted to plate-like characters. This is a
    // heuristic, not a trained plate detector -- expect it to need the
    // vehicle close, square-on, and well-lit, especially given the ESP32's
    // 320x240 source resolution. OCR is also much heavier than object/face
    // detection, so this runs on its own slower ~2.5s cadence rather than
    // the ~500ms AI Vision loop, and only processes one vehicle per cycle.
    function plateScanOnce() {
      if (ocrBusy) return;  // OCR is slow -- never overlap two recognitions
      grabFrame()
        .then(function (ok) {
          if (!ok) return null;
          return objectModel ? objectModel.detect(workCanvas) : [];
        })
        .then(function (objects) {
          if (!objects || !platesRunning) return;
          var vehicles = objects.filter(function (p) {
            return p.class === 'car' || p.class === 'truck' || p.class === 'bus';
          });
          if (!vehicles.length) {
            lastPlates = [];
            redrawOverlay();
            platesStatusEl.textContent = 'No car/truck/bus in view to check for a plate.';
            return;
          }
          // Largest box = closest/most likely to have a legible plate.
          vehicles.sort(function (a, b) { return (b.bbox[2] * b.bbox[3]) - (a.bbox[2] * a.bbox[3]); });
          var v = vehicles[0].bbox;  // [x, y, w, h]
          // Heuristic plate region: bottom-center portion of the vehicle box.
          var px = v[0] + v[2] * 0.20;
          var py = v[1] + v[3] * 0.62;
          var pw = v[2] * 0.60;
          var ph = v[3] * 0.28;
          if (pw < 4 || ph < 4) { platesStatusEl.textContent = 'Vehicle too small/far to check.'; return; }

          var SCALE = 4;  // upscale the crop -- OCR needs more pixels than a 320x240 source gives
          plateCanvas.width = Math.max(1, Math.round(pw * SCALE));
          plateCanvas.height = Math.max(1, Math.round(ph * SCALE));
          pctx.imageSmoothingEnabled = true;
          pctx.drawImage(workCanvas, px, py, pw, ph, 0, 0, plateCanvas.width, plateCanvas.height);

          ocrBusy = true;
          platesStatusEl.textContent = 'Reading plate...';
          return ensureTesseract()
            .then(function () {
              if (!ocrWorker) return Tesseract.createWorker('eng').then(function (w) {
                ocrWorker = w;
                return ocrWorker.setParameters({ tessedit_char_whitelist: 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-' });
              });
            })
            .then(function () { return ocrWorker.recognize(plateCanvas); })
            .then(function (result) {
              var text = (result.data.text || '').replace(/[^A-Z0-9-]/g, '').trim();
              var conf = result.data.confidence || 0;
              if (text.length >= 4 && conf >= 40) {
                lastPlates = [{ x: px, y: py, w: pw, h: ph, text: text + ' ' + Math.round(conf) + '%' }];
                platesStatusEl.textContent = 'Plate: ' + text + ' (' + Math.round(conf) + '% confidence)';
              } else {
                lastPlates = [{ x: px, y: py, w: pw, h: ph, text: 'scanning' }];
                platesStatusEl.textContent = 'Vehicle found, no confident plate read yet (' + Math.round(conf) + '% confidence).';
              }
              redrawOverlay();
            })
            .catch(function (e) {
              platesStatusEl.textContent = "OCR error: " + e.message;
            })
            .then(function () { ocrBusy = false; });
        })
        .catch(function () { /* transient network hiccup -- next tick retries */ });
    }

    // --- QR / Barcode: native browser API first, jsQR fallback ---
    // Unlike everything else in Vision, modern browsers (Chrome/Edge/Android
    // Chrome) have a BUILT-IN barcode/QR scanner (the Shape Detection API's
    // BarcodeDetector) -- no model download, no CDN, no internet needed at
    // all, and it works even while only joined to the car's own isolated
    // AP. Safari/iOS and older browsers don't support it, so this falls
    // back to jsQR (verified against the real published npm tarball, same
    // as blazeface/tesseract above) -- QR codes only in that case, since
    // jsQR doesn't read 1D barcodes (EAN/UPC/Code128/etc).
    function ensureJsQR() {
      if (jsQRLoaded) return Promise.resolve();
      return loadScript('https://cdn.jsdelivr.net/npm/jsqr@1.4.0/dist/jsQR.js').then(function () { jsQRLoaded = true; });
    }

    function codeScanOnce() {
      grabFrame()
        .then(function (ok) {
          if (!ok) return null;

          if (nativeBarcodeSupported) {
            if (!barcodeDetector) barcodeDetector = new BarcodeDetector();
            return barcodeDetector.detect(workCanvas).then(function (results) {
              return results.map(function (r) {
                var bb = r.boundingBox;
                return { x: bb.x, y: bb.y, w: bb.width, h: bb.height, text: r.rawValue, format: r.format };
              });
            });
          }

          var imgData = wctx.getImageData(0, 0, workCanvas.width, workCanvas.height);
          var code = jsQR(imgData.data, imgData.width, imgData.height);
          if (!code) return [];
          var loc = code.location;
          var xs = [loc.topLeftCorner.x, loc.topRightCorner.x, loc.bottomLeftCorner.x, loc.bottomRightCorner.x];
          var ys = [loc.topLeftCorner.y, loc.topRightCorner.y, loc.bottomLeftCorner.y, loc.bottomRightCorner.y];
          var minX = Math.min.apply(null, xs), maxX = Math.max.apply(null, xs);
          var minY = Math.min.apply(null, ys), maxY = Math.max.apply(null, ys);
          return [{ x: minX, y: minY, w: maxX - minX, h: maxY - minY, text: code.data, format: 'qr_code' }];
        })
        .then(function (codes) {
          if (!codes || !codesRunning) return;
          lastCodes = codes;
          redrawOverlay();
          if (codes.length) {
            codesStatusEl.textContent = codes.map(function (c) {
              var t = c.text.length > 40 ? (c.text.slice(0, 40) + '\u2026') : c.text;
              return c.format + ': ' + t;
            }).join(' | ');
          } else {
            codesStatusEl.textContent = nativeBarcodeSupported
              ? 'Watching for QR codes & barcodes...'
              : 'Watching for QR codes... (no native support on this browser, so only QR works, not 1D barcodes)';
          }
        })
        .catch(function () { /* transient network hiccup -- next tick retries */ });
    }

    window.toggleCodes = function () {
      if (!codesRunning) {
        codesBtn.disabled = true;
        codesStatusEl.textContent = nativeBarcodeSupported
          ? 'Starting (native browser support -- no download needed)...'
          : 'Loading QR fallback library (needs internet on this network)...';
        var starter = nativeBarcodeSupported ? Promise.resolve() : ensureJsQR();
        starter
          .then(function () {
            codesRunning = true;
            codesBtn.disabled = false;
            codesBtn.textContent = '\u23F9 Stop QR/Barcode';
            codesBtn.classList.add('active');
            codesStatusEl.textContent = nativeBarcodeSupported
              ? 'Scanning for QR codes & barcodes (native, works offline)...'
              : 'Scanning for QR codes (fallback mode -- 1D barcodes need a browser with native support)...';
            codeLoopTimer = window.setInterval(codeScanOnce, 400);
            codeScanOnce();
          })
          .catch(function (e) {
            codesBtn.disabled = false;
            codesStatusEl.textContent = "Couldn't load the QR library -- " + e.message;
          });
      } else {
        codesRunning = false;
        window.clearInterval(codeLoopTimer);
        lastCodes = [];
        redrawOverlay();
        codesBtn.textContent = '\u25A6 QR / Barcode';
        codesBtn.classList.remove('active');
        codesStatusEl.textContent = 'Stopped.';
      }
    };

    // --- Pose detection (MoveNet) ---
    // @tensorflow-models/pose-detection, MoveNet model, 'tfjs' runtime (not
    // 'mediapipe', which would need a separate @mediapipe/pose script+wasm
    // bundle from yet another host) -- runs entirely on the already-loaded
    // tf.js WebGL backend. Verified the CDN path against the real published
    // npm tarball (same discipline as blazeface/tesseract/jsQR above).
    function ensurePoseScript() {
      if (poseScriptsLoaded) return Promise.resolve();
      poseStatusEl.textContent = 'Loading pose model (needs internet on this network)...';
      return loadScript('https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@4.20.0/dist/tf.min.js')
        .then(function () { return loadScript('https://cdn.jsdelivr.net/npm/@tensorflow-models/pose-detection@2.1.3/dist/pose-detection.min.js'); })
        .then(function () { poseScriptsLoaded = true; });
    }

    function poseScanOnce() {
      grabFrame()
        .then(function (ok) {
          if (!ok) return null;
          return poseModel.estimatePoses(workCanvas);
        })
        .then(function (poses) {
          if (!poses || !posesRunning) return;
          lastPoses = poses;
          redrawOverlay();
          poseStatusEl.textContent = poses.length
            ? (poses.length === 1 ? 'Tracking 1 person.' : ('Tracking ' + poses.length + ' people.'))
            : 'Watching for a person...';
        })
        .catch(function () { /* transient network hiccup -- next tick retries */ });
    }

    window.togglePose = function () {
      if (!posesRunning) {
        poseBtn.disabled = true;
        ensurePoseScript()
          .then(function () {
            if (poseModel) return;
            poseStatusEl.textContent = 'Warming up the pose model...';
            return poseDetection.createDetector(poseDetection.SupportedModels.MoveNet).then(function (m) { poseModel = m; });
          })
          .then(function () {
            posesRunning = true;
            poseBtn.disabled = false;
            poseBtn.textContent = '\u23F9 Stop Pose';
            poseBtn.classList.add('active');
            poseStatusEl.textContent = 'Pose on -- tracking every ~500ms.';
            poseLoopTimer = window.setInterval(poseScanOnce, 500);
            poseScanOnce();
          })
          .catch(function (e) {
            poseBtn.disabled = false;
            poseStatusEl.textContent = "Couldn't load the pose model -- check this network has internet. " + e.message;
          });
      } else {
        posesRunning = false;
        window.clearInterval(poseLoopTimer);
        lastPoses = [];
        redrawOverlay();
        poseBtn.textContent = '\uD83D\uDEB6 Pose';
        poseBtn.classList.remove('active');
        poseStatusEl.textContent = 'Stopped.';
      }
    };

    // --- Hand tracking ---
    // @tensorflow-models/hand-pose-detection, MediaPipeHands model, 'tfjs'
    // runtime for the same reason as pose above (avoids the separate
    // @mediapipe/hands script+wasm bundle). Gives 21 keypoints per hand.
    function ensureHandScript() {
      if (handScriptsLoaded) return Promise.resolve();
      handsStatusEl.textContent = 'Loading hand model (needs internet on this network)...';
      return loadScript('https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@4.20.0/dist/tf.min.js')
        .then(function () { return loadScript('https://cdn.jsdelivr.net/npm/@tensorflow-models/hand-pose-detection@2.0.1/dist/hand-pose-detection.min.js'); })
        .then(function () { handScriptsLoaded = true; });
    }

    function handScanOnce() {
      grabFrame()
        .then(function (ok) {
          if (!ok) return null;
          return handModel.estimateHands(workCanvas);
        })
        .then(function (hands) {
          if (!hands || !handsRunning) return;
          lastHands = hands;
          redrawOverlay();
          handsStatusEl.textContent = hands.length
            ? hands.map(function (h) { return h.handedness; }).join(', ') + ' hand' + (hands.length > 1 ? 's' : '') + ' visible.'
            : 'Watching for hands...';
        })
        .catch(function () { /* transient network hiccup -- next tick retries */ });
    }

    window.toggleHands = function () {
      if (!handsRunning) {
        handsBtn.disabled = true;
        ensureHandScript()
          .then(function () {
            if (handModel) return;
            handsStatusEl.textContent = 'Warming up the hand model...';
            return handPoseDetection.createDetector(handPoseDetection.SupportedModels.MediaPipeHands, { runtime: 'tfjs' }).then(function (m) { handModel = m; });
          })
          .then(function () {
            handsRunning = true;
            handsBtn.disabled = false;
            handsBtn.textContent = '\u23F9 Stop Hands';
            handsBtn.classList.add('active');
            handsStatusEl.textContent = 'Hands on -- tracking every ~500ms.';
            handLoopTimer = window.setInterval(handScanOnce, 500);
            handScanOnce();
          })
          .catch(function (e) {
            handsBtn.disabled = false;
            handsStatusEl.textContent = "Couldn't load the hand model -- check this network has internet. " + e.message;
          });
      } else {
        handsRunning = false;
        window.clearInterval(handLoopTimer);
        lastHands = [];
        redrawOverlay();
        handsBtn.textContent = '\u270B Hands';
        handsBtn.classList.remove('active');
        handsStatusEl.textContent = 'Stopped.';
      }
    };

    // --- Facial expression ---
    // @vladmandic/face-api (actively-maintained fork of the long-unmaintained
    // face-api.js) bundles its own internal tfjs copy, so it doesn't share
    // the tf.js already loaded for the other models -- separate closure,
    // no conflict, just some extra memory. Its model weight files are
    // published inside the SAME npm package as the script (verified against
    // the real tarball), so unlike coco-ssd/blazeface this only depends on
    // ONE host (jsdelivr) rather than two, which is one less thing that can
    // fail independently.
    var FACE_API_BASE = 'https://cdn.jsdelivr.net/npm/@vladmandic/face-api@1.7.15';
    function ensureFaceApi() {
      if (faceApiLoaded) return Promise.resolve();
      exprStatusEl.textContent = 'Loading expression model (needs internet on this network)...';
      return loadScript(FACE_API_BASE + '/dist/face-api.js')
        .then(function () {
          return Promise.all([
            faceapi.nets.tinyFaceDetector.loadFromUri(FACE_API_BASE + '/model'),
            faceapi.nets.faceExpressionNet.loadFromUri(FACE_API_BASE + '/model')
          ]);
        })
        .then(function () { faceApiLoaded = true; });
    }

    function exprScanOnce() {
      grabFrame()
        .then(function (ok) {
          if (!ok) return null;
          return faceapi.detectAllFaces(workCanvas, new faceapi.TinyFaceDetectorOptions()).withFaceExpressions();
        })
        .then(function (detections) {
          if (!detections || !exprRunning) return;
          lastExpressions = detections.map(function (d) {
            var top = 'neutral', topScore = 0;
            for (var k in d.expressions) {
              if (d.expressions[k] > topScore) { topScore = d.expressions[k]; top = k; }
            }
            return { box: d.detection.box, label: top + ' ' + Math.round(topScore * 100) + '%' };
          });
          redrawOverlay();
          exprStatusEl.textContent = lastExpressions.length
            ? lastExpressions.map(function (e) { return e.label; }).join(', ')
            : 'Watching for a face...';
        })
        .catch(function () { /* transient network hiccup -- next tick retries */ });
    }

    window.toggleExpr = function () {
      if (!exprRunning) {
        exprBtn.disabled = true;
        ensureFaceApi()
          .then(function () {
            exprRunning = true;
            exprBtn.disabled = false;
            exprBtn.textContent = '\u23F9 Stop Expression';
            exprBtn.classList.add('active');
            exprStatusEl.textContent = 'Expression on -- checking every ~500ms.';
            exprLoopTimer = window.setInterval(exprScanOnce, 500);
            exprScanOnce();
          })
          .catch(function (e) {
            exprBtn.disabled = false;
            exprStatusEl.textContent = "Couldn't load the expression model -- check this network has internet. " + e.message;
          });
      } else {
        exprRunning = false;
        window.clearInterval(exprLoopTimer);
        lastExpressions = [];
        redrawOverlay();
        exprBtn.textContent = '\uD83D\uDE00 Expression';
        exprBtn.classList.remove('active');
        exprStatusEl.textContent = 'Stopped.';
      }
    };

    window.togglePlates = function () {
      if (!platesRunning) {
        platesBtn.disabled = true;
        platesStatusEl.textContent = 'Loading plate reader (needs internet on this network)...';
        ensureObjectModel()
          .then(function () {
            platesRunning = true;
            platesBtn.disabled = false;
            platesBtn.textContent = '\u23F9 Stop Plates';
            platesBtn.classList.add('active');
            platesStatusEl.textContent = 'Plates on -- checking every ~2.5s.';
            plateLoopTimer = window.setInterval(plateScanOnce, 2500);
            plateScanOnce();
          })
          .catch(function (e) {
            platesBtn.disabled = false;
            platesStatusEl.textContent = "Couldn't load the object model needed to find vehicles -- " + e.message;
          });
      } else {
        platesRunning = false;
        window.clearInterval(plateLoopTimer);
        lastPlates = [];
        redrawOverlay();
        platesBtn.textContent = '\uD83D\uDD22 Plates';
        platesBtn.classList.remove('active');
        platesStatusEl.textContent = 'Stopped.';
      }
    };

    window.toggleVision = function () {
      if (!running) {
        visionBtn.disabled = true;
        var loadErrors = [];
        ensureScripts()
          .then(function () {
            statusEl.textContent = 'Warming up the object model...';
            if (objectModel) return;
            // coco-ssd and blazeface fetch their weight files from Google's
            // model hosting (storage.googleapis.com / tfhub.dev), which is a
            // DIFFERENT host than the jsdelivr CDN the scripts just loaded
            // from -- a network can allow one and block the other, so these
            // are tried and reported independently rather than as one
            // all-or-nothing Promise.all.
            return cocoSsd.load()
              .then(function (m) { objectModel = m; })
              .catch(function (e) { loadErrors.push('object model (storage.googleapis.com): ' + e.message); });
          })
          .then(function () {
            statusEl.textContent = 'Warming up the face model...';
            if (faceModel) return;
            return blazeface.load()
              .then(function (m) { faceModel = m; })
              .catch(function (e) { loadErrors.push('face model (tfhub.dev/storage.googleapis.com): ' + e.message); });
          })
          .then(function () {
            visionBtn.disabled = false;
            if (!objectModel && !faceModel) {
              statusEl.textContent = "Couldn't reach the model files (scripts loaded fine, so it's the model-weight hosts specifically, not jsdelivr) -- " + loadErrors.join('; ');
              return;
            }
            running = true;
            visionBtn.textContent = '\u23F9 Stop Vision';
            visionBtn.classList.add('active');
            var note = loadErrors.length ? (' (' + loadErrors.length + ' model failed to load, running with what did)') : '';
            statusEl.textContent = 'AI Vision on -- looking every ~500ms.' + note;
            loopTimer = window.setInterval(detectOnce, 500);
          })
          .catch(function (e) {
            visionBtn.disabled = false;
            statusEl.textContent = "Couldn't load the AI scripts -- check this network has internet (won't work while only joined to the robot's own WiFi with no internet uplink). " + e.message;
          });
      } else {
        running = false;
        window.clearInterval(loopTimer);
        lastObjects = [];
        lastFaces = [];
        redrawOverlay();
        visionBtn.textContent = '\uD83E\uDD16 AI Vision';
        visionBtn.classList.remove('active');
        statusEl.textContent = 'Stopped.';
      }
    };
  })();

  // Functions for Controls via Keypress
  var keyforward=0;
    var keyleft=0 ;
    var keyright=0;
  // Emulate Keypress with Touch
  var fwdpress = new KeyboardEvent('keydown', {'keyCode':38, 'which':38});
  var fwdrelease = new KeyboardEvent('keyup', {'keyCode':38, 'which':38});
  var leftpress = new KeyboardEvent('keydown', {'keyCode':37, 'which':37});
  var leftrelease = new KeyboardEvent('keyup', {'keyCode':37, 'which':37});
  var rightpress = new KeyboardEvent('keydown', {'keyCode':39, 'which':39});
  var rightrelease = new KeyboardEvent('keyup', {'keyCode':39, 'which':39});
  //Keypress Events
   document.addEventListener('keydown',function(keyon){
    keyon.preventDefault();
      // A human touching the controls always outranks the autonomous loop.
      if (keyon.keyCode == '38' || keyon.keyCode == '40' || keyon.keyCode == '37' || keyon.keyCode == '39') {
        disarmAuto('Manual input \u2014 autonomy disarmed.');
      }
      if ((keyon.keyCode == '38') && (!keyforward)) {keyforward = 1;}
      // Down used to be reverse. With no reverse to give it (SetMotor.h) the
      // key is kept and repurposed as a panic stop rather than left dead --
      // it is the one arrow key whose old meaning nobody can be surprised to
      // find gone, and "the robot is heading for the stairs" wants a key.
      else if (keyon.keyCode == '40') {stopNow(); return;}
      else if ((keyon.keyCode == '37') && (!keyright) && (!keyleft)){keyleft = 1;}
      else if ((keyon.keyCode == '39') && (!keyleft) && (!keyright)){keyright = 1;}
      // Nothing matched: either a key this page doesn't drive with, or an
      // arrow whose flag was already set by the OS repeating the keydown.
      // Either way no command changed, so there is nothing to send -- the
      // keyup handler below guards itself the same way.
      else return;
      // Forced, so a direction change preempts whatever is on the wire rather
      // than queueing behind it. Pressing Left while a Forward request is
      // still in flight is exactly the case that made the robot feel behind
      // the hand: the newest manual command has to win, not wait its turn.
      pushDrive(true);
    });
    //KeyRelease Events
    document.addEventListener('keyup',function(keyoff){
      if (keyoff.keyCode == '38') {keyforward = 0;}
      else if ((keyoff.keyCode == '37') || (keyoff.keyCode == '39')) {keyleft = 0;keyright = 0;}
      else return;
      // Releasing a key used to only clear the flag and leave the stop to the
      // next 100ms tick. Now the release *is* the send -- and it is forced, so
      // it aborts any in-flight movement request instead of waiting behind it.
      // The D-pad buttons dispatch synthetic keyup events, so they inherit
      // this; the joystick release and the panic stop already forced.
      pushDrive(true);
    });

    // Everything that means "stop now, and stay stopped": the D-pad's Stop
    // button and the Down arrow. Clearing the key flags is what makes it
    // stick -- the arbiter below reads them and would otherwise put a held
    // Forward straight back on the wheels. Autonomy is disarmed for the same
    // reason: a Stop that an autonomous mode overrides a tenth of a second
    // later is not a stop.
    window.stopNow = function () {
      keyforward = 0; keyleft = 0; keyright = 0;
      disarmAuto('Stopped.');
      pushDrive(true);   // forced: a stop goes out even if we just sent one
    };
    //Send Commands to Scout
    // ONE arbitrated control sender for every input method. There used to be
    // two independent 100ms timers -- one here and one on the joystick -- and
    // each sent a STOP whenever *its own* input was idle. So dragging the
    // joystick was interrupted by this loop's stop command 10x/sec, and
    // holding the D-pad was interrupted by the joystick loop's x=0&y=0 the
    // same way; both drive modes ran at roughly half duty cycle and felt
    // jerky. That is why arbitration lives in exactly one function and the
    // input handlers only ever set state and call pushDrive(): wiring a
    // sender directly into keyup/pointerup would bring that bug straight back
    // (a key release firing STOP while the joystick is still held).
    //
    // The timer that used to do the sending was also the whole latency story:
    // it sampled every 100ms whether or not anything had changed, fired a
    // fetch() without waiting for the previous one, and kept sending STOP ten
    // times a second forever while nobody was touching anything. A press
    // waited up to 100ms before reaching the wire, and once the ESP32 slowed
    // under MJPEG load the un-awaited requests stacked up, so the car acted on
    // stale steering after the hand had already moved. Now: send on the event,
    // one request in flight at a time, newest state wins, and silence at rest.
    var currentcommand=0;
    // Shared with the joystick block further down (declared here so the
    // arbiter can see them; the joystick block assigns them).
    var joyActive = false;
    var joyX = 0, joyY = 0;  // -100..100, x: + right, y: + forward
    // Autonomous modes (Follow-me, Line, Colour chase) are *sources of
    // setpoints*, never senders: each parks a steer/throttle here and the
    // arbiter forwards it. Giving any of them its own fetch would be exactly
    // the bug described above -- and an autonomous sender racing a manual one
    // is the worst version of it. Exactly one mode holds the arbiter at a
    // time; claiming it disarms whoever had it.
    var autoActive = false, autoName = null;
    var autoX = 0, autoY = 0;
    var autoDisarmers = {};
    function autoRegister(name, fn) { autoDisarmers[name] = fn; }
    function autoClaim(name) {
      for (var k in autoDisarmers) if (k !== name) autoDisarmers[k]('Superseded by ' + name + '.');
      autoActive = true; autoName = name; autoX = 0; autoY = 0;
    }
    function autoRelease(name) {
      if (autoName === name) { autoActive = false; autoName = null; autoX = 0; autoY = 0; }
    }
    function disarmAuto(reason) {
      for (var k in autoDisarmers) autoDisarmers[k](reason);
    }

    // How often an *unchanged* command is repeated while the car is moving.
    // This is the heartbeat that keeps the firmware failsafe
    // (CONTROL_TIMEOUT_MS, 500ms) from auto-stopping mid-drive, so it has to
    // stay comfortably under it -- but it no longer has anything to do with
    // how fast a *new* command reaches the car, which is now immediate.
    var DRIVE_HEARTBEAT_MS = 200;

    // The priority ladder, in one place: a hand on the joystick beats the
    // autonomous loop, which beats the D-pad. Releasing the joystick clears
    // joyActive, which drops us back down -- and the D-pad branch yields
    // command 5 (stop) when no key is held, so the car still stops on release
    // without needing a special case.
    function driveUrl() {
      if (joyActive) return document.location.origin + '/joystick?x=' + joyX + '&y=' + joyY;
      if (autoActive) return document.location.origin + '/joystick?x=' + autoX + '&y=' + autoY;
      // The car's version tested keyforward/keybackward alongside keyleft here
      // and then ignored both, because a spin-in-place turn was the same
      // command either way. Nothing is lost by dropping them: a turn is still
      // a turn, it just swings forward around the undriven wheel now.
      if (keyleft) {currentcommand = 3;} // Turn Left
      else if (keyright) {currentcommand = 4;} // Turn Right
      else if (keyforward) {currentcommand = 1;} //Set Direction Forward
      else {currentcommand = 5;} // Stop
      return document.location.origin+'/control?var=car&val='+currentcommand;
    }

    // Is the car actually being told to move? Decides whether the heartbeat
    // has anything to keep alive -- at a standstill the firmware failsafe
    // only fires while actstate != stp, so there is nothing to feed.
    function driveMoving() {
      if (joyActive) return joyX !== 0 || joyY !== 0;
      if (autoActive) return autoX !== 0 || autoY !== 0;
      return !!(keyforward || keyleft || keyright);
    }

    var driveBusy = false;        // a control request is on the wire right now
    var driveQueued = false;      // input moved while it was
    var driveQueuedForce = false; // ...and at least one of those was a forced send
    var lastDriveUrl = '';        // what we last actually sent
    var lastDriveAt = 0;

    // Hard ceiling on how long a control request is allowed to hold the wire.
    // It has to stay under CONTROL_TIMEOUT_MS (500ms) or a stalled request
    // could keep the heartbeat from being sent until after the firmware has
    // already failsafed. Aborting does not un-send the request -- the ESP32
    // may well have received and acted on it -- it only stops us waiting for
    // a reply that may never come.
    //
    // 400ms, not 250: a release or direction change now aborts the in-flight
    // request the moment it happens, so this no longer has to be tight to keep
    // STOP responsive -- it is only a backstop against a request that has
    // genuinely wedged. Set too low it fires on merely-slow-but-alive
    // responses, and each abort tears down a TCP connection, churning the
    // ESP32's 7-socket pool for nothing.
    var DRIVE_TIMEOUT_MS = 400;
    var driveAbort = null;   // AbortController for the request in flight

    // Every input handler funnels through here. Called with force=true by the
    // panic stop, the joystick release and the heartbeat -- the cases that
    // must reach the car even when the command hasn't changed.
    function pushDrive(force) {
      if (driveBusy) {
        // Never queue the request itself -- queue the *fact* that something
        // changed, and re-read the current state when the wire frees up. That
        // is what stops the car acting on steering the hand has already left.
        driveQueued = true;
        if (force) {
          driveQueuedForce = true;
          // A stop or a release does not wait its turn behind a request that
          // may be stalled. Abort the old one; its rejection re-enters here
          // with the force flag and sends the current state immediately.
          if (driveAbort) driveAbort.abort();
        }
        return;
      }
      var url = driveUrl();
      if (!force && url === lastDriveUrl && (Date.now() - lastDriveAt) < DRIVE_HEARTBEAT_MS) return;

      driveBusy = true;
      var controller = new AbortController();
      driveAbort = controller;
      var timer = window.setTimeout(function () { controller.abort(); }, DRIVE_TIMEOUT_MS);

      // ok is true only when the robot actually answered. Anything else --
      // aborted, network error, or an HTTP status the ESP32 rejected the query
      // with -- leaves lastDriveUrl untouched, so the timer below still sees a
      // difference and tries again. Recording the command as sent before the
      // response came back meant a dropped STOP was never retried: at a
      // standstill driveMoving() is false and driveUrl() already matched
      // lastDriveUrl, so nothing re-sent it and the car coasted until the
      // 500ms watchdog caught it.
      var done = function (ok) {
        window.clearTimeout(timer);
        if (driveAbort === controller) driveAbort = null;
        driveBusy = false;
        if (ok) { lastDriveUrl = url; lastDriveAt = Date.now(); }
        if (driveQueued) {
          var f = driveQueuedForce;
          driveQueued = false; driveQueuedForce = false;
          pushDrive(f);
        }
      };
      // Two callbacks rather than .finally(): this page is ES5 throughout and
      // runs on browsers that have fetch but not Promise.prototype.finally.
      // r.ok matters -- fetch resolves for 4xx/5xx too, and cmd_handler
      // answers a malformed query with 404, which is not a delivered command.
      fetch(url, { cache: 'no-store', signal: controller.signal })
        .then(function (r) { done(!!(r && r.ok)); }, function () { done(false); });
    }

    // What is left of the old 100ms loop. It no longer decides anything and
    // no longer sends at rest: it exists to repeat a held command often enough
    // to feed the failsafe, and to carry the autonomous modes, which park a
    // setpoint in autoX/autoY with no DOM event to hang a send off. A setpoint
    // that changes every tick still goes out at up to 10Hz here (bounded by
    // the in-flight guard, not by a timer); an unchanged one goes out at the
    // heartbeat rate.
    window.setInterval(function(){
      // Two separate reasons to send, and the first one matters most: an
      // autonomous setpoint that just went to zero is a *change* but is not
      // "moving", and gating on movement alone would leave that stop unsent
      // until the 500ms failsafe caught it. So: send whenever the command has
      // changed, and otherwise only keep the heartbeat going while the car is
      // actually moving. At a standstill with nothing changing, this timer
      // sends nothing at all -- which is the whole point.
      if (driveUrl() === lastDriveUrl && !driveMoving()) return;
      pushDrive();
    }, 100);

    // Connection status heartbeat: pings the existing /status endpoint on a
    // timer and flips a dot red/green so you can *see* a dropped connection
    // instead of only discovering it when the car stops responding.
    (function () {
      var dot = document.getElementById('conn-dot');
      var text = document.getElementById('conn-text');
      function setConnected(ok) {
        dot.style.background = ok ? '#2ecc71' : '#e74c3c';
        text.textContent = ok ? 'Connected' : 'Reconnecting\u2026';
      }
      function ping() {
        var controller = new AbortController();
        var timeout = setTimeout(function () { controller.abort(); }, 1500);
        fetch(document.location.origin + '/status', { signal: controller.signal })
          .then(function (r) { clearTimeout(timeout); setConnected(r.ok); })
          .catch(function () { clearTimeout(timeout); setConnected(false); });
      }
      ping();
      window.setInterval(ping, 2000);
    })();

    // Joystick control
    (function () {
      var base = document.getElementById('joystick-base');
      var thumb = document.getElementById('joystick-thumb');
      var radius = 48;      // max thumb travel in px (half of base width minus thumb radius)
      // joyActive / joyX / joyY are declared above, next to the arbiter. This
      // block only sets them and then calls pushDrive() -- it never builds a
      // request of its own, so a joystick release can't stomp on a held key.

      function setThumb(dx, dy) {
        thumb.style.transform = 'translate(' + dx + 'px,' + dy + 'px)';
      }

      function updateFromPoint(clientX, clientY) {
        var rect = base.getBoundingClientRect();
        var cx = rect.left + rect.width / 2;
        var cy = rect.top + rect.height / 2;
        var dx = clientX - cx;
        var dy = clientY - cy;
        var dist = Math.sqrt(dx * dx + dy * dy);
        if (dist > radius) {
          dx = dx * radius / dist;
          dy = dy * radius / dist;
        }
        setThumb(dx, dy);
        joyX = Math.round((dx / radius) * 100);
        joyY = Math.round((-dy / radius) * 100); // screen-down is negative y, so invert for "forward = up"
      }

      function resetThumb() {
        setThumb(0, 0);
        joyX = 0;
        joyY = 0;
      }

      // Ceiling on how often a drag puts a new position on the wire. Grabbing
      // the stick and letting go are discrete events and go out immediately;
      // it is only the continuous sweep in between that is rate-limited. 25Hz
      // is finer than a hand can articulate, and unlike the key and D-pad
      // paths a drag must NOT preempt -- forcing at pointermove's native
      // 60-120Hz would abort a request per frame and churn the ESP32's small
      // socket pool. Skipped moves cost nothing: the state is always current,
      // so the next send carries the newest position and the 100ms timer
      // catches the final one if the finger stops between ticks.
      var JOY_MIN_INTERVAL_MS = 40;
      var lastJoyPush = 0;

      base.addEventListener('pointerdown', function (e) {
        disarmAuto('Joystick taken \u2014 autonomy disarmed.');
        joyActive = true;
        base.setPointerCapture(e.pointerId);
        updateFromPoint(e.clientX, e.clientY);
        lastJoyPush = Date.now();
        pushDrive(true);
      });
      base.addEventListener('pointermove', function (e) {
        if (!joyActive) return;
        updateFromPoint(e.clientX, e.clientY);
        var now = Date.now();
        if (now - lastJoyPush < JOY_MIN_INTERVAL_MS) return;
        lastJoyPush = now;
        // Unforced: pushDrive() coalesces, so whatever the stick reads when
        // the wire frees up is what gets sent, and the positions it swept
        // through in between are dropped rather than queued behind it.
        pushDrive();
      });
      function endDrag() {
        if (!joyActive) return;
        joyActive = false;
        resetThumb();
        // Forced, because the release has to reach the car even if the last
        // thing we sent was already x=0&y=0 from the stick passing through
        // centre a moment ago.
        pushDrive(true);
      }
      base.addEventListener('pointerup', endDrag);
      base.addEventListener('pointercancel', endDrag);
      base.addEventListener('pointerleave', function () { if (joyActive) endDrag(); });
    })();

    // --- Follow-me: closed-loop autonomous following -----------------------
    // The first feature here that *drives* rather than just annotating the
    // picture. COCO-SSD gives a bounding box for the chosen target class; a
    // proportional controller turns that into a steer and a throttle:
    //
    //   steer    <- how far the box centre sits from the frame centre
    //   throttle <- how tall the box is vs. TARGET_FILL (a stand-in for range;
    //               a bigger box means the target is closer)
    //
    // Both land in autoX/autoY, which the single control loop above
    // forwards to /joystick -- the same endpoint manual driving already uses,
    // so the firmware needs no new handler and the 500ms failsafe covers this
    // mode for free.
    //
    // Safety, since this is the one feature that moves the car by itself:
    //   * disarmed by default; any manual input (arrow key, D-pad, joystick)
    //     disarms it immediately, and manual always outranks it in the loop
    //   * output capped at FOLLOW_MAX regardless of the Speed slider, which
    //     the firmware still applies on top as a second ceiling
    //   * it cannot back away from a target that comes too close, because the
    //     chassis has no reverse -- it stops instead and waits
    //   * target lost -> coast to a stop after FOLLOW_LOST_MS, fully disarm
    //     after FOLLOW_GIVEUP_MS, rather than wander off hunting for it
    //   * tab hidden, page hidden, or any error -> disarm
    (function () {
      var followBtn = document.getElementById('follow-btn');
      var statusEl = document.getElementById('follow-status');
      var targetSel = document.getElementById('follow-target');
      var overlay = document.getElementById('follow-overlay');
      var octx = overlay.getContext('2d');
      var vf = document.getElementById('viewfinder');

      // --- tuning knobs -- these are the numbers to play with on real carpet
      var KP_STEER = 1.6;          // gain: horizontal error -> steer
      var KP_THROTTLE = 1.8;       // gain: range error -> throttle
      var TARGET_FILL = 0.55;      // keep the box this tall relative to frame
      var DEADBAND_X = 0.10;       // ignore horizontal error under this
      var DEADBAND_D = 0.12;       // ignore range error under this
      var FOLLOW_MAX = 55;         // max forward magnitude (0..100)
      // Was 30. There is no reverse on this chassis, so backing off when the
      // target gets too close is not available -- the robot can only stop and
      // wait for the gap to open again. Kept as a named zero rather than
      // deleted so the controller below still reads as a two-sided loop, and
      // so a chassis that grows an H-bridge later has one number to change.
      var FOLLOW_MAX_REV = 0;      // no reverse -- see above
      var MIN_SCORE = 0.55;        // ignore weak detections
      var FOLLOW_LOST_MS = 700;    // no target this long -> stop moving
      var FOLLOW_GIVEUP_MS = 3000; // no target this long -> disarm

      var scriptsLoaded = false, model = null;
      var armed = false, busy = false;
      var lastSeen = 0, lastBox = null;
      var frameW = 320, frameH = 240;
      var grab = document.createElement('canvas');
      var gctx = grab.getContext('2d', { willReadFrequently: true });

      function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

      // Fetched, not <script src>, so it goes through the offline cache shim.
      function loadScript(src) {
        return fetch(src)
          .then(function (r) {
            if (!r.ok) throw new Error('HTTP ' + r.status);
            return r.text();
          })
          .then(function (code) {
            var s = document.createElement('script');
            s.text = code + '\n//# sourceURL=' + src;
            document.head.appendChild(s);
          })
          .catch(function (e) { throw new Error('Failed to load ' + src + ' (' + e.message + ')'); });
      }

      // Reuses whatever AI Vision already pulled in, if it ran first.
      function ensureScripts() {
        if (scriptsLoaded) return Promise.resolve();
        if (window.tf && window.cocoSsd) { scriptsLoaded = true; return Promise.resolve(); }
        statusEl.textContent = 'Loading the object model (needs internet on this network)...';
        return Promise.resolve()
          .then(function () { return window.tf ? null : loadScript('https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@4.20.0/dist/tf.min.js'); })
          .then(function () { return window.cocoSsd ? null : loadScript('https://cdn.jsdelivr.net/npm/@tensorflow-models/coco-ssd@2.2.3/dist/coco-ssd.min.js'); })
          .then(function () { scriptsLoaded = true; });
      }

      // Proportional control. Deadbands matter more than the gains here: without
      // them the car hunts left-right forever around the centre because the
      // motors have a stiction floor the controller can't see.
      function control(box) {
        var cx = box[0] + box[2] / 2;
        var errX = (cx - frameW / 2) / (frameW / 2);        // -1 left .. +1 right
        var steer = Math.abs(errX) < DEADBAND_X ? 0 : clamp(errX * KP_STEER, -1, 1);

        var fill = box[3] / frameH;
        var errD = (TARGET_FILL - fill) / TARGET_FILL;      // + = too far away
        var throttle = Math.abs(errD) < DEADBAND_D ? 0 : clamp(errD * KP_THROTTLE, -1, 1);

        autoX = Math.round(steer * FOLLOW_MAX);
        autoY = Math.round(throttle * (throttle >= 0 ? FOLLOW_MAX : FOLLOW_MAX_REV));
      }

      function draw() {
        overlay.width = vf.clientWidth;
        overlay.height = vf.clientHeight;
        octx.clearRect(0, 0, overlay.width, overlay.height);
        if (!armed) return;
        var sx = overlay.width / frameW, sy = overlay.height / frameH;
        var midX = overlay.width / 2;

        // the corridor inside which steer stays at zero
        octx.strokeStyle = 'rgba(74,222,128,0.30)';
        octx.setLineDash([5, 5]);
        octx.lineWidth = 1;
        var half = DEADBAND_X * overlay.width / 2;
        octx.beginPath();
        octx.moveTo(midX - half, 0); octx.lineTo(midX - half, overlay.height);
        octx.moveTo(midX + half, 0); octx.lineTo(midX + half, overlay.height);
        octx.stroke();
        octx.setLineDash([]);

        if (lastBox) {
          var x = lastBox[0] * sx, y = lastBox[1] * sy;
          var w = lastBox[2] * sx, h = lastBox[3] * sy;
          octx.strokeStyle = '#4ADE80';
          octx.lineWidth = 2;
          octx.strokeRect(x, y, w, h);
          octx.beginPath();
          octx.arc(x + w / 2, y + h / 2, 4, 0, Math.PI * 2);
          octx.fillStyle = '#4ADE80';
          octx.fill();
          // line from frame centre to target centre -- the error, drawn
          octx.beginPath();
          octx.moveTo(midX, overlay.height - 8);
          octx.lineTo(x + w / 2, y + h / 2);
          octx.strokeStyle = 'rgba(74,222,128,0.45)';
          octx.lineWidth = 1;
          octx.stroke();
        }

        // steer / throttle readout bars, bottom-left
        var bw = 70, bh = 5, bx = 10, by = overlay.height - 22;
        octx.fillStyle = 'rgba(255,255,255,0.15)';
        octx.fillRect(bx, by, bw, bh);
        octx.fillRect(bx, by + 9, bw, bh);
        octx.fillStyle = '#4ADE80';
        octx.fillRect(bx + bw / 2, by, (autoX / 100) * (bw / 2), bh);
        octx.fillRect(bx + bw / 2, by + 9, (autoY / 100) * (bw / 2), bh);
      }

      function disarm(msg) {
        if (!armed) return;
        armed = false;
        autoRelease('Follow');
        lastBox = null;
        followBtn.textContent = '\uD83C\uDFAF Follow Me';
        followBtn.classList.remove('active');
        targetSel.disabled = false;
        if (msg) statusEl.textContent = msg;
        draw();
      }
      autoRegister('Follow', disarm);

      function tick() {
        if (!armed) return;
        if (busy) { window.setTimeout(tick, 30); return; }
        busy = true;
        VCFrame.grabInto(grab, gctx)
          .then(function (dim) {
            if (!dim) return null;
            frameW = dim.w; frameH = dim.h;
            return model.detect(grab);
          })
          .then(function (preds) {
            if (!armed) return;
            var want = targetSel.value, best = null;
            if (preds) {
              for (var i = 0; i < preds.length; i++) {
                var p = preds[i];
                if (p.class !== want || p.score < MIN_SCORE) continue;
                if (!best || p.bbox[2] * p.bbox[3] > best.bbox[2] * best.bbox[3]) best = p;
              }
            }
            if (best) {
              lastSeen = Date.now();
              lastBox = best.bbox;
              control(best.bbox);
              statusEl.textContent = 'Tracking ' + want + ' ' + Math.round(best.score * 100) + '% \u2014 steer '
                + autoX + ', throttle ' + autoY + (VCFrame.mode() === 'capture' ? ' (/capture fallback)' : '');
            } else {
              var gone = Date.now() - lastSeen;
              lastBox = null;
              if (gone > FOLLOW_LOST_MS) { autoX = 0; autoY = 0; }
              if (gone > FOLLOW_GIVEUP_MS) {
                disarm('Lost the ' + want + ' \u2014 disarmed.');
                return;
              }
              statusEl.textContent = 'Searching for a ' + want + '\u2026';
            }
            draw();
          })
          .catch(function (e) { disarm('Follow stopped after an error: ' + e.message); })
          .then(function () {
            busy = false;
            if (armed) window.setTimeout(tick, 0);
          });
      }

      window.toggleFollow = function () {
        if (armed) { disarm('Stopped.'); return; }
        followBtn.disabled = true;
        ensureScripts()
          .then(function () {
            statusEl.textContent = 'Warming up the object model\u2026';
            if (model) return;
            return cocoSsd.load().then(function (m) { model = m; });
          })
          .then(function () {
            followBtn.disabled = false;
            if (!model) { statusEl.textContent = 'Could not load the object model.'; return; }
            armed = true;
            autoClaim('Follow');
            lastSeen = Date.now();
            targetSel.disabled = true;
            followBtn.textContent = '\u23F9 Stop Following';
            followBtn.classList.add('active');
            statusEl.textContent = 'Armed \u2014 looking for a ' + targetSel.value + '\u2026';
            tick();
          })
          .catch(function (e) {
            followBtn.disabled = false;
            disarm('Could not start: ' + e.message);
          });
      };

      document.addEventListener('visibilitychange', function () {
        if (document.hidden && armed) disarm('Tab hidden \u2014 follow disarmed.');
      });
      window.addEventListener('pagehide', function () { if (armed) disarm(); });
    })();

    // --- Classic CV: no models, no network ---------------------------------
    // Everything above needs a neural net downloaded from the internet, which
    // the car's own AP cannot reach. These three need neither, and they run on
    // raw pixels at full frame rate rather than the 0.4-2.5Hz a model manages
    // on a phone -- which is exactly what a control loop wants.
    //
    //   Motion  passive; frame differencing, draws where something moved
    //   Line    autonomous; follows a dark (or light) line on the floor
    //   Chase   autonomous; drives at the nearest blob of a chosen colour
    //
    // One mode at a time, sharing one overlay. Motion is the exception: it
    // doesn't drive, so it is left running when an autonomous mode elsewhere
    // (Follow-me) claims the arbiter.
    (function () {
      var overlay = document.getElementById('cv-overlay');
      var octx = overlay.getContext('2d');
      var vf = document.getElementById('viewfinder');
      var statusEl = document.getElementById('cv-status');
      var colorIn = document.getElementById('cv-color');
      var invertIn = document.getElementById('cv-invert');
      var btns = {
        motion: document.getElementById('motion-btn'),
        line: document.getElementById('line-btn'),
        chase: document.getElementById('chase-btn')
      };
      var cv = document.createElement('canvas');
      var ctx = cv.getContext('2d', { willReadFrequently: true });

      // --- tuning: these are the numbers to play with on real carpet --------
      var LINE_BAND_TOP = 0.62, LINE_BAND_H = 0.30;  // look at the floor just ahead
      var LINE_MIN_CONTRAST = 18;   // reject a featureless floor rather than chase noise
      var LINE_MIN_PIX = 80;
      // Tuned against a closed-loop simulation rather than guessed. The whole
      // chain -- camera, JPEG, WiFi, browser, HTTP back, I2C -- runs somewhere
      // around 150-350ms, and a proportional loop with that much delay in it
      // goes unstable long before it goes fast. Raising LINE_KP makes it worse,
      // not better: at 250ms, KP 2.5 loses the line where KP 0.9 holds to
      // ~1cm. Speed is the real limiter, so this is deliberately slow.
      // Raise LINE_SPEED once it tracks reliably; that is the knob to move.
      var LINE_KP = 0.9, LINE_SPEED = 26, LINE_TURN_CUT = 0.65, LINE_MAX = 48;
      var CHASE_TOL = 0.10;         // rg-chromaticity radius
      var CHASE_MIN_PIX = 60, CHASE_TARGET_FILL = 0.06;
      var CHASE_KP_STEER = 1.5, CHASE_KP_THROTTLE = 2.2;
      var CHASE_MAX = 50, CHASE_MAX_REV = 0;  // no reverse -- see FOLLOW_MAX_REV
      var MOTION_THR = 25, MOTION_MIN_PIX = 60;
      var LOST_MS = 600, GIVEUP_MS = 2500;

      var mode = null, busy = false, prevBuf = null, lastSeen = 0;
      var W = 320, H = 240, last = null;

      function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }
      function lum(d, i) { return (d[i] * 299 + d[i + 1] * 587 + d[i + 2] * 114) / 1000; }

      // Threshold sits midway between the band's mean and its darkest pixel, so
      // it adapts to whatever contrast the line actually has instead of assuming
      // how dark it is. LINE_MIN_CONTRAST then rejects a blank floor, which
      // would otherwise produce a centroid from sensor noise alone.
      function lineScan(d, w, h, bandTop, bandH, minContrast, minPix, invert) {
        var y0 = Math.max(0, Math.floor(bandTop)), y1 = Math.min(h, y0 + Math.floor(bandH));
        var sum = 0, n = 0, lo = 255, hi = 0, L, x, y;
        for (y = y0; y < y1; y++) for (x = 0; x < w; x++) {
          L = lum(d, (y * w + x) * 4); sum += L; n++;
          if (L < lo) lo = L;
          if (L > hi) hi = L;
        }
        if (!n) return null;
        var mean = sum / n;
        if ((invert ? (hi - mean) : (mean - lo)) < minContrast) return null;
        var thr = invert ? (mean + hi) / 2 : (mean + lo) / 2;
        var sx = 0, cnt = 0, minX = w, maxX = 0;
        for (y = y0; y < y1; y++) for (x = 0; x < w; x++) {
          L = lum(d, (y * w + x) * 4);
          if (invert ? (L > thr) : (L < thr)) {
            sx += x; cnt++;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
          }
        }
        if (cnt < minPix) return null;
        return { x: sx / cnt, count: cnt, minX: minX, maxX: maxX, y0: y0, y1: y1 };
      }

      // rg-chromaticity rather than raw RGB distance, so a shadow falling across
      // the target doesn't lose it -- normalising by total intensity throws away
      // brightness and keeps hue.
      function colorScan(d, w, h, tr, tg, tb, tol, minPix) {
        var ts = tr + tg + tb || 1, tR = tr / ts, tG = tg / ts;
        var sx = 0, sy = 0, cnt = 0, minX = w, maxX = 0, minY = h, maxY = 0;
        for (var y = 0; y < h; y++) for (var x = 0; x < w; x++) {
          var i = (y * w + x) * 4, r = d[i], g = d[i + 1], b = d[i + 2], s = r + g + b;
          if (s < 60) continue;             // too dark for hue to mean anything
          var dR = r / s - tR, dG = g / s - tG;
          if (Math.sqrt(dR * dR + dG * dG) < tol) {
            sx += x; sy += y; cnt++;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
          }
        }
        if (cnt < minPix) return null;
        return { x: sx / cnt, y: sy / cnt, count: cnt, minX: minX, maxX: maxX, minY: minY, maxY: maxY };
      }

      // Frame differencing with the global mean shift removed first. The OV2640
      // runs auto-exposure continuously, so the whole picture brightens and
      // darkens by itself; without this every AEC adjustment reads as full-frame
      // motion and the feature is useless indoors.
      function motionScan(d, prev, w, h, thr, minPix) {
        var np = w * h, cur = new Uint8Array(np), sum = 0, p, L;
        for (p = 0; p < np; p++) { L = lum(d, p * 4) | 0; cur[p] = L; sum += L; }
        if (!prev || prev.length !== np) return { buf: cur, hit: null };
        var sumPrev = 0;
        for (p = 0; p < np; p++) sumPrev += prev[p];
        var delta = (sum - sumPrev) / np;
        var cnt = 0, minX = w, maxX = 0, minY = h, maxY = 0;
        for (var y = 0; y < h; y++) for (var x = 0; x < w; x++) {
          p = y * w + x;
          if (Math.abs(cur[p] - prev[p] - delta) > thr) {
            cnt++;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
          }
        }
        return {
          buf: cur,
          hit: cnt >= minPix ? { count: cnt, minX: minX, maxX: maxX, minY: minY, maxY: maxY, pct: 100 * cnt / np } : null
        };
      }

      function hexToRgb(hex) {
        var m = /^#?([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})$/i.exec(hex || '');
        return m ? [parseInt(m[1], 16), parseInt(m[2], 16), parseInt(m[3], 16)] : [220, 40, 40];
      }

      function draw() {
        overlay.width = vf.clientWidth;
        overlay.height = vf.clientHeight;
        octx.clearRect(0, 0, overlay.width, overlay.height);
        if (!mode || !last) return;
        var sx = overlay.width / W, sy = overlay.height / H;
        var midX = overlay.width / 2;

        if (mode === 'motion' && last.hit) {
          var m = last.hit;
          octx.strokeStyle = '#F472B6';
          octx.lineWidth = 2;
          octx.strokeRect(m.minX * sx, m.minY * sy, (m.maxX - m.minX) * sx, (m.maxY - m.minY) * sy);
          octx.fillStyle = '#F472B6';
          octx.font = '11px monospace';
          octx.fillText('MOTION ' + m.pct.toFixed(1) + '%', m.minX * sx, Math.max(11, m.minY * sy - 4));
        } else if (mode === 'line' && last.hit) {
          var l = last.hit;
          octx.fillStyle = 'rgba(251,191,36,0.14)';
          octx.fillRect(0, l.y0 * sy, overlay.width, (l.y1 - l.y0) * sy);
          octx.strokeStyle = '#FBBF24';
          octx.lineWidth = 2;
          octx.beginPath();
          octx.moveTo(l.x * sx, l.y0 * sy);
          octx.lineTo(l.x * sx, l.y1 * sy);
          octx.stroke();
          octx.setLineDash([4, 4]);
          octx.strokeStyle = 'rgba(255,255,255,0.35)';
          octx.lineWidth = 1;
          octx.beginPath();
          octx.moveTo(midX, l.y0 * sy);
          octx.lineTo(midX, l.y1 * sy);
          octx.stroke();
          octx.setLineDash([]);
        } else if (mode === 'chase' && last.hit) {
          var c = last.hit;
          octx.strokeStyle = '#22D3EE';
          octx.lineWidth = 2;
          octx.strokeRect(c.minX * sx, c.minY * sy, (c.maxX - c.minX) * sx, (c.maxY - c.minY) * sy);
          octx.beginPath();
          octx.arc(c.x * sx, c.y * sy, 4, 0, Math.PI * 2);
          octx.fillStyle = '#22D3EE';
          octx.fill();
        }

        if (mode !== 'motion') {
          var bw = 70, bh = 5, bx = 10, by = overlay.height - 22;
          octx.fillStyle = 'rgba(255,255,255,0.15)';
          octx.fillRect(bx, by, bw, bh);
          octx.fillRect(bx, by + 9, bw, bh);
          octx.fillStyle = mode === 'line' ? '#FBBF24' : '#22D3EE';
          octx.fillRect(bx + bw / 2, by, (autoX / 100) * (bw / 2), bh);
          octx.fillRect(bx + bw / 2, by + 9, (autoY / 100) * (bw / 2), bh);
        }
      }

      function lost(what) {
        var gone = Date.now() - lastSeen;
        if (gone > LOST_MS) { autoX = 0; autoY = 0; }
        if (gone > GIVEUP_MS && mode !== 'motion') {
          stop('Lost the ' + what + ' \u2014 disarmed.');
          return true;
        }
        return false;
      }

      function step() {
        if (!mode) return;
        if (busy) { window.setTimeout(step, 20); return; }
        busy = true;
        VCFrame.grabInto(cv, ctx)
          .then(function (dim) {
            if (!mode) return;
            if (!dim) return;
            W = dim.w; H = dim.h;
            var d = ctx.getImageData(0, 0, W, H).data;

            if (mode === 'motion') {
              var m = motionScan(d, prevBuf, W, H, MOTION_THR, MOTION_MIN_PIX);
              prevBuf = m.buf;
              last = { hit: m.hit };
              statusEl.textContent = m.hit
                ? 'Motion \u2014 ' + m.hit.pct.toFixed(1) + '% of frame changed.'
                : 'Watching \u2014 no motion.';

            } else if (mode === 'line') {
              var l = lineScan(d, W, H, H * LINE_BAND_TOP, H * LINE_BAND_H,
                               LINE_MIN_CONTRAST, LINE_MIN_PIX, invertIn.checked);
              last = { hit: l };
              if (l) {
                lastSeen = Date.now();
                var errL = (l.x - W / 2) / (W / 2);
                var steerL = clamp(errL * LINE_KP, -1, 1);
                autoX = Math.round(steerL * LINE_MAX);
                autoY = Math.round(LINE_SPEED * (1 - LINE_TURN_CUT * Math.abs(steerL)));
                statusEl.textContent = 'Line at ' + errL.toFixed(2) + ' \u2014 steer ' + autoX + ', throttle ' + autoY;
              } else if (!lost('line')) {
                statusEl.textContent = 'Looking for a line\u2026';
              } else { return; }

            } else if (mode === 'chase') {
              var rgb = hexToRgb(colorIn.value);
              var c = colorScan(d, W, H, rgb[0], rgb[1], rgb[2], CHASE_TOL, CHASE_MIN_PIX);
              last = { hit: c };
              if (c) {
                lastSeen = Date.now();
                var errC = (c.x - W / 2) / (W / 2);
                var steerC = clamp(errC * CHASE_KP_STEER, -1, 1);
                var fill = c.count / (W * H);
                var errD = (CHASE_TARGET_FILL - fill) / CHASE_TARGET_FILL;
                var thr2 = clamp(errD * CHASE_KP_THROTTLE, -1, 1);
                autoX = Math.round(steerC * CHASE_MAX);
                autoY = Math.round(thr2 * (thr2 >= 0 ? CHASE_MAX : CHASE_MAX_REV));
                statusEl.textContent = 'Blob ' + (100 * fill).toFixed(1) + '% of frame \u2014 steer '
                  + autoX + ', throttle ' + autoY;
              } else if (!lost('colour')) {
                statusEl.textContent = 'Looking for that colour\u2026';
              } else { return; }
            }
            draw();
          })
          .catch(function (e) { stop('Stopped after an error: ' + e.message); })
          .then(function () {
            busy = false;
            if (mode) window.setTimeout(step, 0);
          });
      }

      function stop(msg) {
        if (!mode) return;
        var was = mode;
        mode = null;
        prevBuf = null;
        last = null;
        if (was === 'line' || was === 'chase') autoRelease(was === 'line' ? 'Line' : 'Chase');
        for (var k in btns) { btns[k].classList.remove('active'); }
        btns.motion.textContent = '\uD83D\uDC40 Motion';
        btns.line.textContent = '\u3030 Line';
        btns.chase.textContent = '\uD83C\uDFA8 Colour chase';
        if (msg) statusEl.textContent = msg;
        draw();
      }

      // Registered so Follow-me can take the arbiter off us. Motion doesn't
      // drive, so it survives -- no reason to stop watching for movement just
      // because something else is steering.
      autoRegister('Line', function (why) { if (mode === 'line') stop(why); });
      autoRegister('Chase', function (why) { if (mode === 'chase') stop(why); });

      window.toggleCV = function (which) {
        if (mode === which) { stop('Stopped.'); return; }
        stop();
        mode = which;
        lastSeen = Date.now();
        prevBuf = null;
        if (which === 'line' || which === 'chase') autoClaim(which === 'line' ? 'Line' : 'Chase');
        btns[which].classList.add('active');
        btns[which].textContent = '\u23F9 Stop';
        statusEl.textContent = 'Starting\u2026';
        step();
      };

      // Easier than eyedroppering a hex value: point the car at the thing and
      // average the middle of the frame.
      window.sampleColour = function () {
        VCFrame.grabInto(cv, ctx).then(function (dim) {
          if (!dim) { statusEl.textContent = 'No frame yet \u2014 is the stream live?'; return; }
          var n = 20;
          var x0 = Math.max(0, (dim.w - n) >> 1), y0 = Math.max(0, (dim.h - n) >> 1);
          var d = ctx.getImageData(x0, y0, n, n).data;
          var r = 0, g = 0, b = 0, c = d.length / 4;
          for (var i = 0; i < d.length; i += 4) { r += d[i]; g += d[i + 1]; b += d[i + 2]; }
          function hx(v) { var s = Math.round(v / c).toString(16); return s.length < 2 ? '0' + s : s; }
          colorIn.value = '#' + hx(r) + hx(g) + hx(b);
          statusEl.textContent = 'Sampled ' + colorIn.value + ' from the centre of frame.';
        }).catch(function (e) { statusEl.textContent = 'Sample failed: ' + e.message; });
      };

      document.addEventListener('visibilitychange', function () {
        if (document.hidden && (mode === 'line' || mode === 'chase')) stop('Tab hidden \u2014 disarmed.');
      });
      window.addEventListener('pagehide', function () { if (mode) stop(); });
    })();
    </script>
    </body>
</html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t *req) {
  // Somebody just opened the control page: the screen's job of advertising
  // the address is done, and the face can have the glass.
  httpd_resp_set_type(req, "text/html");
  // The control page is compiled into the firmware, so a browser holding a
  // cached copy can silently pair an old page with new firmware -- which is
  // exactly the mismatch the build stamp below exists to make visible. Cheaper
  // to prevent it: the page is a few tens of KB over local WiFi, and it is
  // never worth serving a stale one.
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, must-revalidate");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

// Finally, if all is well with the camera, encoding, and all else, here it is, the actual camera server.
// If it works, use your new camera robot to grab a beer from the fridge using function Request.Fridge("beer","buschlite")
// Start an httpd, giving up socket slots rather than giving up entirely.
//
// httpd_start() allocates its socket table and task stack out of internal
// DRAM, which is the scarcest thing on this board -- camera, WiFi, OTA and two
// servers all compete for it, and heap is at its tightest during setup(), right
// after camera init and WiFi association. A single failed attempt used to mean
// the robot came up with no web server at all and no way to ask it why. Trying
// again with a smaller socket pool costs nothing and turns "no server" into "a
// server with fewer simultaneous connections", which is a vastly better robot.
static esp_err_t httpd_start_backoff(httpd_handle_t *out, httpd_config_t *config,
                                     const char *what) {
  const uint16_t fewer[] = { 0, 4, 2 };  // 0 == whatever the caller asked for
  esp_err_t err = ESP_FAIL;
  for (unsigned i = 0; i < sizeof(fewer) / sizeof(fewer[0]); i++) {
    if (fewer[i]) {
      if (config->max_open_sockets <= fewer[i]) continue;  // already at or below
      config->max_open_sockets = fewer[i];
      Serial.printf("  retrying %s with max_open_sockets=%u\n", what, config->max_open_sockets);
    }
    err = httpd_start(out, config);
    if (err == ESP_OK) return ESP_OK;
    *out = NULL;  // httpd_start only writes the handle on success; don't trust it
    Serial.printf("  %s failed: 0x%x (%s)  sockets=%u free=%u largest=%u\n",
                  what, err, esp_err_to_name(err), config->max_open_sockets,
                  ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  }
  return err;
}

// Safe to call more than once: each half is skipped if it is already running,
// so loop() can retry this after a failed boot without leaking a second server.
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  // The control page holds a few keep-alive connections open at once (the
  // drive sender, the /status heartbeat, and the page's own fetches). Without
  // LRU purging, once max_open_sockets fills up the server simply refuses new
  // connections and the page appears to hang. Purging the least-recently-used
  // socket instead keeps it responsive. Set before both httpd_start() calls so
  // the stream server on port 81 inherits it too.
  config.lru_purge_enable = true;

  // 5, not the default 7. This used to need the headroom because the page fired
  // a control request every 100ms without waiting for the previous one, so
  // several were in flight at once. The drive sender now allows exactly one
  // request on the wire at a time, so the realistic peak is the page load, the
  // 2s /status ping and one control request -- and every socket slot given back
  // is internal DRAM the second server can have instead.
  config.max_open_sockets = 5;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
  };

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
  };

  httpd_uri_t joystick_uri = {
    .uri = "/joystick",
    .method = HTTP_GET,
    .handler = joystick_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  httpd_uri_t wifisetup_uri = {
    .uri = "/wifi-setup",
    .method = HTTP_GET,
    .handler = wifisetup_handler,
    .user_ctx = NULL
  };

  httpd_uri_t update_uri = {
    .uri = "/update",
    .method = HTTP_POST,
    .handler = update_handler,
    .user_ctx = NULL
  };

  // --- Say what actually happened -----------------------------------------
  // Both httpd_start() calls used to be bare `if (... == ESP_OK)` with no else,
  // so a server that failed to start left no trace at all: the robot came up,
  // the AP worked, you could associate to it, and then nothing answered on
  // port 80 with nothing on serial to say why. That failure is silent in the
  // one place it most needs not to be, because from the outside "AP up, page
  // dead" looks identical whether WiFi, port 80, port 81 or URI registration
  // was the thing that broke.
  //
  // The heap numbers are printed either side of each server because the most
  // likely cause of httpd_start() failing here is internal RAM: PSRAM does not
  // help, the httpd task stacks and socket structures all come out of internal
  // DRAM, and this firmware is already running the camera, WiFi, OTA and two
  // servers out of it. `largest` matters as much as `free` -- allocation fails
  // on the biggest contiguous block, not the total.
  Serial.printf("Heap before HTTP servers: free=%u largest=%u\n",
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  if (!camera_httpd) {
    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    esp_err_t herr = httpd_start_backoff(&camera_httpd, &config, "httpd_start(port 80)");
    if (herr != ESP_OK) {
      // Not fatal any more, just useless for now: loop() retries this every
      // few seconds, and heap after boot has settled is usually kinder than
      // heap during it. ArduinoOTA is independent of the httpd servers, so a
      // robot stuck here is still reflashable over WiFi.
      Serial.printf("ERROR: control server could not start: 0x%x (%s)\n",
                    herr, esp_err_to_name(herr));
      Serial.println("No control page yet. loop() will keep retrying.");
      return;
    }
    Serial.printf("HTTP control server STARTED on port 80 (max_open_sockets=%u)\n",
                  config.max_open_sockets);
    struct { const char *name; const httpd_uri_t *uri; } handlers[] = {
      { "/", &index_uri },           { "/control", &cmd_uri },
      { "/status", &status_uri },    { "/capture", &capture_uri },
      { "/joystick", &joystick_uri },{ "/wifi-setup", &wifisetup_uri },
      { "/update", &update_uri },
    };
    for (unsigned i = 0; i < sizeof(handlers) / sizeof(handlers[0]); i++) {
      esp_err_t reg = httpd_register_uri_handler(camera_httpd, handlers[i].uri);
      // max_uri_handlers is a fixed-size table; overflowing it fails quietly
      // and the endpoint simply 404s forever, a miserable thing to debug.
      if (reg != ESP_OK) {
        Serial.printf("ERROR: registering %s failed: 0x%x (%s)\n",
                      handlers[i].name, reg, esp_err_to_name(reg));
      }
    }
    Serial.printf("Heap after control server: free=%u largest=%u\n",
                  ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  // Pin the stream to core 1, and only the stream. Left on tskNO_AFFINITY both
  // servers can land on core 0 next to the WiFi stack, where the MJPEG loop --
  // the heaviest thing running -- competes with the radio and with the control
  // requests it is meant to be leaving room for. The control server stays
  // unpinned on purpose: drive commands are short and want whichever core is
  // free, not to queue behind a frame.
  config.core_id = 1;
  // The stream server serves exactly one URI to, realistically, one viewer.
  // Inheriting the control server's full socket pool and 8-entry handler table
  // costs internal RAM for nothing -- and this is the second httpd_start(), so
  // it is the one that fails first when internal RAM runs short. 3 sockets
  // rather than 2: the page's stream auto-recovery opens a new connection
  // before the browser has necessarily torn the old one down, and a reconnect
  // refused because the pool was exactly full is a dead video feed that only
  // a page reload clears.
  config.max_open_sockets = 3;
  config.max_uri_handlers = 2;
  if (!stream_httpd) {
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    esp_err_t serr = httpd_start_backoff(&stream_httpd, &config, "httpd_start(port 81)");
    if (serr != ESP_OK) {
      // Survivable, unlike the control server: the page still loads and the
      // robot still drives, you just get no video. Worth saying precisely,
      // because "page loads, image never appears" is exactly this and nothing
      // else. loop() retries this one too.
      Serial.printf("ERROR: stream server could not start: 0x%x (%s)\n",
                    serr, esp_err_to_name(serr));
      Serial.println("Control page will load but the video will stay black.");
    } else {
      Serial.printf("MJPEG stream server STARTED on port 81 (max_open_sockets=%u)\n",
                    config.max_open_sockets);
      esp_err_t reg = httpd_register_uri_handler(stream_httpd, &stream_uri);
      if (reg != ESP_OK) {
        Serial.printf("ERROR: registering /stream failed: 0x%x (%s)\n",
                      reg, esp_err_to_name(reg));
      }
    }
  }
  Serial.printf("Heap after stream server: free=%u largest=%u\n",
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());
}

#endif
