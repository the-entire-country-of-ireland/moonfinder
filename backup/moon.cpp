#include <WiFi.h>
#include <time.h>
#include <SiderealPlanets.h>

const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";

// Baltimore, MD
constexpr double LAT_DEG = 39.2904;
constexpr double LON_DEG = -76.6122;   // west is negative
constexpr double ELEV_M  = 10.0;

// NTP config.
// We use UTC/GMT for SiderealPlanets, so no local timezone offset here.
constexpr long GMT_OFFSET_SEC = 0;
constexpr int  DST_OFFSET_SEC = 0;

SiderealPlanets astro;

struct Vec3 {
  double n;
  double e;
  double u;
};

static double deg2rad(double x) {
  return x * M_PI / 180.0;
}

Vec3 altAzDegToNEU(double altDeg, double azDeg) {
  // SiderealPlanets / standard astronomy convention:
  // az = 0 deg north, 90 deg east.
  double alt = deg2rad(altDeg);
  double az  = deg2rad(azDeg);

  Vec3 v;
  v.n = cos(alt) * cos(az);
  v.e = cos(alt) * sin(az);
  v.u = sin(alt);
  return v;
}

bool updateMoonNEU(Vec3& moonNEU, double& altDeg, double& azDeg) {
  struct tm utc;
  if (!getLocalTime(&utc, 5000)) {
    Serial.println("Failed to get NTP time.");
    return false;
  }

  int year  = utc.tm_year + 1900;
  int month = utc.tm_mon + 1;
  int day   = utc.tm_mday;
  int hour  = utc.tm_hour;
  int min   = utc.tm_min;
  int sec   = utc.tm_sec;

  astro.setTimeZone(0);
  astro.rejectDST();

  astro.setLatLong(LAT_DEG, LON_DEG);
  astro.setElevationM(ELEV_M);

  astro.setGMTdate(year, month, day);
  astro.setGMTtime(hour, min, sec);

  // 1. Compute geocentric Moon RA/Dec.
  if (!astro.doMoon()) {
    Serial.println("doMoon() failed.");
    return false;
  }

  // 2. Correct Moon RA/Dec for observer location.
  // Important for the Moon because lunar parallax is large enough to matter.
  if (!astro.doLunarParallax()) {
    Serial.println("doLunarParallax() failed.");
    return false;
  }

  // 3. Convert the current RA/Dec to local altitude/azimuth.
  if (!astro.doRAdec2AltAz()) {
    Serial.println("doRAdec2AltAz() failed.");
    return false;
  }

  altDeg = astro.getAltitude();
  azDeg  = astro.getAzimuth();

  moonNEU = altAzDegToNEU(altDeg, azDeg);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  astro.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");

  configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, "pool.ntp.org", "time.nist.gov");

  Vec3 moon;
  double altDeg, azDeg;

  if (updateMoonNEU(moon, altDeg, azDeg)) {
    Serial.println();
    Serial.println("Moon orientation in local Baltimore frame:");
    Serial.print("Altitude deg: "); Serial.println(altDeg, 6);
    Serial.print("Azimuth  deg: "); Serial.println(azDeg, 6);

    Serial.println();
    Serial.println("Moon NEU unit vector:");
    Serial.print("N: "); Serial.println(moon.n, 9);
    Serial.print("E: "); Serial.println(moon.e, 9);
    Serial.print("U: "); Serial.println(moon.u, 9);
  }
}

void loop() {
  static uint32_t last = 0;

  if (millis() - last > 10000) {
    last = millis();

    Vec3 moon;
    double altDeg, azDeg;

    if (updateMoonNEU(moon, altDeg, azDeg)) {
      Serial.print("alt=");
      Serial.print(altDeg, 4);
      Serial.print(" az=");
      Serial.print(azDeg, 4);
      Serial.print(" NEU=(");
      Serial.print(moon.n, 6);
      Serial.print(", ");
      Serial.print(moon.e, 6);
      Serial.print(", ");
      Serial.print(moon.u, 6);
      Serial.println(")");
    }
  }
}