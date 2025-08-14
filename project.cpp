#include <WiFi.h> 
#include <WebServer.h> 
#include <EEPROM.h> 
#include <FirebaseESP32.h> 
#include <WiFiUdp.h> 
#include <NTPClient.h> 
#include <esp_system.h>
#include <HTTPClient.h>

// ---------------------------------------------------------- 
// GLOBAL CONSTANTS AND FLAGS 
// ---------------------------------------------------------- 
#define EEPROM_SIZE 256 
#define RESTART_FLAG_ADDR (EEPROM_SIZE - 10) 
#define MODE_FLAG_ADDR (EEPROM_SIZE - 11) 
#define RESTART_MAGIC_VAL 0xAA 
#define OFFLINE_MODE_VAL 0x01 
#define ONLINE_MODE_VAL 0x02
// Fixed 8-digit hexadecimal Device ID
#define DEVICE_ID "AB!CD@EF"

// ─── Reliable Internet reachability test ───
bool internetAvailable() {
  HTTPClient http;
  http.setConnectTimeout(5000);  // 2 seconds max
  http.begin("http://clients3.google.com/generate_204");
  int httpCode = http.GET();
  http.end();
  return (httpCode == 204);
}

// Define the config structure (added userId)
struct DeviceConfig { 
  char ssid[33]; 
  char password[65]; 
  char userId[33]; // Added to store Firebase userId (UID is typically 28-32 chars)
  bool valid; 
}; 
// Create the global instance of the config structure
DeviceConfig storedConfig; 

// ---------------------------------------------------------- 
// AP MODE VARIABLES 
// ---------------------------------------------------------- 
const char* apSSID = "Demetronics_Config"; 
const char* apPassword = "12345678"; 
WebServer server(80); 
bool apMode = false; 
const int enButtonPin = 0;  // e.g. GPIO0 

// ---------------------------------------------------------- 
// OFFLINE MODE VARIABLES 
// ---------------------------------------------------------- 
bool offlineMode = false; 
// ─── Wi-Fi PROVISIONING & RECONNECT LOGIC ───────────────
const int EEPROM_PROV_ADDR         = 0;            // EEPROM byte for “provisioned” flag
const unsigned long WIFI_RETRY_INTERVAL = 60UL*1000UL; // 60 s between reconnect attempts
unsigned long lastWifiAttempt = 0;
bool provisioned     = false;  // set true if creds valid in EEPROM
bool wifiWasConnected = false;
unsigned long lastWiFiCheckTime = 0; 
const unsigned long WIFI_CHECK_INTERVAL = 60000; // Check WiFi every 60 seconds 
unsigned long offlineModeStartTime = 0; 
unsigned long lastOfflineStatusUpdate = 0; 
const unsigned long OFFLINE_STATUS_UPDATE_INTERVAL = 10000; // Update status every 10 seconds 
// Restart mode flags 
bool skipApModeOnBoot = false;    // Flag set during soft restart 
bool forceOfflineModeAfterBoot = false;  // Used to force offline mode after restart 
bool motorStatusPendingUpdate = false;
String pendingMotorStatus = "";
// ---------------------------------------------------------- 
// FIREBASE VARIABLES 
// ---------------------------------------------------------- 
#define FIREBASE_HOST "demetronics-652b6-default-rtdb.firebaseio.com" 
#define FIREBASE_AUTH "AIzaSyCEm5EMlqd4MSxvCZRRcugEUDwDRZ-LFaU" 
FirebaseData firebaseData; 
FirebaseAuth firebaseAuth; 
FirebaseConfig firebaseConfig; 

// ---------------------------------------------------------- 
// PIN DEFINITIONS 
// ---------------------------------------------------------- 
const int L1 = 34; 
const int L2 = 35; 
const int L3 = 14; 
const int L4 = 12; 
const int L5 = 27; 
const int L6 = 26; 
const int LT = 4;   // Lower tank sensor 
const int RL = 33;  // Relay 
const int Error = 2; 
const int led = 15; 
const int buzzer = 25; 
const int p = 13;  // pushbutton 

// ---------------------------------------------------------- 
// GLOBAL LOGIC VARIABLES 
// ---------------------------------------------------------- 
unsigned long Relaytime = 0; 
unsigned long pb = 0; 
unsigned long diff = 0; 
bool RLmonitering = false; 
String LEVEL; 
String Lowsta;  // "Water Available" or "Water Unavailable" 
bool firebaseControl = false; 
String ssid = ""; 
String password = ""; 
// Offline mode tank threshold variables 
bool tankLevelBasedControl = true;  // Use tank level control in offline mode 
int tankEmptyThreshold = 0;        
// Turn ON when tank level is below this percentage 
int tankFullThreshold = 100;         
// Turn OFF when tank level is above this percentage 

// ---------------------------------------------------------- 
// NTP & TIME VARIABLES 
// ---------------------------------------------------------- 
WiFiUDP ntpUDP; 
NTPClient timeClient(ntpUDP, "pool.ntp.org", 18000, 60000);
const unsigned long NTP_FULL_INTERVAL = 60UL * 60UL * 1000UL;   // 1 hr
const unsigned long NTP_RETRY_DELAY   = 60UL * 1000UL;          // 1 min
const uint8_t       NTP_MAX_RETRIES  = 5;
unsigned long lastNtpAttempt = 0;
uint8_t     ntpRetryCount   = 0;
unsigned long lastSyncEpoch    = 0;     // seconds since 1970 of last good NTP
unsigned long lastSyncMillis    = 0;     // millis() when that happened
// We only check the schedule once per new minute 
static int lastCheckedMinute = -1; 
static int lastCheckedDay = -1; 
// For Pakistan (UTC+5 => 5*3600=18000) 
//const long GMT_PLUS_5 = 18000L; 
// We'll store scheduled times in minutes 
static int scheduledOnMinutes = -1; 
static int scheduledOffMinutes = -1; 
bool inScheduledWindow = false; 
// Flag to indicate if the OFF condition is tank-based 
bool scheduledOffTank = false; 

// ---------------------------------------------------------- 
// BUZZER/ALARM VARIABLES 
// ---------------------------------------------------------- 
// Buzzer if <10% for >=1 min 
bool waitingForLevel10 = false; 
unsigned long zeroStart = 0; 
bool buzzerActivated = false; 

// ---------------------------------------------------------- 
// SCHEDULE VARIABLES 
// ---------------------------------------------------------- 
// If the motor was turned off due to scheduled off time 
bool motorTurnedOffDueToSchedule = false; 
unsigned long motorOffTime = 0;  // Time motor was turned off 
// Tank-level stop logic 
static bool tankStopUsed = false; 
static int lastDayLocalUsed = -1; 
// New flag to prevent motor from turning back on after being turned off due to full tank 
bool motorTurnedOffDueToFullTank = false;  // New flag 
bool tankReachedFullDuringSchedule = false; 
static String lastScheduleSignature = ""; 

// ---------------------------------------------------------- 
// HEARTBEAT Variables 
// ---------------------------------------------------------- 
// Send "POWER: ON" every X ms if connected 
static unsigned long lastHeartbeat = 0; 
static const unsigned long HEARTBEAT_INTERVAL = 10000;  // 10 seconds 
// If Wi-Fi is lost more than 30s, then next time we reconnect, set POWER=OFF once 
static unsigned long lastWiFiOK = 0; 
static const unsigned long MAX_OFFLINE_TIME = 30000;  // 30 seconds 
static bool powerOffPending = false;                  
// if we were offline too long, set to true

// ---------------------------------------------------------- 
// FIREBASE MONITORING Variables 
// ---------------------------------------------------------- 
// Monitor ALARM_STATUS and ERROR_RESET nodes every 15 seconds
static unsigned long lastFirebaseMonitor = 0; 
static const unsigned long FIREBASE_MONITOR_INTERVAL = 15000;  // 15 seconds 
static String lastAlarmStatus = ""; 
static String lastErrorReset = "";

// Monitor DELETE node every 10 seconds
static unsigned long lastDeleteMonitor = 0;
static const unsigned long DELETE_MONITOR_INTERVAL = 10000; // 10 seconds
static String lastDeleteStatus = "";

// ---------------------------------------------------------- 
// FUNCTION DECLARATIONS 
// ---------------------------------------------------------- 
void handleCredentials(); 
void ResetAlarm();
void ErrorReset(); 
void DeleteDevice(); 
void enterAccessPointMode(); 
void checkScheduledTasks(); 
bool parseTimeString(const String& timeStr, int& hourOut, int& minOut); 
void handleOfflineControl(); 
void checkAndReconnectWiFi(); 
void checkFirebaseResetNodes();
void checkFirebaseDeleteNode();
String getDayOfWeekName(int dayNum); 

// ---------------------------------------------------------- 
// HELPER: MAP day number -> name 
// ---------------------------------------------------------- 
String getDayOfWeekName(int dayNum) { 
  switch (dayNum) { 
    case 0: return "Sunday"; 
    case 1: return "Monday"; 
    case 2: return "Tuesday"; 
    case 3: return "Wednesday"; 
    case 4: return "Thursday"; 
    case 5: return "Friday"; 
    case 6: return "Saturday"; 
  } 
  return "Unknown"; 
} 
 
// ---------------------------------------------------------- 
// SETUP 
// ---------------------------------------------------------- 
void setup() { 
  Serial.begin(115200); 
  EEPROM.begin(EEPROM_SIZE); 
   
  // Check if we're recovering from a button-initiated restart 
  byte restartFlag = 0; 
  byte modeFlag = 0; 
  EEPROM.get(RESTART_FLAG_ADDR, restartFlag); 
  EEPROM.get(MODE_FLAG_ADDR, modeFlag); 
   
  if (restartFlag == RESTART_MAGIC_VAL) { 
    Serial.println("*** DETECTED BUTTON-INITIATED RESTART ***"); 
    skipApModeOnBoot = true;  
    // Only force offline mode if the device was in offline mode before restart 
    if (modeFlag == OFFLINE_MODE_VAL) { 
      Serial.println("Device was in OFFLINE mode before restart - continuing in offline mode"); 
      forceOfflineModeAfterBoot = true; 
    } else { 
      Serial.println("Device was in ONLINE mode before restart - will attempt normal WiFi connection"); 
      forceOfflineModeAfterBoot = false; 
    } 
    // Clear the flags immediately 
    restartFlag = 0; 
    EEPROM.put(RESTART_FLAG_ADDR, restartFlag); 
    EEPROM.put(MODE_FLAG_ADDR, 0); 
    EEPROM.commit(); 
     
    Serial.println("Restart flags cleared"); 
  } else { 
    skipApModeOnBoot = false; 
    forceOfflineModeAfterBoot = false; 
  }
  // Read stored config 
  EEPROM.get(0, storedConfig); 
  Serial.print("Stored config valid: "); 
  Serial.println(storedConfig.valid ? "YES" : "NO"); 
   
  // Pin setup 
  pinMode(enButtonPin, INPUT_PULLUP); 
  pinMode(p, INPUT_PULLUP); 
  pinMode(L1, INPUT_PULLDOWN); 
  pinMode(L2, INPUT_PULLDOWN); 
  pinMode(L3, INPUT_PULLDOWN); 
  pinMode(L4, INPUT_PULLDOWN); 
  pinMode(L5, INPUT_PULLDOWN); 
  pinMode(L6, INPUT_PULLDOWN); 
  pinMode(LT, INPUT_PULLDOWN); 
  pinMode(RL, OUTPUT); 
  pinMode(Error, INPUT_PULLDOWN); 
  pinMode(led, OUTPUT); 
  pinMode(buzzer, OUTPUT); 
   
  // Initialize outputs 
  digitalWrite(RL, LOW); 
  digitalWrite(led, LOW); 
  digitalWrite(buzzer, LOW); 
   
  // If we're forcing offline mode after button restart, skip all WiFi connection attempts 
  if (forceOfflineModeAfterBoot) { 
    Serial.println("Forcing offline mode after button restart"); 
    offlineMode = true; 
    apMode = false; 
    return; 
  } 
 
  // Check if the config button is held at boot (skip this if we're in button restart recovery) 
  if (digitalRead(enButtonPin) == LOW && !skipApModeOnBoot) { 
    Serial.println("Configuration button held during boot - entering AP mode directly"); 
    enterAccessPointMode(); 
    return; // Skip the rest of setup if entering AP mode 
  }
  // Read storedConfig from EEPROM
  EEPROM.get(0, storedConfig);
  provisioned = storedConfig.valid;

  // 1) AP mode on first boot or factory reset
  if (!provisioned) {
    Serial.println("⚙️  First-time install or reset → AP mode");
    enterAccessPointMode();  // your existing AP-start function
    return;                  // skip rest of setup
  }

  // 2) Station mode: load credentials and start connect
  ssid     = String(storedConfig.ssid);
  password = String(storedConfig.password);

  Serial.print("➡️  Connecting to SSID: "); Serial.println(ssid);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid.c_str(), password.c_str());

  // 3) Defer Firebase & NTP init until Wi-Fi is actually up
  timeClient.begin();
  
  // ─── INITIAL ONLINE CHECK ───
  if (WiFi.status() == WL_CONNECTED && internetAvailable()) {
    Serial.println("✅ Initial Wi-Fi & Internet check passed → ONLINE mode");
    wifiWasConnected = true;
    onWifiUp();
    // Register /deleteDevice in STA mode
        // server.on("/deleteDevice", DeleteDevice); 
        // server.begin(); 
      lastWiFiOK = millis();
  } else {
    Serial.println("⚠️ Initial Internet unreachable → OFFLINE mode");
    wifiWasConnected = false;
    onWifiDown();
  }
}

// ---------------------------------------------------------- 
// MAIN LOOP 
// ---------------------------------------------------------- 
void loop() {
  // ─── Wi-Fi reconnect watchdog every 60 s ───
  if (provisioned) {
    unsigned long now = millis();

    // 1) Reconnect to Wi-Fi every minute if not connected
    if (WiFi.status() != WL_CONNECTED && now - lastWifiAttempt >= WIFI_RETRY_INTERVAL) {
      lastWifiAttempt = now;
      Serial.println("🔄 Attempting Wi-Fi reconnect…");
      WiFi.reconnect();
    }

    // 2) ONLINE mode if Wi-Fi AND Internet are both available
    if (WiFi.status() == WL_CONNECTED && internetAvailable()) {
      if (!wifiWasConnected) {
        Serial.println("✅ Wi-Fi & Internet OK → ONLINE mode");
        wifiWasConnected = true;
        onWifiUp();
      }
    }
    // 3) OFFLINE if either is down
    else {
      if (wifiWasConnected) {
        Serial.println("⚠️ Lost Wi-Fi or Internet → OFFLINE mode");
        wifiWasConnected = false;
        onWifiDown();
      }
    }
  }

  // Check config button (GPIO0) first for immediate AP mode entry 
  if (digitalRead(enButtonPin) == LOW) { 
    Serial.println("Config button pressed - waiting to confirm..."); 
    unsigned long pressStart = millis(); 
    // Wait to see if this is a sustained press 
    while (digitalRead(enButtonPin) == LOW && (millis() - pressStart < 5000)) { 
      delay(100); // Check every 100ms 
    } 
     
    // If still pressed after 5 seconds, enter AP mode 
    if (digitalRead(enButtonPin) == LOW) { 
      Serial.println("Reconfiguration button pressed for 5s -> AP mode"); 
       
      enterAccessPointMode(); 
       
      if (WiFi.status() == WL_CONNECTED) { 
        // Delete WIFI_IP from both Profile and Device paths
        String profileWifiIpPath = "/users/" + String(storedConfig.userId) + "/Profile/WIFI_IP"; 
        String deviceWifiIpPath = "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/WIFI_IP";
        Firebase.deleteNode(firebaseData, profileWifiIpPath);
        Firebase.deleteNode(firebaseData, deviceWifiIpPath);
        Serial.println("WIFI_IP deleted from both Profile and Device paths in Firebase.");
      } 

      // Clear EEPROM 
      for (int i = 0; i < EEPROM_SIZE; i++) { 
        EEPROM.write(i, 0); 
      } 
      EEPROM.commit(); 

      storedConfig.valid = false; 
      EEPROM.put(0, storedConfig); 
      EEPROM.commit(); 

      WiFi.disconnect(); 
      Serial.println("EEPROM cleared. Wi-Fi disconnected."); 
      ESP.restart(); 
    } 
  } 
   
  // Check if we need to switch between online/offline modes 
  if (WiFi.status() == WL_CONNECTED && internetAvailable() && !apMode) {
    // Just-turned-online?
    if (offlineMode) {
      Serial.println("✅ Wi-Fi & Internet OK → ONLINE mode");
      offlineMode = false;
      onWifiUp();
      // Update Firebase with current status after reconnection 
      if (Firebase.ready()) { 
        Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/UPPER_TANK_LEVEL", LEVEL); 
        Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/LOWER_TANK_LEVEL", Lowsta); 
        Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/MOTOR_STATUS", (digitalRead(RL) == HIGH) ? "ON" : "OFF"); 
        Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/BUZZER_STATUS", (digitalRead(buzzer) == HIGH) ? "ON" : "OFF"); 
      } 
    }
  } else {
    // Just-turned-offline?
    if (!offlineMode) {
      Serial.println("⚠️ Lost Wi‑Fi or Internet → OFFLINE mode");
      offlineMode = true;
      onWifiDown();
    }
  } 

  // If in STA mode, handle possible client requests 
  if (!apMode) { 
    server.handleClient(); 
  } else { 
    // In AP mode, handle client and blink LED as visual indicator 
    server.handleClient(); 
     
    static unsigned long lastAPBlink = 0; 
    if (millis() - lastAPBlink > 1000) { 
      lastAPBlink = millis(); 
      // Blink LED in a pattern to indicate AP mode 
      static int apBlinkState = 0; 
      if (apBlinkState++ % 3 == 0) { 
        digitalWrite(led, !digitalRead(led)); 
      } 
    } 
  } 

  // Handle pushbutton (GPIO13) with different press durations 
  int buttonState = digitalRead(p); 

  // Button pressed 
  if (buttonState == HIGH && pb == 0) { 
    pb = millis();  // Record press start time 
  } 

  // Button released 
  if (buttonState == LOW && pb > 0) { 
    unsigned long pressDuration = millis() - pb; 
    pb = 0;  // Reset press start time 

    // Determine action based on press duration 
    if (pressDuration >= 1000 && pressDuration < 7000) { 
      // 5-second press - Turn off buzzer, LED stays HIGH 
      Serial.println("5 second press - Turning off buzzer only"); 
      digitalWrite(buzzer, LOW); 
      digitalWrite(led, HIGH); 

      // Update Firebase if connected 
      if (WiFi.status() == WL_CONNECTED && Firebase.ready()) { 
        Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/BUZZER_STATUS", "OFF"); 
      } 
    } else if (pressDuration >= 8000 && pressDuration < 15000) { 
      // 8-15 second press - Turn off both buzzer and LED, then restart 
      Serial.println("8-15 second press - Turning off buzzer and LED"); 
      digitalWrite(buzzer, LOW); 
      digitalWrite(led, LOW); 

      // Update Firebase if connected 
      if (WiFi.status() == WL_CONNECTED && Firebase.ready()) { 
        Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/BUZZER_STATUS", "OFF"); 
      } 
       
      // Set restart flag to indicate this is a button-initiated restart 
      byte restartFlag = RESTART_MAGIC_VAL; 
      EEPROM.put(RESTART_FLAG_ADDR, restartFlag); 
       
      // Store the current mode (online or offline) to preserve it after restart 
      byte modeFlag = offlineMode ? OFFLINE_MODE_VAL : ONLINE_MODE_VAL; 
      EEPROM.put(MODE_FLAG_ADDR, modeFlag); 
      EEPROM.commit(); 
       
      // Log the current mode that will be preserved 
      if (offlineMode) { 
        Serial.println("Currently in OFFLINE mode - will preserve after restart"); 
      } else { 
        Serial.println("Currently in ONLINE mode - will preserve after restart"); 
      } 
       
      Serial.println("Setting button restart flag in EEPROM...");
      delay(200); 
      ESP.restart(); 
    } else if (pressDuration >= 30000) { 
      // 30+ second press - Factory reset 
      Serial.println("30+ second press - Factory reset"); 

      // Delete WIFI_IP from Firebase if connected 
      if (WiFi.status() == WL_CONNECTED && Firebase.ready()) { 
        // Delete WIFI_IP from both Profile and Device paths
        String profileWifiIpPath = "/users/" + String(storedConfig.userId) + "/Profile/WIFI_IP"; 
        String deviceWifiIpPath = "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/WIFI_IP";
        Firebase.deleteNode(firebaseData, profileWifiIpPath);
        Firebase.deleteNode(firebaseData, deviceWifiIpPath);
        Serial.println("WIFI_IP deleted from both Profile and Device paths in Firebase");
      } 

      // Clear EEPROM 
      for (int i = 0; i < EEPROM_SIZE; i++) { 
        EEPROM.write(i, 0); 
      } 
      EEPROM.commit(); 

      // Mark config as invalid 
      storedConfig.valid = false; 
      EEPROM.put(0, storedConfig); 
      EEPROM.commit(); 

      // Disconnect WiFi 
      WiFi.disconnect(true); 

      // Enter AP mode 
      enterAccessPointMode(); 

      // Restart 
      Serial.println("Factory reset complete. Restarting..."); 
      ESP.restart(); 
    } 
  }

  // --- Read Sensors --- 
  int l1State = digitalRead(L1); 
  int l2State = digitalRead(L2); 
  int l3State = digitalRead(L3); 
  int l4State = digitalRead(L4); 
  int l5State = digitalRead(L5); 
  int l6State = digitalRead(L6); 
  int lsState = digitalRead(LT); 
  int errState = digitalRead(Error); 
  int ledState = digitalRead(led);
  int buzState = digitalRead(buzzer); 
  bool motorIsOn = (digitalRead(RL) == HIGH); 

  // Determine tank level 
  if (l1State == LOW && l2State == LOW && l3State == LOW && l4State == LOW && l5State == LOW && l6State == LOW) { 
    LEVEL = "0";  // All sensors LOW means 0% (empty) 
  } else if (l1State == HIGH && l2State == LOW && l3State == LOW && l4State == LOW && l5State == LOW && l6State == LOW) { 
    LEVEL = "10"; 
  } else if (l1State == HIGH && l2State == HIGH && l3State == LOW && l4State == LOW && l5State == LOW && l6State == LOW) { 
    LEVEL = "30"; 
  } else if (l1State == HIGH && l2State == HIGH && l3State == HIGH && l4State == LOW && l5State == LOW && l6State == LOW) { 
    LEVEL = "50"; 
  } else if (l1State == HIGH && l2State == HIGH && l3State == HIGH && l4State == HIGH && l5State == LOW && l6State == LOW) { 
    LEVEL = "70"; 
  } else if (l1State == HIGH && l2State == HIGH && l3State == HIGH && l4State == HIGH && l5State == HIGH && l6State == LOW) { 
    LEVEL = "90"; 
  } else if (l1State == HIGH && l2State == HIGH && l3State == HIGH && l4State == HIGH && l5State == HIGH && l6State == HIGH) { 
    LEVEL = "100"; 
  } else { 
    LEVEL = "101";  // Default to 101% if unexpected sensor combination 
  } 

  // Lower tank 
  Lowsta = (lsState == HIGH) ? "Water Available" : "Water Unavailable"; 

  // HANDLE OFFLINE MODE LOGIC 
  if (offlineMode) { 
    // In offline mode, use local logic to control the motor 
    handleOfflineControl(); 
     
    // Try to reconnect to WiFi periodically 
    checkAndReconnectWiFi(); 
     
    // Even in offline mode, check for active errors (no water flow) 
    if (digitalRead(RL) == HIGH) { 
      if (RLmonitering) { 
        if (millis() - Relaytime >= 20000UL) { 
          if (digitalRead(Error) == LOW) { // No water flow detected 
            // Trigger alarm 
            digitalWrite(led, HIGH); 
            digitalWrite(buzzer, HIGH); 
            digitalWrite(RL, LOW); 
            RLmonitering = false; 
            Serial.println("Offline: Error - No water flow detected for 20s → ALARM"); 
            
            // Update Firebase with error status
            if (WiFi.status() == WL_CONNECTED && Firebase.ready()) { 
              Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ERROR_RESET", "1"); 
              Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ALARM_STATUS", "1"); 
            } 
          } 
        } 
      } 
    } 
  }  
  // HANDLE ONLINE MODE LOGIC 
  else if (WiFi.status() == WL_CONNECTED && internetAvailable()) { 
    // Mark WiFi as OK 
    lastWiFiOK = millis(); 

    // HEARTBEAT: every 10s => set POWER=ON 
    if ((millis() - lastHeartbeat) >= HEARTBEAT_INTERVAL) { 
      lastHeartbeat = millis(); 
      if (Firebase.ready()) { 
        Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/POWER", "ON"); 
        Serial.println("Heartbeat -> POWER=ON in Firebase"); 
      } 
    } 

    // If we had a "powerOffPending" from a previous offline, let's do it once 
    if (powerOffPending) { 
      // Set POWER=OFF to indicate device was offline 
      if (Firebase.ready()) { 
        Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/POWER", "OFF"); 
        Serial.println("Device was offline too long, so set POWER=OFF once reconnected."); 
      } 
      powerOffPending = false; 
    } 

    // If Firebase not ready, skip Firebase-based control 
    if (!Firebase.ready()) { 
      delay(500); 
      return; 
    } 

    // Read MOTOR_STATUS from Firebase ("ON"/"OFF") 
    String motorStatusPath = "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/MOTOR_STATUS"; 
    if (Firebase.getString(firebaseData, motorStatusPath)) { 
      String mStatus = firebaseData.stringData(); 
      firebaseControl = (mStatus == "ON"); 
    } 

    // Check schedule 
    checkScheduledTasks(); 

    // Decide if we want the motor ON or OFF 
    bool wantMotorOn = false; 
    int levelVal = LEVEL.toInt(); 
    bool waterAvailable = (lsState == HIGH); 

    // If the motor was forced OFF by time-based schedule, wait 30s 
    if (motorTurnedOffDueToSchedule) { 
      Serial.println("Motor remains OFF due to scheduled off time (time-based)."); 
      if (millis() - motorOffTime >= 30000UL) { 
        motorTurnedOffDueToSchedule = false; 
        Serial.println("Flag reset: Motor can now turn on again."); 
      } 
      // Skip turning motor ON this loop 
      return; 
    } 

    // (1) If Firebase says "ON" => override ON (unless tank reached full during schedule) 
    if (firebaseControl && levelVal < tankFullThreshold) { 
      wantMotorOn = true; 
    } 

    // (2) If schedule says inScheduledWindow => let's see if it's time-based or tank-level 
    if (inScheduledWindow && !firebaseControl && !tankReachedFullDuringSchedule && levelVal < tankFullThreshold) { 
      if (!scheduledOffTank) { 
        wantMotorOn = true; 
      } else { 
        // scheduledOffTank == true 
        if (!tankStopUsed) { 
          wantMotorOn = true; 
        } else { 
          Serial.println("Tank-level stop already used today; not turning ON again."); 
        } 
      } 
    } 
    checkPendingUpdates();
    checkFirebaseResetNodes(); // Monitor Firebase reset nodes every 15 seconds
    checkFirebaseDeleteNode(); // Monitor Firebase delete node every 10 seconds
    // (3) If not in schedule & not firebase => auto logic 
    if (!inScheduledWindow && !firebaseControl) { 
      // Modified condition to include 0% level 
      if (levelVal <= tankEmptyThreshold && waterAvailable && ledState == LOW && buzState == LOW) { 
        wantMotorOn = true; 
        Serial.println("Auto mode: Tank empty or at low level - turning motor ON"); 
      } 
    } 

    // (4) If lower tank unavailable => immediate OFF 
    if (!waterAvailable) { 
      wantMotorOn = false; 
      if (firebaseControl) { 
        Firebase.setString(firebaseData, motorStatusPath, "OFF"); 
        firebaseControl = false; 
      } 
      Serial.println("Immediate OFF => Lower tank water unavailable."); 
    } 

    // (5) If tank is full => OFF and set flag if in scheduled window 
    if (levelVal >= tankFullThreshold) { 
      wantMotorOn = false; 
      if (motorIsOn && inScheduledWindow) { 
        if (scheduledOffTank) { 
          tankStopUsed = true; 
        } 
        tankReachedFullDuringSchedule = true; 
        Serial.println("Tank reached full level - setting flag to prevent re-activation"); 
      } 
    } 

    // (6) If there's LED/buzzer error => OFF 
    if (ledState == HIGH || buzState == HIGH) { 
      wantMotorOn = false;
    } 

    // Implement wantMotorOn with continuous error checking 
    bool motorIsCurrentlyOn = (digitalRead(RL) == HIGH); 

    if (wantMotorOn) { 
      if (!motorIsCurrentlyOn) { 
        // Turn motor ON 
        digitalWrite(RL, HIGH); 
        RLmonitering = true; 
        Relaytime = millis();  // Start timer when motor turns ON 
        Firebase.setString(firebaseData, motorStatusPath, "ON"); 
        Serial.println("Motor turned ON (firebase/sched/auto)."); 

        // Clear any previous error status 
        if (Firebase.ready()) { 
          Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ERROR_STATUS", "NORMAL"); 
        } 
      } 

      // CONTINUOUS ERROR CHECKING WHILE MOTOR IS ON 
      if (digitalRead(RL) == HIGH) { 
        static unsigned long lastErrorCheck = 0; 
        const unsigned long ERROR_CHECK_INTERVAL = 1000;  // Check every 1 second 

        if (millis() - lastErrorCheck >= ERROR_CHECK_INTERVAL) { 
          lastErrorCheck = millis(); 

          if (digitalRead(Error) == LOW) {  // No water detected (Error is LOW) 
            unsigned long currentMillis = millis(); 

            if (currentMillis - Relaytime >= 20000UL) {  // 20-second timeout 
              // Trigger alarm and turn OFF 
              digitalWrite(led, HIGH); 
              digitalWrite(buzzer, HIGH); 
              digitalWrite(RL, LOW); 
              RLmonitering = false; 
              Firebase.setString(firebaseData, motorStatusPath, "OFF"); 
              Serial.println("No water detected for 20s => OFF (error)."); 

              // Update Firebase with error status 
              if (Firebase.ready()) { 
                Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ERROR_RESET", "1"); 
                Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ALARM_STATUS", "1"); 
              } 
            } 
          } else { 
            // Water detected (Error is HIGH) - reset timer 
            Relaytime = millis(); 
            Serial.println("Water flow detected - timer reset"); 

            // Update Firebase with normal status 
            if (Firebase.ready()) { 
              Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ERROR_STATUS", "NORMAL"); 
            } 
          } 
        } 
      } 
    } else { 
      if (motorIsCurrentlyOn) { 
        // Turn motor OFF 
        digitalWrite(RL, LOW); 
        RLmonitering = false; 
        Firebase.setString(firebaseData, motorStatusPath, "OFF"); 
        Serial.println("Motor forced OFF (conditions not met)."); 
      } 
    } 

    // Update Firebase sensor states 
    Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/UPPER_TANK_LEVEL", LEVEL); 
    Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/LOWER_TANK_LEVEL", Lowsta); 
    String buzzerStateStr = (digitalRead(buzzer) == HIGH) ? "ON" : "OFF"; 
    Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/BUZZER_STATUS", buzzerStateStr); 

    // Debug 
    Serial.print("Upper Tank LEVEL: "); 
    Serial.println(LEVEL); 
    Serial.print("Lower tank status: "); 
    Serial.println(Lowsta); 
    Serial.print("Tank reached full during schedule: "); 
    Serial.println(tankReachedFullDuringSchedule ? "YES" : "NO"); 
  } else { 
    // If WiFi status is not connected but we're not yet in offline mode 
    // This can happen when WiFi just disconnected 
    if (!offlineMode) { 
      Serial.println("WiFi disconnected but not yet in offline mode."); 
      // We'll enter offline mode at the beginning of the next loop 
    } 
  } 

  delay(500); 
} 

// ---------------------------------------------------------- 
// OFFLINE MODE FUNCTIONS 
// ---------------------------------------------------------- 
// Function to handle automatic tank level control in offline mode 
void handleOfflineControl() { 
  // Read sensors 
  int l1State = digitalRead(L1); 
  int l2State = digitalRead(L2); 
  int l3State = digitalRead(L3); 
  int l4State = digitalRead(L4); 
  int l5State = digitalRead(L5); 
  int l6State = digitalRead(L6); 
  int lsState = digitalRead(LT); 
  int errState = digitalRead(Error); 
  int levelVal = LEVEL.toInt(); 
  bool waterAvailable = (lsState == HIGH); 
   
  // Determine if motor should be ON or OFF in offline mode 
  bool shouldTurnMotorOn = false; 
   
  // Lower tank must have water 
  if (!waterAvailable) { 
    shouldTurnMotorOn = false; 
    Serial.println("Offline: Lower tank has no water - motor OFF"); 
  }  
  // Tank is full 
  else if (levelVal >= tankFullThreshold) { 
    shouldTurnMotorOn = false; 
    Serial.println("Offline: Upper tank level == " + String(tankFullThreshold) + "% - motor OFF"); 
  }  
  // Tank is empty/low and lower tank has water 
  else if (levelVal <= tankEmptyThreshold && waterAvailable) { 
    shouldTurnMotorOn = true; 
    Serial.println("Offline: Upper tank level <= " + String(tankEmptyThreshold) + "% and water available - motor ON"); 
  } 
  // Keep current state between thresholds (hysteresis) 
  else { 
    shouldTurnMotorOn = (digitalRead(RL) == HIGH); 
    Serial.println("Offline: Maintaining current motor state: " + String(shouldTurnMotorOn ? "ON" : "OFF")); 
  }
  // Check for existing errors (LED/buzzer already ON) 
  if (digitalRead(led) == HIGH || digitalRead(buzzer) == HIGH) { 
    shouldTurnMotorOn = false; 
    Serial.println("Offline: Error detected (LED/buzzer ON) - motor OFF"); 
  } 

  // Apply motor state 
  bool wasMotorOn = (digitalRead(RL) == HIGH); 
  if (shouldTurnMotorOn != wasMotorOn) { 
    digitalWrite(RL, shouldTurnMotorOn ? HIGH : LOW); 
    Serial.println("Offline: Motor turned " + String(shouldTurnMotorOn ? "ON" : "OFF")); 
  } 
   
  // --- ERROR DETECTION LOGIC --- 
  // Continuous monitoring for water flow when motor is ON 
  if (shouldTurnMotorOn) { 
    if (!RLmonitering) { 
      // Just turned ON 
      RLmonitering = true;
      Relaytime = millis(); 
      Serial.println("Offline: Motor turned ON - starting flow monitoring"); 
    } 
     
    // Check for water flow 
    static unsigned long lastErrorCheck = 0; 
    const unsigned long ERROR_CHECK_INTERVAL = 1000;  // Check every 1 second 
     
    if (millis() - lastErrorCheck >= ERROR_CHECK_INTERVAL) { 
      lastErrorCheck = millis(); 
       
      if (digitalRead(Error) == LOW) { // No water detected (Error is LOW) 
        unsigned long currentMillis = millis(); 
         
                  if (currentMillis - Relaytime >= 20000UL) { // 20-second timeout 
            // Trigger alarm and turn OFF 
            digitalWrite(led, HIGH); 
            digitalWrite(buzzer, HIGH); 
            digitalWrite(RL, LOW); 
            RLmonitering = false; 
            Serial.println("Offline: No water flow detected for 20s - ALARM triggered & motor OFF"); 
            
            // Update Firebase with error status
            if (WiFi.status() == WL_CONNECTED && Firebase.ready()) { 
              Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ERROR_RESET", "1"); 
              Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ALARM_STATUS", "1"); 
            } 
          } else { 
          Serial.println("Offline: No water flow detected for " + String((currentMillis - Relaytime) / 1000) + "s, waiting..."); 
        } 
      } else { 
        // Water detected (Error is HIGH) - reset timer 
        Relaytime = millis(); 
      } 
    } 
  } else { 
    RLmonitering = false; 
  } 
   
  // Log current status 
  if (millis() - lastOfflineStatusUpdate > OFFLINE_STATUS_UPDATE_INTERVAL) { 
    lastOfflineStatusUpdate = millis(); 
    Serial.println("--------------------------------"); 
    Serial.println("OFFLINE MODE STATUS UPDATE:"); 
    Serial.println("Upper Tank Level: " + LEVEL + "%"); 
    Serial.println("Lower Tank: " + Lowsta); 
    Serial.println("Motor: " + String(digitalRead(RL) == HIGH ? "ON" : "OFF")); 
    Serial.println("LED: " + String(digitalRead(led) == HIGH ? "ON" : "OFF")); 
    Serial.println("Buzzer: " + String(digitalRead(buzzer) == HIGH ? "ON" : "OFF")); 
    Serial.println("Water Flow Status: " + String(digitalRead(Error) == HIGH ? "NORMAL" : "NO FLOW")); 
    Serial.println("--------------------------------"); 
  } 
} 

// Function to check WiFi and attempt reconnection 
void checkAndReconnectWiFi() { 
  // Skip if we're in AP mode or don't have valid credentials 
  if (!storedConfig.valid || apMode) return;
  static int consecutiveFailures = 0; 
  if (millis() - lastWiFiCheckTime > WIFI_CHECK_INTERVAL) { 
    lastWiFiCheckTime = millis(); 
     
    if (WiFi.status() != WL_CONNECTED) { 
      Serial.println("Attempting to reconnect to WiFi..."); 
       
      // Disconnect first to clear any stale connections 
      WiFi.disconnect(true); 
      delay(1000); 
       
      // Set WiFi mode explicitly 
      WiFi.mode(WIFI_STA); 
      delay(500); 
       
      // Begin connection attempt 
      WiFi.begin(storedConfig.ssid, storedConfig.password); 
       
      // Wait briefly for connection 
      int retries = 15; 
      while (WiFi.status() != WL_CONNECTED && retries-- > 0) { 
        delay(500); 
        Serial.print("."); 
      } 
      Serial.println(); 
       
      if (WiFi.status() == WL_CONNECTED) { 
        Serial.println("WiFi reconnected! IP: " + WiFi.localIP().toString()); 
        offlineMode = false; 
        consecutiveFailures = 0; 
         
        // Initialize Firebase and NTP 
        firebaseConfig.host = FIREBASE_HOST; 
        firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH; 
        Firebase.begin(&firebaseConfig, &firebaseAuth); 
           
        // Update IP in DB (both Profile and Device paths)
        Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/Profile/WIFI_IP", WiFi.localIP().toString());
        Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/WIFI_IP", WiFi.localIP().toString());
           
        // Mark that we need to set POWER=OFF once (we were offline) 
        powerOffPending = true; 
         
        timeClient.begin(); 
         
        // Set the server to listen for commands again 
        // server.on("/deleteDevice", DeleteDevice); 
        // server.begin(); 
      } else { 
        Serial.println("WiFi reconnection failed. Continuing in offline mode."); 
        consecutiveFailures++; 
         
        // If we've had too many consecutive failures AND not in button-restart mode, 
        // try entering AP mode 
        if (consecutiveFailures >= 2) { 
          Serial.println("Too many consecutive WiFi failures. Entering AP mode...");
          consecutiveFailures = 0; 
          enterAccessPointMode();
        } else if (consecutiveFailures >= 10) { 
          Serial.println("Multiple WiFi failures, but staying in offline mode due to button restart flag"); 
          // Reset counter to avoid spamming this message 
          consecutiveFailures = 0; 
        } 
      } 
    } else { 
      consecutiveFailures = 0; // Reset counter when WiFi is connected 
    } 
  } 
} 

// ---------------------------------------------------------- 
// SCHEDULED TASKS 
// ---------------------------------------------------------- 
void checkScheduledTasks() { 
  if (!Firebase.ready()) return; 
  
  // ======== ENHANCED TIME FETCH & FALLBACK ========
  // Get a fresh NTP time if possible, otherwise fall back
  long currentEpoch;
  // try to update NTP, but don't abort scheduling if we've ever had a sync
  if (timeClient.update()) {
    lastSyncEpoch  = timeClient.getEpochTime();
    lastSyncMillis = millis();
    Serial.printf("✅ NTP sync OK: %lu\n", lastSyncEpoch);
  } else {
    if (lastSyncEpoch != 0) {
      Serial.println("⏳ Using fallback time from last good sync");
    } else {
      // first-ever sync failed; we'll run schedule using device's millis()
      Serial.println("⚠️ Still no NTP time—using device uptime as fallback");
      static const uint32_t EPOCH_OFFSET = 1704067200UL;  // Jan 1 2025 00:00:00 UTC
      lastSyncEpoch  = (millis() / 1000UL) + EPOCH_OFFSET;
      lastSyncMillis = millis();
    }
  }
  currentEpoch = lastSyncEpoch;

  // ======== LOCAL TIME CALCULATION ========
  // Pakistan offset = +5 hours = 5*3600 seconds
  const unsigned long PKT_OFFSET = 5UL * 3600UL;
  unsigned long localEpoch = currentEpoch;
  int dayLocal             = (localEpoch / 86400UL + 4) % 7;  // Jan 1 1970 was Thursday (4)
  unsigned long secsInDay  = localEpoch % 86400UL;
  int localHour            = secsInDay / 3600;
  int localMinute          = (secsInDay % 3600) / 60;
  unsigned long rawEpoch   = currentEpoch;
  // ======== END LOCAL CALCULATION ========
  
  // Debug time information
  Serial.println("Raw NTP day: " + String(timeClient.getDay()));
  Serial.println("Time info - Raw epoch: " + String(rawEpoch) +  
                 ", Local day: " + String(dayLocal) +  
                 ", Hour: " + String(localHour) +  
                 ", Minute: " + String(localMinute)); 
   
  // If new day, reset all flags 
  if (dayLocal != lastDayLocalUsed) { 
    tankStopUsed = false; 
    tankReachedFullDuringSchedule = false;  // Reset the flag for new day 
    lastDayLocalUsed = dayLocal; 
    lastScheduleSignature = ""; 
    Serial.println("New day => tankStopUsed & tankReachedFullDuringSchedule reset, signature cleared."); 
  } 

  // Only check once per new minute 
  if (dayLocal == lastCheckedDay && localMinute == lastCheckedMinute) { 
    return; 
  } 
  lastCheckedDay = dayLocal; 
  lastCheckedMinute = localMinute; 

  inScheduledWindow = false; 
  scheduledOffTank = false; 
  scheduledOnMinutes = -1; 
  scheduledOffMinutes = -1; 

  String dayName = getDayOfWeekName(dayLocal); 
  Serial.println("-----------------------------------------------"); 
  Serial.printf("Local dayOfWeek: %s (%d), local time=%02d:%02d\n", 
                dayName.c_str(), dayLocal, localHour, localMinute); 

  String basePath = "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/SCHEDULED_TASK/"; 

  // ======== MULTI-TASK SUPPORT ========
  // Check for multiple tasks (Task1, Task2, Task3, etc.)
  bool foundActiveTask = false;
  String activeTaskSignature = "";
  int taskNumber = 1;
  const int MAX_TASKS = 10; // Maximum tasks to check per day
  
  for (int i = 1; i <= MAX_TASKS && !foundActiveTask; i++) {
    String taskName = "Task" + String(i);
    String taskOnPath = basePath + "ON/" + dayName + "/" + taskName + "/Time";
    String taskOffPath = basePath + "OFF/" + dayName + "/" + taskName + "/Time";
    String taskCondPath = basePath + "OFF/" + dayName + "/" + taskName + "/stopCondition";
    
    // Check if this task exists
    String onTimeStr = "";
    if (Firebase.getString(firebaseData, taskOnPath)) {
      onTimeStr = firebaseData.stringData();
      if (onTimeStr.length() > 0) {
        Serial.println("Found " + taskName + " with ON time: " + onTimeStr);
        
        int onH, onM;
        if (parseTimeString(onTimeStr, onH, onM)) {
          int taskOnMinutes = onH * 60 + onM;
          
          // Check OFF time for this task
          String offTimeStr = "";
          bool haveOffTime = false;
          int taskOffMinutes = -1;
          
          if (Firebase.getString(firebaseData, taskOffPath)) {
            offTimeStr = firebaseData.stringData();
            if (offTimeStr.length() > 0) {
              int offH, offM;
              if (parseTimeString(offTimeStr, offH, offM)) {
                taskOffMinutes = offH * 60 + offM;
                haveOffTime = true;
                Serial.printf("%s OFF time => %02d:%02d => %d\n", taskName.c_str(), offH, offM, taskOffMinutes);
              }
            }
          }
          
          // Check stop condition for this task
          String condVal = "";
          bool taskOffTank = false;
          if (Firebase.getString(firebaseData, taskCondPath)) {
            condVal = firebaseData.stringData();
            if (condVal == "tankLevel") {
              taskOffTank = true;
              Serial.println(taskName + " OFF condition => TANK LEVEL");
            }
          }
          
          int nowMinutes = localHour * 60 + localMinute;
          
          // Check if current time falls within this task's schedule
          if (!taskOffTank && haveOffTime && taskOffMinutes > taskOnMinutes) {
            // Time-based schedule
            if (nowMinutes >= taskOnMinutes && nowMinutes < taskOffMinutes) {
              if (!tankReachedFullDuringSchedule) {
                foundActiveTask = true;
                taskNumber = i;
                scheduledOnMinutes = taskOnMinutes;
                scheduledOffMinutes = taskOffMinutes;
                inScheduledWindow = true;
                motorTurnedOffDueToSchedule = false;
                Serial.println("Currently in " + taskName + " scheduled ON window (time-based).");
              } else {
                Serial.println("In " + taskName + " window but tank already reached full - not activating.");
              }
            } else if (nowMinutes >= taskOffMinutes) {
              // Handle motor OFF for time-based schedule
              if (digitalRead(RL) == HIGH) {
                digitalWrite(RL, LOW);
                motorTurnedOffDueToSchedule = true;
                motorOffTime = millis();
                String motorStatusPath = "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/MOTOR_STATUS";
                if (Firebase.setString(firebaseData, motorStatusPath, "OFF")) {
                  Serial.println(taskName + " scheduled OFF time reached => Motor turned OFF.");
                } else {
                  Serial.println("Failed to update Firebase for MOTOR_STATUS.");
                }
              }
              Serial.println("Outside " + taskName + " scheduled window (time-based).");
            }
          } else if (taskOffTank) {
            // Tank-level based schedule
            if (nowMinutes >= taskOnMinutes && !tankReachedFullDuringSchedule) {
              foundActiveTask = true;
              taskNumber = i;
              scheduledOnMinutes = taskOnMinutes;
              scheduledOffTank = true;
              inScheduledWindow = true;
              motorTurnedOffDueToSchedule = false;
              Serial.println("Currently in " + taskName + " scheduled ON window (tank-level OFF).");
            } else {
              Serial.println("Not entering " + taskName + " window (tank already reached full today).");
            }
          }
          
          // Build signature for this task
          activeTaskSignature += taskName + ":" + onTimeStr + "|" + offTimeStr + "|" + condVal + ";";
        }
      }
    } else {
      // No more tasks found, break the loop
      if (i == 1) {
        Serial.println("No scheduled tasks found for today.");
      }
      break;
    }
  }
  
  // Handle schedule signature changes
  if (activeTaskSignature != lastScheduleSignature && lastScheduleSignature != "") {
    Serial.println("Schedule changed mid-day => resetting tankStopUsed & tankReachedFullDuringSchedule!");
    tankStopUsed = false;
    tankReachedFullDuringSchedule = false;
  }
  lastScheduleSignature = activeTaskSignature;
  
  if (foundActiveTask) {
    Serial.println("Active task: Task" + String(taskNumber));
  } else {
    Serial.println("No active scheduled tasks at current time.");
  }
  
  Serial.println("-----------------------------------------------"); 
}

// ---------------------------------------------------------- 
bool parseTimeString(const String& timeStr, int& hourOut, int& minOut) { 
  if (timeStr.length() < 3) return false; 
  int colonIndex = timeStr.indexOf(':'); 
  if (colonIndex < 0) return false; 

  String hh = timeStr.substring(0, colonIndex); 
  String mm = timeStr.substring(colonIndex + 1); 
  hourOut = hh.toInt(); 
  minOut = mm.toInt(); 
  if (hourOut < 0 || hourOut > 23 || minOut < 0 || minOut > 59) { 
    return false; 
  } 
  return true; 
} 

// ---------------------------------------------------------- 
void enterAccessPointMode() { 
  if (apMode) return; 
  Serial.println("Entering Access Point Mode..."); 
   
  // Clear existing WiFi connections first 
  WiFi.disconnect(true); 
  delay(1000); 
   
  // Explicitly set WiFi mode 
  WiFi.mode(WIFI_AP); 
  delay(500); 
   
  apMode = true; 
  offlineMode = false;  // Not in offline mode when in AP mode 

  // Start access point with more reliable parameters 
  bool apSuccess = WiFi.softAP(apSSID, apPassword, 1, 0, 4); // Channel 1, not hidden, 4 max connections 
   
  if (apSuccess) { 
    Serial.println("Access Point started successfully!"); 
    Serial.print("AP IP address: "); 
    Serial.println(WiFi.softAPIP()); 
     
    // Flash LED to indicate AP mode entry 
    for (int i = 0; i < 5; i++) { 
      digitalWrite(led, HIGH); 
      delay(100); 
      digitalWrite(led, LOW); 
      delay(100); 
    } 
  } else { 
    Serial.println("Failed to start Access Point. Retrying..."); 
    delay(1000); 
    // Try again with default settings 
    WiFi.softAP(apSSID, apPassword); 
    Serial.print("AP IP address: "); 
    Serial.println(WiFi.softAPIP()); 
  } 

  // Register any routes we want available in AP mode 
  server.on("/credentials", handleCredentials); 
  // server.on("/deleteDevice", DeleteDevice); 

  server.begin(); 
   
  Serial.println("Web server started in AP mode"); 
  Serial.println("Waiting for client to connect and configure device..."); 
} 

// ---------------------------------------------------------- 
void handleCredentials() { 
  if (server.hasArg("ssid") && server.hasArg("password") && server.hasArg("userId")) { 
    String newSSID = server.arg("ssid"); 
    String newPassword = server.arg("password"); 
    String newUserId = server.arg("userId"); 

    Serial.println("Received Wi-Fi credentials from Flutter:"); 
    Serial.println("SSID: " + newSSID); 
    Serial.println("Password: " + newPassword); 
    Serial.println("UserId: " + newUserId); 

    // Store in EEPROM 
    newSSID.toCharArray(storedConfig.ssid, sizeof(storedConfig.ssid)); 
    newPassword.toCharArray(storedConfig.password, sizeof(storedConfig.password)); 
    newUserId.toCharArray(storedConfig.userId, sizeof(storedConfig.userId)); 
    storedConfig.valid = true; 
    EEPROM.put(0, storedConfig); 
    EEPROM.commit(); 

    // Stop AP 
    WiFi.softAPdisconnect(true); 
    apMode = false; 
    server.stop(); 

    // Attempt connection 
    WiFi.begin(newSSID.c_str(), newPassword.c_str()); 
    int retries = 20; 
    while (WiFi.status() != WL_CONNECTED && retries-- > 0) { 
      delay(500); 
      Serial.print("."); 
    } 
    Serial.println(); 

    if (WiFi.status() == WL_CONNECTED) { 
      server.send(200, "application/json", "{\"status\":\"connected\"}"); 
      Serial.println("Connected! IP: " + WiFi.localIP().toString()); 

      ssid = newSSID; 
      password = newPassword; 

      if (Firebase.ready()) { 
                Firebase.setString(firebaseData, "/users/" + newUserId + "/Profile/WIFI_IP", WiFi.localIP().toString());
        Firebase.setString(firebaseData, "/users/" + newUserId + "/" + String(DEVICE_ID) + "/Device/WIFI_IP", WiFi.localIP().toString());
      } 

      // Now that we have WiFi, let's start the server in STA mode 
      // server.on("/deleteDevice", DeleteDevice); 
      server.begin(); 

      ESP.restart(); 
      timeClient.begin(); 
    } else { 
      server.send(200, "application/json", "{\"status\":\"failed\"}"); 
      Serial.println("Connection failed!"); 
       
      // Enter offline mode since WiFi connection failed 
      offlineMode = true; 
    } 
  } else { 
    server.send(400, "application/json", "{\"error\":\"Missing one or more parameters\"}"); 
  } 
} 

// ---------------------------------------------------------- 
// RESET ALARM Handler - sets buzzer OFF, updates Firebase, then restarts 
// ---------------------------------------------------------- 
void ResetAlarm() { 
  Serial.println("RESET ALARM endpoint called..."); 

  // Turn off buzzer and turn on LED 
  digitalWrite(buzzer, LOW); 
  digitalWrite(led, HIGH); 

  // Save ALARM_STATUS to "0" in Firebase 
  if (WiFi.status() == WL_CONNECTED && Firebase.ready()) { 
    Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ALARM_STATUS", "0"); 
  } 

  Serial.println("...alarm reset complete."); 
} 



// ---------------------------------------------------------- 
// ERROR RESET Handler - turns off buzzer and LED, then saves ERROR_RESET to "0"
// ---------------------------------------------------------- 
void ErrorReset() { 
  Serial.println("ERROR RESET endpoint called..."); 

  // Turn off buzzer and LED
  digitalWrite(buzzer, LOW); 
  digitalWrite(led, LOW); 

  // Save ERROR_RESET to "0" in Firebase 
  if (WiFi.status() == WL_CONNECTED && Firebase.ready()) { 
    Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ERROR_RESET", "0"); 
  } 

  Serial.println("...error reset complete."); 
}

// ---------------------------------------------------------- 
// DELETE DEVICE Handler 
// ---------------------------------------------------------- 
void DeleteDevice() { 
  Serial.println("Received /deleteDevice => Clearing EEPROM & WiFi, then restarting..."); 

  // Clear stored config from EEPROM 
  for (int i = 0; i < EEPROM_SIZE; i++) { 
    EEPROM.write(i, 0); 
  } 
  EEPROM.commit(); 
  storedConfig.valid = false; 
  EEPROM.put(0, storedConfig); 
  EEPROM.commit(); 

  // Optionally disconnect Wi-Fi 
  WiFi.disconnect(true, true); 
  // Respond to client 
  server.send(200, "text/plain", "Device credentials deleted. Restarting..."); 
  delay(200);  // small delay to ensure the response is sent 
  ESP.restart(); 
} 

// ─── Wi-Fi Transition Helpers ──────────────────────────
void onWifiUp() {
  Serial.println("🔌 Wi-Fi is up. Initializing Firebase, server, and syncing NTP.");

  // Re-register HTTP endpoints and start server
  // server.on("/deleteDevice", DeleteDevice); 
  server.begin(); 

  // (Re)initialize Firebase
  firebaseConfig.host = FIREBASE_HOST;
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&firebaseConfig, &firebaseAuth);
  Firebase.reconnectWiFi(true);
  Serial.println("✅ Firebase initialized.");
  timeClient.begin();
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (timeClient.update()) {
      lastSyncEpoch  = timeClient.getEpochTime();
      lastSyncMillis = millis();
      Serial.printf("✅ Initial NTP sync succeeded: %lu\n", lastSyncEpoch);
      break;
    }
    delay(500);
  }
  if (lastSyncEpoch == 0) {
    Serial.println("❌ Initial NTP sync failed—will retry in background.");
  }
  // Mark we're online
  offlineMode = false;
  lastWiFiOK = millis();

  // Update initial device status in Firebase
  Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/MOTOR_STATUS", "OFF");
  Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/Profile/WIFI_IP", WiFi.localIP().toString());
  Firebase.setString(firebaseData, "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/WIFI_IP", WiFi.localIP().toString());
  
  // Initialize Firebase monitoring variables
  lastFirebaseMonitor = 0;
  lastAlarmStatus = "";
  lastErrorReset = "";
  lastDeleteMonitor = 0;
  lastDeleteStatus = "";
}

// Called once when Wi-Fi goes down
void onWifiDown() {
  Serial.println("🚫 Wi-Fi is down. Switching to offline mode.");

  offlineMode = true;
  offlineModeStartTime = millis();
}

// ─── BELOW YOUR OTHER FUNCTIONS, ADD THIS NEW HELPER ─
void syncNTP() {
  unsigned long now = millis();
  unsigned long interval = (ntpRetryCount == 0) ? NTP_FULL_INTERVAL : NTP_RETRY_DELAY;
  if (now - lastNtpAttempt < interval) return;
  lastNtpAttempt = now;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️  WiFi not connected. Skipping NTP sync.");
    return;
  }

  Serial.println("⏳ Trying NTP sync...");
  if (timeClient.update()) {
    unsigned long epoch = timeClient.getEpochTime();
    Serial.printf("✅ NTP sync succeeded: %lu\n", epoch);
    // remember for fallback
    lastSyncEpoch  = epoch;
    lastSyncMillis = now;
    ntpRetryCount  = 0;
  } else {
    ntpRetryCount = min<uint8_t>(ntpRetryCount + 1, NTP_MAX_RETRIES);
    Serial.printf("❌ NTP sync failed (retry %u/%u)\n",
                  ntpRetryCount, NTP_MAX_RETRIES);
    if (ntpRetryCount >= NTP_MAX_RETRIES) {
      Serial.println("⏸️ Too many failures, backing off for 1 hour");
      ntpRetryCount = 0;
    }
  }
}

// Helper functions
void updateMotorStatusInFirebase(bool isOn) {
    if (WiFi.status() == WL_CONNECTED && Firebase.ready()) {
        String status = isOn ? "ON" : "OFF";
        String path = "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/MOTOR_STATUS";
        
        if (Firebase.setString(firebaseData, path, status)) {
            Serial.println("Firebase updated: MOTOR_STATUS = " + status);
        } else {
            Serial.println("Failed to update Firebase: " + firebaseData.errorReason());
            queueMotorStatusUpdate(isOn); // Queue for retry
        }
    } else {
        queueMotorStatusUpdate(isOn); // Queue for when we're back online
    }
}

void queueMotorStatusUpdate(bool isOn) {
    pendingMotorStatus = isOn ? "ON" : "OFF";
    motorStatusPendingUpdate = true;
    Serial.println("Queued motor status update: " + pendingMotorStatus);
}

void checkPendingUpdates() {
    if (motorStatusPendingUpdate) {
        updateMotorStatusInFirebase(pendingMotorStatus == "ON");
    }
}

// ---------------------------------------------------------- 
// FIREBASE MONITORING FUNCTION 
// ---------------------------------------------------------- 
void checkFirebaseResetNodes() {
    if (!Firebase.ready()) return;
    
    unsigned long now = millis();
    if (now - lastFirebaseMonitor < FIREBASE_MONITOR_INTERVAL) return;
    lastFirebaseMonitor = now;
    
    // Check ALARM_STATUS node
    String alarmStatusPath = "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ALARM_STATUS";
    if (Firebase.getString(firebaseData, alarmStatusPath)) {
        String currentAlarmStatus = firebaseData.stringData();
        if (currentAlarmStatus != lastAlarmStatus) {
            lastAlarmStatus = currentAlarmStatus;
            if (currentAlarmStatus == "0") {
                Serial.println("ALARM_STATUS set to 0 - Triggering ResetAlarm function");
                ResetAlarm();
            }
        }
    }
    
    // Check ERROR_RESET node
    String errorResetPath = "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/ERROR_RESET";
    if (Firebase.getString(firebaseData, errorResetPath)) {
        String currentErrorReset = firebaseData.stringData();
        if (currentErrorReset != lastErrorReset) {
            lastErrorReset = currentErrorReset;
            if (currentErrorReset == "0") {
                Serial.println("ERROR_RESET set to 0 - Triggering ErrorReset function");
                ErrorReset();
            }
        }
    }
}

// ---------------------------------------------------------- 
// FIREBASE DELETE DEVICE MONITORING FUNCTION 
// ---------------------------------------------------------- 
void checkFirebaseDeleteNode() {
    if (!Firebase.ready()) return;
    unsigned long now = millis();
    if (now - lastDeleteMonitor < DELETE_MONITOR_INTERVAL) return;
    lastDeleteMonitor = now;

    String deletePath = "/users/" + String(storedConfig.userId) + "/" + String(DEVICE_ID) + "/Device/DELETE";
    if (Firebase.getString(firebaseData, deletePath)) {
        String currentDeleteStatus = firebaseData.stringData();
        if (currentDeleteStatus != lastDeleteStatus) {
            lastDeleteStatus = currentDeleteStatus;
            if (currentDeleteStatus == "0") {
                Serial.println("DELETE node set to 0 - will set to 1 and trigger DeleteDevice function");
                // Set DELETE node to 1 before deleting
                Firebase.setString(firebaseData, deletePath, "1");
                // Now trigger the delete device logic
                DeleteDevice();
            }
        }
    }
}

