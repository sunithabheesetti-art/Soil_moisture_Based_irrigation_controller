/**
 * Soil Irrigation Controller
 * 
 * Embedded C/C++ sketch implementing:
 *  - Non-blocking sample scheduling using millis()
 *  - Exponential Moving Average (EMA) moisture filter smoothing
 *  - Dual-threshold state machine with hysteresis (Dry Trigger vs. Recovery Threshold)
 *  - Sensor Fault Detection (Implausible values & Stuck sensor checks)
 *  - Telemetry FIFO buffering queue for network outages
 *  - Diagnostic simulation sequence to reproduce the logs
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

// ============================================================================
// CONFIGURATION & PIN DEFINE
// ============================================================================
#define PUMP_PIN          5    // Digital output pin driving the pump relay
#define FAULT_LED_PIN    13    // Digital output pin for the onboard Fault LED
#define MOISTURE_PIN     A0    // Analog input pin for soil moisture sensor
#define TEMP_PIN         A1    // Analog input pin for temperature sensor

// Controller Parameters
const unsigned long SAMPLE_INTERVAL_MS = 2000; // 2 seconds
const float MOISTURE_MIN_SAFE = 30.0;          // Safe range lower bound (%)
const float MOISTURE_MAX_SAFE = 70.0;          // Safe range upper bound (%)
const float RECOVERY_THRESHOLD = 55.0;         // Threshold to turn off pump (%)
const int DEBOUNCE_SAMPLES = 3;                // N consecutive low readings to trigger
const int STUCK_LIMIT = 5;                     // Number of identical readings to trigger stuck fault
const float EMA_ALPHA = 0.50;                  // Filter smoothing coefficient (0.0 to 1.0)

// Plausible Sensor Limits
const float PLAUSIBLE_MIN = 0.0;
const float PLAUSIBLE_MAX = 90.0;

// Set to true to run the automated simulation matching the console log
// Set to false to read actual sensors on the analog pins
#define RUN_SIMULATION true

// ============================================================================
// TELEMETRY QUEUE FOR NETWORK BUFFERING (FIFO)
// ============================================================================
#define QUEUE_MAX_SIZE 20

struct TelemetryRecord {
  unsigned long timestampMs;
  float rawMoisture;
  float smoothedMoisture;
  float temp;
  bool isAlert;
  bool isFault;
};

class TelemetryQueue {
private:
  TelemetryRecord buffer[QUEUE_MAX_SIZE];
  int head;
  int tail;
  int count;

public:
  TelemetryQueue() : head(0), tail(0), count(0) {}

  bool enqueue(TelemetryRecord rec) {
    if (count >= QUEUE_MAX_SIZE) {
      // Buffer full, drop oldest (FIFO)
      head = (head + 1) % QUEUE_MAX_SIZE;
      count--;
    }
    buffer[tail] = rec;
    tail = (tail + 1) % QUEUE_MAX_SIZE;
    count++;
    return true;
  }

  TelemetryRecord dequeue() {
    TelemetryRecord rec = buffer[head];
    head = (head + 1) % QUEUE_MAX_SIZE;
    count--;
    return rec;
  }

  int size() const {
    return count;
  }

  bool isEmpty() const {
    return count == 0;
  }
};

TelemetryQueue telemetryBuffer;

// ============================================================================
// STATE MACHINE & SYSTEM STATE
// ============================================================================
enum ControllerState {
  STATE_SAFE,
  STATE_IRRIGATING,
  STATE_FAULT
};

ControllerState currentState = STATE_SAFE;
float currentSmoothedMoisture = 0.0;
float lastRawMoisture = -999.0;
int consecLowCount = 0;
int consecHighCount = 0;
int stuckCounter = 0;
bool networkAvailable = true;

// Hardware status outputs
bool pumpActive = false;
bool faultLEDActive = false;

// Timer variables
unsigned long lastSampleTime = 0;
unsigned long simulatedTimeMs = 0; // Incremented by 2000ms each loop in simulation

// ============================================================================
// SIMULATION DATA SEQUENCE
// ============================================================================
struct SimReading {
  float rawMoisture;
  float temperature;
  bool networkOnline;
  const char* phaseName;
};

const SimReading simData[] = {
  // --- NORMAL PHASE ---
  {44.7, 25.0, true, "NORMAL"},    // t=2000
  {46.1, 25.0, true, "NORMAL"},    // t=4000
  {43.2, 25.0, true, "NORMAL"},    // t=6000
  {45.0, 25.0, true, "NORMAL"},    // t=8000
  {44.8, 25.0, true, "NORMAL"},    // t=10000
  {45.2, 25.0, true, "NORMAL"},    // t=12000
  {45.1, 25.0, true, "NORMAL"},    // t=14000
  {44.9, 25.0, true, "NORMAL"},    // t=16000
  {45.0, 25.0, true, "NORMAL"},    // t=18000
  {45.3, 25.0, true, "NORMAL"},    // t=20000
  {44.6, 25.0, true, "NORMAL"},    // t=22000
  {44.8, 25.0, true, "NORMAL"},    // t=24000
  {45.1, 25.0, true, "NORMAL"},    // t=26000
  {44.7, 25.0, true, "NORMAL"},    // t=28000
  {45.5, 25.0, true, "NORMAL"},    // t=30000
  
  // --- EXCURSION PHASE (Dry Event) ---
  {18.2, 25.0, true, "EXCURSION (genuine dry event)"}, // t=32000 (Low 1)
  {17.6, 25.0, true, "EXCURSION (genuine dry event)"}, // t=34000 (Low 2)
  {18.9, 25.0, true, "EXCURSION (genuine dry event)"}, // t=36000 (Low 3 -> IRRIGATING, Pump ON)
  {18.4, 25.0, true, "EXCURSION (genuine dry event)"}, // t=38000
  {18.2, 25.0, true, "EXCURSION (genuine dry event)"}, // t=40000
  {18.0, 25.0, true, "EXCURSION (genuine dry event)"}, // t=42000
  {18.3, 25.0, true, "EXCURSION (genuine dry event)"}, // t=44000
  {18.1, 25.0, true, "EXCURSION (genuine dry event)"}, // t=46000
  {18.2, 25.0, true, "EXCURSION (genuine dry event)"}, // t=48000
  {18.0, 25.0, true, "EXCURSION (genuine dry event)"}, // t=50000
  {18.1, 25.0, true, "EXCURSION (genuine dry event)"}, // v=52000
  {17.9, 25.0, true, "EXCURSION (genuine dry event)"}, // t=54000
  {18.2, 25.0, true, "EXCURSION (genuine dry event)"}, // t=56000
  {18.0, 25.0, true, "EXCURSION (genuine dry event)"}, // t=58000
  {18.1, 25.0, true, "EXCURSION (genuine dry event)"}, // t=60000
  
  // --- NOISY PHASE (spikes + stuck sensor prep) ---
  {44.2, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=62000
  {45.8, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=64000
  {97.0, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=66000 (Implausible spike)
  {44.6, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=68000
  {45.0, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=70000
  {44.6, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=72000
  {-5.0, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=74000 (Implausible low spike)
  {45.1, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=76000
  {45.0, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=78000
  {45.2, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=80000
  {45.0, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=82000
  {44.8, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=84000
  {44.5, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=86000
  {44.2, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=88000
  {44.3, 25.0, true, "NOISY (spikes + 1 stuck sample)"}, // t=90000
  
  // --- STUCK SENSOR PHASE ---
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=92000 (Stuck count=1)
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=94000 (Stuck count=2)
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=96000 (Stuck count=3)
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=98000 (Stuck count=4)
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=100000 (Stuck count=5 -> Fault)
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=102000
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=104000
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=106000
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=108000
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=110000
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=112000
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=114000
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=116000
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=118000
  {45.0, 25.0, true, "STUCK SENSOR"}, // t=120000
  
  // --- NETWORK OUTAGE PHASE (Simulate offline buffering) ---
  {44.1, 25.0, false, "NETWORK OUTAGE"}, // t=122000 (buffered 1)
  {45.6, 25.0, false, "NETWORK OUTAGE"}, // t=124000 (buffered 2)
  {44.8, 25.0, false, "NETWORK OUTAGE"}, // t=126000 (buffered 3)
  {45.1, 25.0, false, "NETWORK OUTAGE"}, // t=128000 (buffered 4)
  {44.9, 25.0, false, "NETWORK OUTAGE"}, // t=130000 (buffered 5)
  {45.3, 25.0, false, "NETWORK OUTAGE"}, // t=132000 (buffered 6)
  {44.7, 25.0, false, "NETWORK OUTAGE"}, // t=134000 (buffered 7)
  {44.5, 25.0, false, "NETWORK OUTAGE"}, // t=136000 (buffered 8)
  {45.2, 25.0, false, "NETWORK OUTAGE"}, // t=138000 (buffered 9)
  {45.0, 25.0, false, "NETWORK OUTAGE"}, // t=140000 (buffered 10)
  {44.6, 25.0, false, "NETWORK OUTAGE"}, // t=142000 (buffered 11)
  {44.8, 25.0, false, "NETWORK OUTAGE"}, // t=144000 (buffered 12)
  {45.1, 25.0, false, "NETWORK OUTAGE"}, // t=146000 (buffered 13)
  {44.7, 25.0, false, "NETWORK OUTAGE"}, // t=148000 (buffered 14)
  {44.9, 25.0, false, "NETWORK OUTAGE"}, // t=150000 (buffered 15)
  
  // --- RETURN TO NORMAL ---
  {45.2, 25.0, true, "NORMAL"} // t=152000 (live again, backlog flushed)
};

const int simLength = sizeof(simData) / sizeof(simData[0]);
int simIndex = 0;
const char* lastPhaseName = "";

// ============================================================================
// HELPER LOGGING & ACTUATOR CONTROL FUNCTIONS
// ============================================================================
void printBootHeader() {
  Serial.println(F("====================================================="));
  Serial.println(F(" SOIL IRRIGATION CONTROLLER - BOOT OK"));
  Serial.print(F(" Sampling every "));
  Serial.print(SAMPLE_INTERVAL_MS);
  Serial.print(F(" ms | Safe range "));
  Serial.print(MOISTURE_MIN_SAFE, 0);
  Serial.print(F("-"));
  Serial.print(MOISTURE_MAX_SAFE, 0);
  Serial.print(F("% | N="));
  Serial.print(DEBOUNCE_SAMPLES);
  Serial.println(F(" consec."));
  Serial.println(F("====================================================="));
}

void printTelemetry(const char* prefix, unsigned long timeMs, float moisture, float temp, bool alert, bool fault) {
  Serial.print(prefix);
  Serial.print(F("  t="));
  Serial.print(timeMs);
  Serial.print(F("ms  moisture="));
  Serial.print(moisture, 1);
  Serial.print(F("%  temp="));
  Serial.print(temp, 1);
  Serial.print(F("C  alert="));
  Serial.print(alert ? F("YES") : F("no"));
  Serial.print(F("  fault="));
  Serial.println(fault ? F("YES") : F("no"));
}

void setPump(bool active) {
  if (pumpActive != active) {
    pumpActive = active;
    digitalWrite(PUMP_PIN, pumpActive ? HIGH : LOW);
    Serial.print(F("[ACTUATOR] Pump -> "));
    Serial.println(pumpActive ? F("ON") : F("OFF"));
  }
}

void setFaultLED(bool active) {
  if (faultLEDActive != active) {
    faultLEDActive = active;
    digitalWrite(FAULT_LED_PIN, faultLEDActive ? HIGH : LOW);
  }
}

// Read moisture from analog sensor and convert to percentage (Hardware Mode)
float readAnalogMoisture() {
  int rawADC = analogRead(MOISTURE_PIN);
  // Calibrated maps (change mapping points to match your sensor type)
  float percentage = map(rawADC, 800, 200, 0, 100);
  return constrain(percentage, -10.0, 110.0);
}

// Read temperature from LM35/analog temperature sensor (Hardware Mode)
float readAnalogTemp() {
  int rawADC = analogRead(TEMP_PIN);
  float millivolts = (rawADC / 1024.0) * 5000.0;
  return millivolts / 10.0; 
}

// ============================================================================
// MAIN SYSTEM SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SOIL SYSTEM");

  lcd.setCursor(0,1);
  lcd.print("BOOT OK");

  delay(2000);

  lcd.clear();
  while (!Serial) {
    ; // Wait for Serial connection
  }

  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FAULT_LED_PIN, OUTPUT);
  
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(FAULT_LED_PIN, LOW);
  
  printBootHeader();
  lastSampleTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Periodic sample interval checker
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = currentTime;
    
    float rawMoisture = 0.0;
    float currentTemp = 25.0; 
    
    // Select sensor input based on Mode
    if (RUN_SIMULATION) {
      if (simIndex < simLength) {
        simulatedTimeMs += SAMPLE_INTERVAL_MS;
        
        // Output new test phase title headers
        if (strcmp(simData[simIndex].phaseName, lastPhaseName) != 0) {
          lastPhaseName = simData[simIndex].phaseName;
          Serial.println();
          Serial.print(F("----- TEST PHASE: "));
          Serial.print(lastPhaseName);
          Serial.println(F(" -----"));
        }
        
        rawMoisture = simData[simIndex].rawMoisture;
        currentTemp = simData[simIndex].temperature;
        networkAvailable = simData[simIndex].networkOnline;
        simIndex++;
      } else {
        // Reset and repeat simulation sequence infinitely
        simIndex = 0;
        simulatedTimeMs = 0;
        currentState = STATE_SAFE;
        currentSmoothedMoisture = 0.0;
        lastRawMoisture = -999.0;
        consecLowCount = 0;
        consecHighCount = 0;
        stuckCounter = 0;
        networkAvailable = true;
        setPump(false);
        setFaultLED(false);
        Serial.println();
        Serial.println(F("----- RESTARTING DIAGNOSTIC SIMULATION SEQUENCE -----"));
        return;
      }
    } else {
      // Hardware Mode - Reads from external sensors
      simulatedTimeMs = currentTime;
      rawMoisture = readAnalogMoisture();
      currentTemp = readAnalogTemp();
      networkAvailable = true; 
    }

    // ------------------------------------------------------------------------
    // FAULT STAGE 1: Check Implausible Out-of-Bounds Readings
    // ------------------------------------------------------------------------
    if (rawMoisture < PLAUSIBLE_MIN || rawMoisture > PLAUSIBLE_MAX) {
      Serial.print(F("[FAULT] raw="));
      Serial.print(rawMoisture, 1);
      Serial.println(F("  reason: implausible value"));
      return; // Discard reading entirely
    }

    // ------------------------------------------------------------------------
    // FAULT STAGE 2: Stuck Value Detection
    // ------------------------------------------------------------------------
    if (lastRawMoisture != -999.0 && abs(rawMoisture - lastRawMoisture) < 0.01) {
      stuckCounter++;
    } else {
      stuckCounter = 0;
    }
    lastRawMoisture = rawMoisture;

    if (stuckCounter >= STUCK_LIMIT - 1) { 
      currentState = STATE_FAULT;
      setPump(false);     // Shut pump down immediately as fail-safe
      setFaultLED(true);  // Solid fault indicator
      Serial.print(F("[FAULT] raw="));
      Serial.print(rawMoisture, 1);
      Serial.println(F("  reason: stuck value"));
      return; 
    } else {
      if (currentState == STATE_FAULT) {
        currentState = STATE_SAFE;
        setFaultLED(false);
      }
    }

    // ------------------------------------------------------------------------
    // PROCESS: Smoothing EMA filter
    // ------------------------------------------------------------------------
    if (currentSmoothedMoisture == 0.0) {
      currentSmoothedMoisture = rawMoisture;
    } else {
      currentSmoothedMoisture = (EMA_ALPHA * rawMoisture) + ((1.0 - EMA_ALPHA) * currentSmoothedMoisture);
    }

    // ------------------------------------------------------------------------
    // PROCESS: State machine & Pump Hysteresis control
    // ------------------------------------------------------------------------
    if (rawMoisture < MOISTURE_MIN_SAFE) {
      consecLowCount++;
      consecHighCount = 0;
    } else if (rawMoisture > MOISTURE_MAX_SAFE) {
      consecHighCount++;
      consecLowCount = 0;
    } else {
      consecLowCount = 0;
      consecHighCount = 0;
    }

    if (currentState == STATE_SAFE) {
      if (consecLowCount >= DEBOUNCE_SAMPLES) {
        currentState = STATE_IRRIGATING;
        setPump(true);
      }
    } else if (currentState == STATE_IRRIGATING) {
      // Recovery threshold turns the pump back off
      if (currentSmoothedMoisture >= RECOVERY_THRESHOLD) {
        currentState = STATE_SAFE;
        setPump(false);
        consecLowCount = 0;
      }
    }

    // Log the current raw status
    const char* stateStr = "SAFE";
    if (currentState == STATE_IRRIGATING) stateStr = "IRRIGATING";
    else if (currentState == STATE_FAULT) stateStr = "FAULT";

    Serial.print(F("[READ] raw="));
    Serial.print(rawMoisture, 1);
    Serial.print(F("%  smoothed="));
    Serial.print(currentSmoothedMoisture, 1);
    Serial.print(F("%  temp="));
    Serial.print(currentTemp, 1);
    Serial.print(F("C  consecLow="));
    Serial.print(consecLowCount);
    Serial.print(F(" consecHigh="));
    Serial.print(consecHighCount);
    Serial.print(F("  state="));
    Serial.println(stateStr);

    // ------------------------------------------------------------------------
    // PROCESS: Telemetry buffering & network output
    // ------------------------------------------------------------------------
    TelemetryRecord rec;
    rec.timestampMs = simulatedTimeMs;
    rec.rawMoisture = rawMoisture;
    rec.smoothedMoisture = currentSmoothedMoisture;
    rec.temp = currentTemp;
    rec.isAlert = (currentState == STATE_IRRIGATING);
    rec.isFault = (currentState == STATE_FAULT);

    if (networkAvailable) {
      // If there are buffered records, flush them in chronological order
      if (!telemetryBuffer.isEmpty()) {
        while (!telemetryBuffer.isEmpty()) {
          TelemetryRecord bufferedRec = telemetryBuffer.dequeue();
          printTelemetry("[FLUSH]", bufferedRec.timestampMs, bufferedRec.smoothedMoisture, bufferedRec.temp, bufferedRec.isAlert, bufferedRec.isFault);
        }
        Serial.println(F("[NET] Backlog fully flushed, oldest-first."));
      }
      printTelemetry("[LIVE]", rec.timestampMs, rec.smoothedMoisture, rec.temp, rec.isAlert, rec.isFault);
    } else {
      // Network is down: Buffer reading and display buffer status
      telemetryBuffer.enqueue(rec);
      Serial.print(F("[NET] Offline - buffered (queue size="));
      Serial.print(telemetryBuffer.size());
      Serial.println(F(")"));
    }
  }
}



