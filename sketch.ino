/**
   @file smart_greenhouse.ino
   @brief ESP32 Automated Commercial Micro-Climate Nursery.

   Scenario 2 - COMP50069

   System modes:
   1. Autonomous
   2. Manual Override
   3. Safety / Error

   Priority:
   SAFETY > MANUAL > AUTO

   Hardware:
   - ESP32
   - DHT22
   - LDR analogue light input
   - Two grow-light LEDs
   - SG90 servo vent
   - Pushbutton
   - SSD1306 I2C OLED

   The normal control loop uses non-blocking millis()-based
   scheduling and a GPIO interrupt for the Manual Override button.
*/

#include <Wire.h>
#include "sensor_config.h"
#include <DHT.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


/* ============================================================
   PIN DEFINITIONS
   ============================================================ */

#define DHT_PIN       33
#define DHT_TYPE      DHT22

#define LDR_PIN       34

#define LED1_PIN      16
#define LED2_PIN      17

#define SERVO_PIN     18
#define BUTTON_PIN    19

#define OLED_SDA      21
#define OLED_SCL      22


/* ============================================================
   OLED
   ============================================================ */

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
  );


/* ============================================================
   HARDWARE OBJECTS
   ============================================================ */

DHT dht(DHT_PIN, DHT_TYPE);

Servo ventServo;


/* ============================================================
   OPERATING MODES
   ============================================================ */

enum SystemMode
{
  MODE_AUTO,
  MODE_MANUAL,
  MODE_SAFE
};

SystemMode currentMode = MODE_SAFE;

bool manualRequested = false;


/* ============================================================
   LDR SETTINGS
   ============================================================ */

/*
   Physical prototype calibration:

   Very dark ≈ 16
   Typical brighter readings > 1000

   Hysteresis:

   below 400 -> grow lights ON
   above 700 -> grow lights OFF
*/

const int LIGHT_ON_THRESHOLD  = 400;
const int LIGHT_OFF_THRESHOLD = 700;

const int LDR_CAL_DARK   = 16;
const int LDR_CAL_BRIGHT = 1300;

bool growLightsOn = false;


/* ============================================================
   TEMPERATURE / VENT SETTINGS
   ============================================================ */

/*
   This value can be modified through UART.
*/

float ventStartTemperature = 28.0;

/*
   At this temperature the vent reaches the
   maximum opening position.
*/

const float VENT_FULL_TEMPERATURE = 35.0;


/*
   Physical vent calibration.

   Change these only if your real greenhouse linkage
   needs a different mechanical range.
*/

const int VENT_CLOSED_ANGLE = 0;
const int VENT_OPEN_ANGLE   = 90;


/*
   Sensor-fault safe posture.
*/

const int SAFE_VENT_ANGLE = 45;


/* ============================================================
   SMOOTH SERVO CONTROL
   ============================================================ */

int currentVentAngle = SAFE_VENT_ANGLE;
int targetVentAngle  = SAFE_VENT_ANGLE;

unsigned long lastServoStep = 0;

/*
   40 ms per degree:
   approximately 3.6 seconds for 0 -> 90 degrees.
*/

const unsigned long SERVO_STEP_NORMAL_MS = 40;

/*
   Safety response moves more quickly.
*/

const unsigned long SERVO_STEP_SAFE_MS = 15;


/* ============================================================
   SENSOR VALUES
   ============================================================ */

float temperature = 0.0;
float humidity = 0.0;

// Actual ADC voltage reading, retained for diagnostics and low-input faults.
int lightAdcRaw = 0;
// Calibrated control reading: small = dark, large = bright in both profiles.
int lightRaw = 0;
int lightPercent = 0;


/* ============================================================
   SENSOR VALIDITY
   ============================================================ */

bool dhtFault = false;
bool ldrFault = false;

bool dhtHasValidReading = false;


/* ============================================================
   LDR FAULT DETECTION
   ============================================================ */

/*
   On the real circuit a disconnected LDR supply causes
   GPIO34 to be pulled close to 0 through the 10 kΩ resistor.

   Actual dark reading was approximately 16, therefore <= 5
   is treated as a likely wiring/sensor failure.
*/

const int LDR_FAULT_LEVEL = 5;

const int LDR_FAULT_COUNT = 5;
const int LDR_RECOVERY_COUNT = 5;

int ldrFaultCounter = 0;
int ldrRecoveryCounter = 0;


/* ============================================================
   BUTTON INTERRUPT
   ============================================================ */

volatile bool buttonInterruptFlag = false;

unsigned long lastButtonHandled = 0;

const unsigned long BUTTON_DEBOUNCE_MS = 250;


/* ============================================================
   NON-BLOCKING TASK INTERVALS
   ============================================================ */

unsigned long lastLDRTime = 0;
unsigned long lastDHTTime = 0;
unsigned long lastOLEDTime = 0;
unsigned long lastSerialTime = 0;

const unsigned long LDR_INTERVAL_MS = 200;
const unsigned long DHT_INTERVAL_MS = 2000;
const unsigned long OLED_INTERVAL_MS = 500;
const unsigned long SERIAL_INTERVAL_MS = 2000;


/* ============================================================
   BUTTON ISR
   ============================================================ */

/**
   @brief Interrupt service routine for the Manual Override button.
   @param None This ISR has no parameters.
   @return Nothing.
*/
void IRAM_ATTR buttonISR()
{
  /*
     Keep interrupt service routines short.

     Do not:
     - write to OLED
     - move servo
     - print Serial

     Only notify the main program.
  */

  buttonInterruptFlag = true;
}


/* ============================================================
   MODE TEXT
   ============================================================ */

/**
   @brief Converts a mode value into readable text.
   @param mode System mode to convert.
   @return Text representation of the system mode.
*/
const char* getModeText(SystemMode mode)
{
  switch (mode)
  {
    case MODE_AUTO:
      return "AUTO";

    case MODE_MANUAL:
      return "MANUAL";

    case MODE_SAFE:
      return "SAFETY";

    default:
      return "UNKNOWN";
  }
}


/* ============================================================
   LIGHT PERCENTAGE
   ============================================================ */

/**
   @brief Converts the raw ADC light value into an approximate percentage.
   @param raw Raw ADC reading from GPIO34.
   @return Approximate light percentage from 0 to 100.
*/
int calculateLightPercent(int raw)
{
  long result = map(
                  raw,
                  LDR_CAL_DARK,
                  LDR_CAL_BRIGHT,
                  0,
                  100
                );

  return constrain(
           (int)result,
           0,
           100
         );
}


/* ============================================================
   VENT CALCULATION
   ============================================================ */

/**
   @brief Calculates the required greenhouse vent angle from temperature.
   @param temp Current greenhouse temperature in degrees Celsius.
   @return Calculated target servo angle.
*/
int calculateVentAngle(float temp)
{
  /*
     Temperature below threshold:
     vent closed.
  */

  if (temp <= ventStartTemperature)
  {
    return VENT_CLOSED_ANGLE;
  }


  /*
     Temperature at or above full-open threshold.
  */

  if (temp >= VENT_FULL_TEMPERATURE)
  {
    return VENT_OPEN_ANGLE;
  }


  /*
     Proportional region.

     fraction =
       (current temp - starting temp)
       --------------------------------
       (full-open temp - starting temp)
  */

  float fraction =
    (temp - ventStartTemperature) /
    (VENT_FULL_TEMPERATURE -
     ventStartTemperature);


  int angle =
    VENT_CLOSED_ANGLE +
    (int)(
      fraction *
      (VENT_OPEN_ANGLE -
       VENT_CLOSED_ANGLE)
    );


  return constrain(
           angle,
           VENT_CLOSED_ANGLE,
           VENT_OPEN_ANGLE
         );
}


/* ============================================================
   GROW LIGHT OUTPUT
   ============================================================ */

/**
   @brief Controls both supplemental grow-light LEDs.
   @param state true turns both LEDs on, false turns them off.
   @return Nothing.
*/
void setGrowLights(bool state)
{
  growLightsOn = state;

  digitalWrite(
    LED1_PIN,
    state ? HIGH : LOW
  );

  digitalWrite(
    LED2_PIN,
    state ? HIGH : LOW
  );
}


/* ============================================================
   INTERRUPT PROCESSING
   ============================================================ */

/**
   @brief Processes a Manual Override button event outside the ISR.
   @param now Current millis() timestamp.
   @return Nothing.
*/
void handleButtonInterrupt(unsigned long now)
{
  if (!buttonInterruptFlag)
  {
    return;
  }


  buttonInterruptFlag = false;


  /*
     Software debounce.
  */

  if (now - lastButtonHandled <
      BUTTON_DEBOUNCE_MS)
  {
    return;
  }


  lastButtonHandled = now;


  /*
     Safety has higher priority than Manual Override.
  */

  if (currentMode == MODE_SAFE)
  {
    Serial.println(
      "BUTTON IGNORED: SAFETY MODE ACTIVE"
    );

    return;
  }


  manualRequested =
    !manualRequested;


  if (manualRequested)
  {
    Serial.println();
    Serial.println(
      "INTERRUPT: MANUAL OVERRIDE REQUESTED"
    );
  }

  else
  {
    Serial.println();
    Serial.println(
      "INTERRUPT: AUTO MODE REQUESTED"
    );
  }
}


/* ============================================================
   LDR SAMPLING
   ============================================================ */

/**
   @brief Samples the analogue light sensor and detects wiring faults.
   @param now Current millis() timestamp.
   @return Nothing.
*/
void updateLDR(unsigned long now)
{
  if (now - lastLDRTime <
      LDR_INTERVAL_MS)
  {
    return;
  }


  lastLDRTime = now;


  lightAdcRaw = analogRead(LDR_PIN);

#if USE_WOKWI_LDR
  // Wokwi module voltage falls as illumination increases. Convert it to
  // the existing dark-to-bright control scale without changing GPIOs.
  // Keep valid darkness at 16, separate from the raw-ADC fault detector.
  lightRaw = LDR_CAL_DARK + (int)(
               (4095L - constrain(lightAdcRaw, 0, 4095)) *
               (LDR_CAL_BRIGHT - LDR_CAL_DARK) / 4095L
             );
#else
  // Physical divider described in the original prototype calibration.
  lightRaw = lightAdcRaw;
#endif


  lightPercent =
    calculateLightPercent(
      lightRaw
    );


  /*
     Fault detection.
  */

  if (lightAdcRaw <=
      LDR_FAULT_LEVEL)
  {
    ldrFaultCounter++;

    ldrRecoveryCounter = 0;


    if (ldrFaultCounter >=
        LDR_FAULT_COUNT)
    {
      ldrFault = true;
    }
  }

  else
  {
    ldrFaultCounter = 0;


    /*
       Require several valid readings before
       clearing an existing LDR fault.
    */

    if (ldrFault)
    {
      ldrRecoveryCounter++;


      if (ldrRecoveryCounter >=
          LDR_RECOVERY_COUNT)
      {
        ldrFault = false;

        ldrRecoveryCounter = 0;
      }
    }

    else
    {
      ldrRecoveryCounter = 0;
    }
  }
}


/* ============================================================
   DHT22 SAMPLING
   ============================================================ */

/**
   @brief Reads and validates DHT22 temperature and humidity measurements.
   @param now Current millis() timestamp.
   @return Nothing.
*/
void updateDHT(unsigned long now)
{
  if (now - lastDHTTime <
      DHT_INTERVAL_MS)
  {
    return;
  }


  lastDHTTime = now;


  float newHumidity =
    dht.readHumidity();

  float newTemperature =
    dht.readTemperature();


  /*
     Corrupt / invalid sensor reading checks.
  */

  bool invalid =
    isnan(newHumidity) ||
    isnan(newTemperature) ||
    newHumidity < 0.0 ||
    newHumidity > 100.0 ||
    newTemperature < -40.0 ||
    newTemperature > 80.0;


  if (invalid)
  {
    dhtFault = true;

    return;
  }


  /*
     Store values only when valid.
  */

  temperature =
    newTemperature;

  humidity =
    newHumidity;

  dhtFault = false;

  dhtHasValidReading = true;
}


/* ============================================================
   MODE PRIORITY
   ============================================================ */

/**
   @brief Selects the active mode using the project priority hierarchy.
   @param None This function has no parameters.
   @return Nothing.
*/
void determineMode()
{
  /*
     Wait for the first valid DHT22 measurement.
  */

  if (!dhtHasValidReading &&
      !dhtFault)
  {
    currentMode =
      MODE_SAFE;

    return;
  }


  /*
     PRIORITY 1:
     SAFETY / ERROR
  */

  if (dhtFault ||
      ldrFault)
  {
    /*
       A safety event cancels Manual Override.

       Therefore, after the fault is corrected,
       the controller returns to AUTO rather
       than automatically restoring a previous
       maintenance command.
    */

    manualRequested = false;

    currentMode =
      MODE_SAFE;

    return;
  }


  /*
     PRIORITY 2:
     MANUAL OVERRIDE
  */

  if (manualRequested)
  {
    currentMode =
      MODE_MANUAL;

    return;
  }


  /*
     PRIORITY 3:
     AUTONOMOUS
  */

  currentMode =
    MODE_AUTO;
}


/* ============================================================
   CONTROL DECISIONS
   ============================================================ */

/**
   @brief Calculates actuator targets for the current operating mode.
   @param None This function has no parameters.
   @return Nothing.
*/
void applyControlLogic()
{
  /* ========================================================
     SAFETY / ERROR
     ======================================================== */

  if (currentMode ==
      MODE_SAFE)
  {
    /*
       Predetermined safe physical posture.
    */

    setGrowLights(false);

    targetVentAngle =
      SAFE_VENT_ANGLE;

    return;
  }


  /* ========================================================
     MANUAL OVERRIDE
     ======================================================== */

  if (currentMode ==
      MODE_MANUAL)
  {
    /*
       Automated environmental routines are suspended.

       Maintenance posture:
       vent fully open.
    */

    setGrowLights(false);

    targetVentAngle =
      VENT_OPEN_ANGLE;

    return;
  }


  /* ========================================================
     AUTO - LIGHTING
     ======================================================== */

  /*
     Hysteresis:

     OFF -> ON below 400
     ON -> OFF above 700

     Values between 400 and 700 retain
     the previous lighting state.
  */

  if (!growLightsOn &&
      lightRaw <
      LIGHT_ON_THRESHOLD)
  {
    setGrowLights(true);
  }


  else if (growLightsOn &&
           lightRaw >
           LIGHT_OFF_THRESHOLD)
  {
    setGrowLights(false);
  }


  /* ========================================================
     AUTO - VENTILATION
     ======================================================== */

  if (dhtHasValidReading)
  {
    targetVentAngle =
      calculateVentAngle(
        temperature
      );
  }
}


/* ============================================================
   SMOOTH SERVO MOTION
   ============================================================ */

/**
   @brief Gradually moves the vent servo toward the calculated target.
   @param now Current millis() timestamp.
   @return Nothing.
*/
void updateServoSmoothly(unsigned long now)
{
  unsigned long interval;


  if (currentMode ==
      MODE_SAFE)
  {
    interval =
      SERVO_STEP_SAFE_MS;
  }

  else
  {
    interval =
      SERVO_STEP_NORMAL_MS;
  }


  if (now - lastServoStep <
      interval)
  {
    return;
  }


  lastServoStep = now;


  if (currentVentAngle <
      targetVentAngle)
  {
    currentVentAngle++;

    ventServo.write(
      currentVentAngle
    );
  }


  else if (currentVentAngle >
           targetVentAngle)
  {
    currentVentAngle--;

    ventServo.write(
      currentVentAngle
    );
  }
}


/* ============================================================
   UART HELP
   ============================================================ */

/**
   @brief Prints all UART operator commands.
   @param None This function has no parameters.
   @return Nothing.
*/
void printHelp()
{
  Serial.println();

  Serial.println(
    "========== UART COMMANDS =========="
  );

  Serial.println(
    "A = Autonomous mode"
  );

  Serial.println(
    "M = Manual Override"
  );

  Serial.println(
    "+ = Increase vent threshold by 1 C"
  );

  Serial.println(
    "- = Decrease vent threshold by 1 C"
  );

  Serial.println(
    "S = Print current system status"
  );

  Serial.println(
    "? = Display UART command help"
  );

  Serial.println(
    "==================================="
  );
}


/* ============================================================
   STATUS OUTPUT
   ============================================================ */

/**
   @brief Sends live greenhouse status to the UART terminal.
   @param None This function has no parameters.
   @return Nothing.
*/
void printStatus()
{
  Serial.println();

  Serial.println(
    "========== SYSTEM STATUS =========="
  );


  Serial.print(
    "MODE: "
  );

  Serial.println(
    getModeText(
      currentMode
    )
  );


  /*
     Sensor fault details.
  */

  if (dhtFault)
  {
    Serial.println(
      "FAULT: DHT22"
    );
  }


  if (ldrFault)
  {
    Serial.println(
      "FAULT: LDR"
    );
  }


  /*
     DHT values.
  */

  if (dhtHasValidReading)
  {
    Serial.print(
      "Temperature: "
    );

    Serial.print(
      temperature,
      1
    );

    Serial.println(
      " C"
    );


    Serial.print(
      "Humidity: "
    );

    Serial.print(
      humidity,
      1
    );

    Serial.println(
      " %"
    );
  }

  else
  {
    Serial.println(
      "DHT22: WAITING FOR VALID READING"
    );
  }


  /*
     LDR.
  */

  Serial.print(
    "LDR raw: "
  );

  Serial.println(lightAdcRaw);
  Serial.print("Light control: ");

  Serial.println(
    lightRaw
  );


  Serial.print(
    "Light level: "
  );

  Serial.print(
    lightPercent
  );

  Serial.println(
    " %"
  );


  /*
     Grow lighting.
  */

  Serial.print(
    "Grow LEDs: "
  );

  Serial.println(
    growLightsOn ?
    "ON" :
    "OFF"
  );


  /*
     Vent.
  */

  Serial.print(
    "Vent current: "
  );

  Serial.print(
    currentVentAngle
  );

  Serial.println(
    " degrees"
  );


  Serial.print(
    "Vent target: "
  );

  Serial.print(
    targetVentAngle
  );

  Serial.println(
    " degrees"
  );


  /*
     Configuration.
  */

  Serial.print(
    "Vent start threshold: "
  );

  Serial.print(
    ventStartTemperature,
    1
  );

  Serial.println(
    " C"
  );


  Serial.println(
    "==================================="
  );
}


/* ============================================================
   UART INPUT
   ============================================================ */

/**
   @brief Processes configuration and mode commands received through UART.
   @param None This function has no parameters.
   @return Nothing.
*/
void updateUART()
{
  while (Serial.available() > 0)
  {
    char command =
      Serial.read();


    /*
       Ignore line endings.
    */

    if (command == '\n' ||
        command == '\r' ||
        command == ' ')
    {
      continue;
    }


    /* --------------------------------------------------------
       AUTO COMMAND
       -------------------------------------------------------- */

    if (command == 'A' ||
        command == 'a')
    {
      if (currentMode ==
          MODE_SAFE)
      {
        Serial.println(
          "AUTO REJECTED: SAFETY MODE ACTIVE"
        );
      }

      else
      {
        manualRequested = false;

        Serial.println(
          "UART: AUTO MODE REQUESTED"
        );
      }
    }


    /* --------------------------------------------------------
       MANUAL COMMAND
       -------------------------------------------------------- */

    else if (command == 'M' ||
             command == 'm')
    {
      if (currentMode ==
          MODE_SAFE)
      {
        Serial.println(
          "MANUAL REJECTED: SAFETY MODE ACTIVE"
        );
      }

      else
      {
        manualRequested = true;

        Serial.println(
          "UART: MANUAL OVERRIDE REQUESTED"
        );
      }
    }


    /* --------------------------------------------------------
       INCREASE TEMPERATURE THRESHOLD
       -------------------------------------------------------- */

    else if (command == '+')
    {
      ventStartTemperature += 1.0;


      if (ventStartTemperature >
          32.0)
      {
        ventStartTemperature =
          32.0;
      }


      Serial.print(
        "Vent threshold = "
      );

      Serial.print(
        ventStartTemperature,
        1
      );

      Serial.println(
        " C"
      );
    }


    /* --------------------------------------------------------
       DECREASE TEMPERATURE THRESHOLD
       -------------------------------------------------------- */

    else if (command == '-')
    {
      ventStartTemperature -= 1.0;


      if (ventStartTemperature <
          22.0)
      {
        ventStartTemperature =
          22.0;
      }


      Serial.print(
        "Vent threshold = "
      );

      Serial.print(
        ventStartTemperature,
        1
      );

      Serial.println(
        " C"
      );
    }


    /* --------------------------------------------------------
       STATUS
       -------------------------------------------------------- */

    else if (command == 'S' ||
             command == 's')
    {
      printStatus();
    }


    /* --------------------------------------------------------
       HELP
       -------------------------------------------------------- */

    else if (command == '?')
    {
      printHelp();
    }


    /* --------------------------------------------------------
       UNKNOWN
       -------------------------------------------------------- */

    else
    {
      Serial.print(
        "Unknown command: "
      );

      Serial.println(
        command
      );

      Serial.println(
        "Type ? for help."
      );
    }
  }
}


/* ============================================================
   OLED
   ============================================================ */

/**
   @brief Updates the I2C OLED with live status or safety information.
   @param now Current millis() timestamp.
   @return Nothing.
*/
void updateOLED(unsigned long now)
{
  if (now - lastOLEDTime <
      OLED_INTERVAL_MS)
  {
    return;
  }


  lastOLEDTime = now;


  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(
    1
  );

  display.setCursor(
    0,
    0
  );


  /* ========================================================
     START-UP SENSOR VALIDATION
     ======================================================== */

  if (!dhtHasValidReading &&
      !dhtFault)
  {
    display.println(
      "SMART GREENHOUSE"
    );

    display.println();

    display.println(
      "Checking sensors..."
    );

    display.print(
      "LDR: "
    );

    display.println(
      lightRaw
    );

    display.print(
      "Safe Vent: "
    );

    display.print(
      currentVentAngle
    );

    display.println(
      " deg"
    );

    display.display();

    return;
  }


  /* ========================================================
     SAFETY / ERROR SCREEN
     ======================================================== */

  if (currentMode ==
      MODE_SAFE)
  {
    display.println(
      "*** SENSOR FAULT ***"
    );


    if (dhtFault)
    {
      display.println(
        "DHT22 FAILED"
      );
    }


    if (ldrFault)
    {
      display.println(
        "LDR FAILED"
      );
    }


    display.println(
      "CHECK SENSOR"
    );


    display.print(
      "Vent: "
    );

    display.print(
      currentVentAngle
    );

    display.print(
      "->"
    );

    display.println(
      targetVentAngle
    );


    display.println(
      "Grow lights: OFF"
    );


    display.display();

    return;
  }


  /* ========================================================
     AUTO / MANUAL SCREEN
     ======================================================== */

  display.print(
    "MODE: "
  );

  display.println(
    getModeText(
      currentMode
    )
  );


  display.print(
    "T:"
  );

  display.print(
    temperature,
    1
  );

  display.print(
    "C H:"
  );

  display.print(
    humidity,
    0
  );

  display.println(
    "%"
  );


  display.print(
    "Light:"
  );

  display.print(
    lightRaw
  );

  display.print(
    " ("
  );

  display.print(
    lightPercent
  );

  display.println(
    "%)"
  );


  display.print(
    "Grow:"
  );

  display.println(
    growLightsOn ?
    "ON" :
    "OFF"
  );


  display.print(
    "Vent:"
  );

  display.print(
    currentVentAngle
  );

  display.print(
    "->"
  );

  display.print(
    targetVentAngle
  );

  display.println(
    "deg"
  );


  display.print(
    "Tset:"
  );

  display.print(
    ventStartTemperature,
    0
  );

  display.println(
    "C"
  );


  display.display();
}


/* ============================================================
   SETUP
   ============================================================ */

/**
   @brief Initializes all greenhouse hardware and communications.
   @param None Arduino setup() has no parameters.
   @return Nothing.
*/
void setup()
{
  /* UART */

  Serial.begin(
    115200
  );


  /* DHT22 */

  dht.begin();


  /* ADC */

  analogReadResolution(
    12
  );


  /* Grow LEDs */

  pinMode(
    LED1_PIN,
    OUTPUT
  );

  pinMode(
    LED2_PIN,
    OUTPUT
  );

  setGrowLights(
    false
  );


  /* Manual Override button */

  pinMode(
    BUTTON_PIN,
    INPUT_PULLUP
  );


  /*
     GPIO interrupt:

     The button is normally HIGH because of
     INPUT_PULLUP.

     When pressed it connects GPIO19 to GND,
     creating a HIGH -> LOW falling edge.
  */

  attachInterrupt(
    digitalPinToInterrupt(
      BUTTON_PIN
    ),
    buttonISR,
    FALLING
  );


  /* Servo */

  ventServo.setPeriodHertz(
    50
  );


  ventServo.attach(
    SERVO_PIN,
    500,
    2400
  );


  /*
     Safe startup position.
  */

  currentVentAngle =
    SAFE_VENT_ANGLE;

  targetVentAngle =
    SAFE_VENT_ANGLE;

  ventServo.write(
    currentVentAngle
  );


  /* I2C */

  Wire.begin(
    OLED_SDA,
    OLED_SCL
  );


  /* OLED */

  if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        0x3C))
  {
    Serial.println(
      "FATAL ERROR: OLED NOT FOUND"
    );


    /*
       Fatal initialization stop.

       This is outside normal system operation,
       therefore it is not part of the scheduled
       non-blocking control loop.
    */

    while (true)
    {
    }
  }


  /* Initial OLED */

  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(
    1
  );

  display.setCursor(
    0,
    0
  );

  display.println(
    "SMART GREENHOUSE"
  );

  display.println(
    "Scenario 2"
  );

  display.println();

  display.println(
    "Checking sensors..."
  );

  display.display();


  /* Startup UART */

  Serial.println();

  Serial.println(
    "===================================="
  );

  Serial.println(
    "ESP32 SMART GREENHOUSE"
  );

  Serial.println(
    "SCENARIO 2"
  );

  Serial.println(
    "===================================="
  );

  Serial.println(
    "GPIO19 BUTTON INTERRUPT ACTIVE"
  );

  Serial.println(
    "NON-BLOCKING CONTROL ACTIVE"
  );

  Serial.println(
    "PRIORITY: SAFETY > MANUAL > AUTO"
  );


  printHelp();
}


/* ============================================================
   MAIN LOOP
   ============================================================ */

/**
   @brief Executes the real-time non-blocking greenhouse controller.
   @param None Arduino loop() has no parameters.
   @return Nothing.
*/
void loop()
{
  unsigned long now =
    millis();


  /*
     Process operator communication.
  */

  updateUART();


  /*
     Process any button interrupt event.
  */

  handleButtonInterrupt(
    now
  );


  /*
     Non-blocking sensor tasks.
  */

  updateLDR(
    now
  );

  updateDHT(
    now
  );


  /*
     Determine the highest-priority state.
  */

  determineMode();


  /*
     Calculate actuator targets.
  */

  applyControlLogic();


  /*
     Smooth PWM-controlled vent movement.
  */

  updateServoSmoothly(
    now
  );


  /*
     Update local I2C user interface.
  */

  updateOLED(
    now
  );


  /*
     Scheduled UART status message.
  */

  if (now - lastSerialTime >=
      SERIAL_INTERVAL_MS)
  {
    lastSerialTime = now;

    printStatus();
  }
}
