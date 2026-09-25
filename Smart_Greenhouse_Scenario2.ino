/**
 * @file Smart_Greenhouse_Scenario2.ino
 * @brief ESP32 Smart Greenhouse for COMP50069 Scenario 2.
 *
 * Main control relationship:
 *
 * DHT22 Temperature -> Vent servo
 * DHT22 Humidity    -> Monitoring / OLED / UART
 * LDR Light level   -> Grow LEDs
 * Push button       -> Manual Override
 * DHT22 failure     -> Safety Mode
 *
 * Priority:
 * SAFETY > MANUAL > AUTO
 */

#include <Wire.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


/* =====================================================
   PIN DEFINITIONS
   ===================================================== */

#define DHT_PIN 33
#define DHT_TYPE DHT22

#define LDR_PIN 34

#define LED1_PIN 16
#define LED2_PIN 17

#define SERVO_PIN 18

#define BUTTON_PIN 19

#define OLED_SDA 21
#define OLED_SCL 22


/* =====================================================
   OLED SETTINGS
   ===================================================== */

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);


/* =====================================================
   HARDWARE OBJECTS
   ===================================================== */

DHT dht(
  DHT_PIN,
  DHT_TYPE
);

Servo ventServo;


/* =====================================================
   SYSTEM MODES
   ===================================================== */

enum SystemMode
{
  MODE_AUTO,
  MODE_MANUAL,
  MODE_SAFE
};


/*
   System starts safely.

   AUTO is only entered after the ESP32
   receives a valid DHT22 reading.
*/

SystemMode currentMode =
  MODE_SAFE;

bool manualRequested =
  false;


/* =====================================================
   LIGHT CONTROL SETTINGS
   ===================================================== */

/*
   LDR controls ONLY the grow LEDs.

   LDR < 400:
   dark -> LEDs ON

   LDR > 700:
   bright -> LEDs OFF

   Between 400 and 700:
   retain previous state.

   This hysteresis prevents flickering.
*/

const int LIGHT_ON_THRESHOLD =
  400;

const int LIGHT_OFF_THRESHOLD =
  700;

bool growLightsOn =
  false;


/* =====================================================
   TEMPERATURE / VENT SETTINGS
   ===================================================== */

/*
   Temperature controls ONLY the vent.

   <= 28 C:
   vent closed

   28 C - 35 C:
   proportional opening

   >= 35 C:
   fully open
*/

float ventStartTemperature =
  28.0;

const float VENT_FULL_TEMPERATURE =
  35.0;


/*
   LOGICAL greenhouse vent positions.

   These values describe the greenhouse
   meaning shown on OLED / Serial:

   0 degrees   = fully CLOSED
   140 degrees = fully OPEN
   45 degrees  = Safety position

   The servo is physically mounted in reverse,
   so logical angles are converted before
   being sent to the servo.
*/

const int VENT_CLOSED_ANGLE =
  0;

const int VENT_OPEN_ANGLE =
  140;

const int SAFE_VENT_ANGLE =
  45;


/* =====================================================
   SENSOR VALUES
   ===================================================== */

float temperature =
  0.0;

float humidity =
  0.0;

int lightValue =
  0;


/* =====================================================
   DHT22 SAFETY STATUS
   ===================================================== */

/*
   DHT22 is used as the primary sensor
   for Safety fault detection.

   A disconnected DHT22 normally produces
   NaN readings.

   The LDR is NOT treated as faulty just
   because it reads near zero, because
   near-zero can represent genuine darkness.
*/

bool dhtFault =
  false;

bool dhtHasValidReading =
  false;


/* =====================================================
   BUTTON INTERRUPT
   ===================================================== */

volatile bool buttonInterruptFlag =
  false;

unsigned long lastButtonHandled =
  0;

const unsigned long BUTTON_DEBOUNCE =
  250;


/* =====================================================
   NON-BLOCKING TIMERS
   ===================================================== */

unsigned long lastDHTTime =
  0;

unsigned long lastLDRTime =
  0;

unsigned long lastOLEDTime =
  0;

unsigned long lastSerialTime =
  0;

unsigned long lastServoMoveTime =
  0;


const unsigned long DHT_INTERVAL =
  2000;

const unsigned long LDR_INTERVAL =
  200;

const unsigned long OLED_INTERVAL =
  500;

const unsigned long SERIAL_INTERVAL =
  2000;


/*
   Servo movement timing.

   Normal:
   one logical degree every 40 ms.

   Safety:
   one logical degree every 15 ms.
*/

const unsigned long SERVO_STEP_INTERVAL_NORMAL =
  40;

const unsigned long SERVO_STEP_INTERVAL_SAFE =
  15;


/* =====================================================
   SERVO STATE
   ===================================================== */

/*
   These are LOGICAL vent angles.

   They are not necessarily the physical
   command sent to the servo because the
   servo mounting is reversed.
*/

int currentVentAngle =
  SAFE_VENT_ANGLE;

int targetVentAngle =
  SAFE_VENT_ANGLE;


/* =====================================================
   BUTTON ISR
   ===================================================== */

/**
 * @brief Interrupt Service Routine for Manual Override button.
 * @param None
 * @return Nothing
 */
void IRAM_ATTR buttonISR()
{
  /*
     Keep ISR short.

     Only set a flag.
  */

  buttonInterruptFlag =
    true;
}


/* =====================================================
   MODE TEXT
   ===================================================== */

/**
 * @brief Returns readable text representing current system mode.
 * @param mode Current system operating mode.
 * @return AUTO, MANUAL or SAFETY.
 */
const char* getModeText(
  SystemMode mode
)
{
  if (
    mode ==
    MODE_MANUAL
  )
  {
    return "MANUAL";
  }

  if (
    mode ==
    MODE_SAFE
  )
  {
    return "SAFETY";
  }

  return "AUTO";
}


/* =====================================================
   SERVO DIRECTION CONVERSION
   ===================================================== */

/**
 * @brief Converts logical greenhouse angle to physical servo angle.
 *
 * Logical greenhouse meaning:
 *
 * 0 deg   = CLOSED
 * 140 deg = FULLY OPEN
 *
 * Physical servo mounting is reversed:
 *
 * Servo 140 deg = physical vent CLOSED
 * Servo 0 deg   = physical vent OPEN
 *
 * Therefore:
 *
 * Logical 0   -> physical servo 140
 * Logical 45  -> physical servo 95
 * Logical 70  -> physical servo 70
 * Logical 140 -> physical servo 0
 *
 * @param logicalAngle Logical greenhouse vent angle.
 * @return Physical servo command angle.
 */
int logicalToServoAngle(
  int logicalAngle
)
{
  return map(
    logicalAngle,
    VENT_CLOSED_ANGLE,
    VENT_OPEN_ANGLE,
    VENT_OPEN_ANGLE,
    VENT_CLOSED_ANGLE
  );
}


/* =====================================================
   VENT CALCULATION
   ===================================================== */

/**
 * @brief Calculates required logical vent angle using temperature only.
 * @param temp Current DHT22 temperature in degrees Celsius.
 * @return Logical vent angle from 0 to 140 degrees.
 */
int calculateVentAngle(
  float temp
)
{
  /*
     IMPORTANT:

     No LDR and no humidity value
     are used here.

     Vent is controlled ONLY by
     DHT22 temperature.
  */


  /*
     At or below threshold:
     vent CLOSED.
  */

  if (
    temp <=
    ventStartTemperature
  )
  {
    return
      VENT_CLOSED_ANGLE;
  }


  /*
     At or above 35 C:
     vent FULLY OPEN.
  */

  if (
    temp >=
    VENT_FULL_TEMPERATURE
  )
  {
    return
      VENT_OPEN_ANGLE;
  }


  /*
     Between threshold and 35 C:
     proportional opening.
  */

  float fraction =
    (
      temp -
      ventStartTemperature
    )
    /
    (
      VENT_FULL_TEMPERATURE -
      ventStartTemperature
    );


  int angle =
    VENT_CLOSED_ANGLE +
    (int)(
      fraction *
      (
        VENT_OPEN_ANGLE -
        VENT_CLOSED_ANGLE
      )
    );


  return constrain(
    angle,
    VENT_CLOSED_ANGLE,
    VENT_OPEN_ANGLE
  );
}


/* =====================================================
   GROW-LIGHT OUTPUT
   ===================================================== */

/**
 * @brief Controls both grow-light LEDs.
 * @param state true = ON, false = OFF.
 * @return Nothing
 */
void setGrowLights(
  bool state
)
{
  growLightsOn =
    state;


  digitalWrite(
    LED1_PIN,
    state ?
    HIGH :
    LOW
  );


  digitalWrite(
    LED2_PIN,
    state ?
    HIGH :
    LOW
  );
}


/* =====================================================
   BUTTON PROCESSING
   ===================================================== */

/**
 * @brief Handles Manual Override button event.
 * @param now Current millis() value.
 * @return Nothing
 */
void handleButtonInterrupt(
  unsigned long now
)
{
  if (
    !buttonInterruptFlag
  )
  {
    return;
  }


  buttonInterruptFlag =
    false;


  /*
     Debounce.
  */

  if (
    lastButtonHandled != 0 &&
    now -
    lastButtonHandled <
    BUTTON_DEBOUNCE
  )
  {
    return;
  }


  lastButtonHandled =
    now;


  /*
     Safety has highest priority.
  */

  if (
    currentMode ==
    MODE_SAFE
  )
  {
    Serial.println(
      "Button ignored: SAFETY / SENSOR CHECK active"
    );

    return;
  }


  /*
     Toggle AUTO / MANUAL.
  */

  manualRequested =
    !manualRequested;


  if (
    manualRequested
  )
  {
    Serial.println();

    Serial.println(
      "BUTTON: MANUAL OVERRIDE REQUESTED"
    );
  }

  else
  {
    Serial.println();

    Serial.println(
      "BUTTON: AUTO MODE REQUESTED"
    );
  }
}


/* =====================================================
   LDR READING
   ===================================================== */

/**
 * @brief Reads LDR analogue light level using ESP32 ADC.
 * @param now Current millis() value.
 * @return Nothing
 */
void updateLDR(
  unsigned long now
)
{
  if (
    now -
    lastLDRTime <
    LDR_INTERVAL
  )
  {
    return;
  }


  lastLDRTime =
    now;


  lightValue =
    analogRead(
      LDR_PIN
    );


  /*
     Low light is a valid environmental condition.

     It must NOT trigger Safety Mode.

     Covering the LDR should only affect
     the grow LEDs in AUTO.
  */
}


/* =====================================================
   DHT22 READING / SAFETY VALIDATION
   ===================================================== */

/**
 * @brief Reads and validates DHT22 temperature and humidity.
 * @param now Current millis() value.
 * @return Nothing
 */
void updateDHT(
  unsigned long now
)
{
  /*
     Sample DHT22 every 2 seconds.
  */

  if (
    now -
    lastDHTTime <
    DHT_INTERVAL
  )
  {
    return;
  }


  lastDHTTime =
    now;


  float newHumidity =
    dht.readHumidity();


  float newTemperature =
    dht.readTemperature();


  /*
     Detect invalid readings.
  */

  bool invalid =
    isnan(
      newHumidity
    )
    ||
    isnan(
      newTemperature
    )
    ||
    newHumidity <
    0.0
    ||
    newHumidity >
    100.0
    ||
    newTemperature <
    -40.0
    ||
    newTemperature >
    80.0;


  /*
     Invalid reading activates Safety.
  */

  if (
    invalid
  )
  {
    dhtFault =
      true;

    return;
  }


  /*
     Store valid readings.
  */

  temperature =
    newTemperature;


  humidity =
    newHumidity;


  bool wasFaulted =
    dhtFault;


  dhtFault =
    false;


  dhtHasValidReading =
    true;


  /*
     Recovery message.
  */

  if (
    wasFaulted
  )
  {
    Serial.println();

    Serial.println(
      "DHT22 RECOVERED - valid reading received"
    );
  }
}


/* =====================================================
   MODE PRIORITY
   ===================================================== */

/**
 * @brief Selects Safety, Manual or Auto.
 * @param None
 * @return Nothing
 */
void determineMode()
{
  /*
     PRIORITY:

     1. SAFETY
     2. MANUAL
     3. AUTO
  */


  /*
     Startup stays safe until first
     valid DHT22 reading.
  */

  if (
    !dhtHasValidReading
  )
  {
    currentMode =
      MODE_SAFE;

    return;
  }


  /*
     DHT22 fault overrides
     Manual and Auto.
  */

  if (
    dhtFault
  )
  {
    /*
       Cancel previous Manual request.

       Recovery returns to AUTO.
    */

    manualRequested =
      false;


    currentMode =
      MODE_SAFE;


    return;
  }


  /*
     Manual second priority.
  */

  if (
    manualRequested
  )
  {
    currentMode =
      MODE_MANUAL;

    return;
  }


  /*
     Otherwise AUTO.
  */

  currentMode =
    MODE_AUTO;
}


/* =====================================================
   MAIN CONTROL LOGIC
   ===================================================== */

/**
 * @brief Applies Safety, Manual or Auto greenhouse behaviour.
 * @param None
 * @return Nothing
 */
void applyControlLogic()
{
  /*
     =================================================
     1. SAFETY MODE
     =================================================
  */

  if (
    currentMode ==
    MODE_SAFE
  )
  {
    /*
       Grow lights OFF.
    */

    setGrowLights(
      false
    );


    /*
       Logical Safety angle = 45°.
    */

    targetVentAngle =
      SAFE_VENT_ANGLE;


    return;
  }


  /*
     =================================================
     2. MANUAL OVERRIDE
     =================================================
  */

  if (
    currentMode ==
    MODE_MANUAL
  )
  {
    /*
       Automation bypassed.
    */

    setGrowLights(
      false
    );


    /*
       Logical 140° means FULLY OPEN.

       Servo direction conversion will
       physically send the reversed command.
    */

    targetVentAngle =
      VENT_OPEN_ANGLE;


    return;
  }


  /*
     =================================================
     3. AUTO MODE
     =================================================
  */


  /*
     -----------------------------------------
     AUTO LIGHT CONTROL

     LDR -> LEDs ONLY
     -----------------------------------------
  */

  if (
    !growLightsOn
    &&
    lightValue <
    LIGHT_ON_THRESHOLD
  )
  {
    setGrowLights(
      true
    );
  }


  else if (
    growLightsOn
    &&
    lightValue >
    LIGHT_OFF_THRESHOLD
  )
  {
    setGrowLights(
      false
    );
  }


  /*
     400-700:
     retain previous LED state.
  */


  /*
     -----------------------------------------
     AUTO VENT CONTROL

     Temperature -> Vent ONLY
     -----------------------------------------
  */

  targetVentAngle =
    calculateVentAngle(
      temperature
    );
}


/* =====================================================
   SERVO MOVEMENT
   ===================================================== */

/**
 * @brief Moves vent gradually toward target.
 * @param now Current millis() value.
 * @return Nothing
 */
void updateServoSmoothly(
  unsigned long now
)
{
  unsigned long stepInterval;


  if (
    currentMode ==
    MODE_SAFE
  )
  {
    stepInterval =
      SERVO_STEP_INTERVAL_SAFE;
  }

  else
  {
    stepInterval =
      SERVO_STEP_INTERVAL_NORMAL;
  }


  if (
    now -
    lastServoMoveTime <
    stepInterval
  )
  {
    return;
  }


  lastServoMoveTime =
    now;


  /*
     Increase LOGICAL angle
     toward open.
  */

  if (
    currentVentAngle <
    targetVentAngle
  )
  {
    currentVentAngle++;


    /*
       Convert logical angle into
       reversed physical servo angle.
    */

    ventServo.write(
      logicalToServoAngle(
        currentVentAngle
      )
    );
  }


  /*
     Decrease LOGICAL angle
     toward closed.
  */

  else if (
    currentVentAngle >
    targetVentAngle
  )
  {
    currentVentAngle--;


    ventServo.write(
      logicalToServoAngle(
        currentVentAngle
      )
    );
  }
}


/* =====================================================
   UART HELP
   ===================================================== */

/**
 * @brief Prints available UART commands.
 * @param None
 * @return Nothing
 */
void printHelp()
{
  Serial.println();

  Serial.println(
    "========== UART COMMANDS =========="
  );

  Serial.println(
    "A = AUTO mode"
  );

  Serial.println(
    "M = MANUAL Override"
  );

  Serial.println(
    "+ = Increase vent threshold by 1 C"
  );

  Serial.println(
    "- = Decrease vent threshold by 1 C"
  );

  Serial.println(
    "S = Print full status"
  );

  Serial.println(
    "? = Show help"
  );

  Serial.println(
    "==================================="
  );
}


/* =====================================================
   UART STATUS
   ===================================================== */

/**
 * @brief Prints complete greenhouse status.
 * @param None
 * @return Nothing
 */
void printStatus()
{
  Serial.println();

  Serial.println(
    "========== SYSTEM STATUS =========="
  );


  /*
     MODE
  */

  if (
    currentMode ==
    MODE_SAFE
    &&
    !dhtHasValidReading
    &&
    !dhtFault
  )
  {
    Serial.println(
      "MODE: SAFETY / STARTUP"
    );
  }


  else if (
    currentMode ==
    MODE_SAFE
  )
  {
    Serial.println(
      "MODE: SAFETY / ERROR"
    );
  }


  else if (
    currentMode ==
    MODE_MANUAL
  )
  {
    Serial.println(
      "MODE: MANUAL OVERRIDE"
    );
  }


  else
  {
    Serial.println(
      "MODE: AUTO"
    );
  }


  /*
     Startup status.
  */

  if (
    !dhtHasValidReading
    &&
    !dhtFault
  )
  {
    Serial.println(
      "STATUS: Waiting for first valid DHT22 reading"
    );
  }


  /*
     Sensor fault.
  */

  if (
    dhtFault
  )
  {
    Serial.println(
      "FAULT: DHT22"
    );
  }


  /*
     Temperature.
  */

  Serial.print(
    "Temperature: "
  );


  if (
    dhtHasValidReading
    &&
    !dhtFault
  )
  {
    Serial.print(
      temperature,
      1
    );

    Serial.println(
      " C"
    );
  }

  else
  {
    Serial.println(
      "--"
    );
  }


  /*
     Humidity.
  */

  Serial.print(
    "Humidity: "
  );


  if (
    dhtHasValidReading
    &&
    !dhtFault
  )
  {
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
      "--"
    );
  }


  /*
     LDR ADC.
  */

  Serial.print(
    "Light ADC: "
  );

  Serial.println(
    lightValue
  );


  /*
     Grow lights.
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
     Logical vent angle.
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


  /*
     Logical target.
  */

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
     Temperature threshold.
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


/* =====================================================
   UART INPUT
   ===================================================== */

/**
 * @brief Handles UART commands.
 * @param None
 * @return Nothing
 */
void updateUART()
{
  while (
    Serial.available() >
    0
  )
  {
    char command =
      Serial.read();


    /*
       Ignore line endings and spaces.
    */

    if (
      command == '\n'
      ||
      command == '\r'
      ||
      command == ' '
    )
    {
      continue;
    }


    /*
       AUTO
    */

    if (
      command == 'A'
      ||
      command == 'a'
    )
    {
      if (
        currentMode ==
        MODE_SAFE
      )
      {
        Serial.println(
          "AUTO rejected: SAFETY / SENSOR CHECK active"
        );
      }

      else
      {
        manualRequested =
          false;


        Serial.println(
          "UART: AUTO MODE REQUESTED"
        );
      }
    }


    /*
       MANUAL
    */

    else if (
      command == 'M'
      ||
      command == 'm'
    )
    {
      if (
        currentMode ==
        MODE_SAFE
      )
      {
        Serial.println(
          "MANUAL rejected: SAFETY / SENSOR CHECK active"
        );
      }

      else
      {
        manualRequested =
          true;


        Serial.println(
          "UART: MANUAL OVERRIDE REQUESTED"
        );
      }
    }


    /*
       Increase temperature threshold.
    */

    else if (
      command == '+'
    )
    {
      ventStartTemperature +=
        1.0;


      if (
        ventStartTemperature >
        32.0
      )
      {
        ventStartTemperature =
          32.0;
      }


      Serial.print(
        "Vent threshold increased to "
      );

      Serial.print(
        ventStartTemperature,
        1
      );

      Serial.println(
        " C"
      );
    }


    /*
       Decrease temperature threshold.
    */

    else if (
      command == '-'
    )
    {
      ventStartTemperature -=
        1.0;


      if (
        ventStartTemperature <
        22.0
      )
      {
        ventStartTemperature =
          22.0;
      }


      Serial.print(
        "Vent threshold decreased to "
      );

      Serial.print(
        ventStartTemperature,
        1
      );

      Serial.println(
        " C"
      );
    }


    /*
       Status.
    */

    else if (
      command == 'S'
      ||
      command == 's'
    )
    {
      printStatus();
    }


    /*
       Help.
    */

    else if (
      command == '?'
    )
    {
      printHelp();
    }


    /*
       Unknown command.
    */

    else
    {
      Serial.print(
        "Unknown command: "
      );

      Serial.println(
        command
      );
    }
  }
}


/* =====================================================
   OLED
   ===================================================== */

/**
 * @brief Updates I2C OLED.
 * @param now Current millis() value.
 * @return Nothing
 */
void updateOLED(
  unsigned long now
)
{
  if (
    now -
    lastOLEDTime <
    OLED_INTERVAL
  )
  {
    return;
  }


  lastOLEDTime =
    now;


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


  /*
     =================================================
     STARTUP
     =================================================
  */

  if (
    currentMode ==
    MODE_SAFE
    &&
    !dhtHasValidReading
    &&
    !dhtFault
  )
  {
    display.println(
      "SMART GREENHOUSE"
    );

    display.println(
      "STARTUP CHECK"
    );

    display.println();

    display.println(
      "Checking DHT22..."
    );

    display.print(
      "Light ADC: "
    );

    display.println(
      lightValue
    );

    display.print(
      "Vent safe: "
    );

    display.println(
      SAFE_VENT_ANGLE
    );


    display.display();

    return;
  }


  /*
     =================================================
     SAFETY / SENSOR FAULT
     =================================================
  */

  if (
    currentMode ==
    MODE_SAFE
  )
  {
    display.println(
      "*** SENSOR FAULT ***"
    );

    display.println(
      "DHT22 FAILED"
    );

    display.println(
      "CHECK SENSOR"
    );

    display.println();

    display.print(
      "Light:"
    );

    display.println(
      lightValue
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

    display.println(
      targetVentAngle
    );

    display.println(
      "Grow LEDs: OFF"
    );


    display.display();

    return;
  }


  /*
     =================================================
     AUTO / MANUAL
     =================================================
  */

  if (
    currentMode ==
    MODE_MANUAL
  )
  {
    display.println(
      "MANUAL OVERRIDE"
    );
  }

  else
  {
    display.println(
      "MODE: AUTO"
    );
  }


  /*
     Temperature and humidity.
  */

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


  /*
     Light ADC.
  */

  display.print(
    "Light ADC:"
  );

  display.println(
    lightValue
  );


  /*
     Grow-light state.
  */

  display.print(
    "Grow:"
  );

  display.println(
    growLightsOn ?
    "ON" :
    "OFF"
  );


  /*
     Logical vent position.
  */

  display.print(
    "Vent:"
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


  /*
     Temperature setpoint.
  */

  display.print(
    "Tset:"
  );

  display.print(
    ventStartTemperature,
    1
  );

  display.println(
    "C"
  );


  display.display();
}


/* =====================================================
   SETUP
   ===================================================== */

/**
 * @brief Initializes greenhouse hardware and communication.
 * @param None
 * @return Nothing
 */
void setup()
{
  /*
     UART.
  */

  Serial.begin(
    115200
  );


  /*
     DHT22.
  */

  dht.begin();


  /*
     ADC.
  */

  analogReadResolution(
    12
  );


  /*
     Grow LEDs.
  */

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


  /*
     Manual Override button.

     GPIO19 ---- BUTTON ---- GND

     Released = HIGH
     Pressed  = LOW
  */

  pinMode(
    BUTTON_PIN,
    INPUT_PULLUP
  );


  attachInterrupt(
    digitalPinToInterrupt(
      BUTTON_PIN
    ),
    buttonISR,
    FALLING
  );


  /*
     Servo.
  */

  ventServo.setPeriodHertz(
    50
  );


  ventServo.attach(
    SERVO_PIN,
    500,
    2400
  );


  /*
     Start at logical Safety angle.
  */

  currentVentAngle =
    SAFE_VENT_ANGLE;

  targetVentAngle =
    SAFE_VENT_ANGLE;


  /*
     IMPORTANT:

     Convert the logical 45° Safety angle
     into the reversed physical servo command.
  */

  ventServo.write(
    logicalToServoAngle(
      currentVentAngle
    )
  );


  /*
     OLED / I2C.
  */

  Wire.begin(
    OLED_SDA,
    OLED_SCL
  );


  if (
    !display.begin(
      SSD1306_SWITCHCAPVCC,
      0x3C
    )
  )
  {
    Serial.println(
      "OLED INITIALIZATION FAILED"
    );


    while (
      true
    )
    {
    }
  }


  /*
     Startup display.
  */

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


  /*
     Timer initialization.
  */

  unsigned long now =
    millis();


  /*
     Read LDR immediately.
  */

  lastLDRTime =
    now -
    LDR_INTERVAL;


  /*
     Allow normal 2-second DHT startup.
  */

  lastDHTTime =
    now;


  /*
     Allow OLED refresh immediately.
  */

  lastOLEDTime =
    now -
    OLED_INTERVAL;


  /*
     Startup Serial messages.
  */

  Serial.println();

  Serial.println(
    "SMART GREENHOUSE READY"
  );

  Serial.println(
    "Priority: SAFETY > MANUAL > AUTO"
  );

  Serial.println(
    "LDR controls grow LEDs only"
  );

  Serial.println(
    "DHT22 temperature controls vent only"
  );

  Serial.println(
    "Humidity is monitoring only"
  );

  Serial.println(
    "Logical vent: 0 CLOSED / 140 OPEN"
  );

  Serial.println(
    "Servo direction reversed in software"
  );

  Serial.println(
    "Safety vent position: 45 deg logical"
  );

  Serial.println(
    "AUTO light: <400 ON / >700 OFF"
  );

  Serial.println(
    "AUTO vent: <=28C CLOSED / >=35C FULL OPEN"
  );


  printHelp();
}


/* =====================================================
   MAIN LOOP
   ===================================================== */

/**
 * @brief Runs non-blocking greenhouse controller.
 * @param None
 * @return Nothing
 */
void loop()
{
  unsigned long now =
    millis();


  /*
     UART.
  */

  updateUART();


  /*
     Button interrupt.
  */

  handleButtonInterrupt(
    now
  );


  /*
     LDR.
  */

  updateLDR(
    now
  );


  /*
     DHT22.
  */

  updateDHT(
    now
  );


  /*
     Determine mode.
  */

  determineMode();


  /*
     Apply control logic.
  */

  applyControlLogic();


  /*
     Move servo.
  */

  updateServoSmoothly(
    now
  );


  /*
     OLED.
  */

  updateOLED(
    now
  );


  /*
     Periodic Serial status.
  */

  if (
    now -
    lastSerialTime >=
    SERIAL_INTERVAL
  )
  {
    lastSerialTime =
      now;


    printStatus();
  }
}