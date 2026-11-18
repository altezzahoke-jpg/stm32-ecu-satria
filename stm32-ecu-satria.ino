/* ==============================================================================
 * FIRMWARE ECU TCI SATRIA FU KARBURATOR (STM32F4 BLACK PILL - HARDENED FINAL)
 * 100% LOGIC PRESERVED + HARDWARE PWM TACHOMETER + 32-BIT TIMER
 * ============================================================================== */

#include <Arduino.h>
#include <EEPROM.h>
#include <HardwareSerial.h>
#include <IWatchdog.h>

// --- KONFIGURASI PIN HARDWARE BLACK PILL (F401 / F411) ---
#define PIN_PULSER       PB7   // Input pulser dari isolator CA-IS3722 (EXTI9_5)
#define PIN_TCI          PA8   // Output sinyal ke Isolator -> Gate Driver TC4420 (Coil)
#define PIN_TACH_OUT     PA6   // Output Hardware PWM Tachometer ke Spidometer (TIM3_CH1)
// Serial2 (USART2) untuk Telemetri & Live Tuning ke ESP32-C3 (TX2 = PA2, RX2 = PA3)

#define TCI_COIL_CHARGE() digitalWrite(PIN_TCI, HIGH)
#define TCI_COIL_SPARK()  digitalWrite(PIN_TCI, LOW)

#define ADC_CHAN_TPS     PA1   // Potensiometer TPS (ADC1_IN1)
#define ADC_CHAN_TEMP    PA4   // Sensor Suhu Mesin / NTC (ADC1_IN4)
#define ADC_CHAN_BATT    PA5   // Pembagi Tegangan Aki (ADC1_IN5)

// --- DEFINISI KODE KERUSAKAN HARDWARE (BITMASK) ---
#define FAULT_NONE         0            
#define FAULT_TPS_OUT      (1 << 0)     
#define FAULT_NTC_OUT      (1 << 1)     
#define FAULT_BATT_LOW     (1 << 2)     
#define FAULT_BATT_HIGH    (1 << 3)     

#define ABS_VAL(x)         ((x) < 0 ? -(x) : (x))
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define INV_3600_Q32       1193046ULL 

typedef enum { STATE_STOPPED = 0, STATE_CRANKING, STATE_RUNNING, STATE_LIMP_HOME } EngineState;
typedef enum { MODE_DAILY = 0, MODE_RACING, MODE_EXTREME, MODE_CUSTOM } ModePengapian;
typedef enum { BBM_PERTALITE = 0, BBM_PERTAMAX } JenisBBM;
typedef enum { IGN_IDLE = 0, IGN_WAITING_CHARGE, IGN_WAITING_SPARK } IgnitionSequenceState;

// --- STRUKTUR DATA IGNITION COMMAND (DIPINDAH KE ATAS AGAR TERBACA COMPILER) ---
typedef struct {
    int16_t finalAdvance10;
    uint32_t finalDwellUs;
} IgnitionCommand_t;

// --- PROTEKSI DATA RAM DENGAN BIT-INVERSION MASKING ---
typedef struct {
    uint32_t data;
    uint32_t data_inverted;
} SecureData_t;

SecureData_t sec_mode;
SecureData_t sec_engine_state;

void SecureData_Write(SecureData_t* sec, uint32_t value) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    sec->data = value;
    sec->data_inverted = ~value; 
    if (!primask) __enable_irq();
}

bool SecureData_Read(SecureData_t* sec, uint32_t* out_value) {
    uint32_t d, d_inv;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    d = sec->data;
    d_inv = sec->data_inverted;
    if (!primask) __enable_irq();
    
    if (d == ~d_inv) { 
        *out_value = d;
        return true; 
    }
    return false;
}

// --- VARIABEL GLOBAL VOLATILE ---
volatile uint16_t currentRPM = 0;
volatile uint8_t currentTPS = 0;
volatile uint16_t currentBatteryVoltage10 = 126;
volatile int16_t currentEngineTemp10 = 300;
volatile int16_t currentDegree10 = 100;
volatile bool sensorFaultStatus = false;
volatile uint16_t hardwareFaultCode = FAULT_NONE;
volatile uint8_t activeMapIndex = 0;
volatile uint8_t currentCustomSlot = 0;
volatile uint8_t lastAckStatus = 0; 

volatile uint32_t pulse_interval_us = 0;
volatile uint32_t last_pulse_time_us = 0;
volatile int16_t cachedAdv10 = 100;
volatile uint32_t cachedDwellUs = 3200;
volatile uint16_t activeCustomRpmLimit = 12500;

volatile IgnitionSequenceState ignState = IGN_IDLE;
volatile uint32_t target_spark_tick = 0;
volatile uint32_t charge_start_time_us = 0; 
JenisBBM currentFuel = BBM_PERTALITE;

// TIM2 sebagai Timer 32-bit untuk presisi waktu pengapian mikrodetik
HardwareTimer *timerIgnition = new HardwareTimer(TIM2);
// TIM3 Channel 1 (PA6) untuk Hardware PWM Tachometer
HardwareTimer *timerTach = new HardwareTimer(TIM3);

#define NUM_RPM_POINTS 15
#define NUM_TPS_POINTS 5
#define MAX_CUSTOM_SLOTS 5

// --- STRUKTUR PAKET KOMUNIKASI UART (SINKRON 100% DENGAN ESP32-C3) ---
#pragma pack(push, 1)
typedef struct {
  uint16_t header;     // 0xAA55
  uint16_t rpm;       
  uint8_t  tps;        
  int16_t  degree10;  
  int16_t  temp10;    
  uint16_t battVolt10;
  uint8_t  mode;       
  uint8_t  fuel;      
  uint8_t  state;      
  uint8_t  ackStatus;  
  uint16_t faultCode;  
  uint16_t crc16;      
} TelemetryData;       // Total: 19 Bytes

typedef struct {
  uint16_t header;                                 // 0x55CC
  uint8_t  cmdType;                                // 0x01: Mode, 0x02: RAM Map, 0x03: Save EEPROM
  uint8_t  slotOrMode;   
  int16_t  mapData[NUM_RPM_POINTS][NUM_TPS_POINTS]; // 150 Bytes
  uint16_t dwellData[NUM_RPM_POINTS];               // 30 Bytes
  uint16_t rpmLimit;                                // 2 Bytes
  uint16_t crc16;                                   // 2 Bytes
} CommandPacket;                                    // Total: 188 Bytes
#pragma pack(pop)

const int16_t ROTOR_PULSER_DEGREES_10 = 350; 

const uint16_t rpmAxis[NUM_RPM_POINTS] = { 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000, 13000, 14000 };
const uint8_t  tpsAxis[NUM_TPS_POINTS] = { 0, 25, 50, 75, 100 };

const int16_t mapDaily3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {100, 100, 100, 100, 100}, {100, 100, 100, 100, 100}, {140, 150, 160, 170, 180},
  {180, 200, 220, 230, 240}, {220, 240, 260, 270, 280}, {250, 270, 290, 300, 310},
  {280, 300, 320, 330, 330}, {300, 320, 340, 350, 350}, {310, 330, 350, 350, 350},
  {310, 330, 350, 350, 350}, {300, 320, 340, 340, 340}, {280, 300, 320, 320, 320},
  {260, 280, 300, 300, 300}, {250, 250, 280, 280, 280}, {250, 250, 250, 250, 250}
};

const int16_t mapRacing3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {110, 110, 110, 110, 110}, {120, 120, 125, 130, 130}, {160, 175, 190, 200, 210},
  {210, 230, 250, 260, 270}, {250, 275, 290, 310, 320}, {280, 305, 330, 340, 350},
  {310, 335, 360, 370, 370}, {330, 355, 380, 390, 390}, {340, 365, 390, 390, 390},
  {340, 365, 390, 390, 390}, {330, 355, 380, 380, 380}, {310, 335, 360, 360, 360},
  {290, 315, 340, 340, 340}, {270, 280, 310, 310, 310}, {260, 260, 280, 280, 280}
};

const int16_t mapExtreme3DBase[NUM_RPM_POINTS][NUM_TPS_POINTS] = {
  {120, 120, 120, 120, 120}, {130, 135, 140, 145, 150}, {180, 195, 210, 225, 240},
  {230, 255, 280, 295, 310}, {280, 310, 330, 350, 360}, {310, 340, 370, 385, 400},
  {340, 375, 400, 410, 410}, {360, 395, 420, 430, 430}, {370, 405, 430, 430, 430},
  {370, 405, 430, 430, 430}, {360, 395, 420, 420, 420}, {340, 375, 400, 400, 400},
  {320, 350, 380, 380, 380}, {300, 310, 340, 340, 340}, {280, 280, 300, 300, 300}
};

const uint16_t defaultDwellPerRPM[NUM_RPM_POINTS] = {
  3500, 3500, 3400, 3300, 3200, 3100, 3000, 2900, 2800, 2700, 2600, 2500, 2400, 2300, 2200
};

int16_t mapCustomSlots[MAX_CUSTOM_SLOTS][NUM_RPM_POINTS][NUM_TPS_POINTS];
uint16_t dwellCustomSlots[MAX_CUSTOM_SLOTS][NUM_RPM_POINTS];
uint16_t rpmLimitCustomSlots[MAX_CUSTOM_SLOTS];

int16_t activeCustomMapBuffer[2][NUM_RPM_POINTS][NUM_TPS_POINTS];
uint16_t activeCustomDwellBuffer[2][NUM_RPM_POINTS];

static inline bool isEngineStopped(void) {
  uint32_t engineState;
  if (SecureData_Read(&sec_engine_state, &engineState)) {
      return (engineState == STATE_STOPPED && currentRPM == 0);
  }
  return false;
}

int32_t mapRange(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
  if (in_max == in_min) return out_min;
  int32_t result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return CLAMP(result, out_min, out_max);
}

uint16_t calculateCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

void updateActiveMapBuffer(void) {
  uint8_t slot = currentCustomSlot;
  if (slot >= MAX_CUSTOM_SLOTS) slot = 0;
  uint8_t nextBuf = 1 - activeMapIndex;
  
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  memcpy(activeCustomMapBuffer[nextBuf], mapCustomSlots[slot], sizeof(activeCustomMapBuffer[0]));
  memcpy(activeCustomDwellBuffer[nextBuf], dwellCustomSlots[slot], sizeof(activeCustomDwellBuffer[0]));
  uint16_t limit = rpmLimitCustomSlots[slot];
  if (!primask) __enable_irq();
  
  activeCustomRpmLimit = limit;
  activeMapIndex = nextBuf;
}

// --- PENYIMPANAN EEPROM INTERNAL STM32 ---
void loadMapsFromEEPROM() {
    EEPROM.begin(); 
    
    if (EEPROM.read(0) != 0x54) { 
        for (int i = 0; i < MAX_CUSTOM_SLOTS; i++) {
            memcpy(&mapCustomSlots[i], &mapDaily3DBase, sizeof(mapDaily3DBase));
            memcpy(&dwellCustomSlots[i], &defaultDwellPerRPM, sizeof(defaultDwellPerRPM));
            rpmLimitCustomSlots[i] = 12500;
        }
        SecureData_Write(&sec_mode, MODE_DAILY);
    } else {
        int addr = 1;
        for (int i = 0; i < MAX_CUSTOM_SLOTS; i++) {
            EEPROM.get(addr, mapCustomSlots[i]);
            addr += sizeof(mapCustomSlots[i]);
            EEPROM.get(addr, dwellCustomSlots[i]);
            addr += sizeof(dwellCustomSlots[i]);
            EEPROM.get(addr, rpmLimitCustomSlots[i]);
            addr += sizeof(rpmLimitCustomSlots[i]);
        }
        uint8_t savedMode = MODE_DAILY;
        EEPROM.get(addr, savedMode);
        if (savedMode <= MODE_CUSTOM) {
            SecureData_Write(&sec_mode, savedMode);
        } else {
            SecureData_Write(&sec_mode, MODE_DAILY);
        }
    }
    updateActiveMapBuffer();
}

void saveCustomSlotToEEPROM(uint8_t slot) {
    if (slot >= MAX_CUSTOM_SLOTS) return;
    int addr = 1 + slot * (sizeof(mapCustomSlots[0]) + sizeof(dwellCustomSlots[0]) + sizeof(rpmLimitCustomSlots[0]));
    EEPROM.put(addr, mapCustomSlots[slot]);
    addr += sizeof(mapCustomSlots[slot]);
    EEPROM.put(addr, dwellCustomSlots[slot]);
    addr += sizeof(dwellCustomSlots[slot]);
    EEPROM.put(addr, rpmLimitCustomSlots[slot]);
    EEPROM.write(0, 0x54);
}

void saveModeToEEPROM(uint8_t mode) {
    int addr = 1 + MAX_CUSTOM_SLOTS * (sizeof(mapCustomSlots[0]) + sizeof(dwellCustomSlots[0]) + sizeof(rpmLimitCustomSlots[0]));
    EEPROM.put(addr, mode);
    EEPROM.write(0, 0x54); 
}

IgnitionCommand_t HAL_IgnitionLogic_Calculate(uint16_t rpm, uint8_t tps, int16_t temp10, uint16_t batt10, ModePengapian mode, EngineState state, bool fault) {
    IgnitionCommand_t cmd;

    uint16_t calcRpm = CLAMP(rpm, 0, rpmAxis[NUM_RPM_POINTS - 1]);
    uint8_t calcTps = CLAMP(tps, 0, 100);

    uint8_t r0 = calcRpm / 1000;
    if (r0 >= NUM_RPM_POINTS - 1) r0 = NUM_RPM_POINTS - 2;
    uint8_t t0 = calcTps / 25;
    if (t0 >= NUM_TPS_POINTS - 1) t0 = NUM_TPS_POINTS - 2;
    uint8_t r1 = r0 + 1, t1 = t0 + 1;
    int32_t rF = ((int32_t)(calcRpm - (r0 * 1000)) * 1024) / 1000;

    if (mode == MODE_CUSTOM) {
        uint8_t bufIdx = activeMapIndex;
        uint16_t d0 = activeCustomDwellBuffer[bufIdx][r0];
        uint16_t d1 = activeCustomDwellBuffer[bufIdx][r1];
        cmd.finalDwellUs = (uint32_t)(d0 + (((int32_t)d1 - (int32_t)d0) * rF) / 1024);
    } else {
        if      (batt10 < 100) cmd.finalDwellUs = 4500; 
        else if (batt10 < 115) cmd.finalDwellUs = 3800;
        else if (batt10 < 125) cmd.finalDwellUs = 3200; 
        else if (batt10 < 135) cmd.finalDwellUs = 2800;
        else if (batt10 < 145) cmd.finalDwellUs = 2500; 
        else                   cmd.finalDwellUs = 2200;
    }

    if (fault || state == STATE_LIMP_HOME) {
        cmd.finalAdvance10 = 150; 
        return cmd;
    } 
    if (state == STATE_CRANKING) {
        cmd.finalAdvance10 = 50;  
        return cmd;
    }

    int16_t q11, q21, q12, q22;
    if (mode == MODE_DAILY) {
        q11 = mapDaily3DBase[r0][t0]; q21 = mapDaily3DBase[r1][t0]; 
        q12 = mapDaily3DBase[r0][t1]; q22 = mapDaily3DBase[r1][t1];
    } else if (mode == MODE_RACING) {
        q11 = mapRacing3DBase[r0][t0]; q21 = mapRacing3DBase[r1][t0]; 
        q12 = mapRacing3DBase[r0][t1]; q22 = mapRacing3DBase[r1][t1];
    } else if (mode == MODE_EXTREME) {
        q11 = mapExtreme3DBase[r0][t0]; q21 = mapExtreme3DBase[r1][t0]; 
        q12 = mapExtreme3DBase[r0][t1]; q22 = mapExtreme3DBase[r1][t1];
    } else {
        uint8_t bufIdx = activeMapIndex;
        q11 = activeCustomMapBuffer[bufIdx][r0][t0]; q21 = activeCustomMapBuffer[bufIdx][r1][t0]; 
        q12 = activeCustomMapBuffer[bufIdx][r0][t1]; q22 = activeCustomMapBuffer[bufIdx][r1][t1];
    }

    int32_t tF = ((int32_t)(calcTps - (t0 * 25)) * 1024) / 25;
    int32_t R1 = q11 + ((q21 - q11) * rF) / 1024;
    int32_t R2 = q12 + ((q22 - q12) * rF) / 1024;
    
    int16_t adv = (int16_t)(R1 + ((R2 - R1) * tF) / 1024);

    if (temp10 > 900) { 
        int16_t retard = (int16_t)mapRange(temp10, 900, 1300, 0, 50); 
        adv -= retard;
        if (adv < 100) adv = 100; 
    }
    
    cmd.finalAdvance10 = CLAMP(adv, 0, ROTOR_PULSER_DEGREES_10);
    return cmd;
}

// --- HARDWARE TIMER 32-BIT OUTPUT COMPARE INTERRUPT ---
void ignitionTimerISR() {
  if (ignState == IGN_WAITING_CHARGE) {
    TCI_COIL_CHARGE();
    charge_start_time_us = micros();
    ignState = IGN_WAITING_SPARK;
    
    uint32_t now_cnt = timerIgnition->getCount();
    if ((int32_t)(target_spark_tick - now_cnt) <= 15) {
        TCI_COIL_SPARK();
        ignState = IGN_IDLE;
        timerIgnition->pauseChannel(1);
    } else {
        timerIgnition->setCaptureCompare(1, target_spark_tick, TICK_COMPARE_FORMAT);
    }
  } else if (ignState == IGN_WAITING_SPARK) {
    TCI_COIL_SPARK();
    ignState = IGN_IDLE;
    timerIgnition->pauseChannel(1);
  }
}

// --- EXTERNAL INTERRUPT PULSER (PB7 -> EXTI9_5) ---
void pulserISR() {
  uint32_t now_us = micros(); 
  uint32_t interval_us = now_us - last_pulse_time_us;
  
  if (interval_us < 2000) return; 

  if (pulse_interval_us > 0) {
    if (interval_us < ((pulse_interval_us * 4) / 10)) return; 
    if (interval_us > (pulse_interval_us * 3)) { 
        if (ignState != IGN_IDLE) {
            TCI_COIL_SPARK(); 
            ignState = IGN_IDLE;
            timerIgnition->pauseChannel(1);
        }
    }
  }

  int32_t accel_delta_us = (pulse_interval_us > 0) ? ((int32_t)pulse_interval_us - (int32_t)interval_us) : 0;
  
  uint32_t rpm = 60000000UL / interval_us; 
  EngineState newState = (sensorFaultStatus) ? STATE_LIMP_HOME : ((rpm < 600) ? STATE_CRANKING : STATE_RUNNING);

  pulse_interval_us = interval_us;
  last_pulse_time_us = now_us;
  currentRPM = (uint16_t)rpm;
  SecureData_Write(&sec_engine_state, (uint32_t)newState); 

  int16_t adv10 = cachedAdv10;

  uint32_t safeMode;
  uint16_t currentLimit = 12500;
  uint16_t currentSoftLimit = 12300;
  
  if (SecureData_Read(&sec_mode, &safeMode) && safeMode == MODE_CUSTOM) {
      currentLimit = activeCustomRpmLimit;
      if (currentLimit < 1000) currentLimit = 1000;
      currentSoftLimit = (currentLimit > 200) ? (currentLimit - 200) : currentLimit;
  }

  static uint8_t cut_counter = 0;
  if (newState == STATE_LIMP_HOME && rpm >= 6000) {
      cut_counter++;
      if (cut_counter % 2 != 0) { 
          TCI_COIL_SPARK(); 
          ignState = IGN_IDLE; 
          timerIgnition->pauseChannel(1);
          return; 
      }
  } else if (rpm >= currentLimit) { 
      cut_counter++;
      if (cut_counter % 3 != 0) { 
          TCI_COIL_SPARK(); 
          ignState = IGN_IDLE; 
          timerIgnition->pauseChannel(1);
          return; 
      }
  }

  if (rpm >= currentSoftLimit) adv10 = 50; 
  currentDegree10 = adv10;

  uint32_t targetDwellUs = cachedDwellUs;
  uint32_t maxAllowedDwell = (interval_us > 1200) ? (interval_us - 800) : ((interval_us * 80) / 100);
  if (targetDwellUs > maxAllowedDwell) targetDwellUs = maxAllowedDwell;

  int32_t sparkDegFromPulser = ROTOR_PULSER_DEGREES_10 - adv10;
  if (sparkDegFromPulser < 0) sparkDegFromPulser = 0;
  
  uint32_t sparkDelayUs = (uint32_t)(((uint64_t)sparkDegFromPulser * (uint64_t)interval_us * INV_3600_Q32) >> 32);

  if (accel_delta_us > 0 && sparkDelayUs > 0) {
      uint32_t accel_comp = (sparkDelayUs * (uint32_t)accel_delta_us) / (2 * interval_us);
      if (sparkDelayUs > accel_comp) sparkDelayUs -= accel_comp;
      else sparkDelayUs = 0;
  }

  if (sparkDelayUs >= interval_us) {
    TCI_COIL_SPARK(); 
    ignState = IGN_IDLE;
    timerIgnition->pauseChannel(1);
  } else {
    uint32_t current_tick = timerIgnition->getCount();
    uint32_t spark_tick = current_tick + sparkDelayUs;
    uint32_t charge_tick = (sparkDelayUs > targetDwellUs) ? (spark_tick - targetDwellUs) : (current_tick + 15);
    
    target_spark_tick = spark_tick;
    ignState = IGN_WAITING_CHARGE;
    
    timerIgnition->setCaptureCompare(1, charge_tick, TICK_COMPARE_FORMAT);
    timerIgnition->resumeChannel(1);
  }
}

void processSensors() {
  static uint16_t emaTps = 0, emaBatt = 0, emaTemp = 0;
  static uint8_t lastValidTps = 0;
  static int16_t lastValidTemp = 300;
  
  int rawTps = analogRead(ADC_CHAN_TPS);
  int rawTemp = analogRead(ADC_CHAN_TEMP);
  int rawBatt = analogRead(ADC_CHAN_BATT);

  int mvTps = (rawTps * 3300) / 4095;
  int mvTemp = (rawTemp * 3300) / 4095;
  int mvBatt = (rawBatt * 3300) / 4095;

  uint16_t currentFaults = FAULT_NONE;
  if (rawTps < 50 || rawTps > 4050) currentFaults |= FAULT_TPS_OUT; 
  if (rawTemp < 30 || rawTemp > 4050) currentFaults |= FAULT_NTC_OUT; 
  if (rawBatt < 800) currentFaults |= FAULT_BATT_LOW;  
  else if (rawBatt > 3800) currentFaults |= FAULT_BATT_HIGH; 

  hardwareFaultCode = currentFaults;
  sensorFaultStatus = (currentFaults != FAULT_NONE);
  
  emaTps  = (emaTps == 0)  ? mvTps  : (emaTps - (emaTps >> 2) + (mvTps >> 2));
  emaTemp = (emaTemp == 0) ? mvTemp : (emaTemp - (emaTemp >> 2) + (mvTemp >> 2));
  emaBatt = (emaBatt == 0) ? mvBatt : (emaBatt - (emaBatt >> 2) + (mvBatt >> 2));

  int32_t target_tps   = mapRange(emaTps, 450, 2800, 0, 100); 
  int32_t target_tempC  = mapRange(emaTemp, 300, 2800, -200, 1500);
  uint16_t battV = (uint16_t)mapRange(emaBatt, 0, 3100, 0, 160);

  uint8_t tpsP = (target_tps > lastValidTps + 15) ? (lastValidTps + 15) :
                 (target_tps < lastValidTps - 15) ? (lastValidTps - 15) : (uint8_t)target_tps;
  lastValidTps = tpsP;

  int16_t tempC = (target_tempC > lastValidTemp + 50) ? (lastValidTemp + 50) :
                  (target_tempC < lastValidTemp - 50) ? (lastValidTemp - 50) : (int16_t)target_tempC;
  lastValidTemp = tempC;

  currentTPS = tpsP; 
  currentEngineTemp10 = tempC; 
  currentBatteryVoltage10 = battV; 
  
  uint32_t now = micros();
  if (ignState != IGN_IDLE && (now - charge_start_time_us) > 12000UL) {
      TCI_COIL_SPARK();
      ignState = IGN_IDLE;
      timerIgnition->pauseChannel(1);
  }

  if ((now - last_pulse_time_us) > 600000UL && currentRPM != 0) {
    currentRPM = 0;
    SecureData_Write(&sec_engine_state, STATE_STOPPED);
    pulse_interval_us = 0;
    ignState = IGN_IDLE;
    TCI_COIL_SPARK(); 
    timerIgnition->pauseChannel(1);
  }
}

void processIgnitionCalculation() {
    uint16_t rpm = currentRPM;
    uint8_t tps = currentTPS;
    int16_t temp = currentEngineTemp10;
    uint16_t batt = currentBatteryVoltage10;
    bool fault = sensorFaultStatus;

    uint32_t safeMode, safeState;
    if (!SecureData_Read(&sec_mode, &safeMode) || !SecureData_Read(&sec_engine_state, &safeState)) {
        safeMode = (uint32_t)MODE_DAILY;
        safeState = (uint32_t)STATE_LIMP_HOME;
    }

    IgnitionCommand_t cmd = HAL_IgnitionLogic_Calculate(rpm, tps, temp, batt, (ModePengapian)safeMode, (EngineState)safeState, fault);

    cachedAdv10 = cmd.finalAdvance10;
    cachedDwellUs = cmd.finalDwellUs;
}

// --- PENGGERAK TACHOMETER MENGGUNAKAN HARDWARE PWM (TIMER 3 CH 1 -> PA6) ---
void processTachometer() {
  static uint32_t lastFreqHz = 0;
  uint16_t rpm = currentRPM;
  if (rpm > 400) {
    uint32_t freqHz = rpm / 60; 
    if (freqHz < 1) freqHz = 1;
    if (freqHz != lastFreqHz) {
      timerTach->setOverflow(freqHz, HERTZ_FORMAT);
      timerTach->setCaptureCompare(1, 50, PERCENT_COMPARE_FORMAT); // 50% duty cycle pada Channel 1 (PA6 / TIM3_CH1)
      timerTach->resumeChannel(1);
      lastFreqHz = freqHz;
    }
  } else {
    if (lastFreqHz != 0) {
      timerTach->pauseChannel(1);
      digitalWrite(PIN_TACH_OUT, LOW);
      lastFreqHz = 0;
    }
  }
}

void processMemoryScrubber() {
    uint32_t dummy;
    if (!SecureData_Read(&sec_mode, &dummy)) SecureData_Write(&sec_mode, MODE_DAILY); 
    if (!SecureData_Read(&sec_engine_state, &dummy)) SecureData_Write(&sec_engine_state, STATE_STOPPED);
}

void sendTelemetry() {
  TelemetryData pkt;
  pkt.header = 0xAA55; 
  pkt.rpm = currentRPM; 
  pkt.tps = currentTPS; 
  pkt.degree10 = currentDegree10;
  pkt.temp10 = currentEngineTemp10; 
  pkt.battVolt10 = currentBatteryVoltage10;
  
  uint32_t m, s;
  SecureData_Read(&sec_mode, &m);
  SecureData_Read(&sec_engine_state, &s);
  pkt.mode = (uint8_t)m; 
  pkt.state = (uint8_t)s;
  pkt.fuel = currentFuel; 
  pkt.ackStatus = lastAckStatus;
  pkt.faultCode = hardwareFaultCode;

  pkt.crc16 = calculateCRC16((uint8_t*)&pkt, sizeof(TelemetryData) - sizeof(uint16_t));
  
  Serial2.write((uint8_t*)&pkt, sizeof(TelemetryData));

  if (lastAckStatus != 0) lastAckStatus = 0;
}

void listenCommands() {
  static uint8_t rxBuf[sizeof(CommandPacket)];
  static uint16_t rxIdx = 0;
  static uint8_t syncState = 0; 

  while (Serial2.available() > 0) {
    uint8_t byte = Serial2.read();
    switch (syncState) {
      case 0:
        if (byte == 0x55) { rxBuf[0] = byte; rxIdx = 1; syncState = 1; }
        break;
      case 1:
        if (byte == 0xCC) { rxBuf[1] = byte; rxIdx = 2; syncState = 2; }
        else if (byte == 0x55) { rxBuf[0] = byte; rxIdx = 1; } 
        else { syncState = 0; }
        break;
      case 2:
        rxBuf[rxIdx++] = byte;
        if (rxIdx >= sizeof(CommandPacket)) {
          CommandPacket *cmd = (CommandPacket*)rxBuf;
          uint16_t calcCrc = calculateCRC16(rxBuf, sizeof(CommandPacket) - sizeof(uint16_t));
          
          if (cmd->crc16 == calcCrc) {
            lastAckStatus = 1; 
            
            if (cmd->cmdType == 0x01) { 
              if (cmd->slotOrMode <= MODE_CUSTOM) {
                if (isEngineStopped()) {
                  SecureData_Write(&sec_mode, cmd->slotOrMode);
                  saveModeToEEPROM(cmd->slotOrMode);
                }
              }
            } 
            else if (cmd->cmdType == 0x02 || cmd->cmdType == 0x03) { 
              uint8_t slot = cmd->slotOrMode;
              if (slot < MAX_CUSTOM_SLOTS) {
                uint32_t primask = __get_PRIMASK();
                __disable_irq();
                memcpy(mapCustomSlots[slot], cmd->mapData, sizeof(cmd->mapData));
                memcpy(dwellCustomSlots[slot], cmd->dwellData, sizeof(cmd->dwellData));
                rpmLimitCustomSlots[slot] = cmd->rpmLimit;
                if (!primask) __enable_irq();
                
                currentCustomSlot = slot;
                updateActiveMapBuffer(); 

                if (isEngineStopped()) {
                  saveCustomSlotToEEPROM(slot);
                }
              }
            }
          } else {
            lastAckStatus = 2; 
          }
          syncState = 0; 
        }
        break;
    }
  }
}

void setup() {
  SecureData_Write(&sec_mode, MODE_DAILY);
  SecureData_Write(&sec_engine_state, STATE_STOPPED);

  pinMode(PIN_TCI, OUTPUT);
  TCI_COIL_SPARK(); // Amankan IGBT (LOW) saat awal boot

  pinMode(PIN_PULSER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_PULSER), pulserISR, FALLING);

  Serial2.begin(250000);
  analogReadResolution(12);

  // Inisialisasi TIM2 untuk Output Compare Pengapian (32-bit, 1 tick = 1 µs)
  timerIgnition->pause();
  timerIgnition->setPrescaleFactor(timerIgnition->getTimerClkFreq() / 1000000);
  timerIgnition->setOverflow(0xFFFFFFFF);
  timerIgnition->setMode(1, TIMER_OUTPUT_COMPARE);
  timerIgnition->attachInterrupt(1, ignitionTimerISR);
  timerIgnition->pauseChannel(1);
  timerIgnition->resume();

  // Inisialisasi TIM3 untuk Hardware PWM Tachometer di PIN_TACH_OUT (PA6 / TIM3_CH1)
  timerTach->pause();
  timerTach->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PIN_TACH_OUT);
  timerTach->pauseChannel(1);
  timerTach->resume();

  // Prioritas interupsi tertinggi untuk pengapian (TIM2) dan pulser (PB7 -> EXTI9_5)
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);

  loadMapsFromEEPROM();

  // Aktifkan Independent Watchdog 600ms
  IWatchdog.begin(600000); 
}

void loop() {
  IWatchdog.reload();

  static uint32_t lastSensorMs = 0;
  static uint32_t lastTelMs = 0;
  static uint32_t lastTachMs = 0;
  static uint32_t lastScrubMs = 0;
  uint32_t currentMs = millis();

  processIgnitionCalculation();

  if (currentMs - lastSensorMs >= 10) {
    lastSensorMs = currentMs;
    processSensors();
  }

  if (currentMs - lastTelMs >= 50) {
    lastTelMs = currentMs;
    sendTelemetry();
  }

  if (currentMs - lastTachMs >= 50) {
    lastTachMs = currentMs;
    processTachometer();
  }

  if (currentMs - lastScrubMs >= 100) {
    lastScrubMs = currentMs;
    processMemoryScrubber();
  }

  listenCommands();
}
