#ifndef _extendsensor_h_
#define _extendsensor_h_

#define CONTROL_DATA_SIZE_MAX 240  // 传感器控制参数个数

typedef struct {
  std::string sensorName;
  uint16_t sensorID;
  uint16_t startAddr;
  uint8_t length;
} SensorModule;

typedef struct {
  uint16_t id;
  uint16_t addr;
  uint16_t lenght;
  uint8_t data[CONTROL_DATA_SIZE_MAX];
} SensorControl_t;

bool SensorWrite(uint8_t WriteCount, uint8_t *WriteID, uint16_t *WriteAddress,
                 uint16_t *WriteLenght,
                 uint8_t WriteData[][CONTROL_DATA_SIZE_MAX]);

void ExtendSensorThread(void);
void ExtendSensorTimerThread(void);

#endif