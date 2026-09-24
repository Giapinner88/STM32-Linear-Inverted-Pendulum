#ifndef COMMS_H
#define COMMS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  CONTROL_MODE_IDLE = 0,
  CONTROL_MODE_SWINGUP = 1,
  CONTROL_MODE_LQR = 2
} ControlMode;

typedef struct {
  float x_pos;
  float x_vel;
  float theta;
  float theta_vel;
  float control_effort;
  uint16_t frame;
  uint8_t flags;
} RobotStateData;

#define TELEMETRY_FLAG_CAPTURED 0x01U
#define TELEMETRY_FLAG_RUNNING 0x02U
#define TELEMETRY_FLAG_RAIL_BLOCKED 0x04U

typedef struct {
  float control_effort;
  ControlMode mode;
  uint32_t timestamp_ms;
  uint8_t is_fresh;
} ControlCommandData;

typedef struct {
  uint32_t rx_packets;
  uint32_t rx_parse_resync;
  uint32_t rx_dma_restarts;
  uint32_t rx_errors;
  uint32_t tx_packets;
  uint32_t tx_dropped;
  uint32_t tx_errors;
  uint32_t last_rx_ms;
} CommsStats;

void Comms_Init(UART_HandleTypeDef *huart);
void Comms_Process(void);
void Comms_SendTelemetry(const RobotStateData *state);
uint8_t Comms_GetLatestCommand(ControlCommandData *cmd);
void Comms_GetStats(CommsStats *stats);

#ifdef __cplusplus
}
#endif

#endif
