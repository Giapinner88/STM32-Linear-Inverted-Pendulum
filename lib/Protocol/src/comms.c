#include "comms.h"

#include <string.h>

#define TELEMETRY_HEADER_0 0xAAU
#define TELEMETRY_HEADER_1 0xBBU
#define COMMAND_HEADER_0 0xCCU
#define COMMAND_HEADER_1 0xDDU

typedef struct __attribute__((packed)) {
  uint8_t header[2];
  float x_pos;
  float x_vel;
  float theta;
  float theta_vel;
} RobotStatePacket;

typedef struct __attribute__((packed)) {
  uint8_t header[2];
  float control_effort;
  uint8_t mode;
} ControlCommandPacket;

typedef enum {
  RX_WAIT_HEADER_0 = 0,
  RX_WAIT_HEADER_1,
  RX_WAIT_PAYLOAD
} RxState;

static UART_HandleTypeDef *g_huart = NULL;
static volatile ControlCommandData g_latest_cmd = {0};
static volatile CommsStats g_stats = {0};
static uint8_t g_rx_dma_buf[16] = {0};
static uint8_t g_payload[sizeof(ControlCommandPacket) - 2U] = {0};
static uint8_t g_payload_idx = 0U;
static RxState g_rx_state = RX_WAIT_HEADER_0;
static RobotStatePacket g_tx_pkt = {0};

static void Comms_StartRxDMA(void)
{
  if (g_huart != NULL) {
    if (HAL_UART_Receive_DMA(g_huart, g_rx_dma_buf, sizeof(g_rx_dma_buf)) == HAL_OK) {
      g_stats.rx_dma_restarts++;
    }
  }
}

static void Comms_ResetParser(void)
{
  g_rx_state = RX_WAIT_HEADER_0;
  g_payload_idx = 0U;
}

static void Comms_ParseByte(uint8_t b)
{
  ControlCommandPacket pkt;

  switch (g_rx_state) {
    case RX_WAIT_HEADER_0:
      if (b == COMMAND_HEADER_0) {
        g_rx_state = RX_WAIT_HEADER_1;
      }
      break;

    case RX_WAIT_HEADER_1:
      if (b == COMMAND_HEADER_1) {
        g_rx_state = RX_WAIT_PAYLOAD;
        g_payload_idx = 0U;
      } else if (b == COMMAND_HEADER_0) {
        g_stats.rx_parse_resync++;
        g_rx_state = RX_WAIT_HEADER_1;
      } else {
        g_rx_state = RX_WAIT_HEADER_0;
      }
      break;

    case RX_WAIT_PAYLOAD:
      g_payload[g_payload_idx++] = b;
      if (g_payload_idx >= sizeof(g_payload)) {
        pkt.header[0] = COMMAND_HEADER_0;
        pkt.header[1] = COMMAND_HEADER_1;
        memcpy(&pkt.control_effort, &g_payload[0], sizeof(float));
        pkt.mode = g_payload[sizeof(float)];

        g_latest_cmd.control_effort = pkt.control_effort;
        g_latest_cmd.mode = (pkt.mode <= CONTROL_MODE_LQR) ? (ControlMode)pkt.mode : CONTROL_MODE_IDLE;
        g_latest_cmd.timestamp_ms = HAL_GetTick();
        g_latest_cmd.is_fresh = 1U;
        g_stats.rx_packets++;
        g_stats.last_rx_ms = g_latest_cmd.timestamp_ms;

        Comms_ResetParser();
      }
      break;

    default:
      Comms_ResetParser();
      break;
  }
}

static void Comms_ParseBuffer(const uint8_t *buf, uint16_t len)
{
  uint16_t i;

  if (buf == NULL) {
    return;
  }

  for (i = 0U; i < len; i++) {
    Comms_ParseByte(buf[i]);
  }
}

void Comms_Init(UART_HandleTypeDef *huart)
{
  g_huart = huart;
  memset((void *)&g_stats, 0, sizeof(g_stats));
  memset((void *)&g_latest_cmd, 0, sizeof(g_latest_cmd));
  Comms_ResetParser();
  Comms_StartRxDMA();
}

void Comms_Process(void)
{
  if (g_huart == NULL) {
    return;
  }

  if ((g_huart->RxState == HAL_UART_STATE_READY) && (g_huart->hdmarx != NULL)) {
    if (__HAL_DMA_GET_COUNTER(g_huart->hdmarx) == 0U) {
      Comms_StartRxDMA();
    }
  }
}

void Comms_SendTelemetry(const RobotStateData *state)
{
  HAL_StatusTypeDef status;

  if ((g_huart == NULL) || (state == NULL)) {
    return;
  }

  if (g_huart->gState != HAL_UART_STATE_READY) {
    g_stats.tx_dropped++;
    return;
  }

  g_tx_pkt.header[0] = TELEMETRY_HEADER_0;
  g_tx_pkt.header[1] = TELEMETRY_HEADER_1;
  g_tx_pkt.x_pos = state->x_pos;
  g_tx_pkt.x_vel = state->x_vel;
  g_tx_pkt.theta = state->theta;
  g_tx_pkt.theta_vel = state->theta_vel;

  status = HAL_UART_Transmit_IT(g_huart, (uint8_t *)&g_tx_pkt, sizeof(g_tx_pkt));
  if (status == HAL_OK) {
    g_stats.tx_packets++;
  } else if (status == HAL_BUSY) {
    g_stats.tx_dropped++;
  } else {
    g_stats.tx_errors++;
  }
}

uint8_t Comms_GetLatestCommand(ControlCommandData *cmd)
{
  uint8_t has_new = 0U;

  if (cmd == NULL) {
    return 0U;
  }

  __disable_irq();
  if (g_latest_cmd.is_fresh != 0U) {
    memcpy(cmd, (const void *)&g_latest_cmd, sizeof(ControlCommandData));
    g_latest_cmd.is_fresh = 0U;
    has_new = 1U;
  }
  __enable_irq();

  return has_new;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((g_huart != NULL) && (huart == g_huart)) {
    Comms_ParseBuffer(g_rx_dma_buf, sizeof(g_rx_dma_buf));
    Comms_StartRxDMA();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((g_huart != NULL) && (huart == g_huart)) {
    g_stats.rx_errors++;
    HAL_UART_DMAStop(g_huart);
    Comms_ResetParser();
    Comms_StartRxDMA();
  }
}

void Comms_GetStats(CommsStats *stats)
{
  if (stats == NULL) {
    return;
  }

  __disable_irq();
  memcpy(stats, (const void *)&g_stats, sizeof(CommsStats));
  __enable_irq();
}
