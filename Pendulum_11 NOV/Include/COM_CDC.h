#ifndef COM_CDC
#define COM_CDC
#include "usbd_cdc_if.h"
#include "global.h"
/* ====== Parser paket stick ====== */
typedef struct {
  int16_t ax, ay, rx, ry;
  uint16_t buttons;
  uint8_t  seq;
  uint32_t last_update_ms;
  volatile uint8_t fresh;
} Joy_t;

static Joy_t gJoy = {0};

enum { ST_WAIT_AA, ST_WAIT_55, ST_TYPE, ST_SEQ, ST_PAYLOAD, ST_CRC_LO, ST_CRC_HI };
static uint8_t  st = ST_WAIT_AA;
static uint8_t  typ = 0;
static uint8_t  seq = 0;
static uint8_t  pay[10];           // ax,ay,rx,ry (8) + buttons (2) = 10
static uint8_t  pay_idx = 0;
static uint16_t sum = 0;           // checksum running (low 16 bits)
static uint16_t rx_crc = 0;

static inline void parser_reset(void){
  st = ST_WAIT_AA; typ=0; seq=0; pay_idx=0; sum=0; rx_crc=0;
}

static void joystick_parser_feed(uint8_t b){
  switch(st){
    case ST_WAIT_AA:
      if (b == 0xAA) st = ST_WAIT_55;
      break;
    case ST_WAIT_55:
      st = (b == 0x55) ? ST_TYPE : ST_WAIT_AA;
      break;
    case ST_TYPE:
      typ = b; sum = b; st = ST_SEQ;
      break;
    case ST_SEQ:
      seq = b; sum = (sum + b) & 0xFFFF; pay_idx = 0; st = ST_PAYLOAD;
      break;
    case ST_PAYLOAD:
      pay[pay_idx++] = b; sum = (sum + b) & 0xFFFF;
      if (pay_idx >= sizeof(pay)) 
				st = ST_CRC_LO;
      break;
    case ST_CRC_LO:
      rx_crc  = b;
      st      = ST_CRC_HI;
      break;
    case ST_CRC_HI:
      rx_crc |= ((uint16_t)b << 8);
      if (1 && typ == 0x01){
        /* unpack little-endian */
        int16_t ax =  (int16_t)((pay[1]<<8) | pay[0]);
				int16_t ay =  (int16_t)((pay[3]<<8) | pay[2]);
				int16_t rx =  (int16_t)((pay[5]<<8) | pay[4]);
				int16_t ry =  (int16_t)((pay[7]<<8) | pay[6]);
				uint16_t buttons = (uint16_t)((pay[9]<<8) | pay[8]);

        gJoy.ax = ax; gJoy.ay = ay; gJoy.rx = rx; gJoy.ry = ry;
        gJoy.buttons = buttons;
        gJoy.seq = seq;
        gJoy.last_update_ms = HAL_GetTick();
        gJoy.fresh = 1;
      }
      parser_reset();
      break;
    default:
      parser_reset();
      break;
  }
}

/* ====== Konfigurasi ====== */
#if defined(USBD_USE_HS)
  #define CDC_MAX_PKT   512u
#else
  #define CDC_MAX_PKT   64u
#endif

#define RX_RING_SIZE    2048u   /* power of two */
#define TX_RING_SIZE    2048u   /* power of two */

/* ====== Ring Buffer ====== */
typedef struct { volatile uint32_t head, tail, size; uint8_t *buf; } ring_t;
static uint8_t rx_mem[RX_RING_SIZE], tx_mem[TX_RING_SIZE];
static ring_t RX = {0,0,RX_RING_SIZE, rx_mem};
static ring_t TX = {0,0,TX_RING_SIZE, tx_mem};

static inline uint32_t r_count(const ring_t *r){ return (r->head - r->tail) & (r->size - 1u); }
static inline uint32_t r_free (const ring_t *r){ return (r->size - 1u) - r_count(r); }

static uint32_t r_write(ring_t *r, const uint8_t *src, uint32_t n){
  uint32_t w=0;
  while (w<n && r_free(r)) { r->buf[r->head & (r->size-1u)] = src[w++]; r->head++; }
  return w;
}
static uint32_t r_read(ring_t *r, uint8_t *dst, uint32_t n){
  uint32_t k=0;
  while (k<n && r_count(r)) { dst[k++] = r->buf[r->tail & (r->size-1u)]; r->tail++; }
  return k;
}

/* ====== TX state ====== */
static volatile uint8_t tx_busy = 0;
static uint8_t tx_chunk[CDC_MAX_PKT];

/* ====== API ringan ====== */
static uint32_t COM_Available(void){ return r_count(&RX); }
static uint32_t COM_Read(uint8_t *dst, uint32_t maxlen){ return r_read(&RX, dst, maxlen); }
static uint32_t COM_Write(const uint8_t *src, uint32_t len){ return r_write(&TX, src, len); }

/* ====== Hook dari usbd_cdc_if.c ====== */
void VCP_ReceiveHook(uint8_t *buf, uint32_t len){
	 //for (uint32_t i=0;i<len;i++) joystick_parser_feed(buf[i]);
  (void)r_write(&RX, buf, len);            /* non-blocking */
}
void VCP_TxCpltHook(void){
  tx_busy = 0;                             /* siap kirim batch berikutnya */
}
static void COM_PollAndParseJoystick(void)
{
  
  /* Drain ring sampai habis atau tidak penuh */
  for (;;){
    uint32_t n = COM_Read(buf, sizeof(buf));
    if (n == 0) break;
    for (uint32_t i = 0; i < n; ++i) joystick_parser_feed(buf[i]);
    if (n < sizeof(buf)) break;  // kemungkinan habis
  }
}
/* ====== Loop non-blocking ====== */
void loopCOM(void){
  if (!tx_busy && r_count(&TX)) {
    uint32_t n = r_read(&TX, tx_chunk, CDC_MAX_PKT);
    if (n) {
      uint8_t st = CDC_Transmit_FS(tx_chunk, (uint16_t)n);
      if (st == USBD_OK) {
        tx_busy = 1;
      } else {
        /* push-back bila BUSY (jarang terjadi) */
        for (int i=(int)n-1; i>=0; --i){
          TX.tail = (TX.tail - 1u) & (TX.size - 1u);
          TX.buf[TX.tail] = tx_chunk[i];
        }
      }
    }
  }
}


#include <string.h>
uint8_t sendbuf[128];
/* helper tulis LE */
static inline void wr32(uint8_t *p, uint32_t v){ memcpy(p, &v, 4); }
static inline void wr64(uint8_t *p, double   v){ memcpy(p, &v, 8); }

/* kirim lewat ring TX ? loopCOM() yang mengirimkannya */
void Send_ControlStatus(const control_status *c)
{
    uint8_t buf[64];      // 4 header + 52 payload + 2 CRC = 58
    uint16_t idx = 0;

    // header
    buf[idx++] = 0xAA;
    buf[idx++] = 0xCC;

    // length payload (fix 52)
    const uint16_t payload_len = 52;
    buf[idx++] = (uint8_t)(payload_len & 0xFF);
    buf[idx++] = (uint8_t)(payload_len >> 8);

    // payload (pack manual, no padding)
    wr32(&buf[idx], c->logtick);                idx += 4;
    wr64(&buf[idx], c->degree);                 idx += 8;
    wr64(&buf[idx], c->cmX);                    idx += 8;
    wr64(&buf[idx], c->setspeed);               idx += 8;
    wr64(&buf[idx], c->reserved[0]);            idx += 8;
    wr64(&buf[idx], c->reserved[1]);            idx += 8;
    wr64(&buf[idx], c->reserved[2]);            idx += 8;

    // CRC16 sederhana: sum16(payload)
    uint16_t crc = 0;
    for (uint16_t i = 4; i < 4 + payload_len; ++i) crc += buf[i];
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)(crc >> 8);

    // MASUK ANTRIAN TX (non-blocking); loopCOM() yg ngirim
    COM_Write(buf, idx);
}
#endif