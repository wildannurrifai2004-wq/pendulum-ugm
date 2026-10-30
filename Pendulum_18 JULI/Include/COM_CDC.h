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
static uint8_t  pay[20];           // ax,ay,rx,ry (8) + buttons (2) = 10
static uint8_t  pay_idx = 0;
static uint16_t sum = 0;           // checksum running (low 16 bits)
static uint16_t rx_crc = 0;

static inline void parser_reset(void){
  st = ST_WAIT_AA; typ=0; seq=0; pay_idx=0; sum=0; rx_crc=0;
}


// ============================================================
// TAMBAHKAN DI com_cdc.h - GAINS MANAGEMENT
// ============================================================


Gains_t gGains = {
    .K_TH = -2.50f * 57.0f * 12.0f,  // -1710.0
    .K_TH_D = -0.030f * 57.0f * 18.0f, // -30.78
    .K_X = 3.0f,
    .K_X_D = -1.6f * 2.0f,  // -3.2
    .K_X_INT = 0.0f,
    .updated = 0,
    .last_update_ms = 0
};


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

/* ====== API ringan ====== */
static uint32_t COM_Available(void){ return r_count(&RX); }
static uint32_t COM_Read(uint8_t *dst, uint32_t maxlen){ return r_read(&RX, dst, maxlen); }
static uint32_t COM_Write(const uint8_t *src, uint32_t len){ return r_write(&TX, src, len); }


// ============================================================
// MODIFIKASI joystick_parser_feed() - TAMBAH TYPE 0x02
// ============================================================

void Send_ResetACK(void)
{
    uint8_t ack_buf[8];
    uint16_t idx = 0;

    // Header khusus untuk Reset ACK
    ack_buf[idx++] = 0xAA;
    ack_buf[idx++] = 0xEE;  // Reset ACK identifier

    // Payload: just status byte
    ack_buf[idx++] = 0x01;  // 0x01 = reset success

    // CRC16
    uint16_t crc = 0;
    for (uint16_t i = 2; i < idx; ++i) crc += ack_buf[i];
    ack_buf[idx++] = (uint8_t)(crc & 0xFF);
    ack_buf[idx++] = (uint8_t)(crc >> 8);

    COM_Write(ack_buf, idx);
}

void Send_SwingACK(const SwingParams_t *s)
{
    uint8_t buf[32];
    uint16_t idx = 0;

    // Header khusus untuk Swing ACK: 0xAA 0xBB
    buf[idx++] = 0xAA;
    buf[idx++] = 0xBB;

    // Payload: echo swing params (5 floats = 20 bytes)
    memcpy(&buf[idx], &s->kE,        4); idx += 4;
    memcpy(&buf[idx], &s->kx_swing,  4); idx += 4;
    memcpy(&buf[idx], &s->margin_E,  4); idx += 4;
    memcpy(&buf[idx], &s->catch_deg, 4); idx += 4;
    memcpy(&buf[idx], &s->catch_vel, 4); idx += 4;

    // CRC16 simple: sum16(payload)
    uint16_t crc = 0;
    for (uint16_t i = 2; i < idx; ++i) crc += buf[i];
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)(crc >> 8);

    // Kirim via ring buffer
    COM_Write(buf, idx);
}

void Send_GainsACK(const Gains_t *g)
{
    uint8_t buf[32];
    uint16_t idx = 0;

    // Header khusus untuk ACK
    buf[idx++] = 0xAA;
    buf[idx++] = 0xDD;

    // Payload: echo gains yang diterima (5 floats = 20 bytes)
    memcpy(&buf[idx], &g->K_TH,     4); idx += 4;
    memcpy(&buf[idx], &g->K_TH_D,   4); idx += 4;
    memcpy(&buf[idx], &g->K_X,      4); idx += 4;
    memcpy(&buf[idx], &g->K_X_D,    4); idx += 4;
    memcpy(&buf[idx], &g->K_X_INT,  4); idx += 4;

    // CRC16 simple: sum16(payload)
    uint16_t crc = 0;
    for (uint16_t i = 2; i < idx; ++i) crc += buf[i];
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)(crc >> 8);

    // Kirim via ring buffer
    COM_Write(buf, idx);
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
      
      // Tentukan panjang payload sesuai type
      uint8_t expected_len;
      if      (typ == 0x01) expected_len = 10;  // joystick
      else if (typ == 0x02) expected_len = 20;  // gains (5 floats)
      else if (typ == 0x03) expected_len = 0;   // reset, no payload
      else if (typ == 0x04) expected_len = 8;   // physics params (2 floats)
      else if (typ == 0x05) expected_len = 20;  // swing params (5 floats: kE, kx_swing, margin_E, catch_deg, catch_vel)
      else                  expected_len = 0;
      
      if (pay_idx >= expected_len) 
        st = ST_CRC_LO;
      break;
			
    case ST_CRC_LO:
      rx_crc  = b;
      st      = ST_CRC_HI;
      break;
		
    case ST_CRC_HI:
      rx_crc |= ((uint16_t)b << 8);
      
      // Type 0x01: Joystick (existing code)
      if (typ == 0x01){
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
      // Type 0x02: Set Gains (NEW!)
      else if (typ == 0x02){
        // Unpack 5 floats (little-endian)
        memcpy(&gGains.K_TH,     &pay[0],  4);
        memcpy(&gGains.K_TH_D,   &pay[4],  4);
        memcpy(&gGains.K_X,      &pay[8],  4);
        memcpy(&gGains.K_X_D,    &pay[12], 4);
        memcpy(&gGains.K_X_INT,  &pay[16], 4);
        gGains.updated = 1;
        gGains.last_update_ms = HAL_GetTick();
        
        // Kirim ACK balik ke PC
        Send_GainsACK(&gGains);
      }

      else if (typ == 0x03) {
        // Set flag untuk reset FSM
        extern bool resetReq;  // defined in global.h
        resetReq = true;
        extern char dbg[100];
        sprintf(dbg, "RX RESET CMD");
        Send_ResetACK();
      }

      else if (typ == 0x04) {
        // Paket parameter fisik pendulum dari GUI Python
        // Payload: float length_m (4 byte) + float mass_kg (4 byte)
        float new_length_m, new_mass_kg;
        memcpy(&new_length_m, &pay[0], 4);
        memcpy(&new_mass_kg,  &pay[4], 4);

        // Validasi rentang sebelum update
        if (new_length_m >= 0.01f && new_length_m <= 2.0f &&
            new_mass_kg  >= 0.01f && new_mass_kg  <= 10.0f) {
            gPhysics.length_m   = new_length_m;
            gPhysics.mass_kg    = new_mass_kg;
            gPhysics.updated    = 1;
            gPhysics.last_update_ms = HAL_GetTick();
        }
      }

      else if (typ == 0x05) {
        // Paket parameter swing-up & handover dari GUI Python (5 floats = 20 bytes)
        float new_kE, new_kx_swing, new_margin_E, new_catch_deg, new_catch_vel;
        memcpy(&new_kE,        &pay[0],  4);
        memcpy(&new_kx_swing,  &pay[4],  4);
        memcpy(&new_margin_E,  &pay[8],  4);
        memcpy(&new_catch_deg, &pay[12], 4);
        memcpy(&new_catch_vel, &pay[16], 4);

        // Validasi batas nilai aman sebelum diterapkan
        if (new_kE >= 5.0f && new_kE <= 3000.0f &&
            new_kx_swing >= 0.0f && new_kx_swing <= 5.0f &&
            new_margin_E >= 0.8f && new_margin_E <= 1.5f &&
            new_catch_deg >= 3.0f && new_catch_deg <= 30.0f &&
            new_catch_vel >= 0.5f && new_catch_vel <= 10.0f) {
            
            gSwing.kE        = new_kE;
            gSwing.kx_swing  = new_kx_swing;
            gSwing.margin_E  = new_margin_E;
            gSwing.catch_deg = new_catch_deg;
            gSwing.catch_vel = new_catch_vel;
            gSwing.updated   = 1;
            gSwing.last_update_ms = HAL_GetTick();

            Send_SwingACK(&gSwing);
        }
      }

      parser_reset();
      break;
  }
}



/* ====== TX state ====== */
static volatile uint8_t tx_busy = 0;
static uint8_t tx_chunk[CDC_MAX_PKT];


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
     n = COM_Read(buf, sizeof(buf));
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
    uint8_t buf[72];      // 4 header + 60 payload + 2 CRC = 66
    uint16_t idx = 0;

    // header
    buf[idx++] = 0xAA;
    buf[idx++] = 0xCC;

    // length payload (fix 52)
    const uint16_t payload_len = 60;
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
    wr64(&buf[idx], c->reserved[3]);     				idx += 8;

    // CRC16 sederhana: sum16(payload)
    uint16_t crc = 0;
    for (uint16_t i = 4; i < 4 + payload_len; ++i) crc += buf[i];
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)(crc >> 8);

    // MASUK ANTRIAN TX (non-blocking); loopCOM() yg ngirim
    COM_Write(buf, idx);
}
#endif
