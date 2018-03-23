#ifndef LOG_H
#define LOG_H

#include <stdint.h>
#include <stdbool.h>

#define NAME_LEN  8

struct LogInfo {
  uint16_t stamp;
  uint16_t test;
  int32_t length;
  float sampleRate;
  float mass;
  char name[NAME_LEN];
};

typedef enum {  lc_alpha, 
                lc_dynp, 
                lc_alti,
		lc_accx,
		lc_accy,
		lc_accz,
                lc_roll, 
                lc_rollr, 
                lc_pitch, 
                lc_pitchr, 
                lc_head, 
		lc_yawr,
                lc_aile,
                lc_elev,
		lc_rudder,
		lc_flap,
                lc_ailestk, 
                lc_elevstk,
		lc_thrstk,
		lc_rudstk,
                lc_mode, 
                lc_status, 
                lc_trim, 
                lc_test, 
                lc_gain, 
                lc_channels } ChannelId_t;

typedef enum {
  lt_integer,
  lt_real,
  lt_percent,
  lt_angle
} ChannelType_t;

struct LogChannel {
  const char *name;
  ChannelType_t type;
  float small, large;
  void *object;
  uint16_t value;
  uint32_t stamp;
};

#define TOKEN_MASK (1U<<15)
#define VALUE_MASK (TOKEN_MASK-1)
#define BYTE_MASK ((1<<8)-1)
#define ENTRY_TOKEN(t) (TOKEN_MASK | (t))
#define ENTRY_VALUE(v) (((uint16_t) v) & VALUE_MASK)
#define ENTRY_IS_TOKEN(e) ((e) & TOKEN_MASK)

typedef enum { t_stamp,
               t_mark, 
               t_channel = t_stamp + BYTE_MASK + 1,
               t_start = t_channel + BYTE_MASK + 1
            } LogToken_t;

extern struct LogChannel logChannels[];
extern float flapEncoded;
extern uint16_t modeEncoded, statusEncoded, testEncoded;

#endif

