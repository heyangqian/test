/***************************dbus.h*****************************/
#include "stdint.h"

#ifndef _DBUS_H_
#define _DBUS_H_

#define DBUS_BUF_SIZE    18
#define SBUS_RX_BUF_NUM  36u
/* ----------------------- RC Channel Value Definition---------------------------- */
#define CH_VALUE_MIN                   ((uint16_t)364    ) 
#define CH_VALUE_OFFSET                ((uint16_t)1024   ) 
#define CH_VALUE_MAX                   ((uint16_t)1684   )  
/* ----------------------- RC Switch Value Definition----------------------------- */
#define SW_UP                          ((uint16_t)1      ) 
#define SW_MID                          ((uint16_t)3     ) 
#define SW_DOWN                        ((uint16_t)2      ) 
#define switch_is_down(s) (s == SW_DOWN)
#define switch_is_mid(s) (s == SW_MID)
#define switch_is_up(s) (s == SW_UP)
/* ----------------------- PC Mouse Value Definition-------------------------------- */
#define MOUSE_MOVE_VALUE_MIN         ((uint16_t)-32768 ) 
#define MOUSE_MOVE_VALUE_OFFSET      ((uint16_t)0      ) 
#define MOUSE_MOVE_VALUE_MAX         ((uint16_t)32767  ) 
#define MOUSE_BTN_UP                  ((uint8_t)0      ) 
#define MOUSE_BTN_DN                  ((uint8_t)1      ) 
/* ----------------------- PC Key Value Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W         ((uint16_t)0x01<<0) 
#define KEY_PRESSED_OFFSET_S         ((uint16_t)0x01<<1) 
#define KEY_PRESSED_OFFSET_A         ((uint16_t)0x01<<2) 
#define KEY_PRESSED_OFFSET_D         ((uint16_t)0x01<<3) 
#define KEY_PRESSED_OFFSET_SHIFT     ((uint16_t)0x01<<4) 
#define KEY_PRESSED_OFFSET_CTRL      ((uint16_t)0x01<<5) 
#define KEY_PRESSED_OFFSET_Q     		 ((uint16_t)0x01<<6) 
#define KEY_PRESSED_OFFSET_E         ((uint16_t)0x01<<7)  
#define KEY_PRESSED_OFFSET_R         ((uint16_t)0x01<<8)  
#define KEY_PRESSED_OFFSET_F         ((uint16_t)0x01<<9)  
#define KEY_PRESSED_OFFSET_G         ((uint16_t)0x01<<10)  
#define KEY_PRESSED_OFFSET_Z         ((uint16_t)0x01<<11)  
#define KEY_PRESSED_OFFSET_X         ((uint16_t)0x01<<12)  
#define KEY_PRESSED_OFFSET_C         ((uint16_t)0x01<<13)  
#define KEY_PRESSED_OFFSET_V       	 ((uint16_t)0x01<<14)  
#define KEY_PRESSED_OFFSET_B         ((uint16_t)0x01<<15)  



//W:1 S:2 A:4 D:8 Q:64 E:128 R:256 F:512 G:1024 Z:2048 X:4096 C:8192 V:16384 B:32768 Shift:16 ctrl:32 



typedef struct
{
    struct
    {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        uint8_t  s1;
        uint8_t  s2;
    }rc;
    
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t l;
        uint8_t r;
    }mouse;
    
    struct
    {
        uint16_t v;
    }key;
    
    uint16_t res;
    
}DBUS;
extern DBUS rc_ctrl;
//extern const DBUS *get_rc_ctrl_control_point(void);
extern void DBUS_Enc(const DBUS* rc_ctrl,unsigned char* pbuf);
extern void DBUS_Dec(DBUS* rc_ctrl,volatile const unsigned char* pbuf);

#endif






