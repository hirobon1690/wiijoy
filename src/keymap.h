#include "keicode.h"
#define WIIMOTE_VERTICAL

#define SHAKE_THRESHOLD 50
#define NC_SHAKE_THRESHOLD 80

#ifdef WIIMOTE_VERTICAL
#define WIIMOTE_A XBOX_A
#define WIIMOTE_B XBOX_B
#define WIIMOTE_1 XBOX_LB
#define WIIMOTE_2 XBOX_RB
#define WIIMOTE_MINUS XBOX_BACK
#define WIIMOTE_PLUS XBOX_START
#define WIIMOTE_HOME XBOX_GUIDE
#define WIIMOTE_DPAD_X XBOX_DPAD_X
#define WIIMOTE_DPAD_Y XBOX_DPAD_Y
#define WIIMOTE_SHAKE XBOX_A
#endif

#ifdef WIIMOTE_HORIZONTAL
#define WIIMOTE_A XBOX_X
#define WIIMOTE_B XBOX_Y
#define WIIMOTE_1 XBOX_A
#define WIIMOTE_2 XBOX_B
#define WIIMOTE_MINUS XBOX_BACK
#define WIIMOTE_PLUS XBOX_START
#define WIIMOTE_HOME XBOX_GUIDE
#define WIIMOTE_DPAD_X XBOX_DPAD_Y
#define WIIMOTE_DPAD_Y XBOX_DPAD_X
#define WIIMOTE_SHAKE XBOX_A
#endif

#define NUNCHUCK_C XBOX_Y
#define NUNCHUCK_Z XBOX_X
#define NUNCHUCK_STICK_X XBOX_LSTICK_X
#define NUNCHUCK_STICK_Y XBOX_LSTICK_Y
#define NUNCHUCK_SHAKE XBOX_B

#define CLASSIC_A XBOX_A
#define CLASSIC_B XBOX_B
#define CLASSIC_X XBOX_X
#define CLASSIC_Y XBOX_Y
#define CLASSIC_L XBOX_LB
#define CLASSIC_R XBOX_RB
#define CLASSIC_MINUS XBOX_BACK
#define CLASSIC_PLUS XBOX_START
#define CLASSIC_HOME XBOX_GUIDE
#define CLASSIC_LSTICK_PUSH XBOX_LSTICK_PUSH
#define CLASSIC_RSTICK_PUSH XBOX_RSTICK_PUSH
#define CLASSIC_LSTICK_X XBOX_LSTICK_X
#define CLASSIC_LSTICK_Y XBOX_LSTICK_Y
#define CLASSIC_LTRIGGER XBOX_LTRIGGER
#define CLASSIC_RSTICK_X XBOX_RSTICK_X
#define CLASSIC_RSTICK_Y XBOX_RSTICK_Y
#define CLASSIC_RTRIGGER XBOX_RTRIGGER