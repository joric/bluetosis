#include "nrf_drv_twi.h"

#define SDA_PIN 11
#define SCL_PIN 12

#define SSD1306_I2C_ADDRESS 0x3c
#define SSD1306_LCDWIDTH    128
#define SSD1306_LCDHEIGHT   32

#define SCREEN_W SSD1306_LCDWIDTH
#define SCREEN_H SSD1306_LCDHEIGHT
#define BUFSIZE (SCREEN_W * SCREEN_H / 8)

#define buf_padding 4
static uint8_t buf_alloc[SCREEN_W * SCREEN_H / 8 + buf_padding];
static uint8_t *buf = buf_alloc + buf_padding;
static bool display_present = false;

static const uint8_t ASCII[][5] = {
  {0x00, 0x00, 0x00, 0x00, 0x00} // 20
  , {0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
  , {0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
  , {0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
  , {0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
  , {0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
  , {0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
  , {0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
  , {0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
  , {0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
  , {0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
  , {0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
  , {0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
  , {0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
  , {0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
  , {0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
  , {0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
  , {0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
  , {0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
  , {0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
  , {0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
  , {0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
  , {0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
  , {0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
  , {0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
  , {0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
  , {0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
  , {0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
  , {0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
  , {0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
  , {0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
  , {0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
  , {0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
  , {0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
  , {0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
  , {0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
  , {0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
  , {0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
  , {0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
  , {0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
  , {0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
  , {0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
  , {0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
  , {0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
  , {0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
  , {0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
  , {0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
  , {0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
  , {0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
  , {0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
  , {0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
  , {0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
  , {0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
  , {0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
  , {0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
  , {0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
  , {0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
  , {0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
  , {0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
  , {0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
  , {0x02, 0x04, 0x08, 0x10, 0x20} // 5c ?
  , {0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
  , {0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
  , {0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
  , {0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
  , {0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
  , {0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
  , {0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
  , {0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
  , {0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
  , {0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
  , {0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
  , {0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
  , {0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
  , {0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
  , {0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
  , {0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
  , {0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
  , {0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
  , {0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
  , {0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
  , {0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
  , {0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
  , {0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
  , {0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
  , {0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
  , {0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
  , {0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
  , {0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
  , {0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
  , {0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
  , {0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
  , {0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
  , {0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
  , {0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
  , {0x78, 0x46, 0x41, 0x46, 0x78} // 7f ?
};

void oledWriteCharacter(char character, uint8_t ** pp) {
  uint8_t * p = *pp;
  for (int i = 0; i < 5; i++) {
    *p++ = ASCII[character - 0x20][i];
  }
  *p++ = 0x00; // 1px space
  *pp = p;
}

void oledWriteString(const char *characters, int x, int y) {
  uint8_t * p = buf + y * SCREEN_W / 8 + x;
  while (*characters) {
    oledWriteCharacter(*characters++, &p);
  }
}


static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
static uint16_t send_count = 0;
static volatile bool twi_evt_done = false;  // volatile works omg

void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context) {
    if (p_event->type == NRF_DRV_TWI_EVT_DONE) {
        send_count++;
        twi_evt_done = true;
    }
}

void ssd1306_command(uint8_t c) {

    if (!display_present)
        return;

    uint8_t dta_send[] = { 0x00, c };
    if (!nrf_drv_twi_tx(&m_twi, SSD1306_I2C_ADDRESS, dta_send, 2, false)) {
        while (twi_evt_done != true);
        twi_evt_done = false;
    } else {
        nrf_delay_ms(10);
    }
}

void cmd(uint8_t b, ...) {
    va_list v;
    va_start(v, b);
    ssd1306_command(b);
    for (;;) {
        uint8_t c = va_arg(v, int);
        if (c == 0xff)
            break;
        ssd1306_command(c);
    }
}


void Oled_Draw(uint8_t * buf) {

    if (!display_present)
        return;

    ret_code_t ret;
    cmd(0x21, 0, SCREEN_W - 1, 0x22, 0, SCREEN_H == 64 ? 7 : SCREEN_H == 32 ? 3 : 1, 0xff);

    // ok this is tricky. the fastest way to send the screen to ssd1306
    // is to use large batches with a leading data command byte (0x40)
    // just patch them inplace, make sure that *(buf-1) points to valid memory

    const int batch_size = 32;
    uint8_t *data = buf - 1;

    for (int i = 0; i < BUFSIZE; i += batch_size) {

        uint8_t tmp = *data;
        *data = 0x40;

        ret = nrf_drv_twi_tx(&m_twi, SSD1306_I2C_ADDRESS, data, batch_size + 1, false);
        if (ret != 0) {
            //printf("ssd1306 send failed, err = %d\n", ret);
            nrf_delay_ms(10);
        } else {
            while (twi_evt_done != true);
            twi_evt_done = false;
        }

        *data = tmp;
        data += batch_size;
    }
}

void display_init() {
    //printf("%s\n", __FUNCTION__);


    nrf_drv_twi_config_t twi_config = {
        .scl = SCL_PIN,
        .sda = SDA_PIN,
        .frequency = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    nrf_drv_twi_enable(&m_twi);

    uint8_t dta_send[] = { 0x00, 0x07 };

    if (nrf_drv_twi_tx(&m_twi, SSD1306_I2C_ADDRESS, dta_send, 2, false) == NRF_SUCCESS) {
        nrf_delay_ms(250);
        if (twi_evt_done)
            display_present = true;
    }

    twi_evt_done = false;

    cmd(0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x02, 0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0x2E, 0xAF, -1);

    memset(buf, 0, BUFSIZE);

    Oled_Draw(buf);
}

//////////////////////////////////////////////////////////
// copypasted from ts100tris almost verbatim

/*
#define u8 unsigned char
#define u16 unsigned short
#define u32 unsigned int

#define FIELD_W 7
#define FIELD_H 29
// started to get OOM with that, remind me to use bitmask array
static uint8_t screen[FIELD_W][FIELD_H];

#define NUM_FIGURES 7

#define BMP_WIDTH SCREEN_W
#define BMP_HEIGHT SCREEN_H
#define FALSE 0
#define TRUE 1

static uint8_t map[4][4];
int px, py, score, nextmap;

// chosen by fair dice roll, guaranteed to be random
unsigned int seed = 0x269ec3;
int rnd(int range) {
    seed = seed * 0x343fD + 0x269ec3;
    return ((seed >> 16) & 0x7fff) % range;
}

#define fmap(k,i,j) (p[k]&(1<<(i*4+j)))
u16 p[7] = { 51, 4369, 116, 71, 54, 99, 39 };

void putpixel(int x, int y, int color, u8 * buf) {
    if ((x < 0 || x >= BMP_WIDTH) || (y < 0 || y >= BMP_HEIGHT))
        return;
    u8 b = 1 << (y % 8);
    buf += y / 8 * BMP_WIDTH + (x % BMP_WIDTH);
    if (color)
        *buf |= b;
    else
        *buf &= ~b;
}

void draw_cube(int j, int i) {
    int dx = -10;
    int dy = 2;
    int p = 4; // tetris cube size
    for (int w = 0; w < p; w++)
        for (int h = 0; h < p; h++)
            putpixel(dx + (SCREEN_W - 1 - (i * p + w)), dy + (j * p + h), 1, buf);
}

void print(void) {

    int i, j;
    memset(buf, 0, BUFSIZE);

    // draw field (gotta use bit array for that)
    for (i = 0; i < FIELD_H; i++)
        for (j = 0; j < FIELD_W; j++)
            if (screen[j][i])
                draw_cube(j, i);

    // draw current figure
    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
            if (map[j][i])
                draw_cube(px+j, py+i);

    // draw field borders
    for (i = 0; i < BMP_WIDTH; i++) {
        putpixel(i, 0, 1, buf);
        putpixel(i, BMP_HEIGHT - 1, 1, buf);
    }
    for (i = 0; i < BMP_HEIGHT; i++) {
        putpixel(0, i, 1, buf);
    }

}

int valid(int x, int y) {
    int i, j;
    if (x < 0)
        return FALSE;
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            if (map[j][i]) {
                if ((j + x >= FIELD_W) || (i + y >= FIELD_H))
                    return FALSE;
                if (screen[j + x][i + y])
                    return FALSE;
            }
    return true;
}

#define inv(x) ((x*(-1))+3)

void rotatemap(void) {
    int _map[4][4];
    int i, j, sx = 4, sy = 4;

    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++) {
            _map[j][i] = map[j][i];
            if (map[j][i]) {
                if (i < sx)
                    sx = i;
                if (inv(j) < sy)
                    sy = inv(j);
            }
            map[j][i] = 0;
        }

    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            if (_map[inv(i)][j])
                map[j - sx][i - sy] = 1;

    if (!valid(px, py))
        for (i = 0; i < 4; i++)
            for (j = 0; j < 4; j++)
                map[j][i] = _map[j][i];
}

void deleteline(void) {
    int i, j, k, cl;

    for (i = FIELD_H - 1; i >= 0; i--) {
        cl = 1;

        for (j = 0, cl = 1; j < FIELD_W; j++)
            if (!screen[j][i])
                cl = 0;

        if (cl) {
            score += (((i * (-1)) + FIELD_H) * 10);
            for (k = i; k > 0; k--) {
                for (j = 0; j < FIELD_W; j++) {
                    screen[j][k] = screen[j][k - 1];
                }
            }
            i++;
            print();
        }
    }
}

void createmap(void) {
    int i, j;
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            map[j][i] = fmap(nextmap, i, j) ? 1 : 0;
    py = 0;
    px = FIELD_W / 2;
    nextmap = rnd(NUM_FIGURES);
}

void clearscreen(void) {
    int i, j;
    for (i = 0; i < FIELD_H; i++)
        for (j = 0; j < FIELD_W; j++)
            screen[j][i] = 0;
}

void startgame(void) {
    clearscreen();
    px = FIELD_W / 2;
    py = 0;
    score = 0;
    nextmap = rnd(NUM_FIGURES);
    createmap();
}

int gameover(void) {
    int i;
    for (i = 0; i < FIELD_W; i++)
        if (screen[i][0])
            return true;
    return false;
}

void advancefigure(void) {
    int i, j;

    if (!valid(px, py + 1)) {
        for (i = 0; i < 4; i++)
            for (j = 0; j < 4; j++)
                if (map[j][i])
                    screen[px + j][py + i] = 1;
        createmap();
        deleteline();
    } else
        py++;
}

void dropfigure(void) {
    int i, j;
    for (; valid(px, py + 1); py++);
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            if (map[j][i])
                screen[px + j][py + i] = 1;
}

int bInit = 0;
int UI_TIMER = 50;
int KD_TIMER = 5;

void display_keypress(uint8_t key) {
    switch (key) {
        case KC_UP:
            rotatemap();
            break;
        case KC_DOWN:
            dropfigure();
            print();
            deleteline();
            createmap();
            break;
        case KC_LEFT:
            if (valid(px - 1, py))
                px--;
            break;
        case KC_RIGHT:
            if (valid(px + 1, py))
                px++;
            break;
        default:
            break;
    }
}
*/


#define RINGBUF_SIZE 16
int display_keypos = 0;
uint8_t display_key = 0;
char display_ringbuf[RINGBUF_SIZE+1] = {0};

void display_keypress(uint8_t key) {
    display_keypos = (display_keypos + 1) % RINGBUF_SIZE;
    display_ringbuf[display_keypos] = (char)(key%128)+32;
}

void display_update(void) {

/*
    UI_TIMER--;
    KD_TIMER--;

    if (!bInit) {
        startgame();
        bInit = 1;
    }

    if (UI_TIMER <= 0) {
        UI_TIMER = 20;
        advancefigure();
    }

    print();

    if (gameover())
        startgame();
*/
    oledWriteString("Jorian is ready...", 0,0);
    oledWriteString("Layer 1", 0, 8);
    display_ringbuf[RINGBUF_SIZE] = 0;
    oledWriteString(display_ringbuf, 0, 16);
    Oled_Draw(buf);
}
