#include "neopixel.h"

neopixel_strip_t m_strip;
uint8_t dig_pin_num = RGB_DI_PIN;
uint8_t leds_per_strip = RGBLED_NUM;

#include "ble_radio_notification.h"

// does not work yet
void your_radio_callback_handler(bool radio_active) {
    if (radio_active == false) {
        printf("radio active==false");
        neopixel_show(&m_strip);
    }
}

void rgb_init() {
    neopixel_init(&m_strip, dig_pin_num, leds_per_strip);
    neopixel_clear(&m_strip);
    for (int i=0; i<3; i++) {
        neopixel_set_color_and_show(&m_strip, 0, 0,0,64);
        nrf_delay_ms(100);
        neopixel_set_color_and_show(&m_strip, 0, 0,0,0);
        nrf_delay_ms(100);
    }
}

uint64_t millis(void) {
  return(app_timer_cnt_get() / 32.768);
}

#define RGB_W 8
#define RGB_H 4

const int RGB_MATRIX[RGB_H][RGB_W] = {
    { 0, 13, 14, 15, 16, 17, 18, 0 },
    {12, 11,  8,  7,  4,  3, 19, 0 },
    { 0, 10,  9,  6,  5,  2, 20, 0 },
    { 0,  0,  0,  0,  0,  1, 21, 22}
};

void setColor(int i, uint32_t c) {
    uint8_t r = (c >> 16) & 0xff;
    uint8_t g = (c >> 8) & 0xff;
    uint8_t b = (c) & 0xff;
    neopixel_set_color(&m_strip, i, r,g,b);
}

void setPixel(int x, int y, uint32_t c) {
    int i = RGB_MATRIX[y][x]-1;
    if (i==-1) return;
    setColor(i, c);
}


uint32_t strip_Color(uint8_t r, uint8_t g, uint8_t b) {
    return (r << 16) | (g<<8) | b;
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(uint8_t WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip_Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip_Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip_Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

int usqrt(int x) {
    int a = x, b;
    if (x < 2) return x;
    b = x/a; a = (a+b)/2;
    b = x/a; a = (a+b)/2;
    b = x/a; a = (a+b)/2;
    return a;
}

void rgb_breathing() {
    int ms = millis();
    for (int i=0; i<leds_per_strip; i++) {
        setColor(i, Wheel(( ms/10 ) & 255));
    }
}

void rgb_radial() {
    int j = millis() / 10;
    for (int y=0; y<RGB_H; y++) {
        for (int x=0; x<RGB_W; x++) {
            int dx = x - RGB_H/2, dy = y - RGB_W/2;
            int r = usqrt(dx*dx + dy*dy);
            setPixel(x, y, Wheel(( j + r ) & 255));
        }
    }
}

void rgb_wipe() {
    int n = leds_per_strip, t = 1800;
    int k = n * (millis() % t) / t;
    for (int i=0; i<n; i++) {
        setColor(i, (i==k) ? 0x200000 : 0);
    }
}

void rgb_linear() {
    int j = millis()/2;
    for (int y=0; y<RGB_H; y++) {
        for (int x=0; x<RGB_W; x++) {
            setPixel(x, y, Wheel(( j + x*32 ) & 255));
        }
    }
}

uint64_t last_ms = 0;
uint64_t counter = 0;
int last_rgb_mode = 0;

typedef void (*func_type)(void);
func_type rgb_func[] = { rgb_linear, rgb_breathing, rgb_wipe };

// every 25 ms
// still small glitches
// needs to move to ble_radio_notification_init with radio_active == false
// ble_radio_notification_init(6, NRF_RADIO_NOTIFICATION_DISTANCE_5500US, your_radio_callback_handler);
void rgb_task() {

    int num_modes = sizeof(rgb_func)/sizeof(rgb_func[0]);

    if (rgb_mode<0)
        rgb_mode = 0;

    rgb_mode = rgb_mode % (num_modes + 1);

    // last mode means rgb disabled
    bool disabled = rgb_mode==num_modes;
    if (disabled && last_rgb_mode != rgb_mode) {
        neopixel_clear(&m_strip);
        neopixel_show(&m_strip);
    }

    last_rgb_mode = rgb_mode;

    if (disabled)
        return;

    uint64_t ms = millis();

    int fps = 15;
    if (ms >= last_ms + 1000/fps) {

        if (rgb_mode<sizeof(rgb_func)) rgb_func[rgb_mode]();

        uint8_t region;
        sd_nvic_critical_region_enter(&region);
        neopixel_show(&m_strip);
        sd_nvic_critical_region_exit(region);

        last_ms = ms;
    }
}

