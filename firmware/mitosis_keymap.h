#ifndef KEYMAP_H
#define KEYMAP_H

#include "qmk_defines.h"

enum mitosis_layers
{
  _BL,
  _RAISE,
  _LOWER
};

enum mitosis_keycodes
{
  FNKEY = SAFE_RANGE,
  RAISE,
  RGBMOD
};


#define _______ KC_TRNS
#define XXXXXXX KC_NO
#define DUAL_R  LT(_RAISE,KC_SPC)

#define RAISE_T(kc) LT(_RAISE, kc)
#define LOWER_T(kc) LT(_LOWER, kc)

#define MATRIX_ROWS 5
#define MATRIX_COLS 10

/*
  Mitosis to Jorian remapping, quick and dirty

  mitosis thumb keys:

  {XXXXXXX, S19, S18, S17, S16,   S16, S17, S18, S19, XXXXXXX },
  {XXXXXXX, S23, S22, S21, S20,   S20, S21, S22, S23, XXXXXXX }

  jorian thumb keys:
         
      S19                                    S19
  S22 S20                                    S20 S22
      S21                                    S21
             S17 S16 S18     S18 S16 S17



LBRC,            RGUI_T(KC_RBRC), \
RCTL_T(KC_QUOT), \                 
RALT_T(KC_BSLS), \                 

      GRV                                    LBR
  WIN CTL                                    QOT RBR
      ALT                                    BSL
             RAI SFT LOW     LOW SFT RAI

*/

//#define LSFT_T(kc) MT(MOD_LSFT, kc)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
[_BL] = {

  {KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,          KC_Y,    KC_U,    KC_I,    KC_O,     KC_P    },
  {KC_A,    KC_S,    KC_D,    KC_F,    KC_G,          KC_H,    KC_J,    KC_K,    KC_L,     KC_SCLN },
  {KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,          KC_N,    KC_M,    KC_COMM, KC_DOT,   KC_SLSH },

//  {XXXXXXX, KC_LALT, KC_ESC,  KC_TAB,  KC_QUOT,       KC_LEFT, KC_DOWN, KC_UP,   KC_RIGHT, XXXXXXX },
//  {XXXXXXX, KC_LCTL, KC_LGUI, KC_SPC,  KC_LSFT,       DUAL_R,  KC_BSPC, KC_ENT,  _______,  XXXXXXX }

//  {XXXXXXX, KC_GRV,  LOWER_T(KC_ENT),  RAISE_T(KC_TAB),  LSFT_T(KC_SPC),      LSFT_T(KC_SPC), RAISE_T(KC_TAB), LOWER_T(KC_ENT), KC_LBRC,  XXXXXXX },
//  {XXXXXXX, XXXXXXX,         KC_LGUI,          KC_LALT,         KC_LCTL,             KC_QUOT,         KC_BSLS,  KC_RBRC, XXXXXXX, XXXXXXX }

  {XXXXXXX, KC_GRV,  LOWER_T(KC_ENT),  RAISE_T(KC_TAB),          KC_SPC,             KC_RSFT, RAISE_T(KC_TAB), LOWER_T(KC_ENT), KC_LBRC,  XXXXXXX },
  {XXXXXXX, XXXXXXX,         KC_LGUI,          KC_LALT,         KC_LCTL,             KC_QUOT,         KC_BSLS,  KC_RBRC, XXXXXXX, XXXXXXX }

},

[_RAISE] = {
  {KC_1,    KC_2,    KC_3,   KC_4,     KC_5,          KC_6,    KC_7,    KC_8,    KC_9,     KC_0    },
  {KC_F1,   KC_F2,   KC_F3,  KC_F4,    KC_F5,         KC_F6,   KC_MINS, KC_EQL,  KC_LBRC,  KC_RBRC },
  {KC_F7,   KC_F8,   KC_F9,  KC_F10,   KC_F11,        KC_F12,  KC_BSLS, _______, _______,  KC_QUOT },

//  {XXXXXXX, _______, KC_BRK, KC_CAPS,  KC_GRV,        KC_HOME, KC_PGDN, KC_PGUP, KC_END,   XXXXXXX },
//  {XXXXXXX, _______, _______, _______, _______,       _______, KC_DEL,  KC_PSCR, _______,  XXXXXXX }

  {XXXXXXX, _______, _______, _______, _______,       _______, _______, _______, _______,   XXXXXXX },
  {XXXXXXX, XXXXXXX, RGBMOD, _______, _______,       _______, _______, RGBMOD, XXXXXXX,  XXXXXXX }

}
};

#endif //KEYMAP_H
