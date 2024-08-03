#include <Arduino.h>

#include "font.h"

// LCD connected Pins
#define LCD_SI 2
#define LCD_SCL 3
#define LCD_A0 4
#define LCD_CS 5
#define LCD_RST 6

// LCD control bytes
#define DISPLAY_ON 0xaf
#define DISPLAY_OFF 0xae
#define DISPLAY_ADC_NORMAL 0xa0
#define DISPLAY_ADC_REVERSE 0xa1
#define DISPLAY_DISPLAY_NORMAL 0xa6
#define DISPLAY_DISPLAY_REVERSE 0xa7
#define DISPLAY_ALL_POINTS_OFF 0xa4
#define DISPLAY_ALL_POINTS_ON 0xa5
#define DISPLAY_BIAS_RATIO_ONE_SEVENTH 0xa3
#define DISPLAY_BIAS_RATIO_ONE_NINETH 0xa2
#define DISPLAY_INTERNAL_RESET 0xe2
#define DISPLAY_COM_NORMAL 0xc0   // last 3 bits insignificant
#define DISPLAY_COM_REVERSE 0xc8  // last 3 bits insignificant
#define DISPLAY_STATIC_INDICATOR_OFF 0xac
#define DISPLAY_STATIC_INDICATOR_ON 0xad
#define DISPLAY_STATIC_INDICATOR_FLASHING_ON 0x01   // follows DISPLAY_STATIC_INDICATOR_OFF/ON
#define DISPLAY_STATIC_INDICATOR_FLASHING_OFF 0x00  // follows DISPLAY_STATIC_INDICATOR_OFF/ON
#define DISPLAY_SELECT_BOOSTER_RATIO 0xf8
#define DISPLAY_SELECT_BOOSTER_RATIO_2x3x4x 0x00
#define DISPLAY_SELECT_BOOSTER_RATIO_5x 0x01
#define DISPLAY_NO_OPERATION 0xe3
#define DISPLAY_START_LINE_0 0x40
#define DISPLAY_POWER_CONTROL_BOOSTER_REGULATOR_FOLLOWER 0x2f  // 0x28 - 0x2f select internal power supply operating mode
#define DISPLAY_VOLTAGE_REGULATOR_SET 0x27                     // 0x20 - 0x27 select voltage regulator resistor ratio
#define DISPLAY_ELECTRONIC_VOLUME_SET 0x81                     // followed by 0x00 - 0x3f

#define DISPLAY_SET_PAGE 0xb0              // 0xb0 - 0xbf (but only pages 0 - 7 used)
#define DISPLAY_SET_COLUMN_MOST_SIG 0x10   // 0x10 - 0x1f
#define DISPLAY_SET_COLUMN_LEAST_SIG 0x00  // 0x00 - 0x0f

void setup() {
  pinMode(LCD_SI, OUTPUT);
  pinMode(LCD_SCL, OUTPUT);
  pinMode(LCD_A0, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RST, OUTPUT);

  Serial.begin(115200);
  Serial.println("====================");

  // initialize lcd as described in datasheet
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_RST, LOW);
  digitalWrite(LCD_RST, HIGH);

  lcd_control_byte(DISPLAY_INTERNAL_RESET);

  lcd_control_byte(DISPLAY_START_LINE_0);
  lcd_control_byte(DISPLAY_ADC_REVERSE);
  lcd_control_byte(DISPLAY_COM_NORMAL);
  lcd_control_byte(DISPLAY_DISPLAY_NORMAL);

  lcd_control_byte(DISPLAY_BIAS_RATIO_ONE_NINETH);
  lcd_control_byte(DISPLAY_POWER_CONTROL_BOOSTER_REGULATOR_FOLLOWER);

  lcd_control_byte(DISPLAY_SELECT_BOOSTER_RATIO);
  lcd_control_byte(DISPLAY_SELECT_BOOSTER_RATIO_2x3x4x);

  lcd_control_byte(DISPLAY_VOLTAGE_REGULATOR_SET);
  lcd_control_byte(DISPLAY_ELECTRONIC_VOLUME_SET);
  lcd_control_byte(0x0c);

  lcd_control_byte(DISPLAY_STATIC_INDICATOR_OFF);
  lcd_control_byte(DISPLAY_STATIC_INDICATOR_FLASHING_OFF);

  lcd_control_byte(DISPLAY_ON);
  // end lcd init

  lcd_clear();
  lcd_set_pos(0, 0);
}

void loop() {
  delay(500);
  Serial.println();

  lcd_set_pos(4, 20);
  lcd_write("gehoert in");

  lcd_set_pos(5, 20);
  lcd_write("den muell");
}

void lcd_write(String str) {
  for (uint32_t j = 0; j < str.length(); j++) {
    boolean charDone = false;
    for (uint8_t i = 0; i < 7 && !charDone; i++) {
      byte b = font_standard[str.charAt(j) - 0x20][i];
      if (b == 0xaa) {
        lcd_data_byte(0x00);
        charDone = true;
      } else {
        lcd_data_byte(b);
      }
    }
  }
}

void lcd_clear() {
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 128; j++) {
      lcd_set_pos(i, j);
      lcd_data_byte(0x00);
    }
  }
}

void lcd_set_pos(uint8_t row, uint8_t col) {
  byte least_significant_col = col & 0b00001111;
  byte most_significant_col = (col & 0b11110000) >> 4;

  lcd_control_byte(row | DISPLAY_SET_PAGE);
  lcd_control_byte(most_significant_col | DISPLAY_SET_COLUMN_MOST_SIG);
  lcd_control_byte(least_significant_col | DISPLAY_SET_COLUMN_LEAST_SIG);
}

void lcd_control_byte(byte byte) {
  lcd_byte(0, byte);
}

void lcd_data_byte(byte byte) {
  lcd_byte(1, byte);
}

void lcd_byte(boolean A0, byte byte) {
  // enable chipselect
  digitalWrite(LCD_CS, LOW);
  // prepare clock pulse
  digitalWrite(LCD_SCL, HIGH);

  // set A0 state (0 = display control, 1 = data)
  digitalWrite(LCD_A0, A0 ? HIGH : LOW);

  Serial.print(A0 ? "1" : "0");
  Serial.print(" ");

  // send byte
  for (uint16_t i = 0; i < 8; i++) {
    uint8_t signal = (byte & (0b10000000 >> i)) ? HIGH : LOW;

    digitalWrite(LCD_SCL, LOW);
    digitalWrite(LCD_SI, signal);
    digitalWrite(LCD_SCL, HIGH);

    // debug
    Serial.print(signal);
  }

  Serial.println();

  // disable chipselect
  digitalWrite(LCD_CS, HIGH);
}
