#ifndef OLED_H__
#define OLED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "u8g2.h"
#include "u8x8.h"

#define TWI_ADDRESSES      127

#ifdef CLASSIC_LAYOUT
#define OLED_I2C_PIN_SCL 27
#define OLED_I2C_PIN_SDA 26
#else
#define OLED_I2C_PIN_SCL 28
#define OLED_I2C_PIN_SDA 29
#endif

#define OLED_ADDR        0x3C

#define REDRAW_OLED 500 // Ori: 250


typedef struct {
    double kmh;
    // uint16_t rpm;
    uint16_t power;
    double cadence;
    int8_t gear;
    double gear_ratio;
    bool gear_up_indicator;
    bool gear_down_indicator;
    uint8_t gear_up_counter;
    uint8_t gear_down_counter;

    int16_t sim_resistance;
    uint8_t trainer_resistance;
    int16_t raw_resistance;
    bool tick;
} oled_data_t;

typedef enum
{
    GEAR_UP_INDICATOR,
    GEAR_DOWN_INDICATOR
} gear_indicator_t;

void oled_initialize();
static uint8_t u8g2_nrf_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
static uint8_t u8x8_HW_com_nrf52832(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
static void twi_init (void);
void oled_printStringAt(uint8_t x, uint8_t y, char *str, bool cls, bool printImmediately);
static void oled_printGear(int8_t gear);
static void oled_printGearRatio(double gear_ratio);
static void oled_printPower(uint16_t power);
static void oled_printKmh(double kmh);
static void oled_printCadence(double rpm);
static void oled_printGearIndicator(gear_indicator_t indicator);
static void oled_printTrainerResistance(uint8_t sim_resistance);
static void oled_printSimResistance(int16_t sim_resistance);
static void oled_printRawResistance(int16_t raw_resistance);
void oled_printTick();
void startOLEDUpdates();
void stopOLEDUpdates();


#ifdef __cplusplus
}
#endif

#endif // OLED_H__
