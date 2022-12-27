#ifndef OLED_H__
#define OLED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "u8g2.h"
#include "u8x8.h"

// #define TWI_ADDRESSES      127

#ifdef CLASSIC_LAYOUT
#define OLED_I2C_PIN_SCL 27
#define OLED_I2C_PIN_SDA 26
#else

// P0.28 - SCL (TWI) / CLK (SPI)
// P0.29 - SDA (TWI) / DI (SPI)


typedef enum 
{
    FIRST_ROW = 30,
    SECOND_ROW = 45,
    THIRD_ROW = 60,
    FOURTH_ROW = 75,
    FIFTH_ROW = 90,
    SIXTH_ROW = 105,
} display_row_t;


typedef enum
{
    NONE,
    ERG,
    SIM,
} sim_mode_t;

#define OLED_SPI_PIN_CLK 28 // yellow jumper to yellow wire at OLED
#define OLED_SPI_PIN_DI 29 // white jumper to purple wire at OLED
#define OLED_SPI_PIN_DC 3 // black jumper to green wire at OLED
#define OLED_SPI_PIN_CS 4 //  jumper to orange wire at OLED
#define OLED_SPI_PIN_RESET 31  // green jumper to white wire at OLED

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
    double grade;
    bool gear_up_indicator;
    bool gear_down_indicator;
    bool sterzo_active;
    bool ftms_active;

    uint8_t gear_up_counter;
    uint8_t gear_down_counter;

    uint8_t trigger_up_counter;
    uint8_t trigger_down_counter;

    uint8_t wind_change_counter;
    uint8_t gravitational_change_counter;
    uint8_t rolling_change_counter;
    
    int16_t sim_resistance;
    uint8_t trainer_resistance;
    int16_t raw_resistance;

    int16_t wind_resistance;
    int16_t rolling_resistance;
    int16_t gravitational_resistance;

    bool tick;
    sim_mode_t mode;
} oled_data_t;

typedef enum
{
    GEAR_UP_INDICATOR,
    GEAR_DOWN_INDICATOR
} gear_indicator_t;

void oled_initialize();

static uint8_t u8g2_nrf_gpio_and_delay_spi_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
static uint8_t u8x8_HW_com_spi_nrf52832(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
static void spi_init (void);

static void oled_printGear(int8_t gear);
static void oled_printGearRatio(double gear_ratio);
static void oled_printPower(uint16_t power);
static void oled_printKmh(double kmh);
static void oled_printCadence(double rpm);
static void oled_printSterzoDeactivatedIndicator();
static void oled_printFTMSActivatedIndicator();
static void oled_printGearIndicator(gear_indicator_t indicator);
static void oled_printTrainerResistance(uint8_t sim_resistance);
static void oled_printSimResistance(int16_t sim_resistance);
static void oled_printRawResistance(int16_t raw_resistance);
static void oled_printGravitationalResistance(int16_t gravitational_resistance);
static void oled_printRollingResistance(int16_t rolling_resistance);
static void oled_printWindResistance(int16_t wind_resistance);
static void oled_printGrade(double grade);
static void oled_printTick();
static void oled_printTriggerDownIndicator();
static void oled_printTriggerUpIndicator();
static void oled_printWindChangeIndicator();
static void oled_printGravitationalChangeIndicator();
static void oled_printRollingChangeIndicator();
static void oled_timer_handler(void * p_context);
static void oled_redraw_handler(void * p_context);
static void oled_printmode(sim_mode_t mode);
void oled_printStringAt(display_row_t x, uint8_t y, char *str, bool cls, bool printImmediately);

void startOLEDUpdates();
void stopOLEDUpdates();
void updateOLEDDisplay();

#ifdef __cplusplus
}
#endif

#endif // OLED_H__
