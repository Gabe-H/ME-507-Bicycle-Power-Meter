#ifndef __MAX17048_H
#define __MAX17048_H

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

#define MAX17048_NODE DT_NODELABEL(max17048)

/** MAX17048 REGISTER VALUES **/
#define MAX17048_REG_VCELL 0x02       // ADC measurement of VCELL (78.125uV/cell)
#define MAX17048_REG_SOC 0x04         // Battery state of charge (1%/256)
#define MAX17048_REG_MODE 0x06        // Initates quick-start, reports hib. mode, enables slp. mode
#define MAX17048_REG_VERSION 0x08     // IC production version
#define MAX17048_REG_HIBRT 0x0A       // Controls thresholds for entering and exiting hib. mode
#define MAX17048_REG_CONFIG 0x0C      // Compensation to optimize performance, slp. mode, alrt
                                      // indicators and configuration
#define MAX17048_REG_VALRT 0x14       // Configures the VCELL range outside of which alrerts
                                      // are generated
#define MAX17048_REG_CRATE 0x16       // Approximate charge or discharge rate of the battery
                                      // (0.208%/hr)
#define MAX17048_REG_VRESET_ID 0x18   // Configures VCELL threshold below which the IC resets
                                      // itself, ID is a one-time-factory-programmable ID
#define MAX17048_REG_STATUS 0x1A      // Indicates overvoltage, undervoltage, SOC change,
                                      // SOC low, and reset alerts
#define MAX17048_REG_TABLE_BEGIN 0x40 // Configures battery parameters
#define MAX17048_REG_CMD 0xFE         // Sends POR command

#define MAX17048_VERSION_VAL 0x0012 // Version of installed ICs

// static const struct i2c_dt_spec icm_t;
typedef const struct i2c_dt_spec batt_mon_t;

int battery_monitor_init(batt_mon_t *mon);

uint16_t battery_monitor_read_voltage(batt_mon_t *mon);

uint8_t battery_monitor_read_soc(batt_mon_t *mon);

uint16_t battery_monitor_read_config(batt_mon_t *mon);

uint16_t battery_monitor_read_status(batt_mon_t *mon);

uint16_t battery_monitor_read_mode(batt_mon_t *mon);

uint8_t battery_monitor_read_pct(batt_mon_t *mon);

uint16_t battery_monitor_write_read(batt_mon_t *mon, uint8_t reg);

#endif /* __MAX17048_H */