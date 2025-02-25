#ifndef MODBUS_HANDLER_H
#define MODBUS_HANDLER_H

#include <modbus/modbus.h>
#include <syslog.h>

#define MODBUS_IP "192.168.1.100"  // Change to match your TET-A4 device IP
#define MODBUS_PORT 502            // Modbus TCP port
#define MODBUS_REGISTER 0x00        // Register to write
#define MODBUS_VALUE 1              // Value to write

/**
 * @brief Initializes a Modbus connection.
 * @return Pointer to the initialized Modbus context.
 */
modbus_t* modbus_init();

/**
 * @brief Sends a Modbus signal to the TET-A4 device.
 * @param mb Pointer to the Modbus context.
 * @param register_addr Modbus register address.
 * @param value Value to write.
 */
void modbus_send_signal(modbus_t* mb, int register_addr, int value);

/**
 * @brief Closes the Modbus connection.
 * @param mb Pointer to the Modbus context.
 */
void modbus_cleanup(modbus_t* mb);

#endif // MODBUS_HANDLER_H
