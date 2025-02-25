#include "modbus_handler.h"

modbus_t* modbus_init() {
    modbus_t *mb = modbus_new_tcp(MODBUS_IP, MODBUS_PORT);
    if (mb == NULL) {
        syslog(LOG_ERR, "Failed to create Modbus context");
        return NULL;
    }

    if (modbus_connect(mb) == -1) {
        syslog(LOG_ERR, "Modbus connection failed");
        modbus_free(mb);
        return NULL;
    }

    syslog(LOG_INFO, "Connected to Modbus device at %s", MODBUS_IP);
    return mb;
}

void modbus_send_signal(modbus_t* mb, int register_addr, int value) {
    if (mb == NULL) {
        syslog(LOG_ERR, "Modbus context is NULL. Cannot send signal.");
        return;
    }

    if (modbus_write_register(mb, register_addr, value) == -1) {
        syslog(LOG_ERR, "Failed to write to Modbus register %d", register_addr);
    } else {
        syslog(LOG_INFO, "Modbus signal sent successfully to register %d", register_addr);
    }
}

void modbus_cleanup(modbus_t* mb) {
    if (mb) {
        modbus_close(mb);
        modbus_free(mb);
        syslog(LOG_INFO, "Modbus connection closed");
    }
}
