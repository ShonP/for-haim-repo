#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <curl/curl.h>
#include "modbus_handler.h"  // Include Modbus handler

// Axis Camera Event API (change IP)
#define AXIS_EVENT_API "http://<AXIS_DEVICE_IP>/axis-cgi/param.cgi?action=list&group=Event"

void check_axis_event() {
    CURL *curl;
    CURLcode res;

    curl = curl_easy_init();
    if (!curl) {
        syslog(LOG_ERR, "Failed to initialize CURL");
        return;
    }

    curl_easy_setopt(curl, CURLOPT_URL, AXIS_EVENT_API);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        syslog(LOG_ERR, "Failed to fetch event data from Axis: %s", curl_easy_strerror(res));
    } else {
        syslog(LOG_INFO, "Event detected! Sending Modbus signal...");
        
        // Initialize Modbus connection
        modbus_t *mb = modbus_init();
        if (mb) {
            modbus_send_signal(mb, MODBUS_REGISTER, MODBUS_VALUE);
            modbus_cleanup(mb);
        }
    }

    curl_easy_cleanup(curl);
}

int main() {
    openlog("modbus_event_handler", LOG_PID | LOG_CONS, LOG_USER);
    syslog(LOG_INFO, "Starting Axis event monitoring...");

    while (1) {
        check_axis_event();
    }

    closelog();
    return 0;
}
