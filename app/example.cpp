/**
 * Copyright (C) 2021 Axis Communications AB, Lund, Sweden
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include <opencv2/imgproc.hpp>
#pragma GCC diagnostic pop
#include <opencv2/video.hpp>
#include <stdlib.h>
#include <syslog.h>
#include <modbus/modbus.h>

#include "imgprovider.h"

// Modbus settings
#define MODBUS_IP "192.168.1.100"  // Change to match your device IP
#define MODBUS_PORT 502
#define MODBUS_REGISTER 0x00
#define MOTION_VALUE 1
#define NO_MOTION_VALUE 0

using namespace cv;

int main(void) {
    openlog("opencv_app", LOG_PID | LOG_CONS, LOG_USER);
    syslog(LOG_INFO, "Running OpenCV example with VDO as video source and Modbus output");
    ImgProvider_t* provider = NULL;

    // Initialize Modbus connection
    modbus_t *mb = modbus_new_tcp(MODBUS_IP, MODBUS_PORT);
    if (mb == NULL) {
        syslog(LOG_ERR, "Failed to create Modbus context");
        exit(1);
    }

    if (modbus_connect(mb) == -1) {
        syslog(LOG_ERR, "Modbus connection failed");
        modbus_free(mb);
        exit(1);
    }

    syslog(LOG_INFO, "Connected to Modbus device at %s", MODBUS_IP);

    // The desired width and height of the BGR frame
    unsigned int width  = 1024;
    unsigned int height = 576;

    // chooseStreamResolution gets the least resource intensive stream
    // that exceeds or equals the desired resolution specified above
    unsigned int streamWidth  = 0;
    unsigned int streamHeight = 0;
    if (!chooseStreamResolution(width, height, &streamWidth, &streamHeight)) {
        syslog(LOG_ERR, "%s: Failed choosing stream resolution", __func__);
        modbus_free(mb);
        exit(1);
    }

    syslog(LOG_INFO,
           "Creating VDO image provider and creating stream %d x %d",
           streamWidth,
           streamHeight);
    provider = createImgProvider(streamWidth, streamHeight, 2, VDO_FORMAT_YUV);
    if (!provider) {
        syslog(LOG_ERR, "%s: Failed to create ImgProvider", __func__);
        modbus_free(mb);
        exit(2);
    }

    syslog(LOG_INFO, "Start fetching video frames from VDO");
    if (!startFrameFetch(provider)) {
        syslog(LOG_ERR, "%s: Failed to fetch frames from VDO", __func__);
        modbus_free(mb);
        exit(3);
    }

    // Create the background subtractor
    Ptr<BackgroundSubtractorMOG2> bgsub = createBackgroundSubtractorMOG2();

    // Create the filtering element
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));

    Mat bgr_mat  = Mat(height, width, CV_8UC3);
    Mat nv12_mat = Mat(height * 3 / 2, width, CV_8UC1);
    Mat fg;

    bool last_motion_state = false;  // Track previous motion state

    while (true) {
        VdoBuffer* buf = getLastFrameBlocking(provider);
        if (!buf) {
            syslog(LOG_INFO, "No more frames available, exiting");
            break;
        }

        nv12_mat.data = static_cast<uint8_t*>(vdo_buffer_get_data(buf));
        cvtColor(nv12_mat, bgr_mat, COLOR_YUV2BGR_NV12, 3);
        bgsub->apply(bgr_mat, fg, 0.005);
        morphologyEx(fg, fg, MORPH_OPEN, kernel);

        int nonzero_pixels = countNonZero(fg);
        bool current_motion_state = (nonzero_pixels > 0);

        // Only send Modbus signal when motion state changes
        if (current_motion_state != last_motion_state) {
            if (current_motion_state) {
                syslog(LOG_INFO, "Motion detected: YES - Sending Modbus signal");
                if (modbus_write_register(mb, MODBUS_REGISTER, MOTION_VALUE) == -1) {
                    syslog(LOG_ERR, "Failed to write motion state to Modbus register");
                }
            } else {
                syslog(LOG_INFO, "Motion detected: NO - Sending Modbus signal");
                if (modbus_write_register(mb, MODBUS_REGISTER, NO_MOTION_VALUE) == -1) {
                    syslog(LOG_ERR, "Failed to write motion state to Modbus register");
                }
            }
            last_motion_state = current_motion_state;
        }

        returnFrame(provider, buf);
    }

    // Cleanup
    modbus_close(mb);
    modbus_free(mb);
    return EXIT_SUCCESS;
}
