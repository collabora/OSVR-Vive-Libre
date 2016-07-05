/** @date 2016
    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2016 Sensics Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <thread>
#include <iostream>

#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

#include "org_osvr_Vive_Libre_json.h"

#include "openhmd.h"
#include "platform.h"
#include "vive.h"

static const auto PREFIX = "[OSVR-Vive-Libre] ";


void ohmd_sleep(double seconds)
{
    struct timespec sleepfor;

    sleepfor.tv_sec = (time_t)seconds;
    sleepfor.tv_nsec = (long)((seconds - sleepfor.tv_sec) * 1000000000.0);

    nanosleep(&sleepfor, NULL);
}

// gets float values from the device and prints them
void print_infof(ohmd_device* hmd, const char* name, int len, ohmd_float_value val)
{
    float f[len];
    ohmd_device_getf(hmd, val, f);
    printf("%-20s", name);
    for(int i = 0; i < len; i++)
        printf("%f ", f[i]);
    printf("\n");
}

inline static uint8_t read8(const unsigned char** buffer)
{
    return *(*buffer++);
}

inline static int16_t read16(const unsigned char** buffer)
{
    int16_t ret = **buffer | (*(*buffer + 1) << 8);
    *buffer += 2;
    return ret;
}

inline static int32_t read32(const unsigned char** buffer)
{
    int32_t ret = **buffer | (*(*buffer + 1) << 8) | (*(*buffer + 1) << 16) | (*(*buffer + 1) << 24);
    *buffer += 4;
    return ret;
}

bool vive_decode_imu_packet(vive_imu_packet* pkt, const unsigned char* buffer, int size)
{
    if(size != 52){
        LOGE("invalid vive sensor packet size (expected 52 but got %d)", size);
        return false;
    }

    pkt->report_id = read8(&buffer);

    for(int j = 0; j < 3; j++){
        // acceleration
        for(int i = 0; i < 3; i++){
            pkt->samples[j].acc[i] = read16(&buffer);
        }

        // rotation
        for(int i = 0; i < 3; i++){
            pkt->samples[j].rot[i] = read16(&buffer);
        }

        pkt->samples[j].time_ticks = read32(&buffer);
        pkt->samples[j].seq = read8(&buffer);
    }

    return true;
}


void print_imu_packet(vive_imu_packet* pkt) {
    printf("vive imu sample:\n");
    printf("  report_id: %u\n", pkt->report_id);
    for(int i = 0; i < 3; i++){
        printf("    sample[%d]:\n", i);

        for(int j = 0; j < 3; j++){
            printf("      acc[%d]: %d\n", j, pkt->samples[i].acc[j]);
        }

        for(int j = 0; j < 3; j++){
            printf("      rot[%d]: %d\n", j, pkt->samples[i].rot[j]);
        }

        printf("time_ticks: %d\n", pkt->samples[i].time_ticks);
        printf("seq: %u\n", pkt->samples[i].seq);
        printf("\n");
    }
}

bool vive_decode_watchman_packet(vive_watchman_packet* pkt, const unsigned char* buffer, int size)
{
    if(size != 30){
        LOGE("invalid vive sensor packet size (expected 30 but got %d)", size);
        return false;
    }

    pkt->report_id = buffer[0];
    pkt->time1 = buffer[1];
    pkt->type1 = buffer[2];
    pkt->time2 = buffer[3];
    pkt->type2 = buffer[4];
    pkt->pressed_buttons = buffer[5];

    return true;
}

void print_watchman_packet(vive_watchman_packet * pkt) {
    /*
    printf("vive watchman sample:\n");
    printf("  report_id: %u\n", pkt->report_id);
    printf("  time1: %u\n", pkt->time1);
    printf("  type1: %u\n", pkt->type1);
    printf("  time2: %u\n", pkt->time2);
    printf("  type2: %d\n", pkt->type2);
    */

    printf("type %d %d buttons: %d\n", pkt->type1, pkt->type2, pkt->pressed_buttons);
}

bool vive_decode_lighthouse_packet(vive_lighthouse_packet* pkt, const unsigned char* buffer, int size)
{
    if(size != 64){
        LOGE("invalid vive sensor packet size (expected 52 but got %d)", size);
        return false;
    }

    pkt->report_id = read8(&buffer);

    for(int j = 0; j < 9; j++){
        pkt->samples[j].sensor_id = read8(&buffer);
        pkt->samples[j].length = read16(&buffer);
        pkt->samples[j].time = read32(&buffer);
    }

    return true;
}

bool vive_decode_controller_lighthouse_packet(vive_controller_lighthouse_packet* pkt, const unsigned char* buffer, int size)
{
    if(size != 58){
        LOGE("invalid vive sensor packet size (expected 58 but got %d)", size);
        return false;
    }

    pkt->report_id = read8(&buffer);

    for(int j = 0; j < 7; j++){
        pkt->samples[j].sensor_id = read8(&buffer);
        pkt->samples[j].length = read16(&buffer);
        pkt->samples[j].time = read32(&buffer);
    }
    pkt->unknown = read8(&buffer);

    return true;
}

void print_controller_lighthouse_packet(vive_controller_lighthouse_packet* pkt) {
    printf("vive controller lighthouse sample:\n");
    printf("  report_id: %u\n", pkt->report_id);
    for(int i = 0; i < 7; i++){
        printf("     sensor_id[%d]: %u\n", i, pkt->samples[i].sensor_id);
        printf("      length[%d]: %d\n", i, pkt->samples[i].length);
        printf("      time[%d]: %zd\n", i, pkt->samples[i].time);
        printf("\n");
    }
    printf("unknown: %u\n", pkt->unknown);
}

void print_lighthouse_packet(vive_lighthouse_packet* pkt) {
    printf("vive lighthouse sample:\n");
    printf("  report_id: %u\n", pkt->report_id);
    for(int i = 0; i < 9; i++){
        printf("     sensor_id[%d]: %u\n", i, pkt->samples[i].sensor_id);
        printf("      length[%d]: %d\n", i, pkt->samples[i].length);
        printf("      time[%d]: %zd\n", i, pkt->samples[i].time);
        printf("\n");
    }
}

class TrackerDevice {
  public:
    TrackerDevice(OSVR_PluginRegContext ctx) {
        /* init osvr device */

        std::cout << PREFIX << "Init device." << std::endl;
        
        OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

        // configure our tracker
        osvrDeviceTrackerConfigure(opts, &m_tracker);

        /// Create the device token with the options
        m_dev.initAsync(ctx, "Tracker", opts);

        /// Send JSON descriptor
        m_dev.sendJsonDescriptor(org_osvr_Vive_Libre_json);

        /// Register update callback
        m_dev.registerUpdateCallback(this);

        /* init openhmd */

        ctx_openhmd = ohmd_ctx_create();

        // Probe for devices
        int num_devices = ohmd_ctx_probe(ctx_openhmd);
        if(num_devices < 0){
            printf("failed to probe devices: %s\n", ohmd_ctx_get_error(ctx_openhmd));
            return;
        }

        printf("num devices: %d\n\n", num_devices);

        // Print device information
        for(int i = 0; i < num_devices; i++){
            printf("device %d\n", i);
            printf("  vendor:  %s\n", ohmd_list_gets(ctx_openhmd, i, OHMD_VENDOR));
            printf("  product: %s\n", ohmd_list_gets(ctx_openhmd, i, OHMD_PRODUCT));
            printf("  path:    %s\n\n", ohmd_list_gets(ctx_openhmd, i, OHMD_PATH));
        }

        // Open default device (0)
        hmd = ohmd_list_open_device(ctx_openhmd, 0);

        if(!hmd){
            printf("failed to open device: %s\n", ohmd_ctx_get_error(ctx_openhmd));
            return;
        }
    }

    #define FEATURE_BUFFER_SIZE 256

    OSVR_ReturnCode update() {

        std::this_thread::sleep_for(std::chrono::milliseconds(
            5)); // Simulate waiting a quarter second for data.

        OSVR_TimeValue now;
        osvrTimeValueGetNow(&now);
        double t = 5.0 * ((double)now.seconds + ((double)now.microseconds / 1000000.0));

        /// Report the identity pose for sensor 0
        OSVR_PoseState pose;
        osvrPose3SetIdentity(&pose);
        pose.translation.data[0] = std::sin(t) * 0.25;
        pose.translation.data[1] = std::cos(t + 0.5) * 0.25;
        pose.translation.data[2] = std::sin(t + 0.25) * 0.25;
        osvrDeviceTrackerSendPose(m_dev, m_tracker, &pose, 0);

        vive_priv* priv = (vive_priv*)ctx_openhmd->active_devices[0];

        int size = 0;

/*
        // lighthouse update
        unsigned char buffer[FEATURE_BUFFER_SIZE];
        while((size = hid_read(priv->imu_handle, buffer, FEATURE_BUFFER_SIZE)) > 0){
            if(buffer[0] == VIVE_IRQ_SENSORS){
                vive_imu_packet pkt;
                vive_decode_imu_packet(&pkt, buffer, size);
                print_imu_packet(&pkt);
            }else{
                printf("unhandled message type: %u\n", buffer[0]);
                //LOGE("unknown message type: %u", buffer[0]);
            }
        }

        if(size < 0){
            LOGE("error reading from device");
        }
             */


        unsigned char lighthouse_buffer[FEATURE_BUFFER_SIZE];
        while((size = hid_read(priv->lighthouse_sensor_handle, lighthouse_buffer, FEATURE_BUFFER_SIZE)) > 0){
            if(lighthouse_buffer[0] == 37){
                vive_lighthouse_packet pkt;
                vive_decode_lighthouse_packet(&pkt, lighthouse_buffer, size);
                print_lighthouse_packet(&pkt);
            } else if (lighthouse_buffer[0] == 33) {
                vive_controller_lighthouse_packet pkt;
                vive_decode_controller_lighthouse_packet(&pkt, lighthouse_buffer, size);
                print_controller_lighthouse_packet(&pkt);
            }else{
                printf("unhandled message type: %u\n", lighthouse_buffer[0]);
            }
        }

        if(size < 0){
            LOGE("error reading from device");
        }
/*


        unsigned char watchman_buffer[FEATURE_BUFFER_SIZE];
        while((size = hid_read(priv->watchman_dongle_handle, watchman_buffer, FEATURE_BUFFER_SIZE)) > 0){
            if(watchman_buffer[0] == 35){
                vive_watchman_packet pkt;
                vive_decode_watchman_packet(&pkt, watchman_buffer, size);
                print_watchman_packet(&pkt);
            }else if (watchman_buffer[0] == 36) {
                // todo handle paket 36
            }else{
                printf("unhandled message type: %u\n", watchman_buffer[0]);
                //LOGE("unknown message type: %u", buffer[0]);
            }
        }

        if(size < 0){
            LOGE("error reading from device");
        }
*/

        return OSVR_RETURN_SUCCESS;
    }

  private:
    osvr::pluginkit::DeviceToken m_dev;
    OSVR_TrackerDeviceInterface m_tracker;
    ohmd_context* ctx_openhmd;
    ohmd_device* hmd;
};

class HardwareDetection {
  public:
    HardwareDetection() : m_found(false) {
    
    std::cout << PREFIX << "Detecting VIVE hardware." << std::endl;
    
    }
    OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

        if (m_found) {
            return OSVR_RETURN_SUCCESS;
        }

        /// we always detect device in sample plugin
        m_found = true;

        /// Create our device object
        osvr::pluginkit::registerObjectForDeletion(ctx, new TrackerDevice(ctx));

        return OSVR_RETURN_SUCCESS;
    }

  private:
    bool m_found;
};

OSVR_PLUGIN(org_osvr_Vive_Libre) {

    osvr::pluginkit::PluginContext context(ctx);
    
    std::cout << PREFIX << "Plugin init." << std::endl;

    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
