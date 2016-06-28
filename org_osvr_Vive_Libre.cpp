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

bool vive_decode_sensor_packet(vive_sensor_packet* pkt, const unsigned char* buffer, int size)
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

        // Print hardware information for the opened device
        int ivals[2];
        ohmd_device_geti(hmd, OHMD_SCREEN_HORIZONTAL_RESOLUTION, ivals);
        ohmd_device_geti(hmd, OHMD_SCREEN_VERTICAL_RESOLUTION, ivals + 1);
        printf("resolution:         %i x %i\n", ivals[0], ivals[1]);

        print_infof(hmd, "hsize:",            1, OHMD_SCREEN_HORIZONTAL_SIZE);
        print_infof(hmd, "vsize:",            1, OHMD_SCREEN_VERTICAL_SIZE);
        print_infof(hmd, "lens separation:",  1, OHMD_LENS_HORIZONTAL_SEPARATION);
        print_infof(hmd, "lens vcenter:",     1, OHMD_LENS_VERTICAL_POSITION);
        print_infof(hmd, "left eye fov:",     1, OHMD_LEFT_EYE_FOV);
        print_infof(hmd, "right eye fov:",    1, OHMD_RIGHT_EYE_FOV);
        print_infof(hmd, "left eye aspect:",  1, OHMD_LEFT_EYE_ASPECT_RATIO);
        print_infof(hmd, "right eye aspect:", 1, OHMD_RIGHT_EYE_ASPECT_RATIO);
        print_infof(hmd, "distortion k:",     6, OHMD_DISTORTION_K);

        printf("\n");
    }

    #define FEATURE_BUFFER_SIZE 256

    OSVR_ReturnCode update() {

        //printf("update\n");

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
        unsigned char buffer[FEATURE_BUFFER_SIZE];

        // lighthouse update
        while((size = hid_read(priv->imu_handle, buffer, FEATURE_BUFFER_SIZE)) > 0){
            if(buffer[0] == VIVE_IRQ_SENSORS){
                vive_sensor_packet pkt;
                vive_decode_sensor_packet(&pkt, buffer, size);

                printf("vive sensor sample:\n");
                printf("  report_id: %u\n", pkt.report_id);
                for(int i = 0; i < 3; i++){
                    printf("    sample[%d]:\n", i);

                    for(int j = 0; j < 3; j++){
                        printf("      acc[%d]: %d\n", j, pkt.samples[i].acc[j]);
                    }

                    for(int j = 0; j < 3; j++){
                        printf("      rot[%d]: %d\n", j, pkt.samples[i].rot[j]);
                    }

                    printf("time_ticks: %d\n", pkt.samples[i].time_ticks);
                    printf("seq: %u\n", pkt.samples[i].seq);
                    printf("\n");
                }
            }else{
                LOGE("unknown message type: %u", buffer[0]);
            }
        }

        if(size < 0){
            LOGE("error reading from device");
        }

        //ohmd_ctx_update(ctx_openhmd);

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
