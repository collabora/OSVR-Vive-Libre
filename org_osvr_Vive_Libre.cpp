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
        print_hmd_light_sensors(priv);
        // print_watchman_sensors(priv);
        // print_imu_sensors(priv);

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
