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

#include <Eigen/Geometry>

#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

#include "org_osvr_Vive_Libre_json.h"
#include "vl_driver.h"
#include "vl_math.h"

static const auto PREFIX = "[vive-libre] ";


void vl_print(std::string s) {
    std::cout << PREFIX << s << std::endl;
};

class TrackerDevice {

  private:
    osvr::pluginkit::DeviceToken m_dev;
    OSVR_TrackerDeviceInterface m_tracker;
    vl_driver* vive;

  public:
    TrackerDevice(OSVR_PluginRegContext ctx, vl_driver* vive) {
        // init osvr device

        vl_print("Init Tracker Device.");
        
        OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

        // configure tracker
        osvrDeviceTrackerConfigure(opts, &m_tracker);

        // Create the device token with the options
        m_dev.initAsync(ctx, "Tracker", opts);

        // Send JSON descriptor
        m_dev.sendJsonDescriptor(org_osvr_Vive_Libre_json);

        // Register update callback
        m_dev.registerUpdateCallback(this);

        this->vive= vive;
    }


    OSVR_ReturnCode update() {
        OSVR_TimeValue now;
        osvrTimeValueGetNow(&now);

        OSVR_PoseState pose;
        osvrPose3SetIdentity(&pose);

        // transform pose
        Eigen::AngleAxis<float> rotation_fix(0.5*M_PI, Eigen::Vector3f::UnitY());
        Eigen::Quaternion<float> q = rotation_fix * vl_imu_to_pose(vive);
        pose.rotation = eigen_to_osvr_quaternion(q);

        osvrDeviceTrackerSendPose(m_dev, m_tracker, &pose, 0);

        return OSVR_RETURN_SUCCESS;
    }

};

class HardwareDetection {
    vl_driver* vive;
  public:
    HardwareDetection() : m_found(false) {
        vl_print("Detecting Vive Hardware.");
        // init vive-libre
        this->vive = vl_driver_init();
        if (vive != NULL)
            m_found = true;
    }
    ~HardwareDetection() {
        vl_print("Shutting Down.");
        vl_driver_close(this->vive);
    }

    OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

        if (m_found) {
            /// Create our device object
            osvr::pluginkit::registerObjectForDeletion(ctx, new TrackerDevice(ctx, this->vive));
            return OSVR_RETURN_SUCCESS;
        }

        vl_print("No Vive detected.");

        return OSVR_RETURN_FAILURE;
    }

  private:
    bool m_found;
};

OSVR_PLUGIN(org_osvr_Vive_Libre) {

    osvr::pluginkit::PluginContext context(ctx);
    vl_print("Welcome Human.");
    /// Register a detection callback function object.
    context.registerHardwareDetectCallback(new HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
