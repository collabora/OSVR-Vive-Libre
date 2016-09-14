/*
 * OSVR
 *
 * Copyright (C) 2016 Sensics Inc.
 *
 * Vive Libre
 *
 * Copyright (C) 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Suite 500,
 * Boston, MA 02110-1335, USA.
 */

#include <chrono>
#include <thread>
#include <iostream>

#include <Eigen/Geometry>

#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

#include "org_osvr_Vive_Libre_json.h"
#include "vl_driver.h"
#include "vl_math.h"


#include <osvr/Util/EigenCoreGeometry.h>
#include <osvr/Util/EigenInterop.h>
#include <osvr/Util/TimeValue.h>


namespace ei = osvr::util::eigen_interop;

static const auto PREFIX = "[vive-libre] ";


void vl_print(std::string s) {
    std::cout << PREFIX << s << std::endl;
}

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

        OSVR_Pose3 pose;
        osvrPose3SetIdentity(&pose);

        vive->update_pose();

        if (vive->previous_ticks != 0) {
            Eigen::Quaterniond pose_vl = vive->sensor_fusion->orientation;

            //Fix transformation
            Eigen::AngleAxisd rotation_fix(0.5*M_PI, Eigen::Vector3d::UnitY());
            Eigen::Quaterniond q = rotation_fix * switch_coord_order(&pose_vl);
            //Eigen::Quaterniond q = switch_coord_order(&pose_vl);
            Eigen::Quaterniond inv = eigen_quaternion_inverse_handedness(q);

            // Push pose to OSVR
            osvr::util::toQuat (inv, pose.rotation);
        }

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
        vive = new vl_driver();
        m_found = this->vive->init_devices(0);
    }
    ~HardwareDetection() {
        vl_print("Shutting Down.");
        delete(this->vive);
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
