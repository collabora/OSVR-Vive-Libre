/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2013 Fredrik Hultin.
 * Copyright (C) 2013 Jakob Bornecrantz.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

/* HTC Vive Driver */

#include <vector>

#include "vl_driver.h"

void vl_error(const char* msg) {
    printf("error: %s\n", msg);
}

vl_driver* vl_driver_init() {
    // Probe for devices
    std::vector<int> paths = vl_driver_get_device_paths(HTC_ID, VIVE_HMD);
    if(paths.size() <= 0) {
        printf("failed to probe devices\n");
        return NULL;
    }

    // Open default device (0)
    int index = 0;

    vl_driver* hmd;
    if(index >= 0 && index < paths.size()){
        hmd = vl_driver_open_device(paths[index]);
    } else {
        printf("no device with index: %d\n", index);
        return NULL;
    }

    if(!hmd){
        printf("failed to open device\n");
        return NULL;
    }

    return hmd;
}

void vl_driver_close(vl_driver* priv) {
    hid_close(priv->hmd_device);
    hid_close(priv->hmd_imu_device);
    hid_close(priv->watchman_dongle_device);
    hid_close(priv->hmd_light_sensor_device);
    free(priv);
}

/*
static void dump_indexed_string(hid_device* device, int index)
{
    wchar_t wbuffer[512] = {0};
    char buffer[1024] = {0};

    int hret = hid_get_indexed_string(device, index, wbuffer, 511);

    if(hret == 0){
        wcstombs(buffer, wbuffer, sizeof(buffer));
        printf("indexed string 0x%02x: '%s'\n", index, buffer);
    }
}

static void dumpbin(const char* label, const unsigned char* data, int length)
{
    printf("%s:\n", label);
    for(int i = 0; i < length; i++){
        printf("%02x ", data[i]);
        if((i % 16) == 15)
            printf("\n");
    }
    printf("\n");
}
*/
static void print_info_string(int (*fun)(hid_device*, wchar_t*, size_t), const char* what, hid_device* device)
{
    wchar_t wbuffer[512] = {0};
    char buffer[1024] = {0};

    int hret = fun(device, wbuffer, 511);

    if(hret == 0){
        wcstombs(buffer, wbuffer, sizeof(buffer));
        printf("%s: '%s'\n", what, buffer);
    }
}

void print_device_info(hid_device* dev) {
    print_info_string(hid_get_manufacturer_string, "Manufacturer", dev);
    print_info_string(hid_get_product_string , "Product", dev);
    print_info_string(hid_get_serial_number_string, "Serial Number", dev);
}


static hid_device* open_device_idx(int manufacturer, int product, int iface, int iface_tot, int device_index)
{
    struct hid_device_info* devs = hid_enumerate(manufacturer, product);
    struct hid_device_info* cur_dev = devs;

    int idx = 0;
    int iface_cur = 0;
    hid_device* ret = NULL;

    printf("Opening %04x:%04x %d/%d\n", manufacturer, product, iface+1, iface_tot);

    while (cur_dev) {
        if(idx == device_index && iface == iface_cur)
            ret = hid_open_path(cur_dev->path);
        cur_dev = cur_dev->next;
        iface_cur++;
        if(iface_cur >= iface_tot){
            idx++;
            iface_cur = 0;
        }
    }
    hid_free_enumeration(devs);

    if(hid_set_nonblocking(ret, 1) == -1){
        vl_error("failed to set non-blocking on device.");
        return NULL;
    }

    print_device_info(ret);

    return ret;
}



vl_driver *vl_driver_open_device(int idx)
{
    vl_driver* drv = new vl_driver;

    int hret = 0;

    // Open the HMD device
    drv->hmd_device = open_device_idx(HTC_ID, VIVE_HMD, 0, 1, idx);
    if(!drv->hmd_device)
        goto cleanup;

    // Open the lighthouse device
    drv->hmd_imu_device = open_device_idx(VALVE_ID, VIVE_LIGHTHOUSE_FPGA_RX, 0, 2, idx);
    if(!drv->hmd_imu_device)
        goto cleanup;

    drv->hmd_light_sensor_device = open_device_idx(VALVE_ID, VIVE_LIGHTHOUSE_FPGA_RX, 1, 2, idx);
    if(!drv->hmd_light_sensor_device)
        goto cleanup;

    drv->watchman_dongle_device = open_device_idx(VALVE_ID, VIVE_WATCHMAN_DONGLE, 1, 2, idx);
    if(!drv->watchman_dongle_device)
        goto cleanup;

    // enable lighthouse
    //hret = hid_send_feature_report(priv->hmd_handle, vive_magic_enable_lighthouse, sizeof(vive_magic_enable_lighthouse));
    //printf("enable lighthouse magic: %d\n", hret);

    ofusion_init(&drv->sensor_fusion);

    return drv;

cleanup:
    if(drv)
        free(drv);

    return NULL;
}

std::vector<int> vl_driver_get_device_paths(int vendor_id, int device_id)
{
    struct hid_device_info* devs = hid_enumerate(vendor_id, device_id);
    struct hid_device_info* cur_dev = devs;

    std::vector<int> paths;

    int idx = 0;
    while (cur_dev) {
        paths.push_back(idx);
        cur_dev = cur_dev->next;
        idx++;
    }

    hid_free_enumeration(devs);

    return paths;
}

void vec3f_from_vive_vec_accel(const int16_t* smp, vec3f* out_vec)
{
    float gravity = 9.81;
    out_vec->x = (float)smp[0] * (4.0*gravity/32768.0);
    out_vec->y = (float)smp[1] * (4.0*gravity/32768.0);
    out_vec->z = (float)smp[2] * (4.0*gravity/32768.0);
}

#define VL_GRAVITY 9.8
#define VL_POW_2_15 32768.0

Eigen::Vector3f vec3_from_accel(const int16_t* smp)
{
    Eigen::Vector3f sample(smp[0], smp[1], smp[2]);
    return sample * 4.0 * VL_GRAVITY / VL_POW_2_15;
}

Eigen::Vector3f vec3_from_gyro(const int16_t* smp)
{
    Eigen::Vector3f sample(smp[0], smp[1], smp[2]);
    return sample * 8.0 / VL_POW_2_15;
}

void vec3f_from_vive_vec_gyro(const int16_t* smp, vec3f* out_vec)
{
    float scalar = 8.0 / VL_POW_2_15;
    out_vec->x = (float)smp[0] * scalar;
    out_vec->y = (float)smp[1] * scalar;
    out_vec->z = (float)smp[2] * scalar;
}


#define FEATURE_BUFFER_SIZE 256
void vl_driver_log_watchman(hid_device *dev) {
    int size = 0;
    unsigned char watchman_buffer[FEATURE_BUFFER_SIZE];
    while((size = hid_read(dev, watchman_buffer, FEATURE_BUFFER_SIZE)) > 0){
        if(watchman_buffer[0] == 35){
            vl_msg_watchman pkt;
            vl_msg_decode_watchman(&pkt, watchman_buffer, size);
            vl_msg_print_watchman(&pkt);
        }else if (watchman_buffer[0] == 36) {
            // todo handle paket 36
        }else{
            printf("unhandled message type: %u\n", watchman_buffer[0]);
        }
    }

    if(size < 0){
        printf("error reading from device/n");
    }
}

void vl_driver_log_hmd_light(hid_device* dev) {
    int size = 0;
    unsigned char lighthouse_buffer[FEATURE_BUFFER_SIZE];
    while((size = hid_read(dev, lighthouse_buffer, FEATURE_BUFFER_SIZE)) > 0){
        if(lighthouse_buffer[0] == 37){
            vl_msg_hmd_light pkt;
            vl_msg_decode_hmd_light(&pkt, lighthouse_buffer, size);
            vl_msg_print_hmd_light(&pkt);
        } else if (lighthouse_buffer[0] == 33) {
            vl_msg_controller_light pkt;
            vl_msg_decode_controller_light(&pkt, lighthouse_buffer, size);
            vl_msg_print_controller_light(&pkt);
        }else{
            printf("unhandled message type: %u\n", lighthouse_buffer[0]);
        }
    }

    if(size < 0){
        printf("error reading from device\n");
    }
}

void vl_driver_log_hmd_imu(hid_device* dev) {
    int size = 0;
    unsigned char buffer[FEATURE_BUFFER_SIZE];
    while((size = hid_read(dev, buffer, FEATURE_BUFFER_SIZE)) > 0){
        if(buffer[0] == VL_MSG_HMD_IMU){
            vl_msg_hmd_imu pkt;
            vl_msg_decode_hmd_imu(&pkt, buffer, size);
            vl_msg_print_hmd_imu(&pkt);
        }else{
            printf("unhandled message type: %u\n", buffer[0]);
            //LOGE("unknown message type: %u", buffer[0]);
        }
    }

    if(size < 0){
        printf("error reading from device\n");
    }
}


Eigen::Quaternionf openhmd_to_eigen_quaternion(quatf in) {
    Eigen::Quaternionf q(in.x, in.y, in.z, in.w);
    return q;
}

bool is_timestamp_valid(uint32_t t1, uint32_t t2) {
    return t1 != t2 && (
        (t1 < t2 && t2 - t1 > 0xFFFFFFFF >> 2) ||
        (t1 > t2 && t1 - t2 < 0xFFFFFFFF >> 2)
    );
}

int get_lowest_index(uint8_t s0, uint8_t s1, uint8_t s2) {
    return (s0 == (uint8_t)(s1 + 2) ) ? 1
         : (s1 == (uint8_t)(s2 + 2) ) ? 2
         :                              0;
}

Eigen::Quaternionf imu_to_pose(vl_driver* drv)
{
    int size = 0;
    unsigned char buffer[FEATURE_BUFFER_SIZE];

    while((size = hid_read(drv->hmd_imu_device, buffer, FEATURE_BUFFER_SIZE)) > 0){
        if(buffer[0] == VL_MSG_HMD_IMU){
            vl_msg_hmd_imu pkt;
            vl_msg_decode_hmd_imu(&pkt, buffer, size);
            //vl_msg_print_hmd_imu(&pkt);

            int li = get_lowest_index(
                        pkt.samples[0].seq,
                        pkt.samples[1].seq,
                        pkt.samples[2].seq);

            for (int offset = 0; offset < 3; offset++) {
                int index = (li + offset) % 3;

                vl_imu_sample sample = pkt.samples[index];

                if (drv->previous_ticks == 0) {
                    drv->previous_ticks = sample.time_ticks;
                    continue;
                }

                if (is_timestamp_valid(sample.time_ticks, drv->previous_ticks)) {

                    vec3f raw_accel, raw_gyro;
                    /*
                    Eigen::Vector3f vec3_gyro = vec3_from_gyro(sample.rot);
                    Eigen::Vector3f vec3_accel = vec3_from_accel(sample.acc);
                    */
                    vec3f_from_vive_vec_accel(sample.acc, &raw_accel);
                    vec3f_from_vive_vec_gyro(sample.rot, &raw_gyro);
                    printf("    sample[%d]:\n", index);
                    printf("      acc (%f, %f, %f)\n", raw_accel.x, raw_accel.y, raw_accel.z);
                    printf("      gyro (%f, %f, %f)\n", raw_gyro.x, raw_gyro.y, raw_gyro.z);
                    printf("time_ticks: %u\n", sample.time_ticks);
                    printf("seq: %u\n", sample.seq);
                    printf("\n");

                    float dt = TICK_LEN / FREQ_48KHZ * (sample.time_ticks - drv->previous_ticks);

                    ofusion_update(&drv->sensor_fusion, dt, raw_gyro, raw_accel);

                    drv->previous_ticks = pkt.samples[index].time_ticks;
                }
            }
        }else{
            printf("unknown message type: %u\n", buffer[0]);
        }
    }


    if(size < 0){
        printf("error reading from device\n");
    }

    return openhmd_to_eigen_quaternion(drv->sensor_fusion.orient);
}
