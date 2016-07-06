/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2013 Fredrik Hultin.
 * Copyright (C) 2013 Jakob Bornecrantz.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

/* HTC Vive Driver */

#define FEATURE_BUFFER_SIZE 256

#define HTC_ID                   0x0bb4
#define VIVE_HMD                 0x2c87

#define VALVE_ID                 0x28de
#define VIVE_WATCHMAN_DONGLE     0x2101
#define VIVE_LIGHTHOUSE_FPGA_RX  0x2000


#include <string.h>
#include <wchar.h>
#include <assert.h>
#include <time.h>

#include "vive.h"

static int getf(ohmd_device* device, ohmd_float_value type, float* out)
{
	vive_priv* priv = (vive_priv*)device;

	switch(type){
	case OHMD_ROTATION_QUAT: 
		out[0] = out[1] = out[2] = 0;
		out[3] = 1.0f;
		break;

	case OHMD_POSITION_VECTOR:
		out[0] = out[1] = out[2] = 0;
		break;

	case OHMD_DISTORTION_K:
		// TODO this should be set to the equivalent of no distortion
		memset(out, 0, sizeof(float) * 6);
		break;

	default:
		ohmd_set_error(priv->base.ctx, "invalid type given to getf (%ud)", type);
		return -1;
		break;
	}

	return 0;
}

static void close_device(ohmd_device* device)
{
	int hret = 0;
	vive_priv* priv = (vive_priv*)device;

	LOGD("closing HTC Vive device");

	// turn the display off
	hret = hid_send_feature_report(priv->hmd_handle, vive_magic_power_off1, sizeof(vive_magic_power_off1));
	printf("power off magic 1: %d\n", hret);
	
	hret = hid_send_feature_report(priv->hmd_handle, vive_magic_power_off2, sizeof(vive_magic_power_off2));
	printf("power off magic 2: %d\n", hret);

	hid_close(priv->hmd_handle);
	hid_close(priv->imu_handle);

	free(device);
}

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

static void dump_info_string(int (*fun)(hid_device*, wchar_t*, size_t), const char* what, hid_device* device)
{
	wchar_t wbuffer[512] = {0};
	char buffer[1024] = {0};

	int hret = fun(device, wbuffer, 511);	

	if(hret == 0){
		wcstombs(buffer, wbuffer, sizeof(buffer));
		printf("%s: '%s'\n", what, buffer);
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

static hid_device* open_device_idx(int manufacturer, int product, int iface, int iface_tot, int device_index)
{
	struct hid_device_info* devs = hid_enumerate(manufacturer, product);
	struct hid_device_info* cur_dev = devs;

	int idx = 0;
	int iface_cur = 0;
	hid_device* ret = NULL;

    printf("Opening Manufacturer %04x Product %04x\n", manufacturer, product);

	while (cur_dev) {
        printf("Path %s", cur_dev->path);

		if(idx == device_index && iface == iface_cur){
            ret = hid_open_path(cur_dev->path);
            printf(" [open]\n");
        } else {
            printf(" [skip]\n");
        }

		cur_dev = cur_dev->next;

		iface_cur++;

		if(iface_cur >= iface_tot){
			idx++;
			iface_cur = 0;
		}
	}

	hid_free_enumeration(devs);

    printf("Returning last opened device.\n");

	return ret;
}

static ohmd_device* open_device(ohmd_driver* driver, ohmd_device_desc* desc)
{
	vive_priv* priv = ohmd_alloc(driver->ctx, sizeof(vive_priv));

	if(!priv)
		return NULL;

	int hret = 0;
	
	priv->base.ctx = driver->ctx;

	int idx = atoi(desc->path);

	// Open the HMD device
	priv->hmd_handle = open_device_idx(HTC_ID, VIVE_HMD, 0, 1, idx);

	if(!priv->hmd_handle)
		goto cleanup;
	
	if(hid_set_nonblocking(priv->hmd_handle, 1) == -1){
		ohmd_set_error(driver->ctx, "failed to set non-blocking on device");
		goto cleanup;
	}
	
	// Open the lighthouse device
	priv->imu_handle = open_device_idx(VALVE_ID, VIVE_LIGHTHOUSE_FPGA_RX, 0, 2, idx);

	if(!priv->imu_handle)
		goto cleanup;
	
	if(hid_set_nonblocking(priv->imu_handle, 1) == -1){
		ohmd_set_error(driver->ctx, "failed to set non-blocking on device");
		goto cleanup;
	}


    priv->lighthouse_sensor_handle = open_device_idx(VALVE_ID, VIVE_LIGHTHOUSE_FPGA_RX, 1, 2, idx);
    if(!priv->lighthouse_sensor_handle)
        goto cleanup;

    if(hid_set_nonblocking(priv->lighthouse_sensor_handle, 1) == -1){
        ohmd_set_error(driver->ctx, "failed to set non-blocking on lighthouse sensor");
        goto cleanup;
    }



    priv->watchman_dongle_handle = open_device_idx(VALVE_ID, VIVE_WATCHMAN_DONGLE, 1, 2, idx);

    // Open the controller dongle

    if(!priv->watchman_dongle_handle) {
        printf("failed to open dongle!\n");
        ohmd_set_error(driver->ctx, "failed to open watchman dongle");
        goto cleanup;

    } else {
        printf("opened watchman dongle! :)\n");
    }

    if(hid_set_nonblocking(priv->watchman_dongle_handle, 1) == -1){
        ohmd_set_error(driver->ctx, "failed to set non-blocking on device");
        goto cleanup;
    } else {
        printf("set watchman to non blocking.\n");
    }

    vive_controller_command_packet controller_command;
    controller_command.report_id = 255;
    controller_command.command = 0x8f;
    controller_command.length = 7;

    /*
    hret = hid_send_feature_report(priv->watchman_dongle_handle, vive_controller_power_off, sizeof(vive_controller_power_off));
    printf("vive_controller_haptic_pulse: %d\n", hret);
    */

	dump_info_string(hid_get_manufacturer_string, "manufacturer", priv->hmd_handle);
	dump_info_string(hid_get_product_string , "product", priv->hmd_handle);
	dump_info_string(hid_get_serial_number_string, "serial number", priv->hmd_handle);

	// turn the display on
	hret = hid_send_feature_report(priv->hmd_handle, vive_magic_power_on, sizeof(vive_magic_power_on));
	printf("power on magic: %d\n", hret);
	
	// enable lighthouse
	//hret = hid_send_feature_report(priv->hmd_handle, vive_magic_enable_lighthouse, sizeof(vive_magic_enable_lighthouse));
    //printf("enable lighthouse magic: %d\n", hret);

	// set up device callbacks
	priv->base.close = close_device;
	priv->base.getf = getf;
	
	return (ohmd_device*)priv;

cleanup:
	if(priv)
		free(priv);

	return NULL;
}

static void get_device_list(ohmd_driver* driver, ohmd_device_list* list)
{
	struct hid_device_info* devs = hid_enumerate(HTC_ID, VIVE_HMD);
	struct hid_device_info* cur_dev = devs;

	int idx = 0;
	while (cur_dev) {
		ohmd_device_desc* desc = &list->devices[list->num_devices++];

		strcpy(desc->driver, "OpenHMD HTC Vive Driver");
		strcpy(desc->vendor, "HTC/Valve");
		strcpy(desc->product, "HTC Vive");

		desc->revision = 0;

		snprintf(desc->path, OHMD_STR_SIZE, "%d", idx);

		desc->driver_ptr = driver;

		cur_dev = cur_dev->next;
		idx++;
	}

	hid_free_enumeration(devs);
}

static void destroy_driver(ohmd_driver* drv)
{
	LOGD("shutting down HTC Vive driver");
	free(drv);
}

ohmd_driver* ohmd_create_htc_vive_drv(ohmd_context* ctx)
{
	ohmd_driver* drv = ohmd_alloc(ctx, sizeof(ohmd_driver));
	
	if(!drv)
		return NULL;

	drv->get_device_list = get_device_list;
	drv->open_device = open_device;
	drv->get_device_list = get_device_list;
	drv->open_device = open_device;
	drv->destroy = destroy_driver;

	return drv;
}

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
    printf("== imu sample ==\n");
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
        pkt->samples[j].length = uread16(&buffer);
        pkt->samples[j].time = uread32(&buffer);
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
    printf("== controller light sample ==\n");
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
    printf("== hmd light sample ==\n");
    printf("  report_id: %u\n", pkt->report_id);
    for(int i = 0; i < 9; i++){
        printf("     sensor_id[%d]: %u\n", i, pkt->samples[i].sensor_id);
        printf("      length[%d]: %d\n", i, pkt->samples[i].length);
        printf("      time[%d]: %zd\n", i, pkt->samples[i].time);
        printf("\n");
    }
}

#define FEATURE_BUFFER_SIZE 256
void print_watchman_sensors(vive_priv* priv) {
    int size = 0;
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
}

void print_hmd_light_sensors(vive_priv* priv) {
    int size = 0;
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
}

void print_imu_sensors(vive_priv* priv) {
    int size = 0;
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
}
