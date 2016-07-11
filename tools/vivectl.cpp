#include <stdio.h>
#include <string>
#include "vive.h"


void print_usage() {
    printf("Examples:\n");
    printf("vivectl dump hmd-imu\n");
    printf("vivectl dump hmd-light\n");
    printf("vivectl dump controller-imu\n");
    printf("vivectl dump controller-light\n");
    printf("vivectl send display-off\n");
    printf("vivectl send controller-off\n");
}

bool compare(std::string str1, std::string str2) {
    return str1.compare(str2) == 0;
}

vive_priv* init_hmd_context() {
    /* init openhmd */
    ohmd_context* ctx_openhmd;
    ohmd_device* hmd;

    ctx_openhmd = ohmd_ctx_create();

    // Probe for devices
    int num_devices = ohmd_ctx_probe(ctx_openhmd);
    if(num_devices < 0){
        printf("failed to probe devices: %s\n", ohmd_ctx_get_error(ctx_openhmd));
        return NULL;
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
        return NULL;
    }

    return (vive_priv*)ctx_openhmd->active_devices[0];
}

void free_hmd_context(vive_priv* priv) {
    hid_close(priv->hmd_handle);
    hid_close(priv->imu_handle);
    hid_close(priv->watchman_dongle_handle);
    hid_close(priv->lighthouse_sensor_handle);
    free(priv);
}

void dump_controller() {
    vive_priv* priv = init_hmd_context();
    while(true)
        print_watchman_sensors(priv);
    free_hmd_context(priv);
}

void dump_hmd_imu() {
    vive_priv* priv = init_hmd_context();
    while(true)
        print_imu_sensors(priv);
    free_hmd_context(priv);
}

void dump_hmd_light() {
    vive_priv* priv = init_hmd_context();
    while(true)
        print_hmd_light_sensors(priv);
    free_hmd_context(priv);
}

void send_hmd_on() {
    int hret = 0;
    vive_priv* priv = init_hmd_context();
    printf("hmd on.\n");

    // turn the display on
    hret = hid_send_feature_report(priv->hmd_handle, vive_magic_power_on, sizeof(vive_magic_power_on));
    printf("power on magic: %d\n", hret);
    free_hmd_context(priv);
}

void send_hmd_off() {
    int hret = 0;
    vive_priv* priv = init_hmd_context();
    printf("hmd off.\n");

    // turn the display off
    hret = hid_send_feature_report(priv->hmd_handle, vive_magic_power_off1, sizeof(vive_magic_power_off1));
    printf("power off magic 1: %d\n", hret);

    hret = hid_send_feature_report(priv->hmd_handle, vive_magic_power_off2, sizeof(vive_magic_power_off2));
    printf("power off magic 2: %d\n", hret);

    free_hmd_context(priv);
}

void send_controller_off() {
    int hret = 0;
    printf("controller off.\n");
    vive_priv* priv = init_hmd_context();
    hret = hid_send_feature_report(priv->watchman_dongle_handle, vive_controller_power_off, sizeof(vive_controller_power_off));
    free_hmd_context(priv);
}

int main(int argc, char *argv[]) {
    if ( argc < 2 ) {
        print_usage();
    } else {
        // dump
        if (compare(argv[1], "dump")) {
            if(compare(argv[2], "hmd-imu")) {
                dump_hmd_imu();
            } else if(compare(argv[2], "hmd-light")) {
                dump_hmd_light();
            } else {
                printf("Unknown argument %s\n", argv[2]);
                print_usage();
                return 0;
            }
        // send
        } else if (compare(argv[1], "send")) {
            if(compare(argv[2], "hmd-on")) {
                send_hmd_on();
            } else if(compare(argv[2], "hmd-off")) {
                send_hmd_off();
            } else if(compare(argv[2], "controller-off")) {
                send_controller_off();
            } else {
                printf("Unknown argument %s\n", argv[2]);
                print_usage();
                return 0;
            }
        } else {
            printf("Unknown argument %s\n", argv[1]);
            print_usage();
            return 0;
        }
    }
    return 0;
}
