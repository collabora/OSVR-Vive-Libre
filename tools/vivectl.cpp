#include <stdio.h>
#include <string>
#include "vl_driver.h"


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

void dump_controller() {
    vive_priv* priv = vive_init();
    while(true)
        print_watchman_sensors(priv);
    vive_free(priv);
}

void dump_hmd_imu() {
    vive_priv* priv = vive_init();
    while(true)
        print_imu_sensors(priv);
    vive_free(priv);
}

void dump_hmd_light() {
    vive_priv* priv = vive_init();
    while(true)
        print_hmd_light_sensors(priv);
    vive_free(priv);
}

void send_hmd_on() {
    int hret = 0;
    vive_priv* priv = vive_init();
    printf("hmd on.\n");

    // turn the display on
    hret = hid_send_feature_report(priv->hmd_handle, vive_magic_power_on, sizeof(vive_magic_power_on));
    printf("power on magic: %d\n", hret);
    vive_free(priv);
}

void send_hmd_off() {
    int hret = 0;
    vive_priv* priv = vive_init();
    printf("hmd off.\n");

    // turn the display off
    hret = hid_send_feature_report(priv->hmd_handle, vive_magic_power_off1, sizeof(vive_magic_power_off1));
    printf("power off magic 1: %d\n", hret);

    hret = hid_send_feature_report(priv->hmd_handle, vive_magic_power_off2, sizeof(vive_magic_power_off2));
    printf("power off magic 2: %d\n", hret);

    vive_free(priv);
}

void send_controller_off() {
    int hret = 0;
    printf("controller off.\n");
    vive_priv* priv = vive_init();
    hret = hid_send_feature_report(priv->watchman_dongle_handle, vive_controller_power_off, sizeof(vive_controller_power_off));
    vive_free(priv);
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
