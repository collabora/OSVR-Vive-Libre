#include <stdio.h>
#include <string>
#include "openhmd.h"
#include "platform.h"
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

void dump_controller() {
    vive_priv* priv = init_hmd_context();
    while(true)
        print_watchman_sensors(priv);
}

void dump_hmd_imu() {
    vive_priv* priv = init_hmd_context();
    while(true)
        print_imu_sensors(priv);
}

void dump_hmd_light() {
    vive_priv* priv = init_hmd_context();
    while(true)
        print_hmd_light_sensors(priv);
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
            printf("send!\n");
        } else {
            printf("Unknown argument %s\n", argv[1]);
            print_usage();
            return 0;
        }
    }
    return 0;
}
