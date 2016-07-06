#include <stdio.h>

#include "openhmd.h"
#include "platform.h"
#include "vive.h"


int main() {
    printf("Examples:\n");
    printf("vivectl dump hmd-imu:\n");
    printf("vivectl dump hmd-light:\n");
    printf("vivectl dump controller-imu:\n");
    printf("vivectl dump controller-light:\n");
    printf("vivectl send display-off:\n");
    printf("vivectl send controller-off:\n");

    /* init openhmd */
    ohmd_context* ctx_openhmd;
    ohmd_device* hmd;

    ctx_openhmd = ohmd_ctx_create();

    // Probe for devices
    int num_devices = ohmd_ctx_probe(ctx_openhmd);
    if(num_devices < 0){
        printf("failed to probe devices: %s\n", ohmd_ctx_get_error(ctx_openhmd));
        return 0;
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
        return 0;
    }

    vive_priv* priv = (vive_priv*)ctx_openhmd->active_devices[0];

    while(true) {
        //print_watchman_sensors(priv);
        print_imu_sensors(priv);
        //print_hmd_light_sensors(priv);
    }

    return 0;
}
