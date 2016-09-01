#include <stdio.h>
#include <signal.h>
#include <string>
#include "vl_driver.h"
#include "vl_config.h"

vl_driver* driver;

static void print_usage() {
    printf("Examples:\n");
    printf("vivectl dump hmd-imu\n");
    printf("vivectl dump hmd-light\n");
    printf("vivectl dump controller-imu\n");
    printf("vivectl dump controller-light\n");
    printf("vivectl send hmd-off\n");
    printf("vivectl send controller-off\n");
}

static bool compare(const std::string& str1, const std::string& str2) {
    return str1.compare(str2) == 0;
}

static void dump_controller() {
    while(true)
        vl_driver_log_watchman(driver->watchman_dongle_device);
}

static void dump_hmd_imu() {
    while(true)
        vl_driver_log_hmd_imu(driver->hmd_imu_device);
}

static void dump_hmd_light() {
    while(true)
        vl_driver_log_hmd_light(driver->hmd_light_sensor_device);
}

static void send_hmd_on() {
    // turn the display on
    int hret = hid_send_feature_report(driver->hmd_device,
                                   vive_magic_power_on,
                                   sizeof(vive_magic_power_on));
    printf("power on magic: %d\n", hret);
}

static void dump_config_hmd() {
    char * config = vl_get_config(driver->hmd_imu_device);
    printf("hmd_imu_device config: %s\n", config);
}

static void send_hmd_off() {
    // turn the display off
    int hret = hid_send_feature_report(driver->hmd_device,
                                   vive_magic_power_off1,
                                   sizeof(vive_magic_power_off1));
    printf("power off magic 1: %d\n", hret);

    hret = hid_send_feature_report(driver->hmd_device,
                                   vive_magic_power_off2,
                                   sizeof(vive_magic_power_off2));
    printf("power off magic 2: %d\n", hret);
}

static void send_controller_off() {
    int hret = hid_send_feature_report(driver->watchman_dongle_device,
                                       vive_controller_power_off,
                                       sizeof(vive_controller_power_off));
}


static void signal_interrupt_handler(int sig)
{
    signal(sig, SIG_IGN);
    vl_driver_close(driver);
    exit(0);
}

void run(void (*task)(void)) {
    driver = vl_driver_init();
    if (driver == nullptr)
        return;
    signal(SIGINT, signal_interrupt_handler);
    task();
    vl_driver_close(driver);
}

int main(int argc, char *argv[]) {
    void (*task)(void);

    if ( argc < 3 ) {
        print_usage();
    } else {
        // dump
        if (compare(argv[1], "dump")) {
            if(compare(argv[2], "hmd-imu")) {
                task = &dump_hmd_imu;
            } else if(compare(argv[2], "hmd-light")) {
                task = &dump_hmd_light;
            } else if(compare(argv[2], "hmd-config")) {
                task = &dump_config_hmd;
            } else {
                printf("Unknown argument %s\n", argv[2]);
                print_usage();
                return 0;
            }
            // send
        } else if (compare(argv[1], "send")) {
            if(compare(argv[2], "hmd-on")) {
                task = &send_hmd_on;
            } else if(compare(argv[2], "hmd-off")) {
                task = &send_hmd_off;
            } else if(compare(argv[2], "controller-off")) {
                task = &send_controller_off;
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

    run(task);

    return 0;
}
