#include <stdio.h>
#include <signal.h>
#include <string>
#include <map>
#include "vl_driver.h"
#include "vl_config.h"

vl_driver* driver;

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

static void dump_hmd_imu_pose() {
    while(true)
        driver->update_pose();
}

static void send_hmd_on() {
    // turn the display on
    int hret = hid_send_feature_report(driver->hmd_device,
                                   vive_magic_power_on,
                                   sizeof(vive_magic_power_on));
    printf("power on magic: %d\n", hret);
}

static void dump_hmd_light() {
    // hmd needs to be on to receive light reports.
    send_hmd_on();
    while(true)
        vl_driver_log_hmd_light(driver->hmd_light_sensor_device);
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
    hid_send_feature_report(driver->watchman_dongle_device,
                            vive_controller_power_off,
                            sizeof(vive_controller_power_off));
}

static void signal_interrupt_handler(int sig) {
    signal(sig, SIG_IGN);
    delete(driver);
    exit(0);
}

typedef std::function<void(void)> taskfun;

void run(taskfun task) {
    driver = new vl_driver();
    if (!driver->init_devices(0))
        return;
    signal(SIGINT, signal_interrupt_handler);
    task();
    delete(driver);
}

static std::map<std::string, taskfun> dump_commands {
    { "hmd-imu", dump_hmd_imu },
    { "hmd-light", dump_hmd_light },
    { "hmd-config", dump_config_hmd },
    { "controller", dump_controller },
    { "hmd-imu-pose", dump_hmd_imu_pose }
};

static std::map<std::string, taskfun> send_commands {
    { "hmd-on", send_hmd_on },
    { "hmd-off", send_hmd_off },
    { "controller-off", send_controller_off }
};

static std::string commands_to_str(std::map<std::string, taskfun> commands) {
    std::string str;
    for (const auto& cm : commands)
        str += "  " + cm.first + "\n";
    return str;
}

static void print_usage() {
    std::string dmp_cmd_str = commands_to_str(dump_commands);
    std::string snd_cmd_str = commands_to_str(send_commands);

#define USAGE "\
Receive data from and send commands to Vive.\n\n\
usage: vivectl <command> <message>\n\n\
 dump\n\n\
%s\n\
 send\n\n\
%s\n\
Example: vivectl dump hmd-imu\n"

    printf(USAGE, dmp_cmd_str.c_str(), snd_cmd_str.c_str());
}

static void argument_error(const char * arg) {
    printf("Unknown argument %s\n", arg);
    print_usage();
}

taskfun _get_task_fun(char *argv[], const std::map<std::string, taskfun>& commands) {
    auto search = commands.find(std::string(argv[2]));
    if(search != commands.end()) {
        return search->second;
    } else {
        argument_error(argv[2]);
        return nullptr;
    }
}

int main(int argc, char *argv[]) {
    taskfun task = nullptr;

    if ( argc < 3 ) {
        print_usage();
    } else {
        if (compare(argv[1], "dump")) {
            task = _get_task_fun(argv, dump_commands);
        } else if (compare(argv[1], "send")) {
            task = _get_task_fun(argv, send_commands);
        } else {
            argument_error(argv[1]);
            return 0;
        }
    }

    if (task)
        run(task);

    printf("taks complete!\n");

    return 0;
}
