#include <fstream>
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <string>
#include <map>
#include "vl_config.h"
#include "vl_driver.h"
#include "vl_enums.h"
#include "vl_light.h"
#include "vl_log.h"

static bool should_exit = false;
static vl_driver* driver;

#define CHECK(result, out) \
{ \
    bool success = (result); \
    if (!success) { \
        out; \
    } \
}

static bool compare(const std::string& str1, const std::string& str2) {
    return str1.compare(str2) == 0;
}

static void dump_controller() {
    CHECK(vl_driver_start_watchman_capture(driver, vl_driver_log_watchman), return);
    while (!should_exit)
        CHECK(driver->poll(), break);
    CHECK(vl_driver_stop_watchman_capture(driver), return);
}

static void dump_hmd_mainboard() {
    CHECK(vl_driver_start_hmd_mainboard_capture(driver, vl_driver_log_hmd_mainboard), return);
    while (!should_exit)
        CHECK(driver->poll(), break);
    CHECK(vl_driver_stop_hmd_mainboard_capture(driver), return);
}

static void dump_hmd_imu() {
    CHECK(vl_driver_start_hmd_imu_capture(driver, vl_driver_log_hmd_imu), return);
    while (!should_exit)
        CHECK(driver->poll(), break);
    CHECK(vl_driver_stop_hmd_imu_capture(driver), return);
}

static void dump_hmd_imu_pose() {
    CHECK(vl_driver_start_hmd_imu_capture(driver, vl_driver_update_pose), return);
    while (!should_exit)
        CHECK(driver->poll(), break);
    CHECK(vl_driver_stop_hmd_imu_capture(driver), return);
}

static void send_hmd_on() {
    // turn the display on
    int hret = hid_send_feature_report(driver->hmd_device.handle,
                                   0,
                                   vive_magic_power_on);
    vl_info("power on magic: %d", hret);
}

static void dump_hmd_light() {
    // hmd needs to be on to receive light reports.
    send_hmd_on();
    CHECK(vl_driver_start_hmd_light_capture(driver, vl_driver_log_hmd_light), return);
    while (!should_exit)
        CHECK(driver->poll(), break);
    CHECK(vl_driver_stop_hmd_light_capture(driver), return);
}

static void dump_config_hmd() {
    char * config = vl_get_config(driver->hmd_lighthouse_device, 0);
    vl_info("hmd_lighthouse_device config: %s", config);
}

static void dump_hmd_all() {
    CHECK(vl_driver_start_hmd_mainboard_capture(driver, vl_driver_log_hmd_mainboard), return);
    CHECK(vl_driver_start_watchman_capture(driver, vl_driver_log_watchman), goto out_hmd_mainboard);
    CHECK(vl_driver_start_hmd_imu_capture(driver, vl_driver_log_hmd_imu), goto out_watchman);
    CHECK(vl_driver_start_hmd_light_capture(driver, vl_driver_log_hmd_light), goto out_hmd_imu);
    CHECK(vl_driver_start_watchman_capture(driver, vl_driver_log_hmd_light), goto out_hmd_light);
    while (!should_exit)
        CHECK(driver->poll(), break);
    vl_driver_stop_watchman_capture(driver);
out_hmd_light:
    vl_driver_stop_hmd_light_capture(driver);
out_hmd_imu:
    vl_driver_stop_hmd_imu_capture(driver);
out_watchman:
    vl_driver_stop_watchman_capture(driver);
out_hmd_mainboard:
    vl_driver_stop_hmd_mainboard_capture(driver);
}

static void read_hmd_light(uint8_t* buffer, int size, vl_driver* driver) {
    vl_report_id report_id = static_cast<vl_report_id>(buffer[0]);

    if (report_id != vl_report_id::HMD_LIGHTHOUSE_PULSE2) {
        vl_error("Wrong light message type, expected %d got %d.",
                 static_cast<uint8_t>(vl_report_id::HMD_LIGHTHOUSE_PULSE2),
                 buffer[0]);
        return;
    }

    vive_headset_lighthouse_pulse_report2 pkt;
    vl_msg_decode_hmd_light(&pkt, buffer, size);
    vl_msg_print_hmd_light_csv(&pkt);

    for(int i = 0; i < 9; i++){
        driver->raw_light_samples.push_back(pkt.samples[i]);
    }
}

static void dump_station_angle() {
    send_hmd_on();

    CHECK(vl_driver_start_hmd_light_capture(driver, read_hmd_light), return);
    while (driver->raw_light_samples.size() < 10000)
        CHECK(driver->poll(), break);
    CHECK(vl_driver_stop_hmd_light_capture(driver), return);

    vl_light_classify_samples(driver->raw_light_samples);
}

static vl_lighthouse_samples parse_csv_file(const std::string& file_path) {

    vl_lighthouse_samples samples = {};
    std::string line;
    std::ifstream csv_stream(file_path);

    if (!csv_stream.good()) {
        vl_error("CSV file not found: %s", file_path.c_str());
        return samples;
    }

    while(std::getline(csv_stream, line)) {
        std::istringstream s(line);

        std::string timestamp;
        std::string id;
        std::string length;

        getline(s, timestamp,',');
        getline(s, id,',');
        getline(s, length,',');

        vive_headset_lighthouse_pulse2 sample;

        sample.timestamp = std::stoul(timestamp);
        sample.sensor_id = std::stoul(id);
        sample.length = std::stoul(length);

        samples.push_back(sample);

        //vl_debug("int ts %u id %u length %u", sample.timestamp, sample.sensor_id, sample.length);
    }

    if (samples.empty())
        vl_error("No samples found in %s", file_path.c_str());

    return samples;

}

static void dump_station_angle_from_csv(const std::string& file_path) {
    vl_lighthouse_samples samples = parse_csv_file(file_path);
    if (!samples.empty())
        vl_light_classify_samples(samples);
}


#include <json/value.h>
#include <json/reader.h>

static void pnp_from_csv(const std::string& file_path) {

    // XXX
    std::string config(vl_get_config(driver->hmd_lighthouse_device, 0));
    //vl_info("\n\nconfig:\n\n%s\n\n", config.c_str());

    std::stringstream foo;
    foo << config;

    Json::Value root;
    Json::CharReaderBuilder rbuilder;
    // Configure the Builder, then ...
    std::string errs;
    bool parsingSuccessful = Json::parseFromStream(rbuilder, foo, &root, &errs);
    if (!parsingSuccessful) {
        // report to the user the failure and their locations in the document.
        std::cout  << "Failed to parse configuration\n"
                   << errs;
        return;
    }

    std::string my_encoding = root.get("mb_serial_number", "UTF-32" ).asString();
    vl_info("mb_serial_number: %s", my_encoding.c_str());


    Json::Value modelPoints = root["lighthouse_config"]["modelPoints"];

    vl_info("model points size: %u", modelPoints.size());

    unsigned sensor_id = 0;


    std::map<unsigned, cv::Point3f> config_sensor_positions;

    for ( unsigned index = 0; index < modelPoints.size(); ++index ) {
        // Iterates over the sequence elements.

       Json::Value point = modelPoints[index];

       vl_info("%d: x %s y %s z %s", sensor_id, point[0].asString().c_str(), point[1].asString().c_str(), point[2].asString().c_str());

       cv::Point3f p = cv::Point3f(
               std::stod(point[0].asString()),
               std::stod(point[1].asString()),
               std::stod(point[2].asString()));

       config_sensor_positions.insert(std::pair<unsigned, cv::Point3f>(sensor_id, p));

       sensor_id++;
    }



    vl_lighthouse_samples samples = parse_csv_file(file_path);
    if (!samples.empty())
        dump_pnp_positions(&samples, config_sensor_positions);
}

static void send_hmd_off() {
    // turn the display off
    int hret = hid_send_feature_report(driver->hmd_device.handle,
                                   0,
                                   vive_magic_power_off1);
    vl_debug("power off magic 1: %d", hret);

    hret = hid_send_feature_report(driver->hmd_device.handle,
                                   0,
                                   vive_magic_power_off2);
    vl_debug("power off magic 2: %d", hret);
}

static void send_controller_off() {
    for (int i = 0; i < 2; ++i)
        hid_send_feature_report(driver->watchman_dongle_device[i].handle,
                                1,
                                vive_controller_power_off);
}

static void signal_interrupt_handler(int sig) {
    signal(sig, SIG_DFL);
    should_exit = true;
}

typedef std::function<void(void)> taskfun;

void run(taskfun task) {
    if (!task)
        return;
    vl_set_log_level(Level::INFO);
    driver = new vl_driver();
    if (!driver->init_devices(0))
        return;
    signal(SIGINT, signal_interrupt_handler);
    task();
    delete(driver);
}

static std::map<std::string, taskfun> dump_commands {
    { "hmd-all", dump_hmd_all },
    { "hmd-mainboard", dump_hmd_mainboard },
    { "hmd-imu", dump_hmd_imu },
    { "hmd-light", dump_hmd_light },
    { "hmd-config", dump_config_hmd },
    { "controller", dump_controller },
    { "hmd-imu-pose", dump_hmd_imu_pose },
    { "lighthouse-angles", dump_station_angle }
};

static std::map<std::string, taskfun> send_commands {
    { "hmd-on", send_hmd_on },
    { "hmd-off", send_hmd_off },
    { "controller-off", send_controller_off }
};

static std::string commands_to_str(const std::map<std::string, taskfun>& commands) {
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
Example: vivectl dump hmd-imu"

    vl_info(USAGE, dmp_cmd_str.c_str(), snd_cmd_str.c_str());
}

static void argument_error(const char * arg) {
    vl_error("Unknown argument %s", arg);
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
            run(task);
        } else if (compare(argv[1], "send")) {
            task = _get_task_fun(argv, send_commands);
            run(task);
        } else if (compare(argv[1], "classify")) {
            std::string file_name = argv[2];
            dump_station_angle_from_csv(file_name);
        } else if (compare(argv[1], "pnp")) {
            std::string file_name = argv[2];
            task = [file_name]() {
                pnp_from_csv(file_name);
            };
            run(task);
        } else {
            argument_error(argv[1]);
            return 0;
        }
    }

    return 0;
}
