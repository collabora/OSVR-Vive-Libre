#pragma once

#include <functional>
#include <vector>
#include <string>
#include <map>
#include <opencv2/core/types.hpp>

#define VL_ROTOR_RPS 60 // 60 rps
#define VL_TICK_RATE 48e6 // 48 Mhz

typedef std::vector<vive_headset_lighthouse_pulse2> vl_lighthouse_samples;

typedef std::function<bool(const vive_headset_lighthouse_pulse2&)> sample_filter;


struct vl_light_sample_group {
    char channel;
    char sweep; // rotor
    double epoch;
    int skip;
    int seq;
    vl_lighthouse_samples samples;
};

double median_timestamp(const vl_lighthouse_samples& samples);
int64_t median_length(const vl_lighthouse_samples& samples);
void print_sample_group (const vl_light_sample_group& g);
unsigned unique_sensor_ids(const vl_lighthouse_samples& S);

struct lighthouse_sync_pulse {
    uint16_t duration;
    int skip;
    int sweep; //rotor
    int data;
};

lighthouse_sync_pulse lookup_pulse_class(uint16_t pulselen);

// Decode Vive Lighthouse sync pulses
//
//   [station, sweep, databit] = decode_pulse(S)
//
//   S        Struct of samples for one pulse.
//
// S must contain only pulse samples for the same pulse, which means
// their timestamps must be close. The format of S is the same as
// load_dump() returns.
//
//   skip     Nx1 vector: 0 or 1; -1 for error in decoding.
//   sweep    Nx1 vector: 0 for horizontal sweep, 1 for vertical sweep
//   databit  Nx1 vector: one bit of over-the-light data stream
//
// Reference: https://github.com/nairol/LighthouseRedox/blob/master/docs/Light//20Emissions.md

std::tuple<int,int,int> decode_pulse(const vl_lighthouse_samples& S);

// Recognize the Vive Lighthouse channel from a pulse
//
// ch = channel_detect(last_pulse_time, new_pulse_time);
//
//	last_pulse_time	timestamp of the previous pulse
//	new_pulse_time	timestamp of the pulse to be identified
//
//	ch		Either 'A', 'B', 'C', or 'error' for unrecognized.
//
// This function requires the globals 'tick_rate' and 'rotor_rps'
// to be set.

char channel_detect(uint32_t last_pulse_time, double new_pulse_time);

// Get a sub-set of a struct of arrays
//
// R = subset(S, elms)
//
//	S	A struct with each field being an array of the same size.
//	elms	A logical mask or a vector of indices to choose elements.
//
//	R	A struct identical to S, but with only the chosen
//		elements included in the member vectors.

/*
void subset(S, elms) {
    //R = struct();
    struct R;
    for ([val, key] = S)
        R.(key) = val(elms);
}
*/


// Convert absolute sample ticks to relative angle ticks
//
// angle_ticks = ticks_sample_to_angle(samples, epoch)
//
//	samples		A struct with fields 'timestamp', 'length',
//			as defined by load_dump(), but for a single
//			sweep of a single station.
//	epoch		The sweep starting timestamp.
//
//	angle_ticks	Each sample converted to angle-ticks.
//
// The timestamp is adjusted to point to the middle of the lit up
// period. Assuming the laser line cross section power profile is
// symmetric, this will remove the differences from variations in laser
// line width at the sensor. Then epoch is subtracted to produce
// the time delta directly proportional to the angle.


uint32_t ticks_sample_to_angle(const vive_headset_lighthouse_pulse2& sample, uint32_t epoch);

// Convert tick-delta to millimeters
//
// mm = ticks_to_mm(ticks, dist)
//
//	ticks	delta in Lighthouse tick units
//	dist	distance between lighthouse station and sensors,
//		in meters
//
//	mm	The position delta in millimeters.
/*
double ticks_to_mm(ticks, dist) {
    global tick_rate;
    global rotor_rps;

    angle = ticks / tick_rate * rotor_rps * 2 * pi;
    mm = tan(angle) * dist * 1000;
    return mm;
}
*/

vl_light_sample_group process_pulse_set(const vl_lighthouse_samples& S, int64_t last_pulse);

// Update pulse detection state machine
//
// [last_pulse, current_sweep, seq, out_pulse] = ...
//	update_pulse_state(pulse_samples, last_pulse, current_sweep, seq)
//
// Inputs:
//	pulse_samples	A struct as defined in load_dump().
//
// Input-outputs (state to be updated):
//	last_pulse	epoch of the previous pulse
//	current_sweep	information of the current sweep, a struct
//			with fields:
//			- epoch: the pulse begin timestamp
//			- channel: 'A', 'B', or 'C'
//			- skip: the skip bit
//			- sweep: 'H' or 'V'
//			Can be empty due to detection errors.
//	seq		scanning cycle sequence number
//
// Outputs:
//	out_pulse	Only set for valid non-skipped pulses. A struct
//			with fields:
//			- channel: 'A', 'B', or 'C'
//			- sweep: 'H' or 'V' (old name for 'rotor')
//			- seq: scanning cycle sequence number
//			- epoch: the base timestamp indicating the zero raw angle
//			- samples: the subset of D with the samples indicating this pulse

std::tuple<double, vl_light_sample_group, int, vl_light_sample_group> update_pulse_state(
        const vl_lighthouse_samples& pulse_samples,
        double last_pulse_epoch,
        vl_light_sample_group current_sweep,
        int seq);

struct vl_angles {
    std::vector<uint32_t> x;
    std::vector<uint32_t> y;
    std::vector<uint32_t> t;
};


int find_max_seq(const std::vector<vl_light_sample_group>& sweeps);
std::vector<vl_light_sample_group> filter_sweeps(
        const std::vector<vl_light_sample_group>& sweeps, char ch, int seq, char rotor);
int find_max_sendor_id(vl_lighthouse_samples samples);
vl_lighthouse_samples filter_samples_by_sensor_id(const vl_lighthouse_samples& samples, int sensor_id);
std::map<unsigned, vl_angles> collect_readings(char station, const std::vector<vl_light_sample_group>& sweeps);
vl_lighthouse_samples filter_reports(const vl_lighthouse_samples& reports, const sample_filter& filter_fun);

// Sanitize Vive light samples
//
// D = sanitize(S)
//
//	S	Struct of samples, see load_dump().
//
//	D	The same struct with bad entries dropped.
//
// This function drops all entries { 0xffffffff, 0xff, 0xffff } as there is
// no known purpose for them.

bool is_sample_valid(const vive_headset_lighthouse_pulse2& s);

// Process and classify Lighthouse samples
//
// [sweeps, pulses] = process_lighthouse_samples(D)
//
//	D	A struct as returned from load_dump().
//
//	sweeps	A struct array with the fields:
//		- channel: 'A', 'B', or 'C'
//		- rotor: 'H' or 'V'
//		- seq: scanning cycle sequence number
//		- epoch: the base timestamp indicating the zero raw angle
//		- samples: the subset of D with the samples in this sweep
//
//	pulses	A struct array with the fields:
//		- channel: 'A', 'B', or 'C'
//		- sweep: 'H' or 'V' (old name for 'rotor')
//		- seq: scanning cycle sequence number
//		- epoch: the base timestamp indicating the zero raw angle
//		- samples: the subset of D with the samples indicating this pulse
//
// The struct D should have been sanitized first, see sanitize().
//
// Only the meaningful pulses are returned, i.e. those with the bit skip=false.


vl_lighthouse_samples subset(vl_lighthouse_samples D, std::vector<int> indices);
bool isempty(const vl_light_sample_group& samples);
std::tuple<std::vector<vl_light_sample_group>, std::vector<vl_light_sample_group>> process_lighthouse_samples(const vl_lighthouse_samples& D);
void print_readings(const std::map<unsigned, vl_angles>& readings);
void write_readings_to_csv(const std::map<unsigned, vl_angles>& readings, const std::string& file_name);
std::string epoch_to_string(double epoch);
std::string light_house_samples_to_string(const vl_lighthouse_samples& samples);
void print_pulse(char* buffer, const vl_light_sample_group& g, const std::string& samples, unsigned i);
void print_sweep(char* buffer, const vl_light_sample_group& g, const std::string& samples, unsigned i);


typedef std::function<void(char*, const vl_light_sample_group&, const std::string&, unsigned)> print_fun;


void write_light_groups_to_file(const std::string& title,
                                const std::string& file_name,
                                const std::vector<vl_light_sample_group>& pulses,
                                const print_fun& fun);
void vl_light_classify_samples(vl_lighthouse_samples *raw_light_samples);
void dump_readings_to_csv(const std::string& file_name,
                     const std::map<unsigned, vl_angles>& readings,
                     const std::map<unsigned, cv::Point3f>& config_sensor_positions);
void dump_pnp_positions(vl_lighthouse_samples *raw_light_samples,
             const std::map<unsigned, cv::Point3f>& config_sensor_positions);
