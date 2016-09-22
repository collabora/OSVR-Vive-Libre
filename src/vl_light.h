#pragma once

#include <vector>
#include <string>
#include <set>
#include <map>

#include <iostream>
#include <fstream>

#include "vl_messages.h"

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

double median_timestamp(const vl_lighthouse_samples& samples) {
    std::vector<double> timestamps;

    for (auto s : samples)
        timestamps.push_back(s.timestamp);

    size_t size = timestamps.size();
    std::sort(timestamps.begin(), timestamps.end());
    if (size  % 2 == 0)
          return (timestamps.at(size / 2 - 1) + timestamps.at(size / 2)) / 2;
      else
          return timestamps.at(size / 2);
}

int64_t median_length(const vl_lighthouse_samples& samples) {
    std::vector<int64_t> lengths;

    for (auto s : samples)
        lengths.push_back(s.length);

    size_t size = lengths.size();
    std::sort(lengths.begin(), lengths.end());
    if (size  % 2 == 0)
          return (lengths[size / 2 - 1] + lengths[size / 2]) / 2;
      else
          return lengths[size / 2];
}

void print_sample_group (const vl_light_sample_group& g) {
    printf("channel %c (len %ld, samples %zu): skip %d, sweep %c epoch %f\n",
        g.channel, median_length(g.samples), g.samples.size(), g.skip, g.sweep, g.epoch);
}

unsigned unique_sensor_ids(const vl_lighthouse_samples& S) {
    std::set<uint8_t> unique_ids;

    for (auto s : S)
        unique_ids.insert(s.sensor_id);

    return unique_ids.size();
}

struct lighthouse_sync_pulse {
    uint16_t duration;
    int skip;
    int sweep; //rotor
    int data;
};

static struct lighthouse_sync_pulse pulse_table[10] = {
    { 2500, -1, -1, -1 },
    { 3000, 0, 0, 0 },
    { 3500, 0, 1, 0 },
    { 4000, 0, 0, 1 },
    { 4500, 0, 1, 1 },
    { 5000, 1, 0, 0 },
    { 5500, 1, 1, 0 },
    { 6000, 1, 0, 1 },
    { 6500, 1, 1, 1 },
    { 7000, -1, -1, -1 },
};


lighthouse_sync_pulse lookup_pulse_class(uint16_t pulselen) {
    for (int i = 0; i < 10; i++)
        if (pulselen > (pulse_table[i].duration - 250) &&
                pulselen < (pulse_table[i].duration + 250))
            return pulse_table[i];
    printf("error: no pulse class found for length %u\n", pulselen);
    return lighthouse_sync_pulse();
}

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

std::tuple<int,int,int> decode_pulse(const vl_lighthouse_samples& S) {
    unsigned ndups = S.size() - unique_sensor_ids(S);

    // not fatal
    if (ndups != 0)
        printf("%d duplicate sensors\n", ndups);

    // robust against outlier samples
    uint16_t pulselen = median_length(S);

    lighthouse_sync_pulse pulse = lookup_pulse_class(pulselen);

    //printf("found pulse class: duration %u skip %d sweep %d data %d\n",
    //       pulse.duration, pulse.skip, pulse.sweep, pulse.data);

    return {pulse.skip, pulse.sweep, pulse.data};
}


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

char channel_detect(uint32_t last_pulse_time, double new_pulse_time) {
    // Two sweeps per rotation
    int64_t period = VL_TICK_RATE / VL_ROTOR_RPS / 2;
    // ??
    int64_t space = 20e3;

    int64_t dt = new_pulse_time - last_pulse_time;

    char ch;
    if (abs(dt - period) < 4000)
        ch = 'A';
    else if (abs(dt - (period - space)) < 4000)
        ch = 'B';
    else if (abs(dt - space) < 4000)
        ch = 'C';
    else
        ch = 'e';
    //printf("last %u new %ld channel %c\n", last_pulse_time, new_pulse_time, ch);
    return ch;
}

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


uint32_t ticks_sample_to_angle(const vive_headset_lighthouse_pulse2& sample, uint32_t epoch) {
    uint32_t angle_ticks = sample.timestamp + sample.length / 2 - epoch;
    return angle_ticks;
}

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

vl_light_sample_group process_pulse_set(const vl_lighthouse_samples& S, int64_t last_pulse) {

    int skip;
    int sweepi;
    int databit;
    std::tie(skip, sweepi, databit) = decode_pulse(S);

    // Pick median as the pulse timestamp.

    // Pulse lit duration varies according to the data bit sent
    // over-the-light, and the only correct point is the beginning
    // of the lit period.

    // It seems the starting times for lit periods do not all align
    // exactly, there can be few sensors that activate slightly late.
    // Their stopping time looks to me much better in sync, but
    // let's try simply the starting time concensus.

    double t = median_timestamp(S);

    char ch = channel_detect(last_pulse, t);

    char key[] = { 'e', 'H', 'V' };
    char sweep = key[sweepi + 1];

    if (S.size() < 5)
        printf("Warning: channel %c pulse at %.1f (len %ld, samples %zu): skip %d, sweep %c, data %d\n",
            ch, t, median_length(S), S.size(), skip, sweep, databit);

    // no use for databit here
    vl_light_sample_group p = {
        /*channel*/ ch,
        /*sweep*/ sweep,
        /*epoch*/ t,
        /*skip*/ skip,
        /*seq*/ 0,
        /*samples*/ vl_lighthouse_samples()
    };

    return p;
}


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
        int seq) {

    vl_light_sample_group out_pulse;
    vl_light_sample_group pulse = process_pulse_set(pulse_samples, last_pulse_epoch);
    last_pulse_epoch = pulse.epoch;

    if (pulse.channel == 'e' || pulse_samples.size() < 5) {
        // Invalid pulse, reset state since cannot know which
        // sweep following sweep samples would belong in.
        current_sweep = vl_light_sample_group();
        // printf("Warning: returning incomplete pulse\n");
        return {last_pulse_epoch, current_sweep, seq, out_pulse};
    }

    if (pulse.skip == 0) {
        // Valid pulse starting a new sweep.

        // Count complete sweep sequences. For mode A, that is horz+vert.
        // For mode B+C, that is horz+vert for both stations.
        if ((pulse.channel == 'A' || pulse.channel == 'B') && pulse.sweep == 'H')
            seq += 1;

        printf("Start sweep seq %d: ch %c, sweep %c, pulse detected by %zu sensors\n",
               seq, pulse.channel, pulse.sweep, pulse_samples.size());

        current_sweep = pulse;
        current_sweep.samples = pulse_samples;

        out_pulse = {
            /*channel*/ pulse.channel,
            /*sweep*/ pulse.sweep,
            /*epoch*/ pulse.epoch,
            /*skip*/ 0,
            /*seq*/ seq,
            /*samples*/ pulse_samples
        };
    }

    return {last_pulse_epoch, current_sweep, seq, out_pulse};
}


struct vl_angles {
    std::vector<uint32_t> x;
    std::vector<uint32_t> y;
    std::vector<uint32_t> t;
};


int find_max_seq(const std::vector<vl_light_sample_group>& sweeps) {
    std::vector<int> seqs;
    for (auto g : sweeps) {
        seqs.push_back(g.seq);
    }
    return *std::max_element(seqs.begin(), seqs.end());
}

std::vector<vl_light_sample_group> filter_sweeps(
        const std::vector<vl_light_sample_group>& sweeps, char ch, int seq, char rotor) {
    std::vector<vl_light_sample_group> filtered;
    for (auto g : sweeps)
        if (g.channel == ch && g.seq == seq && g.sweep == rotor)
            filtered.push_back(g);
    return filtered;
}

int find_max_sendor_id(vl_lighthouse_samples samples) {
    std::vector<int> sensor_ids;
    for (auto s : samples) {
        sensor_ids.push_back(s.sensor_id);
    }
    return *std::max_element(sensor_ids.begin(), sensor_ids.end());
}

vl_lighthouse_samples filter_samples_by_sensor_id(const vl_lighthouse_samples& samples, int sensor_id) {
    vl_lighthouse_samples filtered;
    for(auto s : samples)
        if (s.sensor_id == sensor_id)
            filtered.push_back(s);
    return filtered;
}

std::map<unsigned, vl_angles> collect_readings(char station, const std::vector<vl_light_sample_group>& sweeps) {
    // Collect all readings into a nice data structure
    // x and y angles, and a timestamp (x sweep epoch)
    // array R(sensor_id + 1).x, .y, .t

    std::map<unsigned, vl_angles> R;

    int maxseq = find_max_seq(sweeps);

    // loop over sequences
    for (int i = 1; i < maxseq; i++) {
        // choose station and sequence

        std::vector<vl_light_sample_group> x_sweeps = filter_sweeps(sweeps, station, i, 'H');
        std::vector<vl_light_sample_group> y_sweeps = filter_sweeps(sweeps, station, i, 'V');

        if (x_sweeps.size() < 1 || y_sweeps.size() < 1) {
            // Either or both sweeps are empty, ignore.
            printf("Warning: Either or both sweeps are empty, ignore.\n");
            break;
        }

        if (x_sweeps.size() != 1 || y_sweeps.size() != 1)
            printf("error: Unexpected number of indices [%zu %zu], should be just one each\n", x_sweeps.size(), y_sweeps.size());


        for (unsigned sweep_i = 0; sweep_i < x_sweeps.size(); sweep_i++) {

            vl_light_sample_group x_sweep, y_sweep;

            try {
                x_sweep = x_sweeps.at(sweep_i);
                y_sweep = y_sweeps.at(sweep_i);
            } catch (std::out_of_range e) {
                printf("Warning: one dimension is missing for sweep %u\n", sweep_i);
                continue;
            }

            //int max_sensor_id = find_max_sendor_id(x_sweep.samples);

            // loop over sensors ids, only interested in both x and y
            for (unsigned s = 0; s < 32; s++) {
                vl_lighthouse_samples xi = filter_samples_by_sensor_id(x_sweep.samples, s);
                vl_lighthouse_samples yi = filter_samples_by_sensor_id(y_sweep.samples, s);

                if (xi.size() > 1 || yi.size() > 1)
                    printf("error: Same sensor sampled multiple times??\n");

                if (xi.size() < 1 || yi.size() < 1)
                    continue;

                if (!R.count( s ))
                    R.insert(std::pair<unsigned, vl_angles>(s, vl_angles()));


                double x_ang = ticks_sample_to_angle(xi[0], x_sweep.epoch);
                double y_ang = ticks_sample_to_angle(yi[0], y_sweep.epoch);

                R[s].x.push_back(x_ang);
                R[s].y.push_back(y_ang);

                // Assumes all measurements happened at the same time,
                // which is wrong.
                R[s].t.push_back(x_sweep.epoch);


            }
        }
    }
    return R;
}


vl_lighthouse_samples filter_reports (const vl_lighthouse_samples& reports, const sample_filter& filter_fun) {
    vl_lighthouse_samples results;
    for (auto sample : reports)
        if (filter_fun(sample))
            results.push_back(sample);
    return results;
}

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

/*
lighthouse_reports sanitize(const lighthouse_reports& S) {
    return subset(S, !(S.timestamp == 0xffffffff & S.sensor_id == 0xff & S.length == 0xffff));
}
*/

bool is_sample_valid(const vive_headset_lighthouse_pulse2& s) {
    return !(s.timestamp == 0xffffffff && s.sensor_id == 0xff && s.length == 0xffff);
}

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


vl_lighthouse_samples subset(vl_lighthouse_samples D, std::vector<int> indices) {
    vl_lighthouse_samples results;
    for (auto i : indices)
        results.push_back(D[i]);
    return results;
}

bool isempty(const vl_light_sample_group& samples) {
    return samples.samples.empty();
}

std::tuple<std::vector<vl_light_sample_group>, std::vector<vl_light_sample_group>> process_lighthouse_samples(const vl_lighthouse_samples& D) {

    // state for the processing loop
    std::vector<int> pulse_inds;
    std::vector<int> sweep_inds;

    double last_pulse_epoch = -1e6;
    int seq = 0;

    vl_light_sample_group current_sweep;
    //pulse_range = [Inf -Inf]; // begin, end timestamp
    std::pair<uint32_t, uint32_t> pulse_range = {UINT32_MAX, 0};

    std::vector<vl_light_sample_group> pulses;
    std::vector<vl_light_sample_group> sweeps;

    // Process all samples
    for (unsigned i = 0; i < D.size(); i++) {

        vive_headset_lighthouse_pulse2 sample = D[i];
        if (sample.length < 2000) {

            // sweep sample
            if (!pulse_inds.empty()) {
                // process the pulse set
                vl_light_sample_group pulse;
                std::tie(last_pulse_epoch, current_sweep, seq, pulse) = update_pulse_state(subset(D, pulse_inds), last_pulse_epoch, current_sweep, seq);

                pulse_inds.clear();
                pulse_range = {UINT32_MAX, 0};
                if (!isempty(pulse)) {
                    //printf("pushing pulse 1\n");
                    pulses.push_back(pulse);
                } else {
                    //printf("error: sweep has begun but pulse is empty.\n");
                }
            }

            if (isempty(current_sweep)) {
                // printf("warning: current_sweep is empty.\n");
                // do not know which sweep, so skip
                continue;
            }

            // accumulate sweep samples for a single sweep
            sweep_inds.push_back(i);
        } else {
            // pulse sample
            if (!sweep_inds.empty()) {
                // store one sweep

                vl_light_sample_group sweep = {
                    /*channel*/ current_sweep.channel,
                    /*sweep*/ current_sweep.sweep,
                    /*epoch*/ current_sweep.epoch,
                    /*skip*/ 0,
                    /*seq*/ seq,
                    /*samples*/ subset(D, sweep_inds)
                };

                sweep_inds.clear();

                if (!isempty(current_sweep)) {
                    sweeps.push_back(sweep);
                } else {
                    printf("error: pulse has begun but current_sweep is empty.\n");
                }
            }

            // A pulse belongs to the existing set if it overlaps
            // the whole set seen so far.
            if (pulse_inds.empty() || (sample.timestamp <= pulse_range.second && sample.timestamp + sample.length >= pulse_range.first)) {
                // compute the time span of pulses seen so far
                pulse_range = {
                    std::min(pulse_range.first, sample.timestamp),
                    std::max(pulse_range.second, sample.timestamp + sample.length)
                };

                // accumulate a single pulse set
                pulse_inds.push_back(i);
            } else {
                // Otherwise, a new pulse set start immediately after
                // the previous one without any sweep samples in between.

                if (sample.timestamp + sample.length < pulse_range.first)
                    printf("Out of order pulse at index %d\n", i);

                // process the pulse set
                vl_light_sample_group pulse;
                std::tie(last_pulse_epoch, current_sweep, seq, pulse) = update_pulse_state(subset(D, pulse_inds), last_pulse_epoch, current_sweep, seq);

                pulse_inds.clear();
                pulse_range = {UINT_MAX, 0};
                if (!isempty(pulse))
                    pulses.push_back(pulse);

            }
        }
    }
    return {sweeps, pulses};
}

void print_readings(const std::map<unsigned, vl_angles>& readings) {
    for (auto angles : readings) {
        for (unsigned i = 0; i < angles.second.x.size(); i++ ) {
            printf("sensor %u, x %u, y %u, t %u\n",
                   angles.first,
                   angles.second.x[i],
                   angles.second.y[i],
                   angles.second.t[i]);
        }
    }
}

void write_readings_to_csv(const std::map<unsigned, vl_angles>& readings, const std::string& file_name) {
    std::ofstream csv_file;
    csv_file.open (file_name);

    printf("Writing %s\n", file_name.c_str());
    for (auto angles : readings)
        for (unsigned i = 0; i < angles.second.x.size(); i++ )
            csv_file << angles.first << ","
                     << angles.second.x[i] << ","
                     << angles.second.y[i] << ","
                     << angles.second.t[i] << "\n";

    csv_file.close();
}


#define SAMPLES_STRING "\
        .samples = struct [1 1]:\n\
            [1  1] =\n\
                .timestamp = %s\n\
                .sensor_id = %s\n\
                .length = %s\n"

#define PULSE_STRING "\
    [1  %u] =\n\
%s\
        .epoch = %s\n\
        .channel = %c\n\
        .sweep = %c\n\
        .seq = %u\n"

#define SWEEP_STRING "\
    [1  %u] =\n\
        .channel = %c\n\
        .rotor = %c\n\
        .seq = %u\n\
        .epoch = %s\n\
%s"


std::string epoch_to_string(double epoch) {
        std::string epoch_string;

        char* buffer = new char[50];
        double intpart;

        if( modf (epoch , &intpart) == 0 )
            sprintf (buffer, "%.0f", epoch);
        else
            sprintf (buffer, "%.1f", epoch);

        std::string out(buffer);
        delete (buffer);

        return out;
}

std::string light_house_samples_to_string(const vl_lighthouse_samples& samples) {
        std::stringstream timestamps;
        std::stringstream sensor_ids;
        std::stringstream lengths;

        for (unsigned i = 0; i < samples.size(); i++) {

            vive_headset_lighthouse_pulse2 sample = samples[i];

            timestamps << sample.timestamp;
            if (i != samples.size() - 1)
                timestamps << "  ";

            if (sample.sensor_id < 10 && i != 0)
                sensor_ids << " ";

            sensor_ids << (unsigned int)(sample.sensor_id);
            if (i != samples.size() - 1)
                sensor_ids << "  ";

            if (sample.length < 100 && i != 0)
                lengths << " ";

            lengths << sample.length;
            if (i != samples.size() - 1)
                lengths << "  ";
        }

        char* buffer = new char[1000];
        sprintf (buffer, SAMPLES_STRING, timestamps.str().c_str(), sensor_ids.str().c_str(), lengths.str().c_str());

        std::string out(buffer);

        delete (buffer);

        return out;

}


void print_pulse(char* buffer, const vl_light_sample_group& g, const std::string& samples, unsigned i) {
    sprintf (buffer, PULSE_STRING, i+1, samples.c_str(), epoch_to_string(g.epoch).c_str(), g.channel, g.sweep, g.seq);
}

void print_sweep(char* buffer, const vl_light_sample_group& g, const std::string& samples, unsigned i) {
    sprintf (buffer, SWEEP_STRING, i+1, g.channel, g.sweep, g.seq, epoch_to_string(g.epoch).c_str(), samples.c_str());
}

typedef std::function<void(char*, const vl_light_sample_group&, const std::string&, unsigned)> print_fun;


void write_light_groups_to_file(const std::string& title,
                                const std::string& file_name,
                                const std::vector<vl_light_sample_group>& pulses,
                                const print_fun& fun) {
    std::ofstream fid;
    printf("Writing %s.\n", file_name.c_str());
    fid.open (file_name);
    fid << title << " struct [1 " << pulses.size() << "]:\n";
    for (unsigned i = 0; i < pulses.size(); i++) {
        vl_light_sample_group g = pulses.at(i);
        std::string samples = light_house_samples_to_string(g.samples);
        char* buffer = new char[1000];
        fun(buffer, g, samples, i);
        fid << buffer;
        delete(buffer);
    }
    fid.close();
}

void vl_light_classify_samples(vl_lighthouse_samples *raw_light_samples) {

    // Take just a little bit for analysis
    // deliberately start middle of a sweep
    vl_lighthouse_samples sanitized_light_samples = filter_reports(*raw_light_samples, &is_sample_valid);

    printf("raw: %ld\n", raw_light_samples->size());
    printf("valid: %ld\n", sanitized_light_samples.size());

    std::vector<vl_light_sample_group> pulses;
    std::vector<vl_light_sample_group> sweeps;
    std::tie(sweeps, pulses) = process_lighthouse_samples(sanitized_light_samples);

    printf("Found %zu pulses\n", pulses.size());
    write_light_groups_to_file("Pulses", "b_c_still.pulses.cpp.txt", pulses, print_pulse);

    printf("Found %zu sweeps\n", sweeps.size());
    write_light_groups_to_file("Sweeps", "b_c_still.sweeps.cpp.txt", sweeps, print_sweep);

    std::map<unsigned, vl_angles> R_B = collect_readings('B', sweeps);
    std::map<unsigned, vl_angles> R_C = collect_readings('C', sweeps);

    printf("Found %zu sensors with B angles\n", R_B.size());
    // print_readings(R_B);

    printf("Found %zu sensors with C angles\n", R_C.size());
    // print_readings(R_C);

    if (R_B.size() > 0)
        write_readings_to_csv(R_B, "b_angles.csv");
    if (R_C.size() > 0)
        write_readings_to_csv(R_C, "c_angles.csv");
}
