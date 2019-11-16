/*******************************************************************************
 * Copyright (C) 2018-2019 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#include "speedometer.h"
#include "config.h"
#include "gva_utils.h"
#include "gva_buffer_map.h"
#include "gva_roi_meta.h"
#include "inference_backend/logger.h"

#include <assert.h>
#include <chrono>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <math.h>

class Speedometer {
  public:
    using seconds_double = std::chrono::duration<double>;
    virtual ~Speedometer() = default;
    virtual bool NewFrame(const std::string &element_name, FILE *output, GstBuffer *buf) = 0;
    virtual void EOS(FILE *output) = 0;
};

static std::map<std::string, std::shared_ptr<Speedometer>> speedometers;
static std::mutex channels_mutex;
//////////////////////////////////////////////////////////////////////////
// C interface

class IterativeSpeedometer : public Speedometer {
  private:
        std::map<int, std::pair<int, int>> prev_centers_bb;
        
  public:
    IterativeSpeedometer(unsigned interval, bool print_each_stream = true)
        : interval(interval), print_each_stream(print_each_stream) {
            
    }
    bool NewFrame(const std::string &element_name, FILE *output, GstBuffer *buf) override {
        // PrintSpeed(stdout, interval);
        // //fprintf(stdout, "5 \n");
        // //fprintf(stdout, "%d \n", interval);
        GVA::RegionOfInterestList roi_list(buf);

        for (GVA::RegionOfInterest &roi : roi_list) {
            int object_id = roi.meta()->id;
            int cur_x_center = roi.meta()->x + roi.meta()->w / 2;
            int cur_y_center = roi.meta()->y + roi.meta()->h / 2;
            if ( prev_centers_bb.find(object_id) == prev_centers_bb.end() ) {
                prev_centers_bb[object_id] = std::pair<int, int> (cur_x_center, cur_y_center);
            }
            else {
                auto now = std::chrono::high_resolution_clock::now();
                if (!last_time.time_since_epoch().count()) {
                    last_time = now;
                }

                double sec = std::chrono::duration_cast<seconds_double>(now - last_time).count();

                if (sec >= interval) {
                    last_time = now;
                    auto prev_bb = prev_centers_bb[object_id];
                    double d_bb = sqrt( (cur_x_center - prev_bb.first) * (cur_x_center - prev_bb.first) + 
                        (cur_y_center - prev_bb.second) * (cur_y_center - prev_bb.second) );
                    double velocity = d_bb / interval;
                    PrintSpeed(stdout, velocity);
                    prev_centers_bb[object_id] = std::pair<int, int> (cur_x_center, cur_y_center);
                }
            }
            
        }


        std::lock_guard<std::mutex> lock(mutex);
        if (output == nullptr)
            return false;
        num_frames[element_name]++;
        auto now = std::chrono::high_resolution_clock::now();
        if (!last_time.time_since_epoch().count()) {
            last_time = now;
        }

        double sec = std::chrono::duration_cast<seconds_double>(now - last_time).count();
        if (sec >= interval) {
            last_time = now;
            PrintSpeed(output, sec);
            num_frames.clear();
            return true;
        }
        return false;
    }
    void EOS(FILE *) override {
    }

  protected:
    unsigned interval;
    bool print_each_stream;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    std::map<std::string, int> num_frames;
    std::mutex mutex;

    void PrintSpeed(FILE *output, double velocity) {

        
        fprintf(output, "Current speed = %f \n", velocity);

    }
};



extern "C" {

void create_iterative_speedometer (const char *intervals) {
    try {
        std::lock_guard<std::mutex> lock(channels_mutex);
        std::vector<std::string> intervals_list = SplitString(intervals, ',');
        for (const std::string &interval : intervals_list)
            if (not speedometers.count(interval)) {
                std::shared_ptr<Speedometer> speedometer =
                    std::shared_ptr<Speedometer>(new IterativeSpeedometer(std::stoi(interval)));
                speedometers.insert({interval, speedometer});
            }
    } catch (std::exception &e) {
        std::string msg = std::string("Error during creation iterative speedometer: ") + e.what();
        GVA_ERROR(msg.c_str());
    }
}

void speedometer_new_frame(GstBuffer *buf, const char *element_name) {
    try {
        for (auto counter = speedometers.begin(); counter != speedometers.end(); ++counter)
            counter->second->NewFrame(element_name, stdout, buf);
    } catch (std::exception &e) {
        std::string msg = std::string("Error during adding new frame: ") + e.what();
        GVA_ERROR(msg.c_str());
    }
}

void speedometer_eos() {
    try {
        for (auto counter = speedometers.begin(); counter != speedometers.end(); ++counter)
            counter->second->EOS(stdout);
    } catch (std::exception &e) {
        std::string msg = std::string("Error during handling EOS : ") + e.what();
        GVA_ERROR(msg.c_str());
    }
}

} /* extern "C" */