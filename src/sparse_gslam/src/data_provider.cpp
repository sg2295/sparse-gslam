#include "data_provider.h"

#include <Eigen/StdVector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>

struct Data {
    double time;
    g2o::SE2 pose;
    std::vector<float> full_range;
};

class CarmenLogDataProvider : public DataProvider {
    std::vector<Data, Eigen::aligned_allocator<Data>> data;
    std::vector<Data, Eigen::aligned_allocator<Data>>::iterator it;
   public:
    CarmenLogDataProvider(const std::string& fpath) {
        std::ifstream log_file(fpath);
        std::string line, prefix;
        while (std::getline(log_file, line)) {
            std::istringstream iss(line);
            iss >> prefix;
            if (prefix == "FLASER") {
                data.emplace_back();
                auto& d = data.back();
                int num_readings;
                iss >> num_readings;
                d.full_range.resize(num_readings);
                for (int i = 0; i < num_readings; i++) {
                    iss >> d.full_range[i];
                }
                double temp;
                for (int i = 0; i < 3; i++)
                    iss >> temp;
                double data[3];
                for (int i = 0; i < 3; i++)
                    iss >> data[i];
                d.pose = g2o::SE2(data[0], data[1], data[2]);
                std::string hostname;
                iss >> d.time >> hostname >> temp;
            }
        }
        std::sort(data.begin(), data.end(), [](const auto& d1, const auto& d2) { return d1.time < d2.time; });
        it = data.begin();
    }

    bool get_data(double& time, g2o::SE2& pose, std::vector<float>& full_range) override {
        if (it != data.end()) {
            time = it->time;
            pose = it->pose;
            full_range = std::move(it->full_range);
            it++;
            return true;
        }
        return false;
    }
};

class FR079DataProvider : public DataProvider {
    std::ifstream log_file;
    std::string line, prefix;
    g2o::SE2 last_pose;
    double last_tv, last_rv;
    double last_time = -1e10;

   public:
    FR079DataProvider(const std::string& fpath) : log_file(fpath) {
    }

    bool get_data(double& time, g2o::SE2& pose, std::vector<float>& full_range) override {
        while (std::getline(log_file, line)) {
            std::istringstream iss(line);
            iss >> prefix;
            if (prefix == "ODOM") {
                double junk;
                for (int i = 0; i < 3; i++)
                    iss >> junk;
                double tv, rv;
                iss >> tv >> rv >> junk;
                iss >> time;
                if (last_time != -1e10) {
                    double dL = (time - last_time) * last_tv;
                    double new_theta = last_pose[2] + (time - last_time) * last_rv * 1000;
                    last_pose = g2o::SE2(last_pose[0] + cos(new_theta) * dL, last_pose[1] + sin(new_theta) * dL, new_theta);
                }
                last_tv = tv;
                last_rv = rv;
                last_time = time;
            } else if (prefix == "FLASER") {
                if (last_time == -1e10)
                    continue;

                int num_readings;
                iss >> num_readings;
                full_range.resize(num_readings);
                for (int i = 0; i < num_readings; i++) {
                    iss >> full_range[i];
                }
                double temp;
                for (int i = 0; i < 3; i++)
                    iss >> temp;
                double data[3];
                for (int i = 0; i < 3; i++)
                    iss >> data[i];
                // pose = g2o::SE2(data[0], data[1], data[2]);
                iss >> time;
                double dL = (time - last_time) * last_tv;
                double new_theta = last_pose[2] + (time - last_time) * last_rv;
                pose = g2o::SE2(last_pose[0] + cos(new_theta) * dL, last_pose[1] + sin(new_theta) * dL, new_theta);
                return true;
            }
        }
        return false;
    }
};

class StanfordLogDataProvider : public DataProvider {
    std::ifstream log_file;
    std::string line, prefix, junk;

    g2o::SE2 last_pose;

   public:
    StanfordLogDataProvider(const std::string& fpath) : log_file(fpath), last_pose(-1e10, -1e10, -1e10) {
    }

    bool get_data(double& time, g2o::SE2& pose, std::vector<float>& full_range) override {
        while (std::getline(log_file, line)) {
            std::istringstream iss(line);
            // std::cout << line << std::endl;
            iss >> prefix;
            if (prefix[0] == '#')
                continue;
            for (int i = 0; i < 2; i++)
                iss >> junk;
            iss >> prefix;
            if (prefix == "position") {
                for (int i = 0; i < 2; i++)
                    iss >> junk;
                double data[3];
                for (int i = 0; i < 3; i++)
                    iss >> data[i];
                last_pose = g2o::SE2(data[0], data[1], data[2]);
            } else if (prefix == "laser") {
                if (last_pose[0] == -1e10)
                    continue;
                pose = last_pose;
                iss >> junk;
                iss >> time;

                for (int i = 0; i < 4; i++)
                    iss >> junk;
                full_range.resize(181);
                for (int i = 0; i < 181; i++) {
                    float f_junk;
                    iss >> full_range[i] >> f_junk;
                }
                last_pose = g2o::SE2(-1e10, -1e10, -1e10);
                return true;
            }
        }
        return false;
    }
};

class IntelOregonLogDataProvider : public DataProvider {
    std::ifstream log_file;
    std::string line, prefix, junk;

    g2o::SE2 last_pose;

   public:
    IntelOregonLogDataProvider(const std::string& fpath) : log_file(fpath), last_pose(-1e10, -1e10, -1e10) {
    }

    bool get_data(double& time, g2o::SE2& pose, std::vector<float>& full_range) override {
        while (std::getline(log_file, line)) {
            std::istringstream iss(line);
            // std::cout << line << std::endl;
            iss >> prefix;
            if (prefix[0] == '#')
                continue;
            for (int i = 0; i < 2; i++)
                iss >> junk;
            iss >> prefix;
            if (prefix == "position") {
                for (int i = 0; i < 2; i++)
                    iss >> junk;
                double data[3];
                for (int i = 0; i < 3; i++)
                    iss >> data[i];
                last_pose = g2o::SE2(data[0], data[1], data[2]);
            } else if (prefix == "laser") {
                if (last_pose[0] == -1e10)
                    continue;
                pose = last_pose;
                iss >> junk;
                iss >> time;
                full_range.resize(181);
                for (int i = 0; i < 181; i++) {
                    float f_junk;
                    iss >> full_range[i] >> f_junk >> junk;
                }
                last_pose = g2o::SE2(-1e10, -1e10, -1e10);
                return true;
            }
        }
        return false;
    }
};

class USCDataProvider : public DataProvider {
    std::ifstream log_file;
    std::string line, prefix, junk;

    g2o::SE2 last_pose;

   public:
    USCDataProvider(const std::string& fpath) : log_file(fpath), last_pose(-1e10, -1e10, -1e10) {
    }
    bool get_data(double& time, g2o::SE2& pose, std::vector<float>& full_range) override {
        while (std::getline(log_file, line)) {
            std::istringstream iss(line);
            // std::cout << line << std::endl;
            iss >> prefix;
            if (prefix == "position") {
                iss >> junk;
                iss >> junk;
                double data[3];
                for (int i = 0; i < 3; i++)
                    iss >> data[i];
                last_pose = g2o::SE2(data[0], data[1], data[2]);
            } else if (prefix == "laser") {
                iss >> junk;
                iss >> time;
                full_range.resize(181);
                for (int i = 0; i < 181; i++) {
                    float f_junk;
                    iss >> full_range[i] >> f_junk >> f_junk;
                }
                pose = last_pose;
                return true;
            }
        }
        return false;
    }
};


class EV3DataProvider : public DataProvider {
    std::ifstream log_file;
    std::string line, prefix, junk;
    double prev_time = 0;
    static unsigned constexpr num_readings = 13;
    static unsigned constexpr num_readings_per_bearing = 25;
    static unsigned constexpr scaling = 10;  // 100;

   public:
    EV3DataProvider(const std::string& fpath) : log_file(fpath) {}

    bool get_data(double& time, g2o::SE2& pose, std::vector<float>& full_range) override {
        if (!std::getline(log_file, line)) return false;
        std::istringstream iss(line);
        std::array<float, 3> pose_data{};
        for (size_t i = 0; i < pose_data.size(); ++i)
            iss >> pose_data.at(i);
        pose = g2o::SE2(pose_data.at(0) / scaling, pose_data.at(1) / scaling, pose_data.at(2));
        // std::cout << pose_data.at(0) / scaling << ", " << pose_data.at(1) / scaling << ", " << pose_data.at(2) << ": ";

        full_range.resize(num_readings);
        for (size_t i = 0; i < num_readings; ++i) {
            auto bearing = std::array<float, num_readings_per_bearing>{};
            for (size_t j = 0; j < num_readings_per_bearing; ++j) {
                float tmp = 0;
                iss >> tmp;
                bearing.at(i) = tmp;
                // bearing.at(j) = calibrate(tmp); // Calibrate the sensor readings
            }

            // ! Choose what filtering method we will use !
            // 1) N-th value filter (get 1st entry, susceptible to noise)
            full_range.at(i) = nth_filter<0>(bearing) / scaling;
            // 2) N-th value filter (get 25th entry, susceptible to noise)
            // full_range.at(i) = nth_filter<24>(bearing) / scaling;
            // 3) Average filter (incorporates outliers and skews results)
            // full_range.at(i) = average_filter(bearing) / scaling;
            // 4) Median filter (effectively removes outliers & impulse noise)
            // full_range.at(i) = median_filter(bearing) / scaling;
            // std::cout << full_range.at(i) << " ";
        }
        // std::cout << std::endl;
        time = prev_time;
        prev_time += 1.0;
        std::cout << "Read: " << prev_time << " lines" << std::endl;
        return true;
    }
   private:
    float calibrate(float value) const {
        // Inspired by: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7010761
        // if (value > 80)  value = 80;  // ! Cap readings at 80cm !
        if (value < 6)  // [3, 6)
            return 1.461 * value + -1.428;
        if (value < 12)  // [6, 12)
            return 0.819 * value + 2.710;
        if (value < 24)  // [12, 24)
            return 1.0 * value + 0.337;
        if (value < 40)  // [24, 40)
            return 0.856 * value + 4.415;
        return 1.00 * value + -0.476;  // [40, 80]
    }

    template<unsigned n>
    float nth_filter(std::array<float, num_readings_per_bearing> const& bearing) const {
        static_assert(n < bearing.size(), "Given index is out of bounds.");
        return bearing.at(n);
    }

    float average_filter(std::array<float, num_readings_per_bearing> const& bearing) const {
        float total_val = 0;
        for (float const val : bearing)
            total_val += val;
        return total_val / bearing.size();
    }

    float median_filter(std::array<float, num_readings_per_bearing>& bearing) const {
        // Use insertion sort since array is small
        for (size_t i = 1; i < bearing.size(); ++i) {
            auto temp = bearing.at(i);
            size_t j = i - 1;
            while (j != static_cast<size_t>(-1) && bearing.at(j) > temp) {
                bearing.at(j + 1) = bearing.at(j);
                --j;
            }
            bearing.at(j + 1) = temp;
        }
        // Get median
        size_t mid_idx = static_cast<size_t>((bearing.size() - 1) / 2);
        if (bearing.size() % 2 == 0) {
            return (bearing.at(mid_idx) + bearing.at(mid_idx + 1)) / 2;
        }
        return bearing.at(mid_idx);
    }

    float gaussian_filter(std::array<float, num_readings_per_bearing>& bearing) const {
        unsigned constexpr std = 2;  // TODO: What size do we want?...
        unsigned constexpr kernel_size = 6 * std + 1;  // This is a function of std
        // TODO: Fix Gaussian Filter.
        auto kernel = std::array<float, kernel_size>{};
        float k_sum = 0;
        for (size_t i = 0; i < kernel.size(); ++i) {
            kernel.at(i) = std::exp(-(i - kernel_size / 2) * (i - kernel_size / 2) / (2 * std * std));
            k_sum += kernel.at(i);
        }

        std::cout << "Kernel: ";
        for (size_t i = 0; i < kernel.size(); ++i) {
            kernel.at(i) /= k_sum;
            std::cout << kernel.at(i) << ", ";
        }
        std::cout << std::endl;

        std::cout << "Filtered: ";
        auto filtered = std::array<float, num_readings_per_bearing>{};
        for (size_t i = 0; i < bearing.size(); ++i) {
            size_t start = std::max(i - kernel.size() / 2, 0UL);
            size_t end = std::min(i + 1 + kernel.size() / 2, bearing.size());
            auto window = std::vector<float>(end - start);
            for (size_t j = 0; j < window.size(); ++j)
                window.at(j) = (j + start < end) ? bearing.at(j + start) : 0;

            auto kernel_window = std::vector<float>(end - start);
            size_t k_start = std::max(kernel.size() / 2 - (i - start), 0UL);
            size_t k_end = std::min(kernel.size() / 2 + (end - i), kernel.size());
            for (size_t j = k_start; j < k_end; ++j)
                kernel_window.at(j - k_start) = kernel.at(j);

            for (size_t j = 0; j < k_end - k_start; ++j)
                filtered.at(i) += window.at(j + k_start) * kernel_window.at(j);
            std::cout << filtered.at(i) << ", ";
        }
        std::cout << std::endl;

        // TODO: Choose what value we will use...
        return bearing.at(0);  // TODO: Fill in
    }
};

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

#include <Eigen/StdVector>

#include "sparse_gslam/RawData.h"
using sparse_gslam::RawData;

typedef message_filters::sync_policies::ApproximateTime<RawData, RawData> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> ATS;

class ROSBagDataProvider : public DataProvider {
    struct Data {
        double time;
        g2o::SE2 pose;
        float range[4];
    };
    std::vector<Data, Eigen::aligned_allocator<Data>> data;
    int i = 0;

   public:
    rosbag::Bag bag;
    ATS ats;
    ROSBagDataProvider(const std::string& fpath) : ats(MySyncPolicy(5)) {
        ats.registerCallback(&ROSBagDataProvider::callback, this);
        bag.open(fpath);  // BagMode is Read by default
        for (rosbag::MessageInstance const m : rosbag::View(bag)) {
            RawData::ConstPtr i = m.instantiate<RawData>();
            // std::cout << i->header.stamp << std::endl;
            if (m.getTopic() == "/crazyflie2/state_xyzv") {
                ats.add<0>(i);
            } else if (m.getTopic() == "/crazyflie2/state_ranger_qxyzw") {
                ats.add<1>(i);
            } else {
                abort();
            }
        }
        bag.close();
        std::cout << "bag closed" << std::endl;
    }
    void callback(const RawData& state1, const RawData& state2) {
        data.emplace_back();
        auto& d = data.back();

        tf::Quaternion q(state2.raw[5], state2.raw[6], state2.raw[7], state2.raw[8]);
        tf::Matrix3x3 mat(q);
        double _r, _p, _y;
        mat.getRPY(_r, _p, _y);
        d.time = state1.header.stamp.toSec();
        d.pose = g2o::SE2(state1.raw[0], state1.raw[1], _y);
        memcpy(d.range, state2.raw.data(), sizeof(float) * 4);
    }
    bool get_data(double& time, g2o::SE2& pose, std::vector<float>& full_range) {
        if (i >= data.size() / 4)
            return false;
        time = data[i].time;
        pose = data[i].pose;
        full_range.resize(4);
        memcpy(full_range.data(), data[i].range, 4 * sizeof(float));
        i++;
        return true;
    }
};

std::unique_ptr<DataProvider> create_data_provider(const std::string& name, const std::string& fpath) {
    if (name == "carmen") {
        return std::unique_ptr<DataProvider>(new CarmenLogDataProvider(fpath));
    } else if (name == "stanford") {
        return std::unique_ptr<DataProvider>(new StanfordLogDataProvider(fpath));
    } else if (name == "fr079") {
        return std::unique_ptr<DataProvider>(new FR079DataProvider(fpath));
    } else if (name == "usc") {
        return std::unique_ptr<DataProvider>(new USCDataProvider(fpath));
    } else if (name == "drone_bag") {
        return std::unique_ptr<DataProvider>(new ROSBagDataProvider(fpath));
    } else if (name == "oregon") {
        return std::unique_ptr<DataProvider>(new IntelOregonLogDataProvider(fpath));
    } else if (name == "ev3") {
        return std::unique_ptr<DataProvider>(new EV3DataProvider(fpath));
    }
    abort();
}
