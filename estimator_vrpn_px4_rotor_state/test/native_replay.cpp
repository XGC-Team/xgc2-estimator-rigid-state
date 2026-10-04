// A transport-free ABI consumer: no Host implementation or ROS dependency.
#include <xgc-robotics-interfaces/robotics_interfaces_v1.h>

#include <array>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

#include "estimator_vrpn_px4_rotor_state/common/types.h"
#include "estimator_vrpn_px4_rotor_state/native/rigid_state_wire_v1.h"
#include "xgc_rt.h"

namespace rs = estimator_vrpn_px4_rotor_state;
struct Sample {
    int64_t receipt;
    std::vector<uint8_t> data;
};
struct Consumer {
    std::array<std::deque<Sample>, 2> inputs;
    Sample current;
    std::vector<xgc_rigid_state_estimate_v1> estimates;
    bool print_outputs = true;
    xgc_host_api host{};
    const xgc_plugin_vtbl* vtbl = xgc_rt_plugin_v1()->vtbl;
    void* instance = nullptr;
    Consumer(const char* config, bool printing = true) : print_outputs(printing) {
        host.abi_version = XGC_RT_ABI_VERSION;
        host.abi_minor = XGC_RT_ABI_MINOR;
        host.host = this;
        host.publish = [](void* p, uint32_t port, uint64_t, const uint8_t* bytes, uint32_t len) {
            auto& self = *static_cast<Consumer*>(p);
            if (port == 4 && len == sizeof(xgc_rigid_state_estimate_v1)) {
                xgc_rigid_state_estimate_v1 estimate{};
                std::memcpy(&estimate, bytes, len);
                self.estimates.push_back(estimate);
            }
            if (self.print_outputs && (port == 2 || port == 3)) {
                std::printf("%c", port == 2 ? 'S' : 'V');
                for (uint32_t i = 0; i < len / sizeof(double); ++i) {
                    uint64_t bits;
                    std::memcpy(&bits, bytes + i * sizeof(double), sizeof bits);
                    std::printf(" %016" PRIx64, bits);
                }
                std::printf("\n");
            }
            return XGC_OK;
        };
        host.next = [](void* p, uint32_t port, xgc_sample_view* view) {
            auto& self = *static_cast<Consumer*>(p);
            if (port >= self.inputs.size() || self.inputs[port].empty())
                return XGC_ERR_AGAIN;
            self.current = std::move(self.inputs[port].front());
            self.inputs[port].pop_front();
            *view = {};
            view->t_rx = self.current.receipt;
            view->len = static_cast<uint32_t>(self.current.data.size());
            view->data = self.current.data.data();
            return XGC_OK;
        };
        host.log = [](void*, xgc_log_level, const char* message) {
            std::fprintf(stderr, "%s\n", message);
        };
        instance = vtbl->create(&host);
        if (!instance || vtbl->configure(instance, config) != XGC_OK ||
            vtbl->activate(instance) != XGC_OK)
            throw std::runtime_error("native lifecycle setup failed");
    }
    ~Consumer() {
        if (instance) {
            vtbl->deactivate(instance);
            vtbl->destroy(instance);
        }
    }
    template <class T>
    void push(uint32_t port, const T& message, double receipt) {
        Sample s{static_cast<int64_t>(receipt * 1e9), std::vector<uint8_t>(sizeof message)};
        std::memcpy(s.data.data(), &message, sizeof message);
        inputs[port].push_back(std::move(s));
    }
    bool step(double now) {
        xgc_step_ctx ctx{};
        ctx.now = static_cast<int64_t>(now * 1e9);
        ctx.round_advanced = 1;
        ctx.dirty_ports = 3;
        return vtbl->step(instance, &ctx) == XGC_OK;
    }
};

bool read(double* out, int n) {
    for (int i = 0; i < n; ++i) {
        uint64_t bits;
        if (std::scanf("%" SCNx64, &bits) != 1)
            return false;
        std::memcpy(&out[i], &bits, sizeof bits);
    }
    return true;
}

int backward() {
    // Both ports and both time modes must retain an intra-port reversal in one
    // drain. Sorting source stamps would turn all four controls into false passes.
    for (const char* mode : {"session", "input"}) {
        for (unsigned port : {0u, 1u}) {
            const auto config =
                std::string("extrinsic_verified = true\ntime_source = \"") + mode + "\"";
            Consumer c(config.c_str(), false);
            if (port == 0) {
                xgc_imu_v1 sample{};
                sample.accel[2] = 9.81;
                sample.stamp = 10.02;
                c.push(0, sample, 10.02);
                sample.stamp = 10.01;
                c.push(0, sample, 10.03);
            } else {
                xgc_pose_v1 sample{};
                sample.q_wxyz[0] = 1;
                sample.stamp = 10.02;
                c.push(1, sample, 10.02);
                sample.stamp = 10.01;
                c.push(1, sample, 10.03);
            }
            if (!c.step(10.04) || c.estimates.empty() ||
                !(c.estimates.back().flags & rs::kTimeJump)) {
                std::fprintf(stderr, "lost backward time: mode=%s port=%u\n", mode, port);
                return 1;
            }
            std::printf("backward time retained: mode=%s port=%u flags=%u\n", mode, port,
                        c.estimates.back().flags);
        }
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--backward")
        return backward();
    Consumer c("extrinsic_verified = true\ntime_source = \"input\"");
    unsigned port;
    double stamp, now = 0;
    while (std::scanf("%u", &port) == 1 && read(&stamp, 1)) {
        now = stamp;
        if (port == 0) {
            xgc_imu_v1 sample{};
            sample.stamp = stamp;
            if (!read(sample.accel, 3) || !read(sample.gyro, 3))
                return 2;
            c.push(0, sample, stamp);
        } else {
            xgc_pose_v1 sample{};
            sample.stamp = stamp;
            if (!read(sample.position, 3) || !read(sample.q_wxyz, 4))
                return 2;
            c.push(1, sample, stamp);
        }
    }
    return c.step(now) ? 0 : 1;
}
