// Adapter diagnostics, independent of ROS and the estimator's covariance.
#pragma once
#include <cmath>
#include <cstddef>
#include <map>
#include <set>
#include <vector>

namespace docking_quality {
using Failures = std::map<size_t, size_t>;

// Pinned OpenVINS: existing SLAM landmarks with two current stereo observations
// reach UpdaterSLAM::update; a rejected update increments update_fail_count.
// Require the complete normal update path, exclude new/marginalized landmarks,
// and require both cameras even for representations that accept one observation.
inline size_t accepted_stereo_slam(const Failures& before, const Failures& after,
                                 const std::vector<size_t>& left,
                                 const std::vector<size_t>& right, bool completed) {
  if (!completed) return 0;
  std::set<size_t> l(left.begin(), left.end()), r(right.begin(), right.end());
  size_t count = 0;
  for (const auto& item : before) {
    auto found = after.find(item.first);
    if (found != after.end() && found->second == item.second &&
        l.count(item.first) && r.count(item.first)) ++count;
  }
  return count;
}

struct Report {
  size_t continued = 0;
  bool state_current = false, tracking = false, corrected = false;
  double correction_age = -1;
};

class Monitor {
  std::set<size_t> previous_;
  double last_correction_ = -1;
public:
  Report observe(double capture, double state, bool initialized,
                 const std::vector<size_t>& ids, size_t msckf, size_t slam, bool zupt) {
    Report out;
    std::set<size_t> current(ids.begin(), ids.end());
    for (auto id : current) if (previous_.count(id)) ++out.continued;
    out.state_current = initialized && std::isfinite(state) && std::isfinite(capture) &&
                        std::abs(state-capture) < 1e-5;
    out.tracking = out.state_current && current.size() >= 20 && out.continued >= 20;
    out.corrected = out.state_current && (zupt || msckf+slam >= 3);
    if (out.corrected) last_correction_ = capture;
    if (last_correction_ >= 0 && capture >= last_correction_)
      out.correction_age = capture-last_correction_;
    previous_ = std::move(current);
    return out;
  }
};
}
