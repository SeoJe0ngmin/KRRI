#include "quality.hpp"
#include <cassert>
#include <numeric>

int main() {
  using namespace docking_quality;
  std::vector<size_t> ids(40);std::iota(ids.begin(),ids.end(),100);
  Monitor monitor;
  auto r=monitor.observe(0,0,false,ids,0,0,false);
  assert(!r.tracking && r.correction_age<0);
  r=monitor.observe(.1,.1,true,ids,3,0,false);
  assert(r.tracking && r.corrected && r.continued==40);
  r=monitor.observe(.4,.4,true,ids,0,0,false);
  assert(r.tracking && !r.corrected && std::abs(r.correction_age-.3)<1e-8);
  // Existing SLAM updates and ZUPT are not dependent on MSCKF features.
  r=monitor.observe(.5,.5,true,ids,0,3,false);
  assert(r.corrected && r.correction_age==0);
  r=monitor.observe(.6,.6,true,ids,0,0,true);
  assert(r.corrected && r.correction_age==0);
  r=monitor.observe(.7,.6,true,ids,10,0,true);
  assert(!r.tracking && !r.corrected && !r.state_current);
  std::iota(ids.begin(),ids.end(),200);
  r=monitor.observe(.8,.8,true,ids,0,0,false);
  assert(!r.tracking && r.continued==0); // fresh detections are not continued tracks
  assert(std::abs(r.correction_age-.2)<1e-8);
  // Count old landmarks only, excluding rejection, monocular-only observations,
  // removed landmarks and incomplete update paths.
  Failures before{{100,0},{101,1},{102,0},{103,0}},after{{100,0},{101,2},{102,0},{104,0}};
  std::vector<size_t> left{100,101,102,103,104},right{100,101,103,104};
  assert(accepted_stereo_slam(before,after,left,right,true)==1);
  assert(accepted_stereo_slam(before,after,left,right,false)==0);
  // Stationary startup can complete inertial initialization and use ZUPT
  // without ever completing the normal MSCKF path (timelastupdate stays -1).
  // No MSCKF/SLAM update is required to report this initialized state as tracked.
  Monitor stationary;
  stationary.observe(0,0,false,ids,0,0,false);
  for (int i=1;i<100;++i) {
    r=stationary.observe(i*.1,i*.1,true,ids,0,0,true);
    assert(r.tracking && r.corrected && r.correction_age==0);
  }
}
