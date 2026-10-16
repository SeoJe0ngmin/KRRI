// OpenVINS API adapter. Upstream estimator is unmodified and pinned by commit.
// SPDX-License-Identifier: GPL-3.0-or-later
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <cmath>
#include <csignal>
#include <cerrno>
#include <stdexcept>
#include <vector>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <docking_v6_msgs/msg/motion.hpp>
#include <docking_v6_msgs/msg/stereo.hpp>
#include <docking_v6_msgs/msg/vio_state.hpp>
#include <core/VioManager.h>
#include <core/VioManagerOptions.h>
#include <state/StateHelper.h>
#include <state/State.h>
#include <track/TrackBase.h>
#include <utils/sensor_data.h>
#include <types/IMU.h>
#include <types/Landmark.h>
#include "quality.hpp"

using Motion=docking_v6_msgs::msg::Motion;
using Stereo=docking_v6_msgs::msg::Stereo;
using Output=docking_v6_msgs::msg::VioState;
static double seconds(const builtin_interfaces::msg::Time& t) { return t.sec+t.nanosec*1e-9; }
class Estimator : public ov_msckf::VioManager {
public:
  explicit Estimator(ov_msckf::VioManagerOptions& p):VioManager(p) {}
  // Upstream initialized() also requires timelastupdate != -1. That timestamp
  // is only set at the END of the normal visual update; initial clone warmup
  // can return first, followed by ZUPT forever while stationary. The actual
  // inertial initialization flag already certifies a constructed state/P.
  // Tracking, current timestamp and correction evidence are checked separately.
  bool filter_initialized() const { return is_initialized_vio; }
  std::vector<cv::KeyPoint> tracks() { return trackFEATS->get_last_obs()[0]; }
  std::vector<size_t> ids(size_t camera) { return trackFEATS->get_last_ids()[camera]; }
  docking_quality::Failures slam_failures() const {
    docking_quality::Failures failures;
    for (const auto& item : state->_features_SLAM) {
      // ArUco rejection does not increment this counter; do not infer its updates.
      if (item.first > size_t(4*state->_options.max_aruco_features) && !item.second->should_marg)
        failures[item.first]=item.second->update_fail_count;
    }
    return failures;
  }
  bool normal_update_completed(double stamp) const { return std::abs(timelastupdate-stamp)<1e-5; }
  bool zupt() const { return did_zupt_update && !has_moved_since_zupt; }
  void clear_quality() { good_features_MSCKF.clear(); }
};

class Bridge : public rclcpp::Node {
  struct ImageItem { Stereo::ConstSharedPtr msg; std::chrono::steady_clock::time_point arrived; };
  std::unique_ptr<Estimator> estimator_;
  std::mutex mutex_; std::condition_variable wake_;
  std::deque<Motion::ConstSharedPtr> imus_; std::deque<ImageItem> images_;
  std::thread worker_; std::atomic<bool> running_{true};
  uint64_t epoch_=0,sequence_=0,dropped_=0; bool have_epoch_=false;
  int parent_pid_=0; double last_published_=-1;
  docking_quality::Monitor quality_;
  docking_quality::Failures slam_before_;
  Eigen::Matrix4d previous_T_=Eigen::Matrix4d::Identity();
  double last_received_imu_=-1,last_received_image_=-1,max_age_=.3;
  std::atomic<double> last_fed_imu_{-1};
  std::atomic<uint64_t> states_published_{0};
  std::atomic<const char*> stage_{"waiting_for_input"};
  rclcpp::TimerBase::SharedPtr health_timer_;
  Eigen::Vector3d gyro_=Eigen::Vector3d::Zero();
  std::string fault_;
  rclcpp::Subscription<Motion>::SharedPtr imu_sub_;
  rclcpp::Subscription<Stereo>::SharedPtr image_sub_;
  rclcpp::Publisher<Output>::SharedPtr output_;
  bool epoch_ok(uint64_t e) {
    if(!have_epoch_) {epoch_=e;have_epoch_=true;}
    if(e!=epoch_) {fault_="sensor_epoch_changed_restart_required";return false;}
    return true;
  }
  static cv::Mat image(const sensor_msgs::msg::Image& m) {
    if(m.encoding!="mono8" || m.height==0 || m.width==0 || m.step<m.width || m.data.size()!=m.step*m.height)
      throw std::runtime_error("invalid_stereo_image");
    return cv::Mat(m.height,m.width,CV_8UC1,const_cast<unsigned char*>(m.data.data()),m.step).clone();
  }
  void feed_imu(const Motion& m) {
    ov_core::ImuData d;d.timestamp=seconds(m.header.stamp);
    for(int i=0;i<3;i++) {d.wm(i)=m.gyro[i];d.am(i)=m.accel[i];}
    if(!d.wm.allFinite() || !d.am.allFinite())throw std::runtime_error("nonfinite_imu");
    estimator_->feed_measurement_imu(d);last_fed_imu_=d.timestamp;gyro_=d.wm;
  }
  static Eigen::Matrix4d changed(Eigen::Matrix4d T,const Eigen::Matrix<double,6,1>& e) {
    T.block<3,1>(0,3)+=e.head<3>();double angle=e.tail<3>().norm();
    if(angle>0)T.block<3,3>(0,0)=Eigen::AngleAxisd(angle,e.tail<3>()/angle).toRotationMatrix()*T.block<3,3>(0,0);
    return T;
  }
  static Eigen::Matrix<double,6,1> difference(const Eigen::Matrix4d& D,const Eigen::Matrix4d& nominal) {
    Eigen::Matrix<double,6,1> y;y.head<3>()=D.block<3,1>(0,3);
    Eigen::AngleAxisd a(D.block<3,3>(0,0)*nominal.block<3,3>(0,0).transpose());y.tail<3>()=a.angle()*a.axis();return y;
  }
  void relative_quality(Output& out,std::shared_ptr<ov_msckf::State> state,const Eigen::Matrix4d& T) {
    auto found=state->_clones_IMU.find(last_published_);
    if(found!=state->_clones_IMU.end()) {
      auto clone=found->second;auto imu=state->_imu;
      Eigen::Matrix4d A=Eigen::Matrix4d::Identity();A.block<3,3>(0,0)=clone->Rot().transpose();A.block<3,1>(0,3)=clone->pos();
      std::vector<std::shared_ptr<ov_type::Type>> vars={clone->p(),clone->q(),imu->p(),imu->q()};
      Eigen::Matrix<double,12,12> P=ov_msckf::StateHelper::get_marginal_covariance(state,vars);
      Eigen::Matrix<double,12,12> G=Eigen::Matrix<double,12,12>::Identity();
      G.block<3,3>(3,3)=A.block<3,3>(0,0);G.block<3,3>(9,9)=T.block<3,3>(0,0);P=G*P*G.transpose();
      Eigen::Matrix4d nominal=A.inverse()*T, actual=previous_T_.inverse()*T;
      Eigen::Matrix<double,6,12> J;
      for(int i=0;i<12;i++) {
        Eigen::Matrix<double,6,1> e=Eigen::Matrix<double,6,1>::Zero();e(i%6)=1e-5;
        Eigen::Matrix4d plus=i<6?changed(A,e).inverse()*T:A.inverse()*changed(T,e);
        Eigen::Matrix4d minus=i<6?changed(A,-e).inverse()*T:A.inverse()*changed(T,-e);
        J.col(i)=(difference(plus,nominal)-difference(minus,nominal))/(2e-5);
      }
      Eigen::Matrix<double,6,1> revision=difference(actual,nominal)-difference(nominal,nominal);
      Eigen::Matrix<double,6,6> C=2*(J*P*J.transpose()+revision*revision.transpose());C=(C+C.transpose()).eval()*.5;
      if(!C.allFinite())throw std::runtime_error("nonfinite_relative_covariance");
      out.delta_valid=true;out.delta_from_stamp=last_published_;
      for(int r=0;r<4;r++)for(int c=0;c<4;c++)out.relative_transform[r*4+c]=actual(r,c);
      for(int r=0;r<6;r++)for(int c=0;c<6;c++)out.relative_covariance[r*6+c]=C(r,c);
    }
    // ZUPT returns before the normal camera clone augmentation. Retain an
    // exact correlated clone for the next relative covariance query as well.
    if(!state->_clones_IMU.count(state->_timestamp)) {
      state->_clones_IMU[state->_timestamp]=std::dynamic_pointer_cast<ov_type::PoseJPL>(ov_msckf::StateHelper::clone(state,state->_imu->pose()));
      ov_msckf::StateHelper::marginalize_old_clone(state);
    }
    last_published_=state->_timestamp;previous_T_=T;
  }
  void publish(const Stereo& m) {
    Output out;out.header=m.header;out.header.frame_id="odom";out.epoch=m.epoch;
    out.quality_version=1;out.correction_age_s=-1;
    out.last_imu_stamp=last_fed_imu_;out.initialized=estimator_->filter_initialized();
    {std::lock_guard<std::mutex> lock(mutex_);out.stereo_dropped=dropped_;out.fault=fault_;}
    auto tracks=estimator_->tracks();out.features=tracks.size();out.zero_velocity_update=estimator_->zupt();
    for(const auto& key:tracks) {geometry_msgs::msg::Point32 p;p.x=key.pt.x;p.y=key.pt.y;out.left_points.push_back(p);}
    if(out.initialized) {
      auto state=estimator_->get_state();auto imu=state->_imu;
      Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
      // OpenVINS Rot() is R_imu_global; its error is left JPL in IMU axes.
      // Inverse rotation uses a positive right perturbation. Transform that
      // perturbation to world LEFT axes before exporting covariance.
      T.block<3,3>(0,0)=imu->Rot().transpose();T.block<3,1>(0,3)=imu->pos();
      std::vector<std::shared_ptr<ov_type::Type>> vars={imu->p(),imu->q()};
      Eigen::Matrix<double,6,6> P=ov_msckf::StateHelper::get_marginal_covariance(state,vars);
      Eigen::Matrix<double,6,6> J=Eigen::Matrix<double,6,6>::Identity();J.block<3,3>(3,3)=T.block<3,3>(0,0);
      P=J*P*J.transpose();
      if(!T.allFinite() || !P.allFinite())throw std::runtime_error("nonfinite_openvins_state");
      for(int r=0;r<4;r++)for(int c=0;c<4;c++)out.t_odom_imu[4*r+c]=T(r,c);
      for(int r=0;r<6;r++)for(int c=0;c<6;c++)out.covariance[6*r+c]=P(r,c);
      for(int i=0;i<3;i++) {out.gyro_bias[i]=imu->bias_g()(i);out.accel_bias[i]=imu->bias_a()(i);out.velocity_world[i]=imu->vel()(i);}
      out.gyro_norm=(gyro_-imu->bias_g()).norm();
    }
    auto state=estimator_->get_state();out.state_stamp=state->_timestamp;
    out.msckf_update_features=estimator_->get_good_features_MSCKF().size();
    out.slam_update_features=docking_quality::accepted_stereo_slam(slam_before_,estimator_->slam_failures(),
        estimator_->ids(0),estimator_->ids(1),estimator_->normal_update_completed(seconds(m.header.stamp)));
    auto q=quality_.observe(seconds(m.header.stamp),state->_timestamp,out.initialized,
        estimator_->ids(0),out.msckf_update_features,out.slam_update_features,out.zero_velocity_update);
    out.continued_features=q.continued;out.tracking_valid=q.tracking;
    out.correction_applied=q.corrected;out.correction_age_s=q.correction_age;
    // Retained field name for mapping consumers; freshness/uncertainty/control
    // eligibility are evaluated separately in the Python observer.
    out.visual_valid=q.tracking;
    if(q.state_current) {
      Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
      T.block<3,3>(0,0)=state->_imu->Rot().transpose();T.block<3,1>(0,3)=state->_imu->pos();
      relative_quality(out,state,T);
    }
    output_->publish(out);++states_published_;
  }
  void work() {
    while(running_) {
      if(parent_pid_>0 && kill(parent_pid_,0)<0 && errno==ESRCH){rclcpp::shutdown();break;}
      Motion::ConstSharedPtr motion;Stereo::ConstSharedPtr camera;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        // Never sleep once per queued IMU sample: 10 ms would cap a burst at
        // 100 Hz even though the camera produces 200 Hz gyroscope samples.
        wake_.wait_for(lock,std::chrono::milliseconds(10),[this]{
          return !running_ || !fault_.empty() || !imus_.empty() ||
            (!images_.empty() && last_fed_imu_>=seconds(images_.front().msg->header.stamp));
        });
        if(!running_)break;
        if(!fault_.empty()) {
          Output out;out.epoch=epoch_;out.fault=fault_;output_->publish(out);
          wake_.wait_for(lock,std::chrono::milliseconds(10));continue;
        }
        if(!images_.empty() && last_fed_imu_>=seconds(images_.front().msg->header.stamp)) {
          auto item=images_.front();images_.pop_front();
          if(std::chrono::duration<double>(std::chrono::steady_clock::now()-item.arrived).count()>max_age_){dropped_++;continue;}
          camera=item.msg;
        } else if(!imus_.empty()) {motion=imus_.front();imus_.pop_front();}
      }
      try {
        if(motion){stage_="feed_imu";feed_imu(*motion);}
        if(camera) {
          stage_="track_and_update";
          if(seconds(camera->left.header.stamp)!=seconds(camera->header.stamp) ||
             seconds(camera->right.header.stamp)!=seconds(camera->header.stamp))throw std::runtime_error("stereo_stamp_mismatch");
          ov_core::CameraData d;d.timestamp=seconds(camera->header.stamp);d.sensor_ids={0,1};
          d.images={image(camera->left),image(camera->right)};
          d.masks={cv::Mat::zeros(d.images[0].size(),CV_8UC1),cv::Mat::zeros(d.images[1].size(),CV_8UC1)};
          slam_before_=estimator_->slam_failures();
          estimator_->clear_quality();estimator_->feed_measurement_camera(d);
          stage_="publish_state";publish(*camera);
        }
        stage_="waiting_for_input";
      } catch(const std::exception& e) {std::lock_guard<std::mutex> lock(mutex_);fault_=e.what();}
    }
  }
public:
  Bridge():Node("docking_openvins") {
    auto config=declare_parameter<std::string>("config_path","");
    parent_pid_=declare_parameter<int>("parent_pid",0);
    max_age_=declare_parameter<double>("max_input_age_s",.3);
    if(config.empty())throw std::runtime_error("config_path_required");
    auto parser=std::make_shared<ov_core::YamlParser>(config);
    ov_core::Printer::setPrintLevel("WARNING");
    ov_msckf::VioManagerOptions params;params.print_and_load(parser);
    params.use_multi_threading_subs=false;params.use_multi_threading_pubs=false;
    if(!parser->successful() || params.state_options.num_cameras!=2 || !params.state_options.do_fej)
      throw std::runtime_error("invalid_stereo_fej_configuration");
    estimator_=std::make_unique<Estimator>(params);
    output_=create_publisher<Output>("vio_state",rclcpp::QoS(8).reliable());
    imu_sub_=create_subscription<Motion>("imu",rclcpp::QoS(2048).reliable(),[this](Motion::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);if(!epoch_ok(m->epoch))return;
      double t=seconds(m->header.stamp);
      if((sequence_ && m->sequence!=sequence_+1) || (last_received_imu_>=0 && (t<=last_received_imu_ || t-last_received_imu_>.05)))
        fault_="imu_sequence_or_clock_gap";
      sequence_=m->sequence;last_received_imu_=t;
      if(imus_.size()>=4096)fault_="native_imu_overflow";
      if(fault_.empty())imus_.push_back(m);wake_.notify_one();
    });
    image_sub_=create_subscription<Stereo>("stereo",rclcpp::QoS(2).reliable(),[this](Stereo::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);if(!epoch_ok(m->epoch))return;
      double t=seconds(m->header.stamp);
      if(t<=last_received_image_)fault_="stereo_clock_reversed";
      last_received_image_=t;if(!fault_.empty())return;
      if(images_.size()>=2){images_.pop_front();dropped_++;}
      images_.push_back({m,std::chrono::steady_clock::now()});wake_.notify_one();
    });
    // Runs on the ROS executor, independently of the estimator worker. A stuck
    // initializer/tracker therefore leaves useful evidence instead of silence.
    health_timer_=create_wall_timer(std::chrono::seconds(2),[this]{
      std::lock_guard<std::mutex> lock(mutex_);
      fprintf(stderr,"[pipeline] stage=%s imu_sequence=%llu imu_received=%.6f imu_fed=%.6f image_received=%.6f imu_queue=%zu image_queue=%zu dropped=%llu outputs=%llu state_subscribers=%zu fault=%s\n",
        stage_.load(),static_cast<unsigned long long>(sequence_),last_received_imu_,last_fed_imu_.load(),
        last_received_image_,imus_.size(),images_.size(),static_cast<unsigned long long>(dropped_),
        static_cast<unsigned long long>(states_published_.load()),output_->get_subscription_count(),fault_.c_str());
    });
    worker_=std::thread([this]{work();});
  }
  ~Bridge(){running_=false;wake_.notify_one();if(worker_.joinable())worker_.join();}
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<Bridge>());} catch(const std::exception& e) {fprintf(stderr,"OpenVINS bridge: %s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
