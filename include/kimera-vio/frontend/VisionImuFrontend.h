/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionImuFrontend.h
 * @brief  Class describing an abstract VIO Frontend
 * @author Marcus Abate
 */

#pragma once

#include <gflags/gflags.h>

#include <atomic>
#include <memory>

#include "kimera-vio/frontend/FrontendInputPacketBase.h"
#include "kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/imu-frontend/ImuFrontendParams.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/pipeline/PipelineModule.h"
#include "kimera-vio/visualizer/Display-definitions.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

DECLARE_bool(visualize_feature_tracks);
DECLARE_bool(visualize_frontend_images);
DECLARE_bool(save_frontend_images);
DECLARE_bool(log_feature_tracks);
DECLARE_bool(log_mono_tracking_images);
DECLARE_bool(log_stereo_matching_images);

namespace VIO {

class VisionImuFrontend {
 public:
  // 宏定义：定义跟这个类相关的指针.
  KIMERA_POINTER_TYPEDEFS(VisionImuFrontend);
  // 将拷贝和构造函数给删去.
  KIMERA_DELETE_COPY_CONSTRUCTORS(VisionImuFrontend);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
 public:
  VisionImuFrontend(const ImuParams& imu_params,
                 const ImuBias& imu_initial_bias,
                 DisplayQueue* display_queue,
                 bool log_output);

  virtual ~VisionImuFrontend();

 public:
  FrontendOutputPacketBase::UniquePtr spinOnce(
      FrontendInputPacketBase::UniquePtr&& input);

  /* ------------------------------------------------------------------------ */
  // Update Imu Bias. This is thread-safe as imu_frontend_->updateBias is
  // thread-safe.
  // 更新imu bias项.
  inline void updateImuBias(const ImuBias& imu_bias) const {
    imu_frontend_->updateBias(imu_bias);
  }

  /* ------------------------------------------------------------------------ */
   /****************************************************************************
    * @brief 判断前端初始化是不是成功. 这里最主要得是线程安全【frontend_state_本身是原子型】.
    ***************************************************************************/
  inline bool isInitialized() const {
    return frontend_state_ != FrontendState::Bootstrap;
  }

  /* ------------------------------------------------------------------------ */
  // Get Imu Bias. This is thread-safe as imu_frontend_->getCurrentImuBias is
  // thread-safe.
  // 获得IMU内参，这里是线程安全的.
  inline ImuBias getCurrentImuBias() const {
    return imu_frontend_->getCurrentImuBias();
  }

  /* ------------------------------------------------------------------------ */
  // 这里主要是用于初始化过程中（没有多线程，不用考虑线程安全）.
  // 这里主要的操作就是更新IMU bias，然后在初始化的时候重置预积分项.
  inline void updateAndResetImuBias(const ImuBias& imu_bias) const {
    imu_frontend_->updateBias(imu_bias);
    imu_frontend_->resetIntegrationWithCachedBias();
  }

  /* ------------------------------------------------------------------------ */
  // Get IMU Params for IMU Frontend.
  // 在前端过程中获得IMU的参数.
  inline gtsam::PreintegratedImuMeasurements::Params getImuFrontendParams() {
    return imu_frontend_->getGtsamImuParams();
  }

  /* ------------------------------------------------------------------------ */
  // 打印出来当前追踪的状态.
  static void printTrackingStatus(const TrackingStatus& status,
                                  const std::string& type);

  /* ------------------------------------------------------------------------ */
  // 获得tracker的debug信息.
  inline DebugTrackerInfo getTrackerInfo() const {
    return tracker_->debug_info_;
  }

 protected:
  virtual FrontendOutputPacketBase::UniquePtr
      bootstrapSpin(FrontendInputPacketBase::UniquePtr&& input) = 0;

  virtual FrontendOutputPacketBase::UniquePtr
      nominalSpin(FrontendInputPacketBase::UniquePtr&& input) = 0;

  /* ------------------------------------------------------------------------ */
  // Reset ImuFrontend gravity. Trivial gravity is needed for initial alignment.
  // This is thread-safe as imu_frontend_->resetPreintegrationGravity is
  // thread-safe.
  // 重置IMU前端中的重力（这里和imu_frontend模块里的重置预积分重力[resetPreintegrationGravity]
  // 一样，都是线程安全的）.
  inline void resetGravity(const gtsam::Vector3& reset_value) const {
    imu_frontend_->resetPreintegrationGravity(reset_value);
  }

  /* ------------------------------------------------------------------------ */
  // 这里和IMU前端获得预积分重力一样都是线程安全的,主要都
  inline gtsam::Vector3 getGravity() const {
    return imu_frontend_->getPreintegrationGravity();
  }

  // 单目中去除外点.
  void outlierRejectionMono(
      const gtsam::Rot3& keyframe_R_cur_frame,
      Frame* frame_lkf,
      Frame* frame_k,
      TrackingStatusPose* status_pose_mono);

 protected:
  // 前端状态：->要么初始化中，要么运行中.
  enum class FrontendState {
    Bootstrap = 0u,  //! Initialize Frontend
    Nominal = 1u     //! Run Frontend
  };
  // 整个状态在多线程中是原子态.
  std::atomic<FrontendState> frontend_state_;

  // Counters[计数].
  // 有多少图像帧.
  int frame_count_;
  // 有多少图像关键帧.
  int keyframe_count_;

  // Timestamp of last keyframe.
  Timestamp last_keyframe_timestamp_;

  // IMU前端【预积分？】
  ImuFrontend::UniquePtr imu_frontend_;

  // 特征跟踪模块
  Tracker::UniquePtr tracker_;
  // 特征跟踪状态的总结
  TrackerStatusSummary tracker_status_summary_;

  // 展示结果.
  DisplayQueue* display_queue_;

  // Logger记录.
  FrontendLogger::UniquePtr logger_;
};

}  // namespace VIO
