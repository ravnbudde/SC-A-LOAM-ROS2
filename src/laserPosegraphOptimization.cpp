#include <fstream>
#include <filesystem>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <atomic>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lw_messages/msg/reset_event.hpp>

#include <eigen3/Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "aloam_velodyne/ros2_utils.hpp"
#include "aloam_velodyne/laser_pgo_node.hpp"

#include "scancontext/Scancontext.h"

using namespace gtsam;

using std::cout;
using std::endl;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.
double yawAccumulatedSinceKeyframe = 0.0;
double maxYawRateSinceKeyframe = 0.0;
double keyframeIntervalStartTime = 0.0;

bool isNowKeyFrame = false; 

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero 

std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometryBuf;
std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> fullResBuf;
std::queue<sensor_msgs::msg::NavSatFix::ConstSharedPtr> gpsBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;
double timeLaserOdometryPrev = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
std::vector<double> keyframeYawRates;
int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam = nullptr;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius;
double loopIcpMaxCorrespondenceDistance, loopIcpFitnessThreshold;
int loopHistoryKeyframeSearchNum, scNumExcludeRecent;
double loopClosureFrequency, loopMinPathDistance, loopMaxKeyframeYawRate;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;

bool useGPS = true;
std::atomic_bool pgoRunning{false};
std::atomic<double> resetCutoffStamp{0.0};
std::atomic_bool awaitingPostResetBaseline{false};
bool pubLoopScan = true;
bool pubLoopSubmap = true;
// bool useGPS = false;
sensor_msgs::msg::NavSatFix::ConstSharedPtr currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false; 
double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopSubmapLocal;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomRepubVerifier;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

std::string map_frame_id = "camera_init";
std::string body_frame_id = "aft_pgo";
bool enable_tf = true;

std::string save_directory;
std::string pgKITTIformat, pgScansDirectory, pgSCDsDirectory;
std::string odomKITTIformat;
std::fstream pgG2oSaveStream, pgTimeSaveStream;

std::vector<std::string> edges_str; // used in writeEdge

std::string padZeros(int val, int num_digits = 6) 
{
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

std::string getVertexStr(const int _node_idx, const gtsam::Pose3& _Pose)
{
    gtsam::Point3 t = _Pose.translation();
    gtsam::Rot3 R = _Pose.rotation();

    std::string curVertexInfo {
        "VERTEX_SE3:QUAT " + std::to_string(_node_idx) + " "
        + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
        + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
        + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

    // pgVertexSaveStream << curVertexInfo << std::endl;
    // vertices_str.emplace_back(curVertexInfo);
    return curVertexInfo;
}

void writeEdge(const std::pair<int, int> _node_idx_pair, const gtsam::Pose3& _relPose, std::vector<std::string>& edges_str)
{
    gtsam::Point3 t = _relPose.translation();
    gtsam::Rot3 R = _relPose.rotation();

    std::string curEdgeInfo {
        "EDGE_SE3:QUAT " + std::to_string(_node_idx_pair.first) + " " + std::to_string(_node_idx_pair.second) + " "
        + std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z())  + " " 
        + std::to_string(R.toQuaternion().x()) + " " + std::to_string(R.toQuaternion().y()) + " " 
        + std::to_string(R.toQuaternion().z()) + " " + std::to_string(R.toQuaternion().w()) };

    // pgEdgeSaveStream << curEdgeInfo << std::endl;
    edges_str.emplace_back(curEdgeInfo);
}

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ")
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
 
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void saveGTSAMgraphG2oFormat(const gtsam::Values& _estimates)
{
    // save pose graph (runs when programe is closing)
    // cout << "****************************************************" << endl; 
    RCLCPP_DEBUG(rclcpp::get_logger("laserPGO"), "Saving the posegraph ..."); // giseop

    pgG2oSaveStream = std::fstream(save_directory + "singlesession_posegraph.g2o", std::fstream::out);

    int pose_idx = 0;
    for(const auto& _pose6d: keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);    
        pgG2oSaveStream << getVertexStr(pose_idx, pose) << endl;
        pose_idx++;
    }
    for(auto& _line: edges_str)
        pgG2oSaveStream << _line << std::endl;

    pgG2oSaveStream.close();
}

void saveOdometryVerticesKITTIformat(std::string _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

void laserOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr &_laserOdometry)
{
    if (stampToSec(_laserOdometry->header.stamp) <= resetCutoffStamp.load()) return;
	mBuf.lock();
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &_laserCloudFullRes)
{
    if (stampToSec(_laserCloudFullRes->header.stamp) <= resetCutoffStamp.load()) return;
	mBuf.lock();
	fullResBuf.push(_laserCloudFullRes);
	mBuf.unlock();
} // laserCloudFullResHandler

void gpsHandler(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &_gps)
{
    if(useGPS) {
        mBuf.lock();
        gpsBuf.push(_gps);
        mBuf.unlock();
    }
} // gpsHandler

void initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

Pose6D getOdom(nav_msgs::msg::Odometry::ConstSharedPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion quat = _odom->pose.pose.orientation;
    tf2::Matrix3x3(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw}; 
} // getOdom

Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    
    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void pubPath( void )
{
    // pub odom and path 
    nav_msgs::msg::Odometry odomAftPGO;
    nav_msgs::msg::Path pathAftPGO;
    pathAftPGO.header.frame_id = map_frame_id;
    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
        // const gtsam::Pose3& pose_est = isamCurrentEstimate.at<gtsam::Pose3>(node_idx);

        nav_msgs::msg::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = map_frame_id;
        odomAftPGOthis.child_frame_id = body_frame_id;
        odomAftPGOthis.header.stamp = rosTimeFromSec(keyframeTimes.at(node_idx));
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        tf2::Quaternion q;
        q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGOthis.pose.pose.orientation = tf2::toMsg(q);
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::msg::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = map_frame_id;
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    mKF.unlock(); 
    pubOdomAftPGO->publish(odomAftPGO); // last pose 
    pubPathAftPGO->publish(pathAftPGO); // poses 

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odomAftPGO.header.stamp;
    transform.header.frame_id = map_frame_id;
    transform.child_frame_id = body_frame_id;
    transform.transform.translation.x = odomAftPGO.pose.pose.position.x;
    transform.transform.translation.y = odomAftPGO.pose.pose.position.y;
    transform.transform.translation.z = odomAftPGO.pose.pose.position.z;
    transform.transform.rotation = odomAftPGO.pose.pose.orientation;
    if (enable_tf) {
        tfBroadcaster->sendTransform(transform);
    }
} // pubPath

void updatePoses(void)
{
    mKF.lock(); 
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

    mtxRecentPose.unlock();
} // updatePoses

void runISAM2opt(void)
{
    // called when a variable added 
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(), 
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );
    
    int numberOfCores = 8; // TODO move to yaml 
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = key + i; // see https://github.com/gisbi-kim/SC-A-LOAM/issues/7 ack. @QiMingZhenFan found the error and modified as below. 
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        mKF.lock(); 
        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[root_idx]);
        mKF.unlock(); 
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud


double keyframePathDistance(const int start_idx, const int end_idx)
{
    if (start_idx < 0 || end_idx <= start_idx || end_idx >= int(keyframePoses.size()))
        return 0.0;

    double distance = 0.0;
    for (int idx = start_idx + 1; idx <= end_idx; ++idx) {
        const Pose6D& previous = keyframePoses.at(idx - 1);
        const Pose6D& current = keyframePoses.at(idx);
        const double dx = current.x - previous.x;
        const double dy = current.y - previous.y;
        const double dz = current.z - previous.z;
        distance += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return distance;
}


bool keyframeYawRateOk(const int keyframe_idx)
{
    if (loopMaxKeyframeYawRate <= 0.0 || keyframe_idx < 0 ||
        keyframe_idx >= int(keyframeYawRates.size()))
        return true;
    return keyframeYawRates.at(keyframe_idx) <= loopMaxKeyframeYawRate;
}


std::optional<gtsam::Pose3> doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // parse pointclouds
    int historyKeyframeSearchNum = loopHistoryKeyframeSearchNum;
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx); // use same root of loop kf idx 
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx); 

    // loop verification debug topics
    if (pubLoopScan) {
        sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
        pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
        cureKeyframeCloudMsg.header.frame_id = map_frame_id;
        pubLoopScanLocal->publish(cureKeyframeCloudMsg);
    }

    if (pubLoopSubmap) {
        sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
        pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
        targetKeyframeCloudMsg.header.frame_id = map_frame_id;
        pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);
    }

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(loopIcpMaxCorrespondenceDistance);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);
 
    float loopFitnessScoreThreshold = loopIcpFitnessThreshold;
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    } else {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    return poseFrom.between(poseTo);
} // doICPVirtualRelative

void process_pg()
{
    while (rclcpp::ok() && pgoRunning.load())
    {
		while ( !odometryBuf.empty() && !fullResBuf.empty() )
        {
            //
            // pop and check keyframe is or not  
            // 
			mBuf.lock();       
            while (!odometryBuf.empty() && stampToSec(odometryBuf.front()->header.stamp) < stampToSec(fullResBuf.front()->header.stamp))
                odometryBuf.pop();
            if (odometryBuf.empty())
            {
                mBuf.unlock();
                break;
            }

            // Time equal check
            timeLaserOdometry = stampToSec(odometryBuf.front()->header.stamp);
            timeLaser = stampToSec(fullResBuf.front()->header.stamp);
            // TODO

            laserCloudFullRes->clear();
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();

            Pose6D pose_curr = getOdom(odometryBuf.front());
            odometryBuf.pop();

            // find nearest gps 
            double eps = 0.1; // find a gps topioc arrived within eps second 
            while (!gpsBuf.empty()) {
                auto thisGPS = gpsBuf.front();
                auto thisGPSTime = stampToSec(thisGPS->header.stamp);
                if( abs(thisGPSTime - timeLaserOdometry) < eps ) {
                    currGPS = thisGPS;
                    hasGPSforThisKF = true; 
                    break;
                } else {
                    hasGPSforThisKF = false;
                }
                gpsBuf.pop();
            }
            mBuf.unlock(); 

            //
            // Early reject by counting local delta movement (for equi-spereated kf drop)
            // 
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            if (awaitingPostResetBaseline.exchange(false)) {
                odom_pose_prev = pose_curr;
                translationAccumulated = 1000000.0;
                rotaionAccumulated = 1000000.0;
            }
            Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform
            if (keyframeIntervalStartTime <= 0.0)
                keyframeIntervalStartTime = timeLaserOdometry;

            const double odometry_dt = timeLaserOdometryPrev > 0.0 ?
                timeLaserOdometry - timeLaserOdometryPrev : 0.0;
            const double current_yaw_rate = odometry_dt > 1.0e-3 ? dtf.yaw / odometry_dt : 0.0;
            timeLaserOdometryPrev = timeLaserOdometry;
            maxYawRateSinceKeyframe = std::max(maxYawRateSinceKeyframe, current_yaw_rate);
            yawAccumulatedSinceKeyframe += dtf.yaw;

            double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value. 
            translationAccumulated += delta_translation;
            rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  

            if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap ) {
                isNowKeyFrame = true;
                translationAccumulated = 0.0; // reset 
                rotaionAccumulated = 0.0; // reset 
            } else {
                isNowKeyFrame = false;
            }

            if( ! isNowKeyFrame ) 
                continue; 

            if( !gpsOffsetInitialized ) {
                if(hasGPSforThisKF) { // if the very first frame 
                    gpsAltitudeInitOffset = currGPS->altitude;
                    gpsOffsetInitialized = true;
                } 
            }

            //
            // Save data and Add consecutive node 
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext.filter(*thisKeyFrameDS);

            mKF.lock(); 
            keyframeLaserClouds.push_back(thisKeyFrameDS);
            keyframePoses.push_back(pose_curr);
            keyframePosesUpdated.push_back(pose_curr); // init
            keyframeTimes.push_back(timeLaserOdometry);
            const double keyframe_interval_duration = timeLaserOdometry - keyframeIntervalStartTime;
            const double average_yaw_rate = keyframe_interval_duration > 1.0e-3 ?
                yawAccumulatedSinceKeyframe / keyframe_interval_duration : maxYawRateSinceKeyframe;
            keyframeYawRates.push_back(std::max(maxYawRateSinceKeyframe, average_yaw_rate));
            yawAccumulatedSinceKeyframe = 0.0;
            maxYawRateSinceKeyframe = 0.0;
            keyframeIntervalStartTime = timeLaserOdometry;

            scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

            laserCloudMapPGORedraw = true;
            mKF.unlock(); 

            const int prev_node_idx = keyframePoses.size() - 2; 
            const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
            if( ! gtSAMgraphMade /* prior node */) {
                const int init_node_idx = 0; 
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
                // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

                mtxPosegraph.lock();
                {
                    // prior factor 
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                    // runISAM2opt();          
                }   
                mtxPosegraph.unlock();

                gtSAMgraphMade = true; 

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            } else /* consecutive node (and odom factor) after the prior added */ { // == keyframePoses.size() > 1 
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                mtxPosegraph.lock();
                {
                    // odom factor
                    gtsam::Pose3 relPose = poseFrom.between(poseTo);
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relPose, odomNoise));

                    // gps factor 
                    if(hasGPSforThisKF) {
                        double curr_altitude_offseted = currGPS->altitude - gpsAltitudeInitOffset;
                        mtxRecentPose.lock();
                        gtsam::Point3 gpsConstraint(recentOptimizedX, recentOptimizedY, curr_altitude_offseted); // in this example, only adjusting altitude (for x and y, very big noises are set) 
                        mtxRecentPose.unlock();
                        gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
                        RCLCPP_DEBUG(rclcpp::get_logger("laserPGO"), "GPS factor added at node %d", curr_node_idx);
                    }
                    initialEstimate.insert(curr_node_idx, poseTo);                
                    writeEdge({prev_node_idx, curr_node_idx}, relPose, edges_str); // giseop
                    // runISAM2opt();
                }
                mtxPosegraph.unlock();

                if(curr_node_idx % 100 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");

            // save utility 
            std::string curr_node_idx_str = padZeros(curr_node_idx);
            pcl::io::savePCDFileBinary(pgScansDirectory + curr_node_idx_str + ".pcd", *thisKeyFrame); // scan 

            const auto& curr_scd = scManager.getConstRefRecentSCD();
            saveSCD(pgSCDsDirectory + curr_node_idx_str + ".scd", curr_scd);

            pgTimeSaveStream << timeLaser << std::endl; // path 
        }

        // ps. 
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_pg

void performSCLoopClosure(void)
{
    if( int(keyframePoses.size()) < scManager.NUM_EXCLUDE_RECENT) // do not try too early 
        return;

    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) { 
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
        const double loop_path_distance = keyframePathDistance(prev_node_idx, curr_node_idx);
        if (loop_path_distance < loopMinPathDistance) {
            cout << "Loop rejected: path distance " << loop_path_distance << " < "
                 << loopMinPathDistance << " between " << prev_node_idx << " and "
                 << curr_node_idx << endl;
            return;
        }
        if (!keyframeYawRateOk(prev_node_idx) || !keyframeYawRateOk(curr_node_idx)) {
            const double prev_yaw_rate = prev_node_idx < int(keyframeYawRates.size()) ?
                keyframeYawRates.at(prev_node_idx) : 0.0;
            const double curr_yaw_rate = curr_node_idx < int(keyframeYawRates.size()) ?
                keyframeYawRates.at(curr_node_idx) : 0.0;
            cout << "Loop rejected: keyframe yaw rate prev=" << prev_yaw_rate
                 << ", curr=" << curr_yaw_rate << " > " << loopMaxKeyframeYawRate
                 << " between " << prev_node_idx << " and " << curr_node_idx << endl;
            return;
        }
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
} // performSCLoopClosure

void process_lcd(void)
{
    rclcpp::Rate rate(loopClosureFrequency);
    while (rclcpp::ok() && pgoRunning.load())
    {
        rate.sleep();
        performSCLoopClosure();
        // performRSLoopClosure(); // TODO
    }
} // process_lcd

void process_icp(void)
{
    while (rclcpp::ok() && pgoRunning.load())
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 30 ) {
                RCLCPP_WARN(rclcpp::get_logger("laserPGO"), "Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mBuf.lock(); 
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mBuf.unlock(); 

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            if(relative_pose_optional) {
                gtsam::Pose3 relative_pose = relative_pose_optional.value();
                mtxPosegraph.lock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                writeEdge({prev_node_idx, curr_node_idx}, relative_pose, edges_str); // giseop
                // runISAM2opt();
                mtxPosegraph.unlock();
            } 
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp

void process_viz_path(void)
{
    float hz = 10.0; 
    rclcpp::Rate rate(hz);
    while (rclcpp::ok() && pgoRunning.load()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubPath();
        }
    }
}

void process_isam(void)
{
    float hz = 1; 
    rclcpp::Rate rate(hz);
    while (rclcpp::ok() && pgoRunning.load()) {
        rate.sleep();
        if( gtSAMgraphMade ) {
            mtxPosegraph.lock();
            runISAM2opt();
            RCLCPP_DEBUG(rclcpp::get_logger("laserPGO"), "running isam2 optimization ...");
            mtxPosegraph.unlock();

            saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat); // pose
            saveOdometryVerticesKITTIformat(odomKITTIformat); // pose
            saveGTSAMgraphG2oFormat(isamCurrentEstimate);
        }
    }
}

void pubMap(void)
{
    int SKIP_FRAMES = 2; // sparse map visulalization to save computations 
    int counter = 0;

    laserCloudMapPGO->clear();

    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
        if(counter % SKIP_FRAMES == 0) {
            *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
        }
        counter++;
    }
    mKF.unlock(); 

    downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    downSizeFilterMapPGO.filter(*laserCloudMapPGO);

    sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = map_frame_id;
    pubMapAftPGO->publish(laserCloudMapPGOMsg);
}

void process_viz_map(void)
{
    float vizmapFrequency = 0.1; // 0.1 means run onces every 10s
    rclcpp::Rate rate(vizmapFrequency);
    while (rclcpp::ok() && pgoRunning.load()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubMap();
        }
    }
}

void resetPGOState()
{
    std::lock_guard<std::mutex> lock_buf(mBuf);
    std::lock_guard<std::mutex> lock_kf(mKF);
    std::lock_guard<std::mutex> lock_posegraph(mtxPosegraph);

    while (!odometryBuf.empty()) odometryBuf.pop();
    while (!fullResBuf.empty()) fullResBuf.pop();
    while (!gpsBuf.empty()) gpsBuf.pop();
    while (!scLoopICPBuf.empty()) scLoopICPBuf.pop();

    translationAccumulated = 1000000.0;
    rotaionAccumulated = 1000000.0;
    yawAccumulatedSinceKeyframe = 0.0;
    maxYawRateSinceKeyframe = 0.0;
    keyframeIntervalStartTime = 0.0;
    isNowKeyFrame = false;
    odom_pose_prev = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    odom_pose_curr = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    timeLaserOdometry = 0.0;
    timeLaser = 0.0;
    timeLaserOdometryPrev = 0.0;

    laserCloudFullRes->clear();
    laserCloudMapAfterPGO->clear();
    laserCloudMapPGO->clear();
    keyframeLaserClouds.clear();
    keyframePoses.clear();
    keyframePosesUpdated.clear();
    keyframeTimes.clear();
    keyframeYawRates.clear();
    recentIdxUpdated = 0;

    gtSAMgraph.resize(0);
    gtSAMgraphMade = false;
    initialEstimate.clear();
    isamCurrentEstimate.clear();
    if (isam != nullptr) {
        delete isam;
        isam = nullptr;
    }

    laserCloudMapPGORedraw = true;
    currGPS.reset();
    hasGPSforThisKF = false;
    gpsOffsetInitialized = false;
    gpsAltitudeInitOffset = 0.0;
    recentOptimizedX = 0.0;
    recentOptimizedY = 0.0;
    edges_str.clear();
    scManager.clear();
}

void handleResetEvent(const lw_messages::msg::ResetEvent::ConstSharedPtr event)
{
    const double cutoff = rclcpp::Time(event->stamp).seconds();
    resetCutoffStamp.store(cutoff);
    awaitingPostResetBaseline.store(true);

    if (event->mode == lw_messages::msg::ResetEvent::CHECKPOINT_RESTORED) {
        std::lock_guard<std::mutex> lock(mBuf);
        while (!odometryBuf.empty()) odometryBuf.pop();
        while (!fullResBuf.empty()) fullResBuf.pop();
        while (!gpsBuf.empty()) gpsBuf.pop();
        while (!scLoopICPBuf.empty()) scLoopICPBuf.pop();
        translationAccumulated = 1000000.0;
        rotaionAccumulated = 1000000.0;
        yawAccumulatedSinceKeyframe = 0.0;
        maxYawRateSinceKeyframe = 0.0;
        keyframeIntervalStartTime = 0.0;
        timeLaserOdometryPrev = 0.0;
        RCLCPP_WARN(rclcpp::get_logger("laserPGO"),
            "Fast-LIO checkpoint restored at epoch %lu; retained %zu keyframes",
            event->epoch, keyframePoses.size());
        return;
    }

    if (event->mode != lw_messages::msg::ResetEvent::FULL_RESET) return;
    resetPGOState();
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    std::filesystem::remove_all(pgScansDirectory);
    std::filesystem::create_directories(pgScansDirectory);
    std::filesystem::remove_all(pgSCDsDirectory);
    std::filesystem::create_directories(pgSCDsDirectory);

    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = event->stamp;
    empty_path.header.frame_id = map_frame_id;
    pubPathAftPGO->publish(empty_path);
    sensor_msgs::msg::PointCloud2 empty_map;
    empty_map.header = empty_path.header;
    empty_map.height = 1;
    pubMapAftPGO->publish(empty_map);
    RCLCPP_ERROR(rclcpp::get_logger("laserPGO"),
        "Full pose-graph/map reset for Fast-LIO epoch %lu: %s",
        event->epoch, event->reason.c_str());
}

namespace aloam_velodyne
{

LaserPGONode::LaserPGONode(const rclcpp::NodeOptions & options)
: rclcpp::Node("laserPGO", options)
{
    if (pgoRunning.exchange(true)) {
        throw std::runtime_error("LaserPGONode only supports one instance per process");
    }

    resetPGOState();
    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    save_directory = declareAndGet<std::string>(this, "save_directory", "/");
    map_frame_id = declareAndGet<std::string>(this, "map_frame_id", "camera_init");
    body_frame_id = declareAndGet<std::string>(this, "body_frame_id", "aft_pgo");
    enable_tf = declareAndGet<bool>(this, "enable_tf", true);

    pgKITTIformat = save_directory + "optimized_poses.txt";
    odomKITTIformat = save_directory + "odom_poses.txt";

    pgTimeSaveStream = std::fstream(save_directory + "times.txt", std::fstream::out);
    pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);

    pgScansDirectory = save_directory + "Scans/";
    std::filesystem::remove_all(pgScansDirectory);
    std::filesystem::create_directories(pgScansDirectory);

    pgSCDsDirectory = save_directory + "SCDs/";
    std::filesystem::remove_all(pgSCDsDirectory);
    std::filesystem::create_directories(pgSCDsDirectory);

    keyframeMeterGap = declareAndGet<double>(this, "keyframe_meter_gap", 2.0);
    keyframeDegGap = declareAndGet<double>(this, "keyframe_deg_gap", 10.0);
    keyframeRadGap = deg2rad(keyframeDegGap);

    scDistThres = declareAndGet<double>(this, "sc_dist_thres", 0.2);
    scMaximumRadius = declareAndGet<double>(this, "sc_max_radius", 80.0);
    scNumExcludeRecent = declareAndGet<int>(this, "sc_num_exclude_recent", 30);
    loopHistoryKeyframeSearchNum = declareAndGet<int>(this, "loop_history_keyframe_search_num", 25);
    loopIcpMaxCorrespondenceDistance = declareAndGet<double>(this, "loop_icp_max_correspondence_distance", 150.0);
    loopIcpFitnessThreshold = declareAndGet<double>(this, "loop_icp_fitness_threshold", 0.3);
    loopClosureFrequency = declareAndGet<double>(this, "loop_closure_frequency", 1.0);
    loopMinPathDistance = declareAndGet<double>(this, "loop_min_path_distance", 10.0);
    loopMaxKeyframeYawRate = declareAndGet<double>(this, "loop_max_keyframe_yaw_rate", 0.7);
    pubLoopScan = declareAndGet<bool>(this, "pub_loop_scan", true);
    pubLoopSubmap = declareAndGet<bool>(this, "pub_loop_submap", true);

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();

    scManager.setSCdistThres(scDistThres);
    scManager.setMaximumRadius(scMaximumRadius);
    scManager.setNumExcludeRecent(scNumExcludeRecent);

    float filter_size = 0.4;
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    double mapVizFilterSize = declareAndGet<double>(this, "mapviz_filter_size", 0.4);
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

    sub_laser_cloud_full_res_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);
    sub_laser_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);
    sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps/fix", 100, gpsHandler);
    sub_reset_event_ = this->create_subscription<lw_messages::msg::ResetEvent>(
        "~/reset_event", rclcpp::QoS(1).reliable().transient_local(), handleResetEvent);

    pubOdomAftPGO = this->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", 100);
    pubOdomRepubVerifier = this->create_publisher<nav_msgs::msg::Odometry>("/repub_odom", 100);
    pubPathAftPGO = this->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", 100);
    pubMapAftPGO = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", 100);

    pubLoopScanLocal = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", 100);
    pubLoopSubmapLocal = this->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", 100);

    worker_threads_.emplace_back(process_pg);
    worker_threads_.emplace_back(process_lcd);
    worker_threads_.emplace_back(process_icp);
    worker_threads_.emplace_back(process_isam);
    worker_threads_.emplace_back(process_viz_map);
    worker_threads_.emplace_back(process_viz_path);
}

LaserPGONode::~LaserPGONode()
{
    pgoRunning.store(false);
    for (auto & thread : worker_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    if (pgTimeSaveStream.is_open()) {
        pgTimeSaveStream.close();
    }
    if (isam != nullptr) {
        delete isam;
        isam = nullptr;
    }
}

}  // namespace aloam_velodyne

RCLCPP_COMPONENTS_REGISTER_NODE(aloam_velodyne::LaserPGONode)
