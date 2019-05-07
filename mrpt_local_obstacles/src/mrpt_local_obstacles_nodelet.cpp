#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/point_cloud.h>
#include <mrpt_bridge/point_cloud2.h>

#include <map>

#include <mrpt/version.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
using namespace mrpt::maps;
using namespace mrpt::obs;

#include <mrpt/system/string_utils.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>

#include <mrpt/version.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/core/aligned_allocator.h>
using namespace mrpt::system;
using namespace mrpt::config;
using namespace mrpt::img;
#else
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CConfigFile.h>
using namespace mrpt::utils;
#endif

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <laser_geometry/laser_geometry.h>
namespace local {
class MrptLocalObstaclesNodelet : public nodelet::Nodelet {

private:

	ros::NodeHandle m_nh;  //!< The node handle
	ros::NodeHandle m_localn;  //!< "~"

	CTimeLogger m_profiler;

	bool m_show_gui;
	std::string m_frameid_reference;  //!< typ: "odom"
	std::string m_frameid_robot;  //!< typ: "base_link"
	std::string
		m_topic_local_map_pointcloud;  //!< Default: "local_map_pointcloud"
	std::string m_source_topics_2dscan;  //!< Default: "scan,laser1"
	std::string m_source_topics_depthcam;  //!< Default: "depthcam"
	double m_time_window;  //!< In secs (default: 0.2). This can't be smaller
	//! than m_publish_period
	double m_publish_period;  //!< In secs (default: 0.05). This can't be larger
	//! than m_time_window

	ros::Timer m_timer_publish;

	// Sensor data:
	struct TInfoPerTimeStep
	{
		CObservation::Ptr observation;
		CSimplePointsMap::Ptr point_map;
		mrpt::poses::CPose3D robot_pose;
	};
	typedef std::multimap<double, TInfoPerTimeStep> TListObservations;
	TListObservations m_hist_obs;  //!< The history of past observations during
	//! the interest time window.
	boost::mutex m_hist_obs_mtx;

	/** The local maps */
	CSimplePointsMap m_localmap_pts;
	// COccupancyGridMap2D m_localmap_grid;

	mrpt::gui::CDisplayWindow3D::Ptr m_gui_win;

	/** @name ROS pubs/subs
	 *  @{ */
	ros::Publisher m_pub_local_map_pointcloud;
	std::vector<ros::Subscriber>
		m_subs_2dlaser;  //!< Subscriber to 2D laser scans
	std::vector<ros::Subscriber>
		m_subs_depthcam;  //!< Subscriber to depth camera data
	tf::TransformListener m_tf_listener;  //!< Use to retrieve TF data
	/**  @} */

	/**
	 * @brief Subscribe to a variable number of topics.
	 * @param lstTopics String with list of topics separated with ","
	 * @param subs[in,out] List of subscribers will be here at return.
	 * @return The number of topics subscribed to.
	 */
	template <typename CALLBACK_METHOD_TYPE>
	size_t subscribeToMultipleTopics(
		const std::string& lstTopics, std::vector<ros::Subscriber>& subs,
		CALLBACK_METHOD_TYPE cb)
	{
		std::vector<std::string> lstSources;
		mrpt::system::tokenize(lstTopics, " ,\t\n", lstSources);
		subs.resize(lstSources.size());
		for (size_t i = 0; i < lstSources.size(); i++)
			subs[i] = m_nh.subscribe(lstSources[i], 1, cb, this);
		return lstSources.size();
	}
	void downscaleCloud(const sensor_msgs::PointCloud2ConstPtr scan,
	 const sensor_msgs::PointCloud2Ptr scan_voxel) const
	{
		pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
		pcl::PCLPointCloud2ConstPtr cloud_ptr(cloud);
		pcl::PCLPointCloud2* cloud_filt = new pcl::PCLPointCloud2;
		pcl::PCLPointCloud2ConstPtr cloud_ptr_filt(cloud_filt);
		pcl_conversions::toPCL(*scan, *cloud);
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		sor.setInputCloud(cloud_ptr);
		sor.setLeafSize(0.3, 0.3, 0.3);
		sor.filter(*cloud_filt);
		pcl_conversions::fromPCL(*cloud_ptr_filt, *scan_voxel);

	}

	void downscaleLaser(const sensor_msgs::LaserScanConstPtr scan,
	 const sensor_msgs::PointCloud2Ptr scan_voxel) const
	{	
		laser_geometry::LaserProjection proj;
		sensor_msgs::PointCloud2Ptr cloud(new sensor_msgs::PointCloud2);
		proj.projectLaser(*scan,*cloud);
		NODELET_INFO("Header laser cloud: %s", cloud->header.frame_id.c_str());
		downscaleCloud(cloud, scan_voxel);
	}


	/** Callback: On new sensor data (depth camera)
	 */
	void onNewSensor_DepthCam(const sensor_msgs::PointCloud2ConstPtr& scan)
	{
		CTimeLoggerEntry tle(m_profiler, "onNewSensor_DepthCam");
		// Get the relative position of the sensor wrt the robot:
		tf::StampedTransform sensorOnRobot;
		try
		{
			CTimeLoggerEntry tle2(
				m_profiler, "onNewSensor_DepthCam.lookupTransform_sensor");
			// NODELET_INFO("[onNewSensor_DepthCam] %s, %s
			// ",scan->header.frame_id.c_str(), m_frameid_robot.c_str());

			m_tf_listener.lookupTransform(
				m_frameid_robot, scan->header.frame_id, ros::Time(0),
				sensorOnRobot);
		}
		catch (tf::TransformException& ex)
		{
			NODELET_ERROR("%s", ex.what());
			return;
		}

		// Convert data to MRPT format:
		// mrpt::poses::CPose3D sensorOnRobot_mrpt;
		// mrpt_bridge::convert(sensorOnRobot, sensorOnRobot_mrpt);
		// In MRPT, CSimplePointsMap holds sensor data:
		CSimplePointsMap::Ptr obsPointMap =
#if MRPT_VERSION >= 0x199
			mrpt::make_aligned_shared<CSimplePointsMap>();
#else
			CSimplePointsMap::Create();
#endif
		sensor_msgs::PointCloud2Ptr scan_voxel(new sensor_msgs::PointCloud2);
		downscaleCloud(scan, scan_voxel);
		mrpt_bridge::copy(*scan_voxel, *obsPointMap);

		ROS_DEBUG(
			"[onNewSensor_DepthCam] %u points",
			static_cast<unsigned int>(obsPointMap->size()));

		// Get sensor timestamp:
		const double timestamp = scan->header.stamp.toSec();

		// Get camera pose at that time in the reference frame
		mrpt::poses::CPose3D robotPose;
		try
		{
			CTimeLoggerEntry tle3(
				m_profiler, "onNewSensor_DepthCam.lookupTransform_robot");
			tf::StampedTransform tx;

			try
			{
				m_tf_listener.lookupTransform(
					m_frameid_reference, m_frameid_robot, scan->header.stamp,
					tx);
			}
			catch (tf::ExtrapolationException&)
			{
				// if we need a "too much " recent robot pose,be happy with the
				// latest one:
				m_tf_listener.lookupTransform(
					m_frameid_reference, m_frameid_robot, ros::Time(0), tx);
			}
			mrpt_bridge::convert(tx * sensorOnRobot, robotPose);
			ROS_DEBUG(
				"[onNewSensor_DepthCam] robot pose %s",
				robotPose.asString().c_str());
		}
		catch (tf::TransformException& ex)
		{
			NODELET_ERROR("%s", ex.what());
			return;
		}

		// Insert into the observation history:
		TInfoPerTimeStep ipt;
		ipt.point_map = obsPointMap;
		ipt.robot_pose = robotPose;

		m_hist_obs_mtx.lock();
		m_hist_obs.insert(
			m_hist_obs.end(), TListObservations::value_type(timestamp, ipt));
		m_hist_obs_mtx.unlock();

	}  // end onNewSensor_DepthCam

	/** Callback: On new sensor data
	 */
	void onNewSensor_Laser2D(const sensor_msgs::LaserScanConstPtr& scan)
	{
		CTimeLoggerEntry tle(m_profiler, "onNewSensor_Laser2D");

		// Get the relative position of the sensor wrt the robot:
		tf::StampedTransform sensorOnRobot;
		try
		{
			CTimeLoggerEntry tle2(
				m_profiler, "onNewSensor_Laser2D.lookupTransform_sensor");
			m_tf_listener.lookupTransform(
				m_frameid_robot, scan->header.frame_id, ros::Time(0),
				sensorOnRobot);
		}
		catch (tf::TransformException& ex)
		{
			NODELET_ERROR("%s", ex.what());
			return;
		}

		// Convert data to MRPT format:
		mrpt::poses::CPose3D sensorOnRobot_mrpt;
		mrpt_bridge::convert(sensorOnRobot, sensorOnRobot_mrpt);
		// In MRPT, CObservation2DRangeScan holds both: sensor data + relative
		// pose:

		CSimplePointsMap::Ptr obsPointMap =
#if MRPT_VERSION >= 0x199
			mrpt::make_aligned_shared<CSimplePointsMap>();
#else
			CSimplePointsMap::Create();
#endif
		sensor_msgs::PointCloud2Ptr scan_voxel(new sensor_msgs::PointCloud2);
		
		downscaleLaser(scan, scan_voxel);
		mrpt_bridge::copy(*scan_voxel, *obsPointMap);

	//	auto obsScan = CObservation2DRangeScan::Create();
	//	sensor_msgs::LaserScan scan_trimmed;
	//	mrpt_bridge::convert(scan_trimmed, sensorOnRobot_mrpt, *obsScan);

		/*ROS_DEBUG(
			"[onNewSensor_Laser2D] %u rays, sensor pose on robot %s",
			static_cast<unsigned int>(obsScan->scan.size()),
			sensorOnRobot_mrpt.asString().c_str());*/

		// Get sensor timestamp:
		const double timestamp = scan->header.stamp.toSec();

		// Get robot pose at that time in the reference frame, typ: /odom ->
		// /base_link
		mrpt::poses::CPose3D robotPose;
		try
		{
			CTimeLoggerEntry tle3(
				m_profiler, "onNewSensor_Laser2D.lookupTransform_robot");
			tf::StampedTransform tx;

			try
			{
				m_tf_listener.lookupTransform(
					m_frameid_reference, m_frameid_robot, scan->header.stamp,
					tx);
			}
			catch (tf::ExtrapolationException&)
			{
				// if we need a "too much " recent robot pose,be happy with the
				// latest one:
				m_tf_listener.lookupTransform(
					m_frameid_reference, m_frameid_robot, ros::Time(0), tx);
			}
			mrpt_bridge::convert(tx*sensorOnRobot, robotPose);
			ROS_DEBUG(
				"[onNewSensor_Laser2D] robot pose %s",
				robotPose.asString().c_str());
		}
		catch (tf::TransformException& ex)
		{
			NODELET_ERROR("%s", ex.what());
			return;
		}

		// Insert into the observation history:
		TInfoPerTimeStep ipt;
		ipt.point_map = obsPointMap;
		ipt.robot_pose = robotPose;

		m_hist_obs_mtx.lock();
		m_hist_obs.insert(
			m_hist_obs.end(), TListObservations::value_type(timestamp, ipt));
		m_hist_obs_mtx.unlock();

	}  // end onNewSensor_Laser2D

	/** Callback: On recalc local map & publish it */
	void onDoPublish(const ros::TimerEvent&)
	{
		CTimeLoggerEntry tle(m_profiler, "onDoPublish");

		// Purge old observations & latch a local copy:
		TListObservations obs;
		{
			CTimeLoggerEntry tle(m_profiler, "onDoPublish.removingOld");
			m_hist_obs_mtx.lock();

			// Purge old obs:
			if (!m_hist_obs.empty())
			{
				const double last_time = m_hist_obs.rbegin()->first;
				TListObservations::iterator it_first_valid =
					m_hist_obs.lower_bound(last_time - m_time_window);
				const size_t nToRemove =
					std::distance(m_hist_obs.begin(), it_first_valid);
				ROS_DEBUG(
					"[onDoPublish] Removing %u old entries, last_time=%lf",
					static_cast<unsigned int>(nToRemove), last_time);
				m_hist_obs.erase(m_hist_obs.begin(), it_first_valid);
			}
			// Local copy in this thread:
			obs = m_hist_obs;
			m_hist_obs_mtx.unlock();
		}

		ROS_DEBUG(
			"Building local map with %u observations.",
			static_cast<unsigned int>(obs.size()));
		if (obs.empty()) return;

		// Build local map(s):
		// -----------------------------------------------
		m_localmap_pts.clear();
		mrpt::poses::CPose3D curRobotPose;
		{
			CTimeLoggerEntry tle2(m_profiler, "onDoPublish.buildLocalMap");

			// Get the latest robot pose in the reference frame (typ: /odom ->
			// /base_link)
			// so we can build the local map RELATIVE to it:
			try
			{
				tf::StampedTransform tx;
				m_tf_listener.lookupTransform(
					m_frameid_reference, m_frameid_robot, ros::Time(0), tx);
				mrpt_bridge::convert(tx, curRobotPose);
				ROS_DEBUG(
					"[onDoPublish] Building local map relative to latest robot "
					"pose: %s",
					curRobotPose.asString().c_str());
			}
			catch (tf::TransformException& ex)
			{
				NODELET_ERROR("%s", ex.what());
				return;
			}

			// For each observation: compute relative robot pose & insert obs
			// into map:
			for (TListObservations::const_iterator it = obs.begin();
				 it != obs.end(); ++it)
			{
				const TInfoPerTimeStep& ipt = it->second;

				// Relative pose in the past:
				mrpt::poses::CPose3D relPose(mrpt::poses::UNINITIALIZED_POSE);
				relPose.inverseComposeFrom(ipt.robot_pose, curRobotPose);
				ROS_DEBUG(
					"[onDoPublish] Building local map relative to latest robot "
					"pose: %s",
					relPose.asString().c_str());
				// Insert obs:
				if (ipt.observation)
				{
					m_localmap_pts.insertObservationPtr(
						ipt.observation, &relPose);
				}
				else if (ipt.point_map)
				{
					m_localmap_pts.insertAnotherMap(
						ipt.point_map.get(), relPose);
				}
				else
				{
					ROS_DEBUG(
						"[onDoPublish] Observation is empty, could not be "
						"added to local map");
				}

			}  // end for
		}

		// Publish them:
		if (m_pub_local_map_pointcloud.getNumSubscribers() > 0)
		{
			sensor_msgs::PointCloud2Ptr msg_pts =
				sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);
			sensor_msgs::PointCloud2Ptr msg_pts2 =
				sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);
			msg_pts->header.frame_id = m_frameid_robot;
			msg_pts->header.stamp = ros::Time(obs.rbegin()->first);
			mrpt_bridge::copy(m_localmap_pts,msg_pts->header, *msg_pts);
		//	mrpt_bridge::point_cloud::mrpt2ros(
		//		m_localmap_pts, msg_pts->header, *msg_pts);


		

			m_pub_local_map_pointcloud.publish(msg_pts);
		}

		// Show gui:
		if (m_show_gui)
		{
			if (!m_gui_win)
			{
				m_gui_win = mrpt::gui::CDisplayWindow3D::Create(
					"LocalObstaclesNode", 800, 600);
				mrpt::opengl::COpenGLScene::Ptr& scene =
					m_gui_win->get3DSceneAndLock();
				scene->insert(mrpt::opengl::CGridPlaneXY::Create());
				scene->insert(
					mrpt::opengl::stock_objects::CornerXYZSimple(1.0, 4.0));

				auto gl_obs = mrpt::opengl::CSetOfObjects::Create();
				gl_obs->setName("obstacles");
				scene->insert(gl_obs);

				auto gl_pts = mrpt::opengl::CPointCloud::Create();
				gl_pts->setName("points");
				gl_pts->setPointSize(2.0);
				gl_pts->setColor_u8(TColor(0x0000ff));
				scene->insert(gl_pts);

				m_gui_win->unlockAccess3DScene();
			}

			auto& scene = m_gui_win->get3DSceneAndLock();
			auto gl_obs = mrpt::ptr_cast<mrpt::opengl::CSetOfObjects>::from(
				scene->getByName("obstacles"));
			ROS_ASSERT(!!gl_obs);
			gl_obs->clear();

			auto gl_pts = mrpt::ptr_cast<mrpt::opengl::CPointCloud>::from(
				scene->getByName("points"));

			for (const auto& o : obs)
			{
				const TInfoPerTimeStep& ipt = o.second;
				// Relative pose in the past:
				mrpt::poses::CPose3D relPose(mrpt::poses::UNINITIALIZED_POSE);
				relPose.inverseComposeFrom(ipt.robot_pose, curRobotPose);

				mrpt::opengl::CSetOfObjects::Ptr gl_axis =
					mrpt::opengl::stock_objects::CornerXYZSimple(0.9, 2.0);
				gl_axis->setPose(relPose);
				gl_obs->insert(gl_axis);
			}  // end for

			gl_pts->loadFromPointsMap(&m_localmap_pts);

			m_gui_win->unlockAccess3DScene();
			m_gui_win->repaint();
		}

	}  // onDoPublish

public:
  virtual void onInit(){
    m_nh = getNodeHandle();
    m_localn = getPrivateNodeHandle();
    m_show_gui = true;
    m_frameid_reference = "odom";
    m_frameid_robot = "base_link";
    m_topic_local_map_pointcloud = "local_map_pointcloud";
    m_source_topics_2dscan = "scan,laser1";
    m_source_topics_depthcam = "depthcam";
    m_time_window = 0.2;
    m_publish_period = 0.05;

    // Load params:
		m_localn.param("show_gui", m_show_gui, m_show_gui);
		m_localn.param(
			"frameid_reference", m_frameid_reference, m_frameid_reference);
		m_localn.param("frameid_robot", m_frameid_robot, m_frameid_robot);
		m_localn.param(
			"topic_local_map_pointcloud", m_topic_local_map_pointcloud,
			m_topic_local_map_pointcloud);
		m_localn.param(
			"source_topics_2dscan", m_source_topics_2dscan,
			m_source_topics_2dscan);
		m_localn.param("time_window", m_time_window, m_time_window);
		m_localn.param(
			"source_topics_depthcam", m_source_topics_depthcam,
			m_source_topics_depthcam);
		m_localn.param("publish_period", m_publish_period, m_publish_period);

		ROS_ASSERT(m_time_window > m_publish_period);
		ROS_ASSERT(m_publish_period > 0);

		// Init ROS publishers:
		m_pub_local_map_pointcloud = m_nh.advertise<sensor_msgs::PointCloud2>(
			m_topic_local_map_pointcloud, 10);

		// Init ROS subs:
		// Subscribe to one or more laser sources:
		size_t nSubsTotal = 0;
		nSubsTotal += this->subscribeToMultipleTopics(
			m_source_topics_2dscan, m_subs_2dlaser,
			&MrptLocalObstaclesNodelet::onNewSensor_Laser2D);
		nSubsTotal += this->subscribeToMultipleTopics(
			m_source_topics_depthcam, m_subs_depthcam,
			&MrptLocalObstaclesNodelet::onNewSensor_DepthCam);
		NODELET_INFO(
			"Total number of sensor subscriptions: %u\n",
			static_cast<unsigned int>(nSubsTotal));
		ROS_ASSERT_MSG(
			nSubsTotal > 0,
			"*Error* It is mandatory to set at least one source topic for "
			"sensory information!");

		// Local map params:
		m_localmap_pts.insertionOptions.minDistBetweenLaserPoints = 0;
		m_localmap_pts.insertionOptions.also_interpolate = false;

		// Init timers:
		m_timer_publish = m_nh.createTimer(
			ros::Duration(m_publish_period), &MrptLocalObstaclesNodelet::onDoPublish,
			this);
  }

};

}

PLUGINLIB_EXPORT_CLASS(local::MrptLocalObstaclesNodelet, nodelet::Nodelet);
