#pragma once

#include <mutex>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "grid_map_core/TypeDefs.hpp"

namespace dmce {
	using pos_t = grid_map::Position;
	using pose_t = geometry_msgs::PoseStamped;
	using plan_t = std::vector<pose_t>;

	using mutex_t = std::mutex;
	using lockGuard_t = std::lock_guard<mutex_t>;

	using index_t = grid_map::Index;
	using indexList_t = std::vector<index_t>;

	/**
	 * Convert a 2D position to a 3D pose
	 *  with z=0 and orientation (0,0,0).
	 */
	inline pose_t posToPose(const pos_t& position) {   //定义一个内联函数posToPose，输入为pos_t 类型的二维位置position，返回类型是 pose_t
		pose_t poseStamped;  //创建一个空的PoseStamped 对象，用于保存最终的位姿
		poseStamped.pose.position.x = position.x();
		poseStamped.pose.position.y = position.y();
		poseStamped.pose.position.z = 0;
		tf2::Quaternion orientation;   //创建一个 tf2::Quaternion 对象，用来设置方向
		orientation.setRPY(0, 0, 0);
		poseStamped.pose.orientation = tf2::toMsg(orientation);  //将 tf2 四元数 orientation 转换成 ROS 消息格式
		return poseStamped;
	}

	inline pos_t poseToPos(const pose_t& pose) {
		pos_t position;
		position.x() = pose.pose.position.x;
		position.y() = pose.pose.position.y;
		return position;
	}
}

// Specialize std::less for the intex type
namespace std {
	template<>
	struct less<dmce::index_t>
	{
	   bool operator()(const dmce::index_t& k1, const dmce::index_t& k2) const
	   {
			// Lexicographic comparison k1 < k2
			if (k1.x() != k2.x())
				return k1.x() < k2.x();
			return k1.y() < k2.y();
	   }
	};
}

