#include "dmce_mcplanner/MCAction.hpp"

namespace dmce {

	double MCAction::simulate_(MCState& state, const double& idleValue) const {   //内部
		unsigned int rayCount_ = params_.robotLidarRayCount;   //获取传感器配置
		double sensorRange_ = params_.robotSensorRange;    

		double angleIncrement = 2 * M_PI / rayCount_;    //角度间隔计算
		double actualRange = sensorRange_ - state.map.getResolution();   //修正最大距离
		for (unsigned int i = 0; i < rayCount_; i++) {  //对每条射线进行模拟
			double angle = i * angleIncrement;  //计算方向角
			auto rayTarget = state.robot.pos;   //计算目标点
			rayTarget.x() += std::cos(angle) * actualRange;
			rayTarget.y() += std::sin(angle) * actualRange;
			castRay_(state.robot.pos, rayTarget, state.map); //调用 castRay_()：模拟一条激光束的效果，更新地图（state.map）中的未知区域
		}

		double value = 1 - state.map.getRelativeEntropy();  //计算信息价值，RelativeEntropy = 未知单元格数量 / 总单元格数量，1 - entropy这个值越大，说明地图越清晰，探测越有价值
		if (params_.useLocalReward)  //如果启用了 useLocalReward（本地奖励），
			return value - idleValue;
		else
			return value;
	}

	bool MCAction::castRay_(const pos_t& from, const pos_t& to, RobotMap& map) {
		try {
			auto it = map.getLineIterator(from, to);  //使用地图对象 map 的 getLineIterator() 获取一条从 from 到 to 的线段栅格遍历器
			grid_map::Position cellPos;
			bool stop = false;  //用于控制何时终止 ray 的传播（比如遇到障碍物）
			if (it.isPastEnd()) {  //如果射线起点/终点无效，或者路径越界，直接失败
				return false;
			}
			do {
				map.getPosition(*it, cellPos);  //根据迭代器位置，获取地图上该格子的实际坐标
				bool isOccupied = map.isOccupied(*it);  //查询该格子是否为障碍物
				if (!isOccupied)  //如果不是障碍物
					map.setOccupancy(*it, map.freeValue);   //设置为“空闲区域”
				++it;  //迭代器前进一步
				stop = isOccupied || it.isPastEnd();  //如果遇到障碍或越界，则停止
			} while (!stop);
		} catch (std::invalid_argument e) {
			return false;
		}
		return true;
	}
}
