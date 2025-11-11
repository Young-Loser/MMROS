#pragma once

#include "MCAction.hpp"

namespace dmce {
	class CachedMCAction : public MCAction {   //声明CachedMCAction类
		bool wasInfeasible_ = false;  //两个类内的成员变量
		bool hasNoExplorationValue_ = false;

	public:
		using MCAction::MCAction;

		bool isCacheable() const override {   //表示该动作可以缓存结果，方便后续在树中复用
			return true;
		}

		virtual hashKey_t getHash() const override = 0;

		virtual bool isFeasible(const MCState& state) override {
			if (params_.useActionCaching && wasInfeasible_)
				return false;

			bool ret = isFeasible_(state);
			wasInfeasible_ = wasInfeasible_ || !ret;
			return ret;
		}

		virtual double simulate(MCState& state, const double& idleValue) override {   //动作模拟
			state.robot = this->getFinalRobotState();  //将当前动作的最终机器人状态赋值到模拟状态中，即假设动作已经执行完毕，机器人达到了这个动作终点的状态
			if (params_.useActionCaching && hasNoExplorationValue_)    //如果启用了动作缓存（useActionCaching == true）并且这个动作在之前就已经被判定为“无探索价值”（hasNoExplorationValue_ == true）
				return 1 - state.map.getRelativeEntropy();  //直接返回当前地图的“相对已知程度”（1 减去相对熵），这样做是为了加速模拟，避免重复执行“无效”动作

			unsigned int initUnknownCells = state.map.getNUnknownCells();  //记录动作执行前，地图中未知单元格的数量
			double value = simulate_(state, idleValue);  //调用真正的核心模拟函数 simulate_()，这个函数由子类具体实现，执行动作的模拟，并返回一个数值作为动作的“探索价值”
			unsigned int finalUnknownCells = state.map.getNUnknownCells();   //动作执行之后，再次统计地图中未知单元格数量
			if (finalUnknownCells == initUnknownCells) {   //如果未知单元格数量无变化
				hasNoExplorationValue_ = true;  //说明此动作没有任何探索价值
			}
			return value;
		}
	};
}
