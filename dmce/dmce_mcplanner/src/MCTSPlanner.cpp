#include "dmce_mcplanner/MCTSPlanner.hpp"

namespace dmce {

	std::unique_ptr<MCTree> MCTSPlanner::buildTree_(MCState state) {   //创建一棵MCTS树
		initialAction_ = std::make_shared<DisplacementAction<NONE>>(state, params_);  //创建一个默认动作 DisplacementAction<NONE>，初始化时使用当前状态 state 和规划参数 params_，这个动作赋值给成员变量 initialAction_
		MCNodePtr rootNode = std::make_shared<MCTreeNode>(state, params_, initialAction_);  //使用state, params_, initialAction_构造
		return std::make_unique<MCTree>(state, rootNode, params_);
	}

	void MCTSPlanner::resetTree(const MCState& state) {    //定义一个重置树的函数，输入是当前状态state
		tree_ = buildTree_(state);   //调用上面的buildTree函数，重新构建一棵树，将其赋值给成员变量 tree_
	}

	plan_t MCTSPlanner::convertMCPlanToPoses_(const MCPlan& mcPlan) const {  //
		plan_t plan;  //声明一个空路径容器plan
		pos_t rootInitialPos = (*mcPlan.begin())->getAction()->getInitialRobotState().pos;  //获取路径中第一个节点对应动作的初始位置，也就是整条路径开始时机器人所在的位置
		plan.push_back(posToPose(rootInitialPos));  //将起始位置转换成 ROS 的 Pose 类型（或类似格式），并加入路径中
		for (auto node_ptr : mcPlan) {
			plan.push_back(posToPose(node_ptr->getAction()->getFinalRobotState().pos));
		}
		return plan;
	}

	MCState MCTSPlanner::getCurrentState_() {     //定义 MCTSPlanner 类的一个私有成员函数 getCurrentState_，返回一个 MCState 类型的对象
		return {getPosition(), robotDiameter_, getMap(), clusterHandler_};    //构造一个新的 MCState 对象并返回，构造参数为当前机器人的位置信息、机器人的直径、当前的地图信息、聚类处理器
	}

	pos_t MCTSPlanner::getRootPosition_() const {  //定义 MCTSPlanner 的一个私有常量成员函数 getRootPosition_，返回值是 pos_t 类型
		return tree_->getRoot()->getFinalRobotState().pos;   //返回树根节点的最终位置，可以理解为树的起点
	}

	std::pair<bool, plan_t> MCTSPlanner::getLatestPlan_() {  //定义函数 getLatestPlan_，返回值第一个是bool表示是否成功生成路径，第二个是plan_t，通常是一个位姿序列（路径）
		auto failure = std::make_pair(false, plan_t{});  //路径失败时 返回空路径

		if (revertToFallback_) {  //如果为真
			return goToFallbackPlan_();  //// 则调用备用路径规划器 ClusteredFrontierPlanner 返回备用路径
		}

		if (!tree_ || tree_->getNRollouts() < params_.minRollouts)  //如果当前 MCTS 树不存在，或者树中的 rollouts（模拟次数）不足设定的最小值 minRollouts，则返回失败（failure 是一个 false 的空 plan）
			return failure;

		MCState currentState = getCurrentState_();  //获取当前完整状态，包括机器人的当前位置、地图、聚类信息等
		MCPlan mcPlan;  //定义一个名为 mcPlan 的变量，其类型是 MCPlan，空的 MCTS 计划变量 mcPlan，准备后续填入最优路径

		double sqDistFromRoot = (currentState.robot.pos - getRootPosition_()).squaredNorm();  //计算当前机器人位置到 MCTS 树根节点位置之间的平方距离
		bool reset = sqDistFromRoot > params_.navigationCutoff*params_.navigationCutoff;  //判断是否超过设定的距离阈值 navigationCutoff，如果是，则认为偏离过远，需要重置规划树？？？？？？？？？？没有找到赋值
		if (reset) {
			ROS_WARN("[MCTSPlanner::getLatestPlan_] Resetting due to distance from root!");   
			resetTree(currentState);  // 用当前状态重新建立新的树
			return failure;  // 并返回失败（暂不提供路径）
		}

		mcPlan = tree_->getCurrentBestPlan();  //从当前的 MCTS 树中获取当前最优路径 mcPlan
		if (mcPlan.size() < params_.minPlanDepth) {  //如果返回的路径长度小于设定的最小长度，就视为无效
			return failure;
		}

		if (params_.minPlanValue > 0) {   //如果设置了路径价值评估门槛（minPlanValue > 0），就进行路径价值计算（这个逻辑是进一步过滤低质量路径的）
			double initialEntropy = currentState.map.getRelativeEntropy();   //获取当前地图的相对熵（entropy），代表地图中还有多少“未知/不确定区域”！！！！！！！！！！！！
			MCState simState = currentState;  //拿 currentState 拷贝一份为 simState，创建一个模拟状态，防止修改真实的 currentState
			for (MCNodePtr& np : mcPlan) {   //遍历当前最优路径中的所有动作节点
				np->getAction()->simulate(simState, 1-initialEntropy);  //在 simState 中模拟执行动作，同时影响地图中的未知区域变少
			}
			double finalEntropy = simState.map.getRelativeEntropy();  //看模拟执行完后，地图“已知程度”提升了多少
			double planValue = (initialEntropy - finalEntropy);
			if (planValue < params_.minPlanValue) {   //路径如果不能带来明显的地图熵下降（即探索价值低），就会被丢弃或回退到 fallback 方案
				return goToFallbackPlan_();    //跳转83行
			}
		}
//std::vector<dmce::MCRobotState> robotStates;
//for (const auto& node : mcPlan) {
//    robotStates.push_back(node->getAction()->getFinalRobotState());
//}

//if (isPlanConflict(robotStates)) {
//    ROS_WARN("[MCTSPlanner::getLatestPlan_] Detected path conflict, fallback!");
//    return goToFallbackPlan_();
//}

		if (params_.reuseBranches)  //如果参数 reuseBranches 为 true，表示希望复用已有 MCTS 树中的分支结构
			tree_->changeRoot(mcPlan.front());  //于是将 MCTS 树的根节点 rootNode_ 替换为当前路径 mcPlan 中的第一个节点
		else {  //如果不复用树结构，则重建整棵 MCTS 树
			MCState state = getCurrentState_();  //获取当前状态 state
			state.robot = mcPlan.front()->getFinalRobotState();  //将其中的 robot 设置为路径中第一个动作执行后的结果，即当前动作的终点状态
			resetTree(state);  //清空旧树，以这个新状态作为根节点重新构建 MCTS 树
		}
		plan_t posPlan = convertMCPlanToPoses_(mcPlan);  //将 MCTS 中的节点序列 mcPlan 转换为机器人实际导航所需要的路径点序列 posPlan（从“策略层动作”转成“轨迹层位置”）
//        sharedMap.updateRobotPlan(params_.robotID, robotStates);
		return std::make_pair(true, posPlan);  //返回一个结果，表示规划成功，true 说明成功，posPlan 是路径结果
	}

	std::pair<bool, plan_t> MCTSPlanner::goToFallbackPlan_() {     //返回一个 std::pair<bool, plan_t> 类型的结果，用于表示是否成功获取备用路径（fallback plan）以及对应的路径点序列 plan_t
		ClusteredFrontierPlanner frontierPlanner(robotDiameter_);   //构造函数的参数是机器人的直径
		frontierPlanner.setPosition(getPosition());  //设置当前机器人位置为frontierPlanner的起点
		FrontierClustering fc;  //定义前沿聚类器对象 fc，用于从地图中提取未探索区域（frontier）并进行聚类处理
		fc.fromMapFrontiers(getMap());  //从当前地图中提取“前沿区域”，并在内部进行聚类
		frontierPlanner.frontierClusterCallback(fc.getMessage());  //////////////////将聚类后的前沿区域发送给 frontierPlanner，供其进行路径生成
		frontierPlanner.setMap(getMap());  //给 frontierPlanner 设置当前地图信息
		frontierPlanner.updatePlan();  //调用前沿规划器生成一条前往未知区域的路径
		auto frontierPlan = frontierPlanner.getLatestPlan();  //获取 frontierPlanner 最新生成的路径计划，frontierPlan 是一个 std::pair<bool, plan_t>，表示是否规划成功以及具体的路径！！！！！！！！！！！！！！
		if (frontierPlan.first) {   //如果规划成功（即 frontierPlan.first == true）
			auto newState = getCurrentState_();    //获取当前的 MCState（机器人状态+地图信息+聚类信息）
			auto newPos = frontierPlan.second.front().pose.position;  //取出备用路径的第一个位置点（目的地位置），提取其坐标
			newState.robot.pos = {newPos.x, newPos.y};  //将新状态中的机器人的位置设置为备用路径的第一个目标点
			resetTree(newState);  //重置 MCTS 树的根节点状态为 newState，即让 MCTS 从新的前沿目标点重新开始搜索
			ROS_WARN("[MCTSPlanner] Fell back on ClusteredFrontierPlanner.");   ///输出
		} else
		 	ROS_ERROR("[MCTSPlanner] ClusteredFrontierPlanner fallback failed!");   //输出
		return frontierPlan;
	}

	void MCTSPlanner::updatePlan_() {
		MCState currentState = getCurrentState_();  //获取当前机器人状态

		if (firstUpdate_) {  //第一次调用 updatePlan_()，也就是第一次规划
			firstUpdate_ = false;  //表示后面不是第一次了
			tree_ = buildTree_(currentState);  //调用 buildTree_() 来创建一棵新的 MCTS 树，并以当前状态为根节点
		} else {  //如果不是第一次
			tree_->updateState(currentState);  //调用 updateState() 来更新 MCTS 树的状态，例如根节点状态、路径有效性等，而不是重建整棵树
		}

		{
			bool iterationSuccess;    //进行第一次迭代
			MCNodePtr node;
			std::tie(iterationSuccess, node) = tree_->iterate();   //调用 tree_->iterate() 做一次完整的 MCTS 迭代
			if (!iterationSuccess) {      //如果当前迭代失败
				++consecutiveIterationFailures_;   //失败次数加1
				if (consecutiveIterationFailures_ >= 25) {  //如果失败次数达到阈值 25
					ROS_WARN("[MCTSPlanner::updatePlan] Resetting tree due to %u consecutive iteration failures.", consecutiveIterationFailures_);
					resetTree(currentState);   //重建树
					consecutiveIterationFailures_ = 0;   //清0
					revertToFallback_ = true;    //启用备用策略
				}
			} else {
				consecutiveIterationFailures_ = 0;  //若成功一次迭代，则清零失败计数
				revertToFallback_ = false;  //回到正常路径搜索流程
			}
		}

		if (outputUpdateRate_) {  //每次 updatePlan_() 调用时：记录当前时间，和上次时间比，得出这次耗时，保存在一个队列里，每次都用这些历史数据算平均更新时间，最后输出“每秒更新频率”（Hz）
			ros::Time curTime = ros::Time::now();
			latestTimings_.push((curTime - timeOfLastUpdate_).toSec());
			timeOfLastUpdate_ = curTime;
			double timingSum = 0;
			latestTimings_.reduce<double>(timingSum,
				[](double& r, const double& e) {
					r += e;
				});
			double avgTime = timingSum / latestTimings_.size();
			ROS_INFO("[MCTSPlanner::updatePlan] Update rate: %.2f Hz", 1/avgTime);
		}
	}
}

//bool MCTSPlanner::isPlanConflict(const std::vector<dmce::MCRobotState>& candidatePlan) {
//    for (const auto& state : candidatePlan) {
//        if (sharedMap.hasRecentPlanNearby(state, params_.robotID)) {
//            return true;  // 发现冲突
//        }
//    }
//    return false;  // 无冲突
//}
