#include "dmce_mcplanner/MCTreeNode.hpp"
#include "dmce_mcplanner/MCActionGenerator.hpp"
//#include "shared_map.hpp"    // 包含 SharedMap 类声明
//extern SharedMap sharedMap;  // 声明：sharedMap 在别处（shared_map.cpp）定义

namespace dmce {
	MCTreeNode::MCTreeNode(   //创建一个节点，并调用 resetPotentialChildren() 为当前状态生成可能的下一步动作
			const MCState& initialState,
			const MCParams& params,
			MCActionPtr action,
			MCNodePtr parentNode
		) : action_(action), parentNode(parentNode), params_(params), explorationFactor(params.explorationFactor)
	{
		// std::cout << "[MCTreeNode] InitialPos x: " << initialState.robot.pos.x() << " y: " << initialState.robot.pos.y() << std::endl;
		resetPotentialChildren(initialState, parentNode == nullptr);
	}

	void MCTreeNode::resetPotentialChildren(const MCState& initialState, bool useFrontiers) {   //
		potentialChildren_.clear();   //清空 potentialChildren_
		MCState finalState = initialState;
		finalState.robot = getFinalRobotState();
		MCActionGenerator generator;   //用 MCActionGenerator 生成新的一组 可行动作
		potentialChildren_ = generator.generateFeasibleActions(
				finalState,
				params_,
				parentNode,
				useFrontiers  //useFrontiers 决定是否使用“frontier-based”探索策略
			);

		std::shuffle(
			potentialChildren_.begin(),
			potentialChildren_.end(),
			utils::RNG::get()
		);
	}
	
	void MCTreeNode::addVisit(double visitValue) {   //用于模拟（simulation）结束后，把模拟的结果反向传播到树节点上
		double gamma = params_.iterationDiscountFactor;   //
		discountedValue_ = discountedValue_*gamma + visitValue;
		discountedVisits_ = discountedVisits_*gamma + 1;
		nVisits_++;
	}

	double MCTreeNode::getEstimatedValue() const {   //MCTreeNode 类的一个成员函数，返回该节点的估计价值（estimated value），类型为 double
		if (discountedVisits_ == 0) return 0;  //如果该节点的折扣访问次数 discountedVisits_为 0，说明这个节点从未被访问过，那么就返回 0，表示当前没有可靠的价值估计
		else {
			double expectation = discountedValue_ / discountedVisits_;  //否则，计算期望值（expectation）：这是该节点所有采样值的加权平均（即累计折扣值除以累计折扣访问次数）
			double duration = action_->getDuration();  //获取该节点对应的动作持续时间
			double timeDiscount = std::min(1., std::pow(params_.timeDiscountFactor, duration));  //计算一个时间折扣系数
			return timeDiscount * expectation;  //返回最终的折扣后估计价值！！！！！
		}
	}

//	double MCTreeNode::getEstimatedValue() const {
    // 如果未访问过，直接返回 0
//   if (discountedVisits_ == 0) 
//        return 0.0;

    // 1. 计算基本的折扣期望值
//  / double expectation  = discountedValue_ / discountedVisits_;
//    double duration     = action_->getDuration();
//    double timeDiscount = std::min(1.0, std::pow(params_.timeDiscountFactor, duration));
//    double value        = timeDiscount * expectation;

    // 2. 多机器人避让：如果开启避让并且节点末端位置靠近其他机器人路径，则扣分
//    if (params_.multiRobotAvoidance) {
        // 获取该节点动作执行完后的机器人状态
//     MCRobotState finalState = this->getFinalRobotState();
        // 判断是否靠近其他机器人已规划路径
//        if (sharedMap.hasRecentPlanNearby(finalState, params_.robotID)) {
//            value -= params_.robotRepulsionWeight;
//        }
//    }
//    return value;
//}

	double MCTreeNode::getDiscountedVisits() const {
		return discountedVisits_;   //返回当前节点被访问的折扣计数值
	}

	double MCTreeNode::getUCB() const {      //UCB  Upper Confidence Bound，选择最优节点时的评分
		if (isRootNode() || discountedVisits_ == 0)  //如果是根节点，或者当前节点还没有被访问（访问次数为 0），直接返回 0
			return 0;
		return getEstimatedValue() + explorationFactor * std::sqrt(std::log(parentNode->getDiscountedVisits()) / discountedVisits_);  //值高的节点更容易被选  访问少的节点更有机会被尝试  
	}

	bool MCTreeNode::isRootNode() const {   //判断节点是否为根节点，根节点没有父节点
		return parentNode == nullptr;
	}

	bool MCTreeNode::isExpandable(MCState& refState) {   //是否还有潜在动作可以扩展
		MCState initialState = refState;
		initialState.robot = getFinalRobotState();

		for (auto it = potentialChildren_.begin(); it != potentialChildren_.end(); ) {
			if (!(*it)->isFeasible(initialState)) {
				it = potentialChildren_.erase(it);
			} else {
				++it;
			}
		}

		return potentialChildren_.size() > 0;
	}

	std::pair<bool, MCNodePtr> MCTreeNode::expand(MCState& refState, MCNodePtr parentPtr) {
		MCState initialState = refState;
		initialState.robot = getFinalRobotState();

		if (!isExpandable(refState)) {
			return {false, nullptr};
		}

		MCNodePtr newNode = std::make_shared<MCTreeNode>(initialState, params_, potentialChildren_.back(), parentPtr);
		childNodes.push_back(newNode);
		potentialChildren_.pop_back();
		return {true, newNode};
	}

	MCRobotState MCTreeNode::getFinalRobotState() const {   //获取当前节点对应动作执行完后的机器人状态
		return action_->getFinalRobotState();
	}

	MCRobotState MCTreeNode::getInitialRobotState() const {   //获取节点动作开始前的状态
		return action_->getInitialRobotState();
	}

	unsigned int MCTreeNode::countDescendants() const {  //获取当前节点下的所有子孙节点数量
		unsigned int descendantCount = 0;
		auto it = childNodes.begin();
		for ( ; it != childNodes.end(); ++it) {
			descendantCount += 1 + (*it)->countDescendants();   //递归计算当前节点下所有的子孙节点数量
		}
		return descendantCount;
	}

	MCActionPtr MCTreeNode::getAction() const {  //获取当前节点对应的动作指针
		return action_;
	}

	void MCTreeNode::pruneChild(MCNodePtr& child) {  //从当前节点的子节点列表中删除指定的子节点 child
		auto it = childNodes.begin();  //获取 childNodes 容器的迭代器起点
		for ( ; it != childNodes.end(); ) {  //遍历所有子节点
			if (*it == child) {  //如果找到了要删除的那个子节点：使用 erase() 将它从容器中删除
				it = childNodes.erase(it);
			} else {
				++it;
			}
		}
		child.reset();  //最后，把传入的 child 智能指针置空，释放其引用计数。如果这个子节点没有其他共享引用，内存也会被释放
	}

	unsigned int MCTreeNode::getNVisits() const {
		return nVisits_;
	}
}

