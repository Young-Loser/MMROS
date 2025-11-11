#include "dmce_mcplanner/MCTree.hpp"

namespace dmce {
	std::pair<bool, MCNodePtr> MCTree::selection_(MCState& state, const double& idleValue) {  //选择
		// std::cout << "SELECTION" << std::endl;
		MCNodePtr selectedNode = rootNode_;
		selectedNode->getAction()->simulate(state, idleValue);

		while (!selectedNode->hasPotentialChildren()) {
			if (!selectedNode->hasChildren()) {
				return {false, selectedNode};
			}

			MCNodePtr bestChild = getBestChild_(
				selectedNode,
				[](const MCNodePtr& n) { return n->getUCB(); }
			);

			bool isFeasible = bestChild->getAction()->isFeasible(currentState_);
			if (isFeasible) {
				selectedNode = bestChild;
				selectedNode->getAction()->simulate(state, idleValue);
			} else {
				// ROS_INFO("[MCTree::selection_] Pruning infeasible node: %s", bestChild->getAction()->toString().c_str());
				selectedNode->pruneChild(bestChild);
			}
		}

		return {true, selectedNode};
	}


	std::pair<bool, MCNodePtr> MCTree::expansion_(MCState& state, MCNodePtr leafNode) {
		// std::cout << "EXPANSION" << std::endl;
		auto result = leafNode->expand(state, leafNode);
		return result;
	}

	double MCTree::rollout(MCState state, MCNodePtr newNode, const double& idleValue) {
		// std::cout << "ROLLOUT" << std::endl;
		unsigned int rolloutDepth = params_.rolloutDepth;
		MCActionGenerator actionGenerator;
		double value = newNode->getAction()->simulate(state, idleValue);
		for (unsigned int i = 0; i < rolloutDepth; i++) {
			auto potentialChildren = actionGenerator.generateFeasibleActions(state, params_);
			if (potentialChildren.size() == 0)
				return value;
			size_t idx = utils::randomIndex(potentialChildren.size());
			value = potentialChildren[idx]->simulate(state, idleValue);
		}
		return value;
	}

	void MCTree::backPropagation_(MCNodePtr startingNode, const double& value) {
		// std::cout << "BACKPROPAGATION" << std::endl;
		double curVal = value;
		startingNode->addVisit(value);
		MCNodePtr target = startingNode;
		while (!target->isRootNode()) {
			// curVal *= 0.95;
			target = target->parentNode;
			target->addVisit(curVal);
		}
	}

	std::pair<double, MCNodePtr> MCTree::iterate() {    //迭代
		// std::cout << "iterate" << std::endl;
		MCState state = currentState_;
		double idleValue = 1 - currentState_.map.getRelativeEntropy();
		MCNodePtr selectedNode, newNode;
		bool selectionSuccess, expansionSuccess;
		std::tie(selectionSuccess, selectedNode) = selection_(state, idleValue);   //选择
		if (selectionSuccess) {
			std::tie(expansionSuccess, newNode) = expansion_(state, selectedNode);  //扩展  对选中的叶节点 selectedNode，从其可能动作中选一个未尝试过的动作，创建新子节点 newNode
			if (expansionSuccess) {
				double value = rollout(state, newNode, idleValue);   //模拟，从新扩展出的 newNode 开始，执行一次模拟动作序列（通常是随机的），直到达到一定深度或终止条件，返回一个 value（例如减小的熵，探索价值），即这条路径的价值
				backPropagation_(newNode, value);   //回传，将这个 value 从 newNode 沿着父节点链一路向上传播，用于更新各节点的 discountedValue_ 和 visitCount_ 等
			} else {  //扩展失败时，不会进入 rollout 和 backpropagation
				// ROS_INFO("Expansion hit dead end at x=%.1f y=%.1f", state.robot.pos.x(), state.robot.pos.y());
			}
		} else {  //选择失败时，也不会扩展
			// ROS_INFO("Selection hit dead end at x=%.1f y=%.1f", state.robot.pos.x(), state.robot.pos.y());
		}

		if (!selectionSuccess || !expansionSuccess) {  //如果任一步失败（选择或扩展）
			backPropagation_(selectedNode, -1);  //仍然对 selectedNode 执行一次负值回传 -1，以惩罚这条无效路径
			selectedNode->resetPotentialChildren(state);  //并重置它的潜在子节点列表（可能是因为地图变化导致动作不再可行）
			return {false, selectedNode};  //一切顺利：返回新扩展出的 newNode 以及成功状态
		}
		return {true, newNode};
	}

	MCNodePtr MCTree::getBestChild_(     //作用：在给定的父节点的所有子节点中，根据某种评分函数选择评分最高的一个子节点，并返回它
		const MCNodePtr& parent,  //父节点（MCNodePtr，一个智能指针）
		double (*valueFcn)(const MCNodePtr&)  //一个函数指针，接受一个 MCNodePtr 并返回一个 double，用于对每个子节点评分
	) const {
		MCNodePtr bestChild = nullptr; //当前找到的最优子节点，初始设为空
		double bestValue = -std::numeric_limits<double>::infinity();  //当前找到的最大值，初始为负无穷（确保任何值都比它大）

		if (!parent->hasChildren()) {  //如果传入的 parent 没有任何子节点，说明调用逻辑有误，抛出运行时异常
			throw std::runtime_error("[MCTree::getBestChild_] Called on node with no children!");
		}

		auto it = parent->childNodes.begin();  //遍历父节点的所有子节点（通过迭代器访问 childNodes 容器）
		for (; it != parent->childNodes.end(); ++it) {
			MCNodePtr candidate = *it;
			double candidateValue = valueFcn(candidate);  //将当前迭代到的子节点赋值为 candidate，并使用传入的评分函数 valueFcn 计算其评分值 candidateValue
			if (candidateValue > bestValue) {  //如果这个候选子节点的评分比当前记录的最优评分还高，就更新 bestChild 和 bestValue
				bestChild = candidate;
				bestValue = candidateValue;
			}
		}

		return bestChild;
	}

	MCPlan MCTree::getCurrentBestPlan() {  //定义一个成员函数 getCurrentBestPlan()，返回类型是 MCPlan（实质是 std::list<MCNodePtr>），表示当前树中“最优的动作序列”
		MCPlan plan;  //创建一个空的路径变量 plan，用于存储最终选出的节点序列
		MCNodePtr currentNode = rootNode_;  //从树的根节点 rootNode_ 开始向下进行搜索
		while (currentNode->hasChildren()) {  //如果当前节点还有子节点，则继续向下搜索
			currentNode = getBestChild_(  //调用 getBestChild_() 获取当前节点中估值最高的子节点，选择策略是通过 getEstimatedValue() 评估
				currentNode,
				[](const MCNodePtr& n) { return n->getEstimatedValue(); }  
			);

			bool isFeasible = currentNode->getAction()->isFeasible(currentState_);  //判断该节点的动作是否在当前状态下可行
			if (!isFeasible) {
				// ROS_WARN("[MCTree::getCurrentBestPlan] Encountered infeasible node! Pruning: %s", currentNode->getAction()->toString().c_str());
				currentNode->parentNode->pruneChild(currentNode);   //将该不可行的子节点从其父节点的子集中移除！！！！！
				return { };  //返回一个空plan
			}

			plan.push_back(currentNode);    //如果当前节点是可行的，加入到路径plan中
		}

		return plan;  //返回最终回到的路径
	}

	unsigned int MCTree::size() const {
		return 1 + rootNode_->countDescendants();
	}

	MCNodePtr MCTree::getRoot() const {
		return rootNode_;
	}

	void printPlan(const MCPlan& plan) {
		std::stringstream msg;
		for (auto nodePtr : plan) {
			msg << nodePtr->getAction()->toString() << " | ";
		}
		ROS_INFO("Plan: %s", msg.str().c_str());
	}

	void MCTree::changeRoot(MCNodePtr newRoot) {
		rootNode_ = newRoot;
		newRoot->parentNode.reset();
		MCState stateCopy = currentState_;
		stateCopy.robot = rootNode_->getAction()->getFinalRobotState();
		rootNode_->resetPotentialChildren(stateCopy, true);
	}

	unsigned int MCTree::getNRollouts() const {
		return rootNode_->getNVisits();
	}
};

