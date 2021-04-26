#include "rosplan_planning_system/ProblemGeneration/HDDLProblemGenerator.h"

namespace KCL_rosplan {
	void HDDLProblemGenerator::makeTasks(std::ofstream &pFile) {
		ros::NodeHandle nh;
		ros::ServiceClient getCurrentTasksClient = nh.serviceClient<rosplan_knowledge_msgs::GetTaskService>(task_service);

		pFile << "(:htn\n :tasks (and" << std::endl;

		// get current tasks
		rosplan_knowledge_msgs::GetTaskService currentTaskSrv;
		if (!getCurrentTasksClient.call(currentTaskSrv)) {
			ROS_ERROR("KCL: (HDDLProblemGenerator) Failed to call service %s", task_service.c_str());
		} else {
			for(size_t i=0;i<currentTaskSrv.response.attributes.size();i++) {
				rosplan_knowledge_msgs::KnowledgeItem attr = currentTaskSrv.response.attributes[i];
				if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::TASK) {
                    pFile << "    (" + attr.attribute_name;
                    for(size_t j=0; j<attr.values.size(); j++) {
                        pFile << " " << attr.values[j].value;
                    }
                    pFile << ")" << std::endl;
				}
			}
		}
		pFile << "))" << std::endl;
	}

	void HDDLProblemGenerator::makeProblem(std::ofstream &pFile) {
		makeHeader(pFile);
		makeInitialState(pFile);
		makeTasks(pFile);
		makeGoals(pFile);
		makeMetric(pFile);

		// add end of problem file
		pFile << ")" << std::endl;
	};
}
