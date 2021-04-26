#ifndef ROSPLAN_HDDL_Knowledgebase
#define ROSPLAN_HDDL_Knowledgebase

#include "PDDLKnowledgeBase.h"
#include "HDDLDomainParser.h"
#include "HDDLProblemParser.h"

namespace KCL_rosplan {

	class HDDLKnowledgeBase : public PDDLKnowledgeBase {
	public:
		HDDLKnowledgeBase(ros::NodeHandle& n) : PDDLKnowledgeBase(n) {};
		~HDDLKnowledgeBase() = default;

        virtual void addInitialState() override;
	};
}
#endif
