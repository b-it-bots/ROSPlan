/**
 * This class is responsible for generating the PDDL instance.
 * This is done by using the objects requested from Knowedge services.
 */
#include "ros/ros.h"
#include "ProblemGenerator.h"

#ifndef KCL_PDDLproblemgenerator
#define KCL_PDDLproblemgenerator

namespace KCL_rosplan {

	class PDDLProblemGenerator : public ProblemGenerator {
	protected:
		virtual void makeHeader(std::ofstream &pFile);
		virtual void makeInitialState(std::ofstream &pFile);
		virtual void makeGoals(std::ofstream &pFile);
		virtual void makeMetric(std::ofstream &pFile);
		virtual void printExpression(std::ofstream &pFile, rosplan_knowledge_msgs::ExprComposite &e);

		virtual void makeProblem(std::ofstream &pFile);
	public:
	    PDDLProblemGenerator(const std::string& kb): ProblemGenerator(kb) {};
	};
} // close namespace

#endif
