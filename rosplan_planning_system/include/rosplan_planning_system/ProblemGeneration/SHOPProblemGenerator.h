/**
 * This class is responsible for generating the PDDL instance.
 * This is done by using the objects requested from Knowedge services.
 */
#include "ros/ros.h"
#include "ProblemGenerator.h"

#ifndef KCL_SHOPProblemGenerator
#define KCL_SHOPProblemGenerator

namespace KCL_rosplan {

	class SHOPProblemGenerator : public ProblemGenerator {
	private:
		void makeHeader(std::ofstream &pFile);
		void makeInitialState(std::ofstream &pFile);
		void makeGoals(std::ofstream &pFile);
		void makeMetric(std::ofstream &pFile);
		void printExpression(std::ofstream &pFile, rosplan_knowledge_msgs::ExprComposite &e);

		void makeProblem(std::ofstream &pFile);
	public:
	    SHOPProblemGenerator(const std::string& kb): ProblemGenerator(kb) {};
	};
} // close namespace

#endif
