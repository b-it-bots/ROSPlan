#ifndef ROSPLAN_HDDL_PROBLEM_GENERATOR_H
#define ROSPLAN_HDDL_PROBLEM_GENERATOR_H

#include "PDDLProblemGenerator.h"

namespace KCL_rosplan {

	class HDDLProblemGenerator : public PDDLProblemGenerator {
	protected:
        virtual void makeProblem(std::ofstream &pFile);
		void makeTasks(std::ofstream &pFile);
	public:
	    HDDLProblemGenerator(const std::string& kb): PDDLProblemGenerator(kb) {};
	};
} // close namespace

#endif
