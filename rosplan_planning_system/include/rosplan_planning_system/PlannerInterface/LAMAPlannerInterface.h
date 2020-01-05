#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

#ifndef KCL_LAMA_planner_interface
#define KCL_LAMA_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

	class LAMAPlannerInterface: public PlannerInterface
	{
	private:

		/* runs external commands */
		std::string runCommand(std::string cmd);

		/* remove all white space from a string */
        std::string strip(std::string str);

	protected:

		bool runPlanner();

	public:

		LAMAPlannerInterface(ros::NodeHandle& nh);
		virtual ~LAMAPlannerInterface();
	};

} // close namespace

#endif
