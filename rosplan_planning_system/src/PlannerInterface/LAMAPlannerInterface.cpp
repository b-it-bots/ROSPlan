#include "rosplan_planning_system/PlannerInterface/LAMAPlannerInterface.h"

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	LAMAPlannerInterface::LAMAPlannerInterface(ros::NodeHandle& nh)
	{
		node_handle = &nh;

		plan_server = new actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>((*node_handle), "start_planning", boost::bind(&PlannerInterface::runPlanningServerAction, this, _1), false);

		// publishing raw planner output
		std::string plannerTopic = "planner_output";
		node_handle->getParam("planner_topic", plannerTopic);
		plan_publisher = node_handle->advertise<std_msgs::String>(plannerTopic, 1, true);

		// start planning action server
		plan_server->start();
	}
	
	LAMAPlannerInterface::~LAMAPlannerInterface()
	{
		delete plan_server;
	}

	/**
	 * Runs external commands
	 */
	std::string LAMAPlannerInterface::runCommand(std::string cmd) {
        /* the command should be as follows */
		/* EXECUTABLE --search-time-limit TIMELIMIT --alias seq-sat-lama-2011 --plan-file DATA_PATH DOMAIN PROBLEM */
        std::string data;
		FILE *stream;
		char buffer[1000];
		stream = popen(cmd.c_str(), "r");
		while ( fgets(buffer, 1000, stream) != NULL )
			data.append(buffer);
		pclose(stream);
		return data;
	}

	/*------------------*/
	/* Plan and process */
	/*------------------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 */
	bool LAMAPlannerInterface::runPlanner() {

		// save problem to file for LAMA
		if(use_problem_topic && problem_instance_received) {
			ROS_INFO("KCL: (%s) (%s) Writing problem to file.", ros::this_node::getName().c_str(), problem_name.c_str());
			std::ofstream dest;
			dest.open((problem_path).c_str());
			dest << problem_instance;
			dest.close();
		}

        /* create a temporary directory in data_path directory */
        std::string cd_cmd = "mkdir -p " + data_path + "plan";
		std::string cd_cmd_output = runCommand(cd_cmd.c_str());

		// prepare the planner command line
		std::string str = planner_command;
		std::size_t dit = str.find("DOMAIN");
		if(dit!=std::string::npos) str.replace(dit,6,domain_path);
		std::size_t pit = str.find("PROBLEM");
		if(pit!=std::string::npos) str.replace(pit,7,problem_path);
        /* change current directory to the temp directory then call planner */
		std::string commandString = "cd " + data_path + "plan/; " + str + " > " + data_path + "plan.pddl";

		// call the planner
		ROS_INFO("KCL: (%s) (%s) Running: %s", ros::this_node::getName().c_str(), problem_name.c_str(),  commandString.c_str());
		std::string plan = runCommand(commandString.c_str());
        std::cout << plan << std::endl;
		ROS_INFO("KCL: (%s) (%s) Planning complete", ros::this_node::getName().c_str(), problem_name.c_str());

        /* get the best plan file (LAMA generates multiple plans) */
        std::string best_plan_cmd = "ls " + data_path + "plan | grep sas_plan | tail -n 1";
		std::string best_plan_cmd_output = runCommand(best_plan_cmd.c_str());
        std::string best_plan_file = strip(best_plan_cmd_output);
        std::cout << "Best plan is:" << best_plan_file << ";" << std::endl;

		// check if planner solved the problem or not
		bool solved = best_plan_file.length() > 0;

        if (solved) {
            std::ifstream planfile;
            planfile.open((data_path + "plan/" + best_plan_file).c_str());
            std::stringstream ss;
            ss << planfile.rdbuf();
            planner_output = ss.str();
            planfile.close();
        }

        /* remove the temporary directory and all files inside it */
        std::string rm_cmd = "rm -r " + data_path + "plan";
		std::string rm_cmd_output = runCommand(rm_cmd.c_str());


		if(!solved) ROS_INFO("KCL: (%s) (%s) Plan was unsolvable.", ros::this_node::getName().c_str(), problem_name.c_str());
		else ROS_INFO("KCL: (%s) (%s) Plan was solved.", ros::this_node::getName().c_str(), problem_name.c_str());

		return solved;
	}

	/**
	 * strips all white characters from a string
     * source: https://stackoverflow.com/a/58090517/10460994
	 */
    std::string LAMAPlannerInterface::strip(std::string str){
        if  (!(str.length() == 0)) {
            auto w = std::string(" ") ;
            auto n = std::string("\n") ;
            auto t = std::string("\t") ;
            auto r = std::string("\r") ;
            auto v = std::string(1, str.front()); 
            while((v == w) or (v==t) or (v==r) or (v==n)) {
                str.erase(str.begin());
                v = std::string(1, str.front());
            }

            v = std::string(1 , str.back()); 

            while((v ==w) or (v==t) or (v==r) or (v==n)) {
                str.erase(str.end() - 1);
                v = std::string(1, str.back());
            }
        }
        return str;
    }


} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		srand (static_cast <unsigned> (time(0)));

		ros::init(argc,argv,"rosplan_planner_interface");
		ros::NodeHandle nh("~");

		KCL_rosplan::LAMAPlannerInterface pi(nh);
		
		// subscribe to problem instance
		std::string problemTopic = "problem_instance";
		nh.getParam("problem_topic", problemTopic);
		ros::Subscriber problem_sub = nh.subscribe(problemTopic, 1, &KCL_rosplan::PlannerInterface::problemCallback, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

		// start the planning services
		ros::ServiceServer service1 = nh.advertiseService("planning_server", &KCL_rosplan::PlannerInterface::runPlanningServerDefault, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));
		ros::ServiceServer service2 = nh.advertiseService("planning_server_params", &KCL_rosplan::PlannerInterface::runPlanningServerParams, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}
