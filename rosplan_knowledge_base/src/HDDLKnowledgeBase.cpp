
#include "rosplan_knowledge_base/HDDLKnowledgeBase.h"

namespace KCL_rosplan {

    /* get the initial state from the domain and problem files */
    void HDDLKnowledgeBase::addInitialState() {
        VALVisitorProblem problem_visitor(domain_parser.domain, problem_parser.problem);
        model_instances = problem_visitor.returnInstances();
        model_facts = problem_visitor.returnFacts();
        model_functions = problem_visitor.returnFunctions();
        model_goals = problem_visitor.returnGoals();
        model_tasks = problem_visitor.returnTasks();
        model_timed_initial_literals = problem_visitor.returnTimedKnowledge();
        if (problem_parser.problem->metric) {
            model_metric = problem_visitor.returnMetric();
        }
    }
} // close namespace
