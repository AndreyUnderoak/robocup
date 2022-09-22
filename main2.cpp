#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
  
#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state){
     // cast the abstract state type to the type we expect
     const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
  
     // extract the first component of the state and cast it to what we expect
     const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
  
     // extract the second component of the state and cast it to what we expect
     const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
     
     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
     return (const void*)rot != (const void*)pos;
}

int main()
{
    auto space(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    auto si(std::make_shared<ob::SpaceInformation>(space));


    si->setStateValidityChecker(isStateValid);

    ob::ScopedState<> start(space);
    start.random();

    ob::ScopedState<> goal(space);
    goal.random();

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    pdef->setStartAndGoalStates(start, goal);

    auto planner(std::make_shared<og::RRTConnect>(si));

    planner->setProblemDefinition(pdef);

    planner->setup();

    si->printSettings(std::cout);

    pdef->print(std::cout);
  
     // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);


     if (solved)
    {
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        path->print(std::cout);
    }
    else
         std::cout << "No solution found" << std::endl;
}