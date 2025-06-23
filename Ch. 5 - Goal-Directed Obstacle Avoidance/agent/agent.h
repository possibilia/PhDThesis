#ifndef AGENT_H
#define AGENT_H

// TODO: review
#include "logger.h"
#include <vector>
#include <cassert>
#include <math.h>
#include <memory>
#include <string>
#include <stack>
#include <set>
#include <algorithm>
#include <random>
#include <list>
#include <cstdio>
#include <cmath>
#include <string>
#include <chrono>
#include <map>
#include <limits>

using namespace std;
using namespace std::chrono;

extern int nEvents;
extern float step;
extern float detectionThreshold;
extern int maxRecursion;

const int run = 10;
const char west[] = "west";
const char east[] = "east";
const char northWest[] = "northWest";
const char southWest[] = "southWest";
const char northEast[] = "northEast";
const char southEast[] = "southEast";

const float leftTurnCoeff = 1;
const float rightTurnCoeff = 1;
const float rightWheelCoeff = 1.034; 

const float L = 0.147;
const float tol = L * 0.1;
const float wheelRadius = 0.033;
const float dSafe = 0.2;
const float lidarMinRange = 0.15;
const float beta = 1;
const float alpha = 0.3;
//const float dMax = dSafe + dSafe * beta + dSafe * alpha;

////////////////////////////////// Observations ///////////////////////////////////

/*
	wrapper
*/
struct Point {
	float x = 0;
	float y = 0;
}; 

class Observation {
public:

	/*
		enum for type of observation
	*/
	enum class ObsType {invalid, obstacle, target};

	/* 
		default constructor 
		(default obs type == invalid)
	*/
	Observation() {}

	/*
		initialises lidar observation with location and 
		observation type (default == obstacle)
	*/
	Observation(float x, float y, ObsType t = ObsType::obstacle) {
		setObservation(x, y, t);
	}

	/*
		sets location and type of obstacle 
		(default == obstacle)
	*/
	void setObservation(float x, float y, ObsType t = ObsType::obstacle) {
		location.x = x;
		location.y = y; 
		label = t;
	}

	/*
		getter for observation location
		@returns location (struct Point)
	*/
	Point getLocation() const {
		assert(isValid());
		return location;
	}

	/*
		getter for radial distance to observation
		@returns distance from origin to observation
	*/
	float getDistance() const {
		assert(isValid());
		float a = pow(location.x, 2); 
		float b = pow(location.y, 2);
		return sqrt(a + b);
	}

	float getDistanceTo(Observation o) const {
		assert(isValid());
		float x = location.x - o.getLocation().x;
		float y = location.y - o.getLocation().y;
		float a = pow(x, 2); 
		float b = pow(y, 2);
		return sqrt(a + b);
	}

	/*
		getter for angle from heading to observation
		@returns angle in rads (pos left, neg right)
	*/
	float getAngle() const {
		assert(isValid());
		return atan2(location.y, location.x);
	}

	/*
		getter for observation type
		@returns observation label 
	*/
	ObsType getLabel() const {
		return label;
	}

	/*
		invalidates observation when avoided
	*/
	void invalidate() {
		//assert(isValid());
		label = ObsType::invalid;
	}

	/*
		checks whether observation is invalid
		@returns true if valid, false otherwise
	*/
	bool isValid() const {
		return ObsType::invalid != label;
	}

private:
	Point location;
	ObsType label = ObsType::invalid;
};

////////////////////////////////// Tasks ///////////////////////////////////////

struct ActionInterface {

	/*
		callback interface for task controls
		interface implemented in main thread
	*/
	virtual void executeMotorAction(float speedLeft, float speedRight) = 0;
};

class AbstractTask {
public:

	/* 
		enum for task success/failure criteria
	*/
	enum ResultCodes { nothing = 0, disturbance_gone = 1, 
	failed = 2, new_disturbance = 3, success = 4, running = 5};

	/*
		task result helper struct
	*/
	struct TaskResult {
		float displacement = 0;
		float angleRotated = 0;

		ResultCodes result = nothing;
		Observation newDisturbance; 

		void setDisturbance(Observation d) {
			newDisturbance = d;
			result = new_disturbance;
		}
	};

	/*
		initialises with previous task variables for chaining
		called by straight tasks, overriden by avoid tasks
	*/
	virtual void init(shared_ptr<AbstractTask> &t, Observation d) {
		// FIXME: review variables
		takeAction = t->takeAction;
		desiredMotorDrive = t->desiredMotorDrive;
		motorDriveLeft = t->desiredMotorDrive;
		motorDriveRight = t->desiredMotorDrive;
		disturbance = d;
	}

	virtual void init(shared_ptr<AbstractTask> &t) {
		// FIXME: review variables
		takeAction = t->takeAction;
		desiredMotorDrive = t->desiredMotorDrive;
		motorDriveLeft = t->desiredMotorDrive;
		motorDriveRight = t->desiredMotorDrive * rightWheelCoeff;
	}

	/*
		pure virtual method for task execution to be 
		implemented by the child class
	*/
	virtual TaskResult taskExecutionStep(float samplingrate, 
		const vector<Observation>& obs) = 0;

	virtual int getLabel() = 0;

	virtual float getActualDisplacement() = 0;

	/*
		registers actuator interface with target task
		(otherwise pointer passed in task init method)
	*/
	void registerInterface(ActionInterface* ta) {
		takeAction = ta;
	}

	/*
		sets the initial/desired motor speed
	*/
	void setInitialSpeed(const float speed) {
		// FIXME: review variables
		desiredMotorDrive = speed;
		motorDriveLeft = desiredMotorDrive;
		motorDriveRight = desiredMotorDrive * rightWheelCoeff;
	}

	void resetTaskDuration() {
		taskDuration = 0;
	}

	void setDisplacement(float d) {
		targetDisplacement = d;
	}

	float getDisplacement() {
		return targetDisplacement;
	}

	/*
		getter for motor linear velocity
		@returns kinematic linear velocity
	*/
	float getMotorLinearVelocity() {
		// average of both motors converted to m/s
		return ((motorDriveLeft+motorDriveRight) / 2.0) * robotDrive2realSpeed;
	}

	/*
		getter for motor angular velocity
		@returns kinematic angular velocity
	*/
	float getMotorAngularVelocity() {
		// FIXME: doesn't seem quite right
		return (motorDriveLeft - motorDriveRight) / L * robotDrive2realSpeed;
	}

protected:

	/*
		event handler for motors
	*/
	void eventNewMotorAction() {
		if (nullptr != takeAction) {
			takeAction->executeMotorAction(motorDriveLeft, motorDriveRight);
		}
	}

	// inhereted for chaining
	ActionInterface* takeAction = nullptr;
	float motorDriveLeft = 0;
	float motorDriveRight = 0;
	float desiredMotorDrive = 0;

	// the disturbance
	Observation disturbance;

	// based on lidar rpm
	float taskDuration = 0.0;

	float targetDisplacement = 0;

	// TODO: motor speed conversion
	float robotDrive2realSpeed = 0.16;
};

struct StraightTask : AbstractTask {

	int label = 0;
	bool firstCall = true;
	float actualDisplacement = 0;
	float disturbanceLookahead = 2; 

	/*
		task execution step called by agent event handler
		handles steering error then disturbance detection
	*/
	virtual TaskResult taskExecutionStep(float samplingrate, 
		const vector<Observation>& obs);

	int getLabel() {
		return label;
	}

	float getActualDisplacement() {
		return actualDisplacement;
	}

};

template <int signedYawScalar>
struct Rotate90Task : AbstractTask {

	bool firstCall = true;
	int label = signedYawScalar;

	const float angleConstant = 1.17; // 1.5
	const float idealSamplingRate = 0.16; 

	float angle = 0;

	virtual void init(shared_ptr<AbstractTask> &t, Observation d) {
		AbstractTask::init(t, d);
		motorDriveLeft = desiredMotorDrive *(float)signedYawScalar;
		motorDriveRight = -desiredMotorDrive *(float)signedYawScalar;
	}

	int getLabel() {
		return label;
	}

	float getActualDisplacement() {
		return -1;
	}

	virtual TaskResult taskExecutionStep(float samplingrate, 
		const vector<Observation>& obs) {
		TaskResult tr;
		tr.result = ResultCodes::running;

		float startAngle = 0;
		if (disturbance.isValid()) {
			startAngle = disturbance.getAngle();
		} else {
			tr.result = AbstractTask::nothing;
			return tr;
		}

		float omega = getMotorAngularVelocity();


		if (signedYawScalar == 1) {
			omega = omega * rightTurnCoeff;
		} else {
			omega = omega * leftTurnCoeff;
		}

		angle += startAngle + omega * (1/samplingrate) * angleConstant;
		logger.printf("angle = %f", angle);

	
		if (abs(angle - startAngle) > M_PI/2) {
			tr.result = ResultCodes::success;
			tr.angleRotated = signedYawScalar * M_PI/2;
			disturbance.invalidate();
			return tr;
		}

		eventNewMotorAction();
		if (samplingrate > 0) {
			taskDuration += (1/samplingrate);
		}
		return tr;
	}
};

struct Rotate90Left : Rotate90Task<-1> {};
struct Rotate90Right : Rotate90Task<1> {};


////////////////////////////// Planner /////////////////////////////////////////////

struct IntentionTransition {
	int src;
	int dest;
};

struct ActionTransition {
	int src;
	int dest;
	int label;
};

class AbstractPlanner {
public:

	virtual vector<shared_ptr<AbstractTask>> eventNewPlan(
		const vector<Observation>& obs, const Observation& disturbance, const Observation& goal, bool reactive) = 0;

	virtual vector<vector<shared_ptr<AbstractTask>>> getIntentionStack() = 0; 
	virtual vector<int> getIntentionStackKeys() = 0; 
};



struct StateMachineLTL : AbstractPlanner {

	int nStates;
	int nIStates;
	const int init = 0;

	vector<int> intentionStackKeys;
	vector<vector<shared_ptr<AbstractTask>>> intentionStack; 

	list<int> *igraph;
	
	vector<IntentionTransition> itrans;
	map<int, int> imap{{5, 0}, {0, 5}, {1, 3}, 
	{3, 1}, {2, 4}, {4, 2}};

	list<int> *graph;
	vector<ActionTransition> trans;

	StateMachineLTL(int _nIStates, int _nStates) {
		// setup intentions model
		igraph = new list<int>[_nIStates]();
		nIStates = _nIStates;

		// set up actions model
		graph = new list<int>[_nStates]();
		nStates = _nStates;
	}

	virtual vector<shared_ptr<AbstractTask>> eventNewPlan(
		const vector<Observation>& obs, const Observation& disturbance, const Observation& goal, bool reactive);

	// consider making const method
	void setIntentionTransition(int src, int dest) {
		igraph[src].push_back(dest);
        IntentionTransition st = {src, dest};
        itrans.push_back(st);
	}

	// consider making const method
	void setActionTransition(int src, int dest, int label) {
		graph[src].push_back(dest);
        ActionTransition st = {src, dest, label};
        trans.push_back(st);
	}

	virtual vector<int> getIntentionStackKeys() {
		vector<int> intentionStackKeys_ = intentionStackKeys;
		intentionStackKeys.erase(intentionStackKeys.begin(), intentionStackKeys.end());
		return intentionStackKeys_;
	}

	virtual vector<vector<shared_ptr<AbstractTask>>> getIntentionStack() {
		vector<vector<shared_ptr<AbstractTask>>> intentionStack_ = intentionStack;
		intentionStack.erase(intentionStack.begin(), intentionStack.end());
		return intentionStack_;
	}

	Observation getNearestLateralDisturbance(const vector<Observation>& obs, float dMax) {
		float miny = abs(dMax);
		Observation nearest(0, dMax);
	
		for (unsigned i = 0; i < obs.size(); i++) {
			if (obs[i].isValid()) {
				// left
				if (abs(obs[i].getLocation().y) < miny) {
					miny = abs(obs[i].getLocation().y);
					nearest = obs[i];
				}
			} 
		}
		return nearest;
	}

	Observation getNearestLongitudinalDisturbance(const vector<Observation>& obs, float dMax) {
		float minx = abs(dMax);
		Observation nearest(0, dMax);
	
		for (unsigned i = 0; i < obs.size(); i++) {
			if (obs[i].isValid()) {
				// left
				if (abs(obs[i].getLocation().x) < minx
					&& obs[i].getLocation().x > lidarMinRange) {
					minx = obs[i].getLocation().x;
					nearest = obs[i];
				}
			} 
		}
		return nearest;
	}

	vector<Observation> getDisturbanceLateralSegment(const vector<Observation>& obs) {
		int index;
		float minx = dSafe + dSafe * beta;

		vector<Observation> obs_;
		vector<Observation> segment;

		//get all valid observationsmake
		for (unsigned i = 0; i < obs.size(); i++) {
			if (obs[i].isValid()) {
				obs_.push_back(obs[i]);
			}
		}

		// get index of nearest longitudinal oi 
		for (unsigned i = 0; i < obs_.size(); i++) {
			Observation ob(obs_[i].getLocation().x, 
				obs_[i].getLocation().y);

			if (abs(ob.getLocation().y) <= L+tol
				&& ob.getLocation().x > 0
				&& (ob.getLocation().x < minx)) {
				minx = ob.getLocation().x;
			    index = i;
			}
		}

		//get segment
		int i = index;
		int j = index;
		
		bool running = true;
		bool finishedLeft = false;
		bool finishedRight = false;

		Observation left;
		Observation right;

		while (running) {
			float d;

			if (!finishedLeft) {
				d = abs(obs_[i-1].getDistance() - obs_[i].getDistance());
				if (d > 0.1) {
					left = obs_[i];
					finishedLeft = true;
				}
				i--;
			} 

			if (!finishedRight) {
				d = abs(obs_[j+1].getDistance() - obs_[j].getDistance());
				if (d > 0.1) {
					right = obs_[j];
					finishedRight = true;
				}
				j++;
			} 

			if (finishedLeft && finishedRight) {
				segment.push_back(left);
				segment.push_back(right);
				running = false;
			}
		}

		return segment;
	}

	Observation getDisturbanceLongitudinalMax(const vector<Observation>& o) {
		float max = -dSafe;
		Observation maxxObs;

		for (unsigned i = 0; i < o.size(); i++) {
			if (o[i].isValid()) {
				if (o[i].getLocation().x > max
					&& o[i].getLocation().x <= dSafe * 2) {
					max = o[i].getLocation().x;
					maxxObs = o[i];
				}
			}
		}
		return maxxObs;
	}

	int selectIntention(int init_, set<int> accept) {
		// performance stats
		vector<int> queueSize;
		vector<int> queueCapacity;
		vector<int> setSize;

		auto start = imap[init_];
		list<int>::iterator it;

		// random stuff
		random_device rd; 
		mt19937 gen(rd()); 
		vector<int> diff;

		vector<int> queue;
		set<int> reachable;
		bool invariant = true;

		queue.push_back(start);
		reachable.insert(start);

		int acceptingState;

		while(!queue.empty() && invariant) {
			int s = queue[0];
			queue.erase(queue.begin());

			// performance stats
			queueSize.push_back(queue.size());
			queueCapacity.push_back(queue.capacity());
			setSize.push_back(reachable.size());

			acceptingState = imap[s];
			invariant = accept.find(imap[s]) == accept.end();

			for (it = igraph[s].begin(); it != igraph[s].end(); ++it) {
	        	if (reachable.find(*it) == reachable.end()) {
	        		diff.push_back(*it);
	        	}
	        }

	        while (!diff.empty()) {
				uniform_int_distribution<> distr(0, diff.size()-1);
				int index = distr(gen);
				queue.push_back(diff[index]);
				reachable.insert(diff[index]);
				diff.erase(diff.begin()+index);
			}
			
			diff.clear();
		}

		saveAlgoData("bfs", queueSize, sizeof(queue), queueCapacity, 
			setSize, sizeof(reachable));

		if (invariant) {
			return 11;
		} else {
			return acceptingState;
		}
	}

	vector<shared_ptr<AbstractTask>> generatePlan(set<int> accept) {
		vector<int> path = generatePath(accept);
		vector<shared_ptr<AbstractTask>> plan;

		// logging stuff
		vector<const char*> trajectory;

		// get tasks for state transitons
		for (unsigned i = 1; i < path.size(); i++) {
			for (auto& st : trans) {
				if (st.src == path[i-1]
					&& st.dest == path[i]) {
					shared_ptr<AbstractTask> task;
			        switch (st.label) {
			            case 0:
			            	trajectory.push_back("straight");
			                task = make_shared<StraightTask>();
			                break;
			            case 1:
			            	trajectory.push_back("right");
			            	task = make_shared<Rotate90Right>();
			                break;
			            case -1:
			            	trajectory.push_back("left");
			            	task = make_shared<Rotate90Left>();
			                break;
			        }
					plan.push_back(task);
				}
			}
		}

		logMe(accept, path, trajectory);
		return plan;
	}

	vector<int> generatePath(set<int> accept) {
		// performance stats
		vector<int> stackSize;
		vector<int> stackCapacity;
		vector<int> setSize;

		// random stuff
		random_device rd; 
		mt19937 gen(rd()); 
		// set difference
		vector<int> diff;

		vector<int> stack;
		set<int> reachable;
		bool invariant = true;

		stack.push_back(init);
		reachable.insert(init);

		while(!stack.empty() && invariant) {
			int s = stack.back();

			// performance stats
			stackSize.push_back(stack.size());
			stackCapacity.push_back(stack.capacity());
			setSize.push_back(reachable.size());

			bool subset = includes(reachable.begin(), reachable.end(),
			graph[s].begin(), graph[s].end());

			if (subset) {
				invariant = accept.find(s) == accept.end();
				if (invariant) {
					stack.pop_back();
				} 
			} else {
				set_difference(graph[s].begin(), graph[s].end(),
				reachable.begin(), reachable.end(),
				inserter(diff, diff.begin()));

				if (!diff.empty()) {
				    uniform_int_distribution<> distr(0, diff.size()-1);  
				    // break symmtery using list order
				    // as is for left, reversed for right
					int s_ = diff[distr(gen)];
					diff.clear();

					stack.push_back(s_);
					reachable.insert(s_);
				} 
			}
		}

		saveAlgoData("dfs", stackSize, sizeof(stack), stackCapacity, 
			setSize, sizeof(reachable));

		if (invariant) {
			// empty path
			stack.clear();
			return stack;
		} else {
			// the path
			return stack;
		}
	}

	void logMe(set<int> accept, vector<int> path, 
		vector<const char*> trajectory) {
		logger.printf("************************************* PLAN **********************************************\n");

		// log accept states
		logger.printf("ACCEPT: ");
		for (auto& a : accept) {
			logger.printf("%d ", a);
		}
		logger.printf("\n");

		// log path
		logger.printf("PATH: ");
		for (auto&  p: path) {
			logger.printf("%d ", p);
		}
		logger.printf("\n");

		// log plan
		logger.printf("PLAN: ");
		for (unsigned i = 0; i < trajectory.size(); i++) {
			if (i < (trajectory.size()-1)) {
				logger.printf("%s -> ", trajectory[i]);
			} else {
				logger.printf("%s \n", trajectory[i]);
			}
		}
	}

	void saveObs(const char* name, const vector<Observation>& obs) {
		char tmp[256];
		sprintf(tmp,"../%d/%s%03d.dat", run, name, nEvents);
		//fprintf(stderr,"%s\n",tmp);
		FILE* f = fopen(tmp,"wt");
		fprintf(f,"x\ty\tr\tphi\n");
		for(unsigned i = 0; i < obs.size();i++) {
			if (obs[i].isValid()) {
				fprintf(f,"%4.4f\t%4.4f\t%4.4f\t%4.4f\n",
				obs[i].getLocation().x,
				obs[i].getLocation().y,
				obs[i].getDistance(),
				obs[i].getAngle());
			}
		}
		fclose(f);
	}

	void saveAlgoData(const char* name, const vector<int> stackSize, const int stackMemory, 
		const vector<int> stackCapacity, const vector<int> setSize, const int setMemory) {
		char tmp[256];
		sprintf(tmp,"../10/%s%03d_%03d.dat", name, nEvents, maxRecursion);
		//fprintf(stderr,"%s\n",tmp);
		FILE* f = fopen(tmp,"wt");
		fprintf(f,"STACK:\nn\tcapacity\n");
		for(unsigned i = 0; i < stackSize.size();i++) {
			fprintf(f,"%d\t%d\n",
			stackSize[i], stackCapacity[i]);
		}
		fprintf(f,"COMPILE MEM: %d bytes\n\n", + stackMemory);
		fprintf(f,"SET:\nn\n");
		for(unsigned i = 0; i < setSize.size();i++) {
			fprintf(f,"%d\n", setSize[i]);
		}
		fprintf(f,"COMPILE MEM: %d bytes", + setMemory);
		fprintf(f,"\n\nADJ. LIST COMPILE MEM: %d bytes", sizeof(graph) + sizeof(list<int>) * nStates);
		fprintf(f,"\nSTATE SIZE: %d bytes", sizeof(int));
		fprintf(f,"\nSTATE SPACE SIZE: %d bytes", sizeof(int) * nStates);
		fclose(f);
	}
};


////////////////////////////// Agent ////////////////////////////////////////////////

class Agent {
public:

	void eventNewRelativeCoordinates(float samplingrate,
		const vector<Observation>& obs);

	void initTasks(shared_ptr<AbstractTask> t) {
		currentTask = t;
	}

	void setGoalBelief(float x, float y) {
		goal = Observation(x, y, Observation::ObsType::target);
		gotogoal = true;
	}

	void setPlanner(shared_ptr<AbstractPlanner> p) {
		planner = p;
	}

private:
	// the goal
	Observation goal;

	// desire to go to goal
	bool gotogoal = false;

	// the task we are in
	shared_ptr<AbstractTask> currentTask;

	// the planner
	shared_ptr<AbstractPlanner> planner;	

	// the plan if available
	vector<shared_ptr<AbstractTask>> plan;

	vector<int> intentionStackKeys;
	vector<vector<shared_ptr<AbstractTask>>> intentionStack;

	// desire is binary, go to goal or stop
	void updateGoalBelief(float theta, float displacement) {
		float x;
		float y;

		if (theta == 0 && displacement > 0) {
			x = goal.getLocation().x - displacement;
			y = goal.getLocation().y;
		} else {
			x = goal.getLocation().x * cos(theta) - goal.getLocation().y * sin(theta);
			y = goal.getLocation().x * sin(theta) + goal.getLocation().y * cos(theta);
		}
		goal = Observation(x, y, Observation::ObsType::target);

		// update desire for goal
		if (abs(goal.getLocation().x) < dSafe
			&& abs(goal.getLocation().y) <= dSafe) {
			gotogoal = false;
		} 
	}

	void saveMap(const vector<Observation>& obs, const Observation& goal) {
		char tmp[256];
		sprintf(tmp,"../%d/map%03d.dat", run, nEvents);
		//fprintf(stderr,"%s\n",tmp);
		FILE* f = fopen(tmp,"wt");
		fprintf(f,"gx\tgy\tx\ty\tr\tphi\n");
		for(unsigned i = 0; i < obs.size();i++) {
			if (obs[i].isValid()) {
				fprintf(f,"%4.4f\t%4.4f\t%4.4f\t%4.4f\t%4.4f\t%4.4f\n",
				goal.getLocation().x,
				goal.getLocation().y,
				obs[i].getLocation().x,
				obs[i].getLocation().y,
				obs[i].getDistance(),
				obs[i].getAngle());
			}
		}
		fclose(f);
	}
};


#endif