#include "agent.h"

int nEvents = 0;
float detectionThreshold = 0;

Logger logger;

bool jobdone = false;
int maxRecursion = 0; 

//float step = 0;

void Agent::eventNewRelativeCoordinates(float samplingrate,
	const vector<Observation>& obs) { 

	AbstractTask::TaskResult tr = currentTask->taskExecutionStep(samplingrate, obs); 
	updateGoalBelief(tr.angleRotated, tr.displacement);

	saveMap(obs, goal);

	if (nEvents > 0) {
		logger.printf(" Go to goal? = %d ", gotogoal);
		logger.printf("RESULT %d | ", tr.result);
		logger.printf("G x = %f y = %f \n", goal.getLocation().x, goal.getLocation().y);
	}

	if (gotogoal) {
		if (tr.result == AbstractTask::nothing && plan.empty()) {
			//if (!rulebased) {
			auto start = high_resolution_clock::now();
			planner->eventNewPlan(obs, tr.newDisturbance, goal, false);
			auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
			logger.printf("DELIBERATIVE COMPLETED  %f ms\n", duration * 0.001);
			logger.printf("-----------------------------------------------------------------------------------------\n\n");

			//exit(1);

			intentionStack = planner->getIntentionStack();

			plan = intentionStack[0];
			intentionStack.erase(intentionStack.begin());

			// initiate plan
			if (plan[0]->getLabel() != 0) {
				plan[0]->init(currentTask, Observation(dSafe, 0));
			} else {
				plan[0]->init(currentTask);
			}

			currentTask = plan[0];
			plan.erase(plan.begin());
		}

		if (tr.result == AbstractTask::success && plan.empty()) {
			if (intentionStack.empty()) {
				// last behaviour successful and intention stack empty
				auto start = high_resolution_clock::now();
				planner->eventNewPlan(obs, tr.newDisturbance, goal, false);
				auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
				logger.printf("DELIBERATIVE COMPLETED  %f ms\n", duration * 0.001);
				logger.printf("-----------------------------------------------------------------------------------------\n\n");
				intentionStack = planner->getIntentionStack();

				plan = intentionStack[0];
				intentionStack.erase(intentionStack.begin());
			} else {
				// last behaviour successful and intention stack not empty
				int actual = planner->eventNewPlan(obs, tr.newDisturbance, goal, true)[0]->getLabel();
				int expected = intentionStack[0][0]->getLabel();

				logger.printf("actual = %d  expected = %d \n", actual, expected);

				if (actual != expected) {
					// replan if behaviours don't match
					auto start = high_resolution_clock::now();
					planner->eventNewPlan(obs, tr.newDisturbance, goal, false);
					auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
					logger.printf("DELIBERATIVE COMPLETED  %f ms\n", duration * 0.001);
					logger.printf("-----------------------------------------------------------------------------------------\n\n");
					intentionStack = planner->getIntentionStack();
				} 

				// load next behaviour to plan
				plan = intentionStack[0];
				intentionStack.erase(intentionStack.begin());
			}
		}

		if (tr.result == AbstractTask::success && !plan.empty()) {
			// execute next task in plan
			if (plan[0]->getLabel() != 0) {
				plan[0]->init(currentTask, Observation(dSafe, 0));
			} else {
				plan[0]->init(currentTask);
			}
			
			currentTask = plan[0];
			plan.erase(plan.begin());
		}

		if (tr.result == AbstractTask::new_disturbance) {
			// safe zone crossed so replan and execute
			auto start = high_resolution_clock::now();
			planner->eventNewPlan(obs, tr.newDisturbance, goal, false);
			auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
			logger.printf("DELIBERATIVE COMPLETED  %f ms\n", duration * 0.001);
			logger.printf("-----------------------------------------------------------------------------------------\n\n");
			intentionStack = planner->getIntentionStack();

			plan = intentionStack[0];
			intentionStack.erase(intentionStack.begin());

			// initiate plan
			if (plan[0]->getLabel() != 0) {
				plan[0]->init(currentTask, Observation(dSafe, 0));
			} else {
				plan[0]->init(currentTask);
			}

			currentTask = plan[0];
			plan.erase(plan.begin());
		}
	} else {
		exit(1);
	}

	nEvents++;
}

AbstractTask::TaskResult StraightTask::taskExecutionStep(float samplingrate,
	const vector<Observation>& obs) {

	/* check if idle */
	TaskResult tr;
	if (targetDisplacement == 0 && nEvents == 0) {
		tr.result = AbstractTask::nothing;
		return tr;
	}

	float motorLinearVelocity = getMotorLinearVelocity();
	eventNewMotorAction();

	/* look for disturbance  */
	detectionThreshold = motorLinearVelocity * disturbanceLookahead;

	Observation disturbance;
	if (taskDuration > 0.1) {
		for (unsigned i = 0; i < obs.size(); i++) {
			if ((obs[i].isValid())) {
				Point location = obs[i].getLocation();
				if (location.x <= dSafe 
					&& location.x > lidarMinRange
					&& (abs(location.y) <= L+tol * 0.5)) {
					disturbance = obs[i];
				}
			} 
		}
	}

	/* update progress */
	if (samplingrate > 0) {
		actualDisplacement = motorLinearVelocity * taskDuration;
		taskDuration += (1/samplingrate);
	}

	if (!firstCall) {
		tr.displacement = motorLinearVelocity * (1/samplingrate);
	} else {
		firstCall = false;
	}

	logger.printf("actual dist %f target dist %f ", actualDisplacement, targetDisplacement);

	/* update task result */
	if (actualDisplacement >= targetDisplacement) {
		tr.result = AbstractTask::success;
	} else {
		tr.result = AbstractTask::running;
	}

	if (disturbance.isValid()) {
		tr.setDisturbance(disturbance);
		logger.printf("\n Disturbance !!! x = %f y = %f \n", disturbance.getLocation().x, disturbance.getLocation().y);
	}

	return tr;
}

// vector<shared_ptr<AbstractTask>> StateMachineLTL::eventNewPlan(
// 	const vector<Observation>& obs, const Observation& disturbance, const Observation& goal, bool reactive) {

// 	auto start = high_resolution_clock::now();
// 	logger.printf("\n-----------------------------------------------------------------------------------------\n");

// 	set<int> intention;
// 	vector<shared_ptr<AbstractTask>> plan;

// 	if (disturbance.isValid()) {
// 		logger.printf("D x = %f y = %f \n", 
// 			disturbance.getLocation().x, 
// 			disturbance.getLocation().y);
// 	}

// 	float dMax = dSafe * 2;
// 	logger.printf("L+tol = %f, dSafe = %f alpha = %f beta = %f dMax = %f \n", 
// 		L+tol, dSafe, alpha, beta, dMax);
	
// 	logger.printf("******************************** INTENTIONS *********************************************\n");

// 	int currentGoalBeliefState;
// 	float tuningParam = 0.5;

// 	logger.printf("G x = %f y = %f\n", goal.getLocation().x, goal.getLocation().y);
// 	logger.printf("Intentions model tuning param = %f \n", tuningParam);

// 	set<int> intentions = {3, 4, 5};

// 	if (goal.getLocation().x > dSafe && abs(goal.getLocation().y) <= dSafe * tuningParam) {
// 		currentGoalBeliefState = 5;
// 	} else if (goal.getLocation().y > dSafe * tuningParam 
// 		|| (goal.getLocation().x < -dSafe && goal.getLocation().y > 0)) {
// 		currentGoalBeliefState = 3;
// 		intentions = {3, 5};
// 	} else if (goal.getLocation().y < -dSafe * tuningParam
// 		|| (goal.getLocation().x < -dSafe && goal.getLocation().y < 0)) {
// 		currentGoalBeliefState = 4;
// 		intentions = {4, 5};
// 	} 

// 	logger.printf("Current goal belief state (init) = %d \n", currentGoalBeliefState);
// 	logger.printf("******************************* PERCEPTION **********************************************\n");

	//check resting state
	// vector<Observation> o0;
	// for (unsigned i = 0; i < obs.size(); i++) {
	// 	if ((obs[i].isValid())) {
	// 		// forward simulate
	// 		Observation ob(obs[i].getLocation().x, 
	// 			obs[i].getLocation().y);

	// 		// subset o0
	// 		if ((ob.getLocation().x > 0) 
	// 			&& (ob.getLocation().x <= dSafe + dSafe * alpha) 
	// 			&& (abs(ob.getLocation().y) <= L+tol * 0.5)) {
	// 			o0.push_back(ob);
	// 		}
	// 	} 
	// }

	// logger.printf("Subset o0 = %d \n", o0.size());
	// vector<Observation> segment;

	// if (o0.size() > 0) {
	// 	if (currentGoalBeliefState == 5) {
	// 		Observation d = getNearestLongitudinalDisturbance(o0, dSafe + dSafe * alpha);
	// 		if (d.getLocation().y > 0) {
	// 			logger.printf("D x = %f y = %f\n", d.getLocation().x, d.getLocation().y);
	// 			plan.push_back(make_shared<Rotate90Right>());
	// 			logger.printf("right -> straight \n");
	// 		} else {
	// 			plan.push_back(make_shared<Rotate90Left>());
	// 			logger.printf("left -> straight \n");
	// 		}
	// 	}

	// 	if (currentGoalBeliefState == 3) {
	// 		plan.push_back(make_shared<Rotate90Left>());
	// 		logger.printf("left -> straight \n");
	// 	}

	// 	if (currentGoalBeliefState == 4) {
	// 		plan.push_back(make_shared<Rotate90Right>());
	// 		logger.printf("right -> straight \n");
	// 	}

	// 	plan.push_back(make_shared<StraightTask>());
	// 	plan[1]->setDisplacement(dSafe * beta);
	// } else {
	// 	if (currentGoalBeliefState == 3) {
	// 		if (abs(goal.getLocation().x) < dSafe || abs(goal.getLocation().y) < dSafe) {
	// 			plan.push_back(make_shared<Rotate90Left>());
	// 			plan.push_back(make_shared<StraightTask>());
	// 			plan[1]->setDisplacement(dSafe * beta);
	// 			logger.printf("left -> straight \n");
	// 		}
	// 	} 

	// 	if (currentGoalBeliefState == 4) {
	// 		if (abs(goal.getLocation().x) < dSafe || abs(goal.getLocation().y) < dSafe) {
	// 			plan.push_back(make_shared<Rotate90Right>());
	// 			plan.push_back(make_shared<StraightTask>());
	// 			plan[1]->setDisplacement(dSafe * beta);
	// 			logger.printf("right -> straight \n");
	// 		}
	// 	}

	// 	if (plan.empty()) {
	// 		plan.push_back(make_shared<StraightTask>());
	// 		plan[0]->setDisplacement(dSafe * alpha);
	// 		logger.printf("straight \n");
	// 	}
	// }

	// if (!reactive) {
	// 	intentionStack.push_back(plan);
		
	// 	float x = 0;
	// 	float y = 0;

	// 	if (plan[0]->getLabel() == 0) {
	// 		x = plan[0]->getDisplacement();
	// 	} else {
	// 		y = plan[1]->getDisplacement();
		// }

		// vector<Observation> obs_;
		// for (unsigned i = 0; i < obs.size(); i++) {
		// 	Observation o;
		// 	if (obs[i].isValid()) {
		// 		if (plan.size() == 2) {
		// 			if (plan[0]->getLabel() == -1) {
		// 				o.setObservation(obs[i].getLocation().y - y,
		// 					-obs[i].getLocation().x);
		// 			} else {
		// 				o.setObservation(-obs[i].getLocation().y - y,
		// 					obs[i].getLocation().x);
		// 			}
		// 		} else {
		// 			o.setObservation(obs[i].getLocation().x - x,
		// 		        obs[i].getLocation().y - y);
		// 		}
		// 	} 
		// 	obs_.push_back(o);
		// }

		// Observation goal_;
		// if (plan.size() == 2) {
		// 	if (plan[0]->getLabel() == -1) {
		// 		goal_.setObservation(goal.getLocation().y - y, 
		// 			-goal.getLocation().x);
		// 	} else {
		// 		goal_.setObservation(-goal.getLocation().y - y, 
		// 			goal.getLocation().x);
		// 	}	
		// } else {
		// 	goal_.setObservation(goal.getLocation().x - x,
		// 	    goal.getLocation().y - y);
		// }

		// Observation disturbance_;
		// float minx = detectionThreshold;

		// for (unsigned i = 0; i < obs_.size(); i++) {
		// 	if ((obs_[i].isValid())) {
		// 		Point location = obs_[i].getLocation();
		// 		if ((abs(location.x) < detectionThreshold) 
		// 			&& (abs(location.y) <= L) 
		// 			&& (location.x > dSafe) 
		// 			&& (location.x < minx)) {
		// 			disturbance_ = obs_[i];
		// 			minx = location.x;
		// 		}
		// 	} 
		// }

		// auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
// 		logger.printf("STEP COMPLETED  %f ms\n", duration * 0.001);
// 		logger.printf("-----------------------------------------------------------------------------------------\n\n");

// 		maxRecursion++;

// 		if (maxRecursion < 50) {
// 			if (abs(goal_.getLocation().x) > dSafe || abs(goal_.getLocation().y) > dSafe) {
// 				return eventNewPlan(obs_, disturbance_, goal_, false);
// 			}
// 		} else {
// 			logger.printf("MAXED OUT !!!!!!!!! \n");
// 		}
// 	} 

// 	if (reactive) {
// 		for (unsigned i = 0; i < plan.size(); i++) {
// 			logger.printf("distance %d = %f \n", i, plan[i]->getDisplacement());
// 		}
// 	}

// 	maxRecursion = 0;
// 	return plan;
// }


// MODEL
vector<shared_ptr<AbstractTask>> StateMachineLTL::eventNewPlan(
	const vector<Observation>& obs, const Observation& disturbance, const Observation& goal, bool reactive) {

	auto start = high_resolution_clock::now();
	logger.printf("\n-----------------------------------------------------------------------------------------\n");

	set<int> intention;
	vector<shared_ptr<AbstractTask>> plan;

	if (disturbance.isValid()) {
		logger.printf("D x = %f y = %f \n", 
			disturbance.getLocation().x, disturbance.getLocation().y);
	}

	float dMax = dSafe * 2;
	logger.printf("L+tol = %f, dSafe = %f alpha = %f beta = %f dMax = %f \n", 
		L+tol, dSafe, alpha, beta, dMax);
	
	logger.printf("******************************** INTENTIONS *********************************************\n");

	int currentGoalBeliefState;
	float tuningParam = 0.5;

	logger.printf("G x = %f y = %f\n", goal.getLocation().x, goal.getLocation().y);
	logger.printf("Intentions model tuning param = %f \n", tuningParam);

	set<int> intentions = {3, 4, 5};

	if (goal.getLocation().x > dSafe && abs(goal.getLocation().y) <= dSafe * tuningParam) {
		currentGoalBeliefState = 5;
	} else if (goal.getLocation().y > dSafe * tuningParam 
		|| (goal.getLocation().x < -dSafe && goal.getLocation().y > 0)) {
		currentGoalBeliefState = 3;
		intentions = {3, 5};
	} else if (goal.getLocation().y < -dSafe * tuningParam
		|| (goal.getLocation().x < -dSafe && goal.getLocation().y < 0)) {
		currentGoalBeliefState = 4;
		intentions = {4, 5};
	} 

	logger.printf("Current goal belief state (init) = %d \n", currentGoalBeliefState);
	logger.printf("******************************* PERCEPTION **********************************************\n");

	//check resting state
	vector<Observation> o0;
	for (unsigned i = 0; i < obs.size(); i++) {
		if ((obs[i].isValid())) {
			// forward simulate
			Observation ob(obs[i].getLocation().x, 
				obs[i].getLocation().y);

			// subset o0
			if ((ob.getLocation().x > lidarMinRange) 
				&& (ob.getLocation().x <= dSafe + dSafe * alpha) 
				&& (abs(ob.getLocation().y) <= L+tol * 0.5)) {
				o0.push_back(ob);
			}
		} 
	}

	logger.printf("Subset o0 = %d \n", o0.size());
	vector<Observation> segment;

	if (currentGoalBeliefState == 5) {
		if (o0.size() > 0) {
			intentions.erase(5);
			logger.printf("Removed state 5 \n");
		} else {
			intention = {selectIntention(currentGoalBeliefState, intentions)};
			plan = generatePlan(intention);
		}
	}

	if (currentGoalBeliefState == 3 || currentGoalBeliefState == 4) {
		if (o0.size() > 0) {
			Observation d = getNearestLongitudinalDisturbance(o0, dSafe + dSafe * alpha);
			if (d.getLocation().x <= dSafe) {
				intentions.erase(5);
				logger.printf("Removed state 5 Dx x = %f y = %f \n", 
					d.getLocation().x, d.getLocation().y);
			} 
		} 
	}

	if (plan.empty()) {
		// left and right
		vector<Observation> o1;
	 	vector<Observation> o2;
	 	for (unsigned i = 0; i < obs.size(); i++) {
			if (obs[i].isValid()) {
				// forward simulate
				Observation ob(obs[i].getLocation().x, 
					obs[i].getLocation().y);

				// subset o1
				if ((ob.getLocation().y < dMax)
					&& (ob.getLocation().y > lidarMinRange) 
					&& (abs(ob.getLocation().x) <= dSafe)) {
						o1.push_back(ob);
				}
				// subset o2
				if ((ob.getLocation().y > -dMax)
					&& (ob.getLocation().y < -lidarMinRange) 
					&& (abs(ob.getLocation().x) <= dSafe)) {
					o2.push_back(ob);
				}
			} 
		}

		logger.printf("Subsets o1 = %d o2 = %d \n", o1.size(), o2.size());

		if (o1.size() > 0) {
			if (currentGoalBeliefState == 5) {
				intentions.erase(3);
				logger.printf("Removed state 3 \n");
			}

			if (currentGoalBeliefState == 3 && goal.getLocation().x > 0) {
				Observation yPosNearest = getNearestLateralDisturbance(o1, dMax);

				if (yPosNearest.getLocation().y <= dSafe
					&& abs(yPosNearest.getLocation().x) <= L+tol * 0.5) {
					intentions.erase(3);
					logger.printf("Removed state 3 +Dy x = %f y = %f \n", 
						yPosNearest.getLocation().x, yPosNearest.getLocation().y);
				}
			}
		}

		if (o2.size() > 0) {
			if (currentGoalBeliefState == 5) {
				intentions.erase(4);
				logger.printf("Removed state 4 \n");
			}

			if (currentGoalBeliefState == 4 && goal.getLocation().x > 0) {
				Observation yNegNearest = getNearestLateralDisturbance(o2, dMax);

				if (abs(yNegNearest.getLocation().y) <= dSafe
					&& abs(yNegNearest.getLocation().x) <= L+tol * 0.5) {
					intentions.erase(4);
					logger.printf("Removed state 4 -Dy x = %f y = %f \n", 
						yNegNearest.getLocation().x, yNegNearest.getLocation().y);
				} 
			}
		}
		
		intention = {selectIntention(currentGoalBeliefState, intentions)};
		plan = generatePlan(intention);
	}

	/* add distances to straight tasks */

	if (*intention.begin() == 5) {
		// goal or disturbance distance
		plan[0]->setDisplacement(dSafe * alpha);
	} else {
		plan[1]->setDisplacement(dSafe);
	}

	/* do recursion steps */

	if (!reactive) {
		intentionStack.push_back(plan);
		intentionStackKeys.push_back(*intention.begin());
		
		float x = 0;
		float y = 0;

		if (*intention.begin() == 5) {
			x = plan[0]->getDisplacement();
		} else {
			y = plan[1]->getDisplacement();
		}

		vector<Observation> obs_;
		for (unsigned i = 0; i < obs.size(); i++) {
			Observation o;
			if (obs[i].isValid()) {
				if (plan.size() == 2) {
					if (plan[0]->getLabel() == -1) {
						o.setObservation(obs[i].getLocation().y - y,
							-obs[i].getLocation().x);
					} else {
						o.setObservation(-obs[i].getLocation().y - y,
							obs[i].getLocation().x);
					}
				} else {
					o.setObservation(obs[i].getLocation().x - x,
				        obs[i].getLocation().y - y);
				}
			} 
			obs_.push_back(o);
		}

		Observation goal_;
		if (plan.size() == 2) {
			if (plan[0]->getLabel() == -1) {
				goal_.setObservation(goal.getLocation().y - y, 
					-goal.getLocation().x);
			} else {
				goal_.setObservation(-goal.getLocation().y - y, 
					goal.getLocation().x);
			}	
		} else {
			goal_.setObservation(goal.getLocation().x - x,
			    goal.getLocation().y - y);
		}

		Observation disturbance_;
		float minx = detectionThreshold;

		for (unsigned i = 0; i < obs_.size(); i++) {
			if ((obs_[i].isValid())) {
				Point location = obs_[i].getLocation();
				if ((abs(location.x) < detectionThreshold) 
					&& (abs(location.y) <= L) 
					&& (location.x > dSafe) 
					&& (location.x < minx)) {
					disturbance_ = obs_[i];
					minx = location.x;
				}
			} 
		}

		auto duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
		logger.printf("STEP COMPLETED  %f ms\n", duration * 0.001);
		logger.printf("-----------------------------------------------------------------------------------------\n\n");

		maxRecursion++;

		if (maxRecursion < 5) {
			if (abs(goal_.getLocation().x) > dSafe || abs(goal_.getLocation().y) > dSafe) {
				return eventNewPlan(obs_, disturbance_, goal_, false);
			}
		} else {
			logger.printf("MAXED OUT !!!!!!!!! \n");
		}
	} 

	if (reactive) {
		for (unsigned i = 0; i < plan.size(); i++) {
			logger.printf("distance %d = %f \n", i, plan[i]->getDisplacement());
		}
	}

	maxRecursion = 0;
	return plan;
}