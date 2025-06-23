#include "agent.h"
#include "alphabot.h"
#include "a1lidarrpi.h"
#include <vector>
#include <signal.h>
#include <iostream>
#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>

using namespace std;

const float minDetectRange = 0.15; 
const float maxDetectRange = 2.0; 

bool running = true;

// every 200 ms, 8192 data points 
class A1LidarScanEvent : public A1Lidar::DataInterface {
public:
    Agent& agent;

    A1LidarScanEvent(Agent& _agent) : agent(_agent) {};

public:
    void newScanAvail(float rpm, A1LidarData (&data)[A1Lidar::nDistance]) {
        if ((rpm > 0) && (rpm < 50)) {
            logger.printf("LiDAR underpowered. Exiting...\n");
            exit(1);
        }

        if (rpm >= 50) {
            int nData = 0; 
            vector<Observation> obs;
            for(A1LidarData &data: data) {
                Observation ob; // invalid by default
                if (data.valid && data.r > minDetectRange 
                    && data.r < maxDetectRange) {
                    ob.setObservation(data.x, data.y);
                    nData++;
                } 
                obs.push_back(ob);
            }
            agent.eventNewRelativeCoordinates(rpm/60.0f, obs);   
        }
    }
};

class MotorActionEvent : public ActionInterface {
public:
    MotorActionEvent(AlphaBot& _alphabot) : alphabot(_alphabot) {}

    AlphaBot& alphabot;
    virtual void executeMotorAction(float l, float r) {
        alphabot.setLeftWheelSpeed(l);
        alphabot.setRightWheelSpeed(r);
    }
};

void sig_handler(int signo) {
    if (signo == SIGINT) {
        running = false;
    }
}

int main(int argc, char* argv[]) { 
    signal(SIGINT, sig_handler);
    logger.startLogging("../10/autoctrl.txt", true);

    float Gx = 1;
    float Gy = 0;

    if (argc > 1) {
        Gx = atof(argv[1]);
        Gy = atof(argv[2]);
    }

    Agent agent;

    A1LidarScanEvent data(agent);
    A1Lidar lidar;

    lidar.registerInterface(&data);

    try {
        logger.printf("Starting LiDAR...");
        lidar.start();
        logger.printf("SUCCESS \n");
    } 
    catch (string m) {
        logger.printf("Error: %s \n", m);
        lidar.stop();
        return 0;
    }

    AlphaBot alphabot;
    MotorActionEvent takeAction(alphabot);

    shared_ptr<AbstractTask> initialTask = make_shared<StraightTask>();
    shared_ptr<StateMachineLTL> planner = make_shared<StateMachineLTL>(3, 6);

    // intentions model 
    planner->setIntentionTransition(0, 1);
    planner->setIntentionTransition(0, 2);
    planner->setIntentionTransition(1, 0);
    planner->setIntentionTransition(2, 0);

    // actions model
    planner->setActionTransition(0, 5, 0);
    planner->setActionTransition(0, 1, -1);
    planner->setActionTransition(0, 2, 1);
    planner->setActionTransition(1, 3, 0);
    planner->setActionTransition(2, 4, 0);

    initialTask->registerInterface(&takeAction);
    initialTask->setInitialSpeed(0.8f);

    agent.initTasks(initialTask);  
    agent.setGoalBelief(Gx, Gy);
    agent.setPlanner(planner);

    try {
        logger.printf("Starting motors...");
        alphabot.start();
        logger.printf("SUCCESS \n");
    }
    catch (string m) {
        logger.printf("Error: %s \n", m);
        lidar.stop();
        alphabot.stop();
        return 0;
    }

    logger.printf("Starting resource logging...");
    logger.startResourceLogging("../10/usage.txt");
    logger.printf("SUCCESS");

    while(running) {
        // blocking
        int ch = getchar();
        switch (ch) {
            case 27:
                running = false;
                break;
        }
    }

    lidar.stop();
    alphabot.stop();
    return 0;
}
