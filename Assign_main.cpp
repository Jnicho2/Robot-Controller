#include <Aria.h>
#include <stdio.h>
#include <iostream>
#include <conio.h>
#include <cstring>

using namespace std;

//Variables
int base = 100;
double angleF, angleL;
int distance_f, distance_s;

const double p = 0.6, i = 0 , d = 0.95;
double e = 0, ei = 0, ed = 0, eprev = 0;

//Robot and devises
ArLaser *laser;
ArRobot robot;

double laserRange[18], laserAngle[18];

void getReadings() {

	//Takes 5 readings and averages them
	laser->lockDevice();

	//Get the first reading
	for (int i = 0; i < 18; i++) {
		laserRange[i] = laser->currentReadingPolar(10 * i - 90, 10 * (i + 1) - 90, &laserAngle[i]);
	}

	//Add another 4 readings on
	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 18; i++) {
			laserRange[i] += laser->currentReadingPolar(10 * i - 90, 10 * (i + 1) - 90, &laserAngle[i]);
		}
	}

	//Divide by 5 to get the average
	for (int i = 0; i < 18; i++) {
		laserRange[i] = laserRange[i] / 5;
	}
	laser->unlockDevice();

}

bool obstacleAvoidance() {

	//Check if anything is within 500mm in front of robot and if return true
	double shortestDist = 320000;
	for (int i = 5; i < 13; i++) {
		if (laserRange[i] < shortestDist)
			shortestDist = laserRange[i];	
	}

	if (shortestDist < 500) {
		return true;
	}
	else {
		return false;
	}	
}

void rightEdgeFollowPID() {

	//Basic REF will follow right edge and then turn as needed. 
	double leftVel, rightVel;

	eprev = e;
	e = 500 - laserRange[0];
	ei += e;
	ed = e - eprev;

	double output = p*e + i*ei + d*ed;

	leftVel = base - output;
	rightVel = base + output;

	robot.setVel2(leftVel, rightVel);
}

class FuzzySet {

	double a, b, c, d;
	char* name;

public:

	FuzzySet(double a1, double b1, double c1, double d1, char* n) {
		a = a1;
		b = b1;
		c = c1;
		d = d1;
		name = n;
	}

	FuzzySet(double a1, double b1, double c1, char* n) {
		a = a1;
		b = b1;
		c = b1;
		d = c1;
		name = n;
	}

	double getValue(double in){
		if (in < a || in > d)
			return 0;
		else if (in < b)
			return (in - a) / (b - a);
		else if ((in > b)  && (in < c))
			return 1;
		else if (in < d)
			return (d - in) / (d - c);
	}

	char* getName() {
		return name;
	}

	double getMid(char* n) {
		if (n == name)
			return b;
		return 0;
	}
};

class FuzzyRule {
	
	char* in1;
	char* in2;
	char* out1;
	char* out2;

public:

	FuzzyRule(char* i1, char* i2, char* o1, char* o2) {
		in1 = i1;
		in2 = i2;
		out1 = o1;
		out2 = o2;
	}

	char* getOut1(char* i1, char* i2) {
		if (i1 == in1 && i2 == in2) {
			return out1;
		}
	}

	char* getOut2(char* i1, char* i2) {
		if (i1 == in1 && i2 == in2) {
			return out2;
		}
	}
};

void rightEdgeFollowFuzzy(double front, double back) {

	//The fuzzyset for the distance
	FuzzySet *FuzzySetDist[3];
	FuzzySetDist[0] = new FuzzySet(0, 0, 400, 500, "close");
	FuzzySetDist[1] = new FuzzySet(450, 550, 650, 750, "ok");
	FuzzySetDist[2] = new FuzzySet(600, 700, 1000, 1000, "far");

	//The fuzzyset for the speed
	FuzzySet *FuzzySetSpeed[3];
	FuzzySetSpeed[0] = new FuzzySet(25, 50, 75, "slow");
	FuzzySetSpeed[1] = new FuzzySet(75, 100, 125, "norm");
	FuzzySetSpeed[2] = new FuzzySet(125, 150, 175, "fast");

	//The rules
	FuzzyRule *REWRules[9];
	REWRules[0] = new FuzzyRule("close", "close", "fast", "norm");
	REWRules[1] = new FuzzyRule("close", "ok", "fast", "norm");
	REWRules[2] = new FuzzyRule("close", "far", "fast", "slow");
	REWRules[3] = new FuzzyRule("ok", "close", "norm", "fast");
	REWRules[4] = new FuzzyRule("ok", "ok", "norm", "norm");
	REWRules[5] = new FuzzyRule("ok", "far", "norm", "slow");
	REWRules[6] = new FuzzyRule("far", "close", "slow", "fast");
	REWRules[7] = new FuzzyRule("far", "ok", "norm", "fast");
	REWRules[8] = new FuzzyRule("far", "far", "norm", "norm");
	
	//Fuzzify both inputs, use the same rule set.
	double frontVals[3];
	frontVals[0] = FuzzySetDist[0]->getValue(front);
	frontVals[1] = FuzzySetDist[1]->getValue(front);
	frontVals[2] = FuzzySetDist[2]->getValue(front);

	double backVals[3];
	backVals[0] = FuzzySetDist[0]->getValue(back);
	backVals[1] = FuzzySetDist[1]->getValue(back);
	backVals[2] = FuzzySetDist[2]->getValue(back);

	double lT = 0, lB = 0, rT = 0, rB = 0;

	for (int i = 0; i < 9; i++) {

		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {

				double firingStrn;
				if (frontVals[j] < backVals[k])
					firingStrn = frontVals[j];
				else  firingStrn = backVals[k];

				char* f = FuzzySetDist[j]->getName();
				char* b = FuzzySetDist[k]->getName();

				char* rightSpeed = REWRules[i]->getOut1(f, b);
				char* leftSpeed = REWRules[i]->getOut2(f, b);

				for (int l = 0; l < 3; l++) {
					lT += FuzzySetSpeed[l]->getMid(leftSpeed)*firingStrn;
					rT += FuzzySetSpeed[l]->getMid(rightSpeed)*firingStrn;
				}

				lB += firingStrn;
				rB += firingStrn;
			}
		}
	}

	double leftVel = (lT / lB)*10;
	double rightVel = (rT / rB)*10;

	robot.setVel2(leftVel, rightVel);

}

int main(int argc, char** argv) {
	
	//initialize aria and aria's logging destination and level
	Aria::init();
	ArPose pose;
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())
		cout << "Robot Connected!" << endl;
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();

	//Laser Connection;
	argParser.addDefaultArgument("-connectLaser");
	ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);
	if (laserConnector.connectLasers())
		cout << "Laser Connected!" << endl;
	laser = robot.findLaser(1);
	
	//**ROBOT SETUP & CONNECTION**
	robot.setVel2(base, base);
	printf("Connected\n");
	ArUtil::sleep(500);

	//********************************************** Add your code **********************************************/
	int count = 0;
	double front = 0;
	double back = 0;
	while (true){

		getReadings();
		//rightEdgeFollowPID();


		
		front = laserRange[3];
		back = laserRange[0];

		//Lowers the value read so that it will always fire the rules
		if (front > 1000)
			front = 999;
		if (back > 1000)
			back = 999;

		if (obstacleAvoidance()) {
			robot.setVel2(-base, base);
		}
		else {
			rightEdgeFollowFuzzy(front, back);
		}

		
	}
	ArLog::log(ArLog::Normal, "Ending robot thread...");
	robot.setVel2(0, 0);
	robot.stopRunning();
	robot.waitForRunExit();	// wait for robot task loop to end before exiting the program
	ArLog::log(ArLog::Normal, "Exiting.");	//exit

	return 0;
}