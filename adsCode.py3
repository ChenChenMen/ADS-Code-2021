# Active Drag System Code, Coded by Rocketry at Virginia Tech

# Import files from Python
import numpy as np
import math
import time

# Import ODE Solver
from scipy.integrate import odeint

# Import plotting file - delete it later
import matplotlib.pyplot as plt

# Declaration of Constant Variable and State Vector------------------------------------
g = 9.80665; ## gravity acceleration, m/s^2
L0 = -0.0065; ## Temperature lapse rate, K/m
M = 28.9645; ## molar mass of air, g/mol
R = 8.3144998 * 1000; ## Universal gas constant, J/kmol

# Launch associated variables----------------------------------------------------------
LAUNCH_THRESHOLD = -4; ## in Gs
BURNOUT_THRESHOLD = 0; ## in Gs
APOGEE_THRESHOLD = 1; ## in Gs

# Variables Associated with ODE--------------------------------------------------------
# from 0 to 20 seconds with 50 evenly spaced intervals
step = 50;
timeInterval = np.linspace(0, 20, step);

# Constants
GAMMA = 1.4;
RBAR = R / M;

# Flap CD
flapPercentCD = 0;

# Launch Day Input Variables
P_GROUND = 29.91 * 3386.39;
T_GROUND = 295.15;
RHO_GROUND = P_GROUND / RBAR / T_GROUND;

# Drag Information
CD_BODY = 0.39;
CD_FLAPS = 1.28;

# Intermediate Variables
# The 1000 is needed to get the numerator in kg so that kgm2/s2 will cancel out with J
ONE_PLUS_GM_OVER_RL0 = 1 + (g*M / R / L0);
NEGATIVE_ONE_OVER_2M0 = -1 / 25.207000000000000 / 2;

# Initial servo write
servoWrite = (60,);

# Initial Conditions
X0 = 0; ## initial horizontal position meters
V0 = 248.93; ## burnout velocity, m/s
Z0 = 1191.8; ## burnout altitude (ALG), m
THETA0 = 1.412302977421291; ## burout zenith angle, rad

Y_prev = [X0, Z0, THETA0, V0];
Y_current = [39.3743811851986, 1433.08267455424, 1.40553400310464, 229.313640927776]; ## one timestep of the ODE

''' main '''
def main():
	global servoWrite;

	# Runs the system setup
	setup();

	detectLaunch();

	launchTime = time.time();
	detectBurnout(launchTime);
	burnoutTime = time.time();

	# servo setup and record its seted value
	servoWrite = (108,);

	while (~detectApogee(burnoutTime)):
		servoWrite += (prediction(),);
		#print(servoWrite)

''' setup the system before the main function 
	
	Blank for now'''
def setup():
	pass;

''' This method detects the launch of the rocket through a threshold value
	and reads data while waiting 

	Blank for now'''
def detectLaunch():
	pass;


''' This method detects burnout of the rocket motor either throught an 
	acceleration threshold or through a time threshold

	@param: the rocket launch time

	Blank for now'''
def detectBurnout(launchTime):
	pass;

''' This method determines when the rocket has reached apogee
	
	@param: the rocket burnout time
	@return: if the rocket has reached apogee

	'''
def detectApogee(burnoutTime):
	return False;

''' This method predicts the expected apogee of the rocket and 
	the CD necessary to actuate the ADS to

	@return: What the CD needs to be charged to

	'''
def prediction():
	global servoWrite;

	finalResult = [];
	minDiff = 3048;
	minIndex = 0;
	for i in range(5):
		finalResult.append(odeSolver(i * .25));

		if (abs(finalResult[i] - 3048) < minDiff):
			minIndex = i;
			minDiff = abs(finalResult[i] - 3048);

	startP = (minIndex * .25) - .1;
	specificResult = [];
	minDiff = 3048;
	minIndex = 0;
	for i in range(5):
		currP = startP + (0.05 * i);
		specificResult.append(odeSolver(currP));

		if (abs(specificResult[i] - 3048) < minDiff):
			minIndex = i;
			minDiff = abs(specificResult[i] - 3048);

	# Converts Flap Percentages to Servo actuations
	# Numbers might change based on ADS system updates
	sPred = ((startP + (0.05 * minIndex)) * 50) + 60;
	if (sPred > 60 and sPred < 110):
		servoWrite += (sPred,);

	# Shall be an output to print out information
	pass;

	return sPred;

''' This method solves the coupled equations using scipy.integrate
	
	@param: what flap percent that the rocket has currently
			input scale from 0 to 1
	@return: the final altitude of the state vector after 20 seconds from calcs

	'''
def odeSolver(flapPercent):
	global Y_current;
	global Y_prev;
	global flapPercentCD;

	Y_prev = Y_current;
	maxAlt = 0;

	flapPercentCD = flapPercent * 7.656 * 0.00064516 / 2 * 1.28;

	# Update Y_current
	complementaryFilter();

	Y_prediction = odeint(ODE, Y_current, timeInterval);

	for i in range(step):
		if (Y_prediction[i][1] > maxAlt):
			maxAlt = Y_prediction[i][1];

	return maxAlt;

''' ODE for horizontal displacement x
			altitude z
			zenith angle theta
			velocity v

	@param: current state vector
	@param: time step
	@return: derivative state vector

	'''
def ODE(Y, t):

	dx_dt = Y[3] * math.cos(Y[2]);
	dz_dt = Y[3] * math.sin(Y[2]);
	dtheta_dt = -g * math.cos(Y[2]) / Y[3];

	dv_dt = (NEGATIVE_ONE_OVER_2M0 * (RHO_GROUND * ((1 - L0 * Y[1] / T_GROUND)**ONE_PLUS_GM_OVER_RL0))\
		* (Y[3]**2)) * (((30.8451 * 0.00064516) * (0.44562314583995 + -0.290001570655917 * Y[3]\
			/ math.sqrt(R * GAMMA * (T_GROUND + L0 * Y[1])) + 0.561689194232028 * (Y[3]**2) / R \
			/ GAMMA / (T_GROUND + L0 * Y[1]))) + flapPercentCD) - (g * math.sin(Y[2]));

	return [dx_dt, dz_dt, dtheta_dt, dv_dt];

''' This method takes data from two sensors and applies a complementary filter
	to mauck sure that we get more accurate data

	Update current state vector
	
	Filter update expected
	Blank for now'''
def complementaryFilter():
	global Y_current;
	pass;

if __name__ == "__main__":
	main();