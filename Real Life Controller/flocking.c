

#include <ircom/e_ad_conv.h>
#include <epfl/e_init_port.h>
#include <epfl/e_epuck_ports.h>
#include <epfl/e_uart_char.h>
#include <epfl/e_led.h>
#include <epfl/e_led.h>
#include <epfl/e_motors.h>
#include <epfl/e_agenda.h>
#include <stdio.h>
#include <ircom/ircom.h>
#include <btcom/btcom.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>




#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          5    // Minimum sensibility value
#define MAX_SENS          4000    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
#define LEFT 0
#define RIGHT 1
#define DELTA_T 0.1
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  2	  // Size of flock
#define M_PI 3.1415

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)


#define RULE1_THRESHOLD     0.10  // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.03/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.03   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.03/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (0.0/10)   // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (0.03/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction
#define ROBOT_ID 0
#define NEIGH1_ID 1
#define NEIGH2_ID 2

#define ABS(x) ((x>=0)?(x):-(x))
#define SGN(x) ((x>=0)?(1):(-1))
#define EPS 1e-5


int e_puck_matrix_right[8] = {34,20,5,0,0,-6,-30,-40};
int e_puck_matrix_left[8] = {-40,-30,-6,0,0,5,20,34}; // for obstacle avoidance
float sensorDir[NB_IR_SENSORS] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};


float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3]={0,0,0};     		// X, Z, Theta of the current robot
float prev_my_position[3]={0,0,0};  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
float migr[2] = {0, -25};	        // Migration vector
float dist=0;
char* robot_name;
long int motor_steps[2]={0,0};
long int old_motor_steps[2]={0,0};

float theta_robots[FLOCK_SIZE];
int need_update=1;

int translate(float val){
  int val_trans=0;
  val_trans = (int)(val/MAX_SPEED_WEB)*1000;
  return val_trans;
}
int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void update_self_motion(int msl, int msr) {
  motor_steps[LEFT]=e_get_steps_left();
  motor_steps[RIGHT]=e_get_steps_right();
	float theta = my_position[2];
	float dr = ((float)(motor_steps[RIGHT]-old_motor_steps[RIGHT])*M_PI*2/1000)*WHEEL_RADIUS;
	float dl = ((float)(motor_steps[LEFT]-old_motor_steps[LEFT])*M_PI*2/1000)*WHEEL_RADIUS;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;
  dist+=du;

	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);

	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;

	// Keep orientation within 0, 2pi
	if (my_position[2] >= 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;

  old_motor_steps[RIGHT]=motor_steps[RIGHT];
  old_motor_steps[LEFT]=motor_steps[LEFT];

}
void compute_wheel_speeds(int *msl, int *msr)
{
	float x = speed[ROBOT_ID][0]*cosf(my_position[2]) + speed[ROBOT_ID][1]*sinf(my_position[2]); // x in robot coordinates
	float z = -speed[ROBOT_ID][0]*sinf(my_position[2]) + speed[ROBOT_ID][1]*cosf(my_position[2]); // z in robot coordinates
	float Ku = 0.2;   // Forward control coefficient
	float Kw = 1;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position
	float u = Ku*range*cosf(bearing);
	float w = Kw*bearing;
	*msl = ((u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS));
	*msr = ((u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS));
}
void compute_center_of_mass(float CoF[2]){
  int i;
  float area = 0;

  for(i=0; i < FLOCK_SIZE-1;i++){
      area += ((relative_pos[i][0]*relative_pos[i+1][1]) - (relative_pos[i+1][0]*relative_pos[i][1]));
      CoF[0] += ((relative_pos[i][0]+relative_pos[i+1][0])*((relative_pos[i][0]*relative_pos[i+1][1])-(relative_pos[i+1][0]*relative_pos[i][1])));
      CoF[1] += ((relative_pos[i][1]+relative_pos[i+1][1])*((relative_pos[i][0]*relative_pos[i+1][1])-(relative_pos[i+1][0]*relative_pos[i][1])));
  }
  area *= 0.5;
  CoF[0] *= (1/(6*area));
  CoF[1] *= (1/(6*area));
}
void reynolds_rules() {
	int i, j, k;			// Loop counters
	float rel_avg_loc[2] = {0,0};	// Flock average positions
	float rel_avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};
	float CoF[2]={0,0};

	/* Compute averages over the whole flock */
          for(i=0; i<FLOCK_SIZE; i++) {
              if (i == ROBOT_ID)
                  continue; // don't consider yourself for the average
               for (j=0;j<2;j++) {
                   rel_avg_speed[j] += relative_speed[i][j];
                   rel_avg_loc[j] += relative_pos[i][j];
               }
           }
           for (j=0;j<2;j++) {
               rel_avg_speed[j] /= FLOCK_SIZE-1;
               rel_avg_loc[j] /= FLOCK_SIZE-1;
           }

          /* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
           for (j=0;j<2;j++)
               cohesion[j] = rel_avg_loc[j];
          /* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
          for (k=0;k<FLOCK_SIZE;k++) {
              if (k != ROBOT_ID){ // Loop on flockmates only
              // If neighbor k is too close (Euclidean distance)
                  if(pow(relative_pos[k][0],2)+pow(relative_pos[k][1],2) <
                  RULE2_THRESHOLD){
                       for (j=0;j<2;j++) {
                           dispersion[j] -= 1/relative_pos[k][j]; // Relative
                       }
                   }
               }
          }

          /* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
           for (j=0;j<2;j++)
               consistency[j] = 0;

         //aggregation of all behaviors with relative influence determined by weights
         for (j=0;j<2;j++)
	{
                 speed[ROBOT_ID][j] = cohesion[j] * RULE1_WEIGHT;
                 speed[ROBOT_ID][j] +=  dispersion[j] * RULE2_WEIGHT;
                 speed[ROBOT_ID][j] +=  consistency[j] * RULE3_WEIGHT;
         }
        speed[ROBOT_ID][1] *= -1; //y axis of webots is inverted

        //move the robot according to some migration rule
        if(MIGRATORY_URGE == 0){
          speed[ROBOT_ID][0] += 0.01*cos(my_position[2] + M_PI/2);
          speed[ROBOT_ID][1] += 0.01*sin(my_position[2] + M_PI/2);
        }
        else {
            speed[ROBOT_ID][0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT;
            speed[ROBOT_ID][1] -= (migr[1]-my_position[1]) * MIGRATION_WEIGHT; //y axis of webots is inverted
        }
}

void send_ping(void)
{
  int i;
	for (i = 0; i < 30; i++)
	{
	    ircomSend(ROBOT_ID);
	    while (ircomSendDone() == 0);
	}
}

void process_received_ping_messages(float prev_relative_pos[FLOCK_SIZE][3],float relative_pos[FLOCK_SIZE][3],float relative_speed[FLOCK_SIZE][2])
{
  ircomDisableContinuousListening ();
  int counter=0;
  float direction=0;
  float distance=0;
  int value = 0;
  int other_ROBOT_ID=-3;
  while(1){
  IrcomMessage imsg;
  ircomPopMessage(&imsg);
  if(imsg.error==-1)
    break;
  else{
    if(imsg.error==0){
      direction=imsg.direction;
      distance=imsg.distance;
      if((int) (imsg.value)==NEIGH1_ID){
        other_ROBOT_ID=NEIGH1_ID;
        float theta =	direction;
        float range = distance;
        // Get position update
        //theta += dtheta_g[other_ROBOT_ID];
        //theta_robots[other_ROBOT_ID] = 0.8*theta_robots[other_ROBOT_ID] + 0.2*theta;
        prev_relative_pos[other_ROBOT_ID][0] = relative_pos[other_ROBOT_ID][0];
        prev_relative_pos[other_ROBOT_ID][1] = relative_pos[other_ROBOT_ID][1];
        relative_pos[other_ROBOT_ID][0] = range*cos(theta);  // relative x pos
        relative_pos[other_ROBOT_ID][1] = -1.0 * range*sin(theta);   // relative y pos
        //printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_ROBOT_ID,relative_pos[other_ROBOT_ID][0],relative_pos[other_ROBOT_ID][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);
        relative_speed[other_ROBOT_ID][0] = relative_speed[other_ROBOT_ID][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_ROBOT_ID][0]-prev_relative_pos[other_ROBOT_ID][0]);
        relative_speed[other_ROBOT_ID][1] = relative_speed[other_ROBOT_ID][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_ROBOT_ID][1]-prev_relative_pos[other_ROBOT_ID][1]);
      }
      if((int)(imsg.value)==NEIGH2_ID){
        other_ROBOT_ID=NEIGH2_ID;
        float theta =	direction;
        float range = distance;
        // Get position update
        //theta += dtheta_g[other_ROBOT_ID];
        //theta_robots[other_ROBOT_ID] = 0.8*theta_robots[other_ROBOT_ID] + 0.2*theta;
        prev_relative_pos[other_ROBOT_ID][0] = relative_pos[other_ROBOT_ID][0];
        prev_relative_pos[other_ROBOT_ID][1] = relative_pos[other_ROBOT_ID][1];
        relative_pos[other_ROBOT_ID][0] = range*cos(theta);  // relative x pos
        relative_pos[other_ROBOT_ID][1] = -1.0 * range*sin(theta);   // relative y pos
        //printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_ROBOT_ID,relative_pos[other_ROBOT_ID][0],relative_pos[other_ROBOT_ID][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);
        relative_speed[other_ROBOT_ID][0] = relative_speed[other_ROBOT_ID][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_ROBOT_ID][0]-prev_relative_pos[other_ROBOT_ID][0]);
        relative_speed[other_ROBOT_ID][1] = relative_speed[other_ROBOT_ID][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_ROBOT_ID][1]-prev_relative_pos[other_ROBOT_ID][1]);
      }
      }
  }
  }

  ircomEnableContinuousListening ();
}


void init(){
  e_init_port();
  e_init_ad_scan();
  e_init_uart1();
  e_led_clear();
  e_init_motors();
  e_start_agendas_processing();
  int selector = getselector();
  // wait for s to start
  btcomSendInt(selector);
  btcomSendString("...");
  btcomWaitForCommand('s',1);
  btcomSendString("==== READY - IR TESTING ====\n\n");
  e_calibrate_ir();
  // initialize ircom and start reading
  ircomStart();
  ircomEnableContinuousListening();
  ircomListen();
}
void flocking_algo(){
  int msl, msr;			// Wheel speeds
  /*Webots 2018b*/
  float msl_w, msr_w;
  /*Webots 2018b*/
  int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
  int i;
  int j;				// Loop counter
  int distances[NB_SENSORS];	// Array for the distance sensor readings
  int max_sens;			// Store highest sensor value
  int count = 0;
  msl = 0; msr = 0;
  max_sens = 0;
  bmsl = 0;
  bmsr = 0;
  sum_sensors = 0;
  max_sens = 0;

  /* Braitenberg */
  for(i=0;i<NB_SENSORS;i++)
  {
        distances[i]=(e_get_calibrated_prox(i)); //Read sensor values
        sum_sensors += distances[i]; // Add up sensor values
        max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value

        // Weighted sum of distance sensor values for Braitenburg vehicle
        bmsl += e_puck_matrix_left[i] * distances[i];
        bmsr += e_puck_matrix_right[i] * distances[i];
   }
   // Adapt Braitenberg values (empirical tests)
   bmsl/=MIN_SENS; bmsr/=MIN_SENS;
   bmsl+=90; bmsr+=90;

  /* Send and get information */


  /// Compute self position
  prev_my_position[0] = my_position[0];
  prev_my_position[1] = my_position[1];

  update_self_motion(msl,msr);


  speed[ROBOT_ID][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
  speed[ROBOT_ID][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);

  // Reynold's rules with all previous info (updates the speed[][] table)
  reynolds_rules();
  btcomSendString(" vefore : ");
  btcomSendInt(msl);
  // Compute wheels speed from reynold's speed
  compute_wheel_speeds(&msl, &msr);
  btcomSendString(" after : ");
  btcomSendInt(msl);

  // Adapt speed instinct to distance sensor values
  if (sum_sensors > NB_SENSORS*MIN_SENS) {
    msl -= msl*max_sens/(2*MAX_SENS);
    msr -= msr*max_sens/(2*MAX_SENS);
  }

  // Add Braitenberg
  msl += bmsl;
  msr += bmsr;



  // Set speed

  if(ABS(msl_w)>1000){
  msl=(1000)*SGN(msl_w);
  }
  if(ABS(msr_w)>1000){
  msr=(1000)*SGN(msr_w);
  }
  e_set_speed_left(msl);
  e_set_speed_right(msr);
}
int main()
{

  init();
  e_activate_agenda(flocking_algo,1000);
  while(1){send_ping();
  process_received_ping_messages(prev_relative_pos,relative_pos,relative_speed);}
}
