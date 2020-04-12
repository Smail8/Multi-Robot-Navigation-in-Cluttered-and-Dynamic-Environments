/*****************************************************************************/
/* File:         Group1                                                 */
/* Version:      10.0                                                         */
/* Date:         16-December-18                                                   */
/* Description:  Supervisor		     */
/*                                                                           */
/* Author: 	Yassine Ahaggach, Ait Bouhsain Smail, Charles Coster and Malo Simondin 				     */
/*				     */				     
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE	 6		// Number of robots in flock
// TO BE CHANGE TO 6 FOR OBSTACLE
#define TIME_STEP	64		// [ms] Length of time step
#define MAX_SPEED      0.1287    // Maximum speed webots

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock
float prev_loc[FLOCK_SIZE][3];

#define RULE1_THRESHOLD 0.2
#define fit_cluster_ref 0.03
#define fit_orient_ref 1.0


int offset = 0;				// Offset of robots number
float migrx, migrz;			// Migration vector
float orient_migr; 			// Migration orientation
int t;
int t_old;

int count = 0 ; 
float metrics[3] = {0,0,0}; 


/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i+offset);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
	for(int i=0;i<FLOCK_SIZE;i++){
	prev_loc[i][0]=-9999;
	prev_loc[i][1]=-9999;
	prev_loc[i][2]=-9999;
	}
	migrx=0;
	migrz=-25;
}


/*
 * Compute performance metrics.
 */
void compute_mean(float loc[FLOCK_SIZE][3],float* mean){

  for(int i=0;i<3;i++){
    for(int j=0;j<FLOCK_SIZE;j++){
      mean[i]+=loc[j][i];
    }
    mean[i]/=FLOCK_SIZE;
  }

}

float dist_to(float loc1[3],float loc2[3]){
  float dist =0;
  float difference[2]={ 0, 0};
  for(int i=0;i<2;i++){
    difference[i]=loc2[i]-loc1[i];
    dist+=(difference[i]*difference[i]);
  }
  dist=sqrt(dist);
  return dist;
}
 
float compute_orientation(float loc[FLOCK_SIZE][3]){

  float orient_cos=0;
  float orient_sin=0;
  float orient=0;
    for(int i=0;i<FLOCK_SIZE;i++){
    orient_cos+=cos(loc[i][2]);
    orient_sin+=sin(loc[i][2]);
    }
    orient = sqrt(orient_cos*orient_cos+orient_sin*orient_sin)/FLOCK_SIZE;
  return orient;

}

float compute_cohesion(float loc[FLOCK_SIZE][3]){
  float cohesion=0;
  float mean[3]={0,0,0};
  compute_mean(loc,mean);
  for(int i=0;i<FLOCK_SIZE;i++){
    cohesion+=dist_to(loc[i],mean);
  }
  cohesion/=FLOCK_SIZE;
  cohesion+=1;
  cohesion=1/cohesion;
  return cohesion;

}

float compute_velocity(float loc[FLOCK_SIZE][3],float prev_loc[FLOCK_SIZE][3],int t_old,int t){

  float speed[3]={0,0,0};
  float proj_speed=0;
  float migr_direction[3]={0,0,0};
  float length_migr=sqrt(migrx*migrx+migrz*migrz);
  migr_direction[0]=-migrz/length_migr;
  migr_direction[1]=0;
  migr_direction[2]=migrx/length_migr;

  for(int i=0;i<FLOCK_SIZE;i++){
    speed[0]+=wb_supervisor_node_get_velocity(robs[i])[0];
    speed[1]+=wb_supervisor_node_get_velocity(robs[i])[1];
    speed[2]+=wb_supervisor_node_get_velocity(robs[i])[2];
  }
  speed[0]/=FLOCK_SIZE;
  speed[1]/=FLOCK_SIZE;
  speed[2]/=FLOCK_SIZE;
  float reply=(speed[0]*migr_direction[0]+speed[1]*migr_direction[1]+speed[2]*migr_direction[2])/MAX_SPEED;
  return reply;
}




/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
	int i;			// Index
          
	reset();

		
	for(;;) {
		wb_robot_step(TIME_STEP);
		t += TIME_STEP;
		if (t % 10 == 0) {
			for (i=0;i<FLOCK_SIZE;i++) {
				// Get data
				loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
				
    			}
                                  float orient_perf=compute_orientation(loc);
                                  float cohesion_perf=compute_cohesion(loc);
                                  float velocity_perf=0;
                                  if(prev_loc[0][0]!=-9999)
                                    velocity_perf=compute_velocity(loc,prev_loc,t_old,t);
                                  for(int i=0;i<FLOCK_SIZE;i++){
                                    prev_loc[i][0]=loc[i][0];
                                    prev_loc[i][1]=loc[i][1];
                                    prev_loc[i][2]=loc[i][2];
                                  }
                                  t_old=t;
			printf("time:%d, Orientation: %f, Cohesion: %f,Velocity : %f\n", t, orient_perf,cohesion_perf,velocity_perf);
			count ++ ; 
			metrics[0] += cohesion_perf ; 
			metrics[1] += orient_perf ; 
			metrics[2] += velocity_perf ;
			
			float mean[3] = {metrics[0]/count,metrics[1]/count,metrics[2]/count} ;
			
			float perf = mean[0]*mean[1]*mean[2] ;
			
			printf("orientation mean: %f, Cohesion mean %f, Velocity mean: %f, perf: %f \n", 
			mean[0], mean[1], mean[2], perf); 						
			
		}
		
		
	}

}
