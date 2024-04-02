#ifndef DP_H_INCLUDED_
#define DP_H_INCLUDED_


#ifndef DT			
#define DT			0.1//Control cycle[sec]
#endif

#ifndef CASE	
#define CASE		100//100:Proposed strategy 101:Conventional strategy
#endif

#ifndef ANIMATION
#define ANIMATION		1//0: no screen, 1:show screen
#endif

//////////CASE1, 4, 11, 14, 100, 101(Setting for recruiting period change)//////////
#ifndef MIN_TIME_RECRUIT	
#define MIN_TIME_RECRUIT	100//min recruiting period
#endif

#ifndef DIFF_TIME_RECRUIT_CHANGE
#define DIFF_TIME_RECRUIT_CHANGE			10	//interval
#endif

#ifndef MAX_TIME_RECRUIT	
#define MAX_TIME_RECRUIT	1000//max recruiting period
#endif

//////////CASE2, 100, 101(Setting for population change)//////////
#ifndef MIN_NUM_AGENT		
#define MIN_NUM_AGENT		70//min population
#endif

#ifndef DIFF_NUM_AGENT_CHANGE
#define DIFF_NUM_AGENT_CHANGE			30	//interval
#endif

#ifndef MAX_NUM_AGENT		
#define MAX_NUM_AGENT		160//max population
#endif

//////////CASE3, 100, 101(Setting for environmental change)//////////
#ifndef MIN_NUM_INIT_PREY		
#define MIN_NUM_INIT_PREY		10//min food num
#endif

#ifndef DIFF_NUM_INIT_PREY_CHANGE
#define DIFF_NUM_INIT_PREY_CHANGE		2	//interval
#endif

#ifndef MAX_NUM_INIT_PREY		
#define MAX_NUM_INIT_PREY		12//max food num
#endif

//////////TYPICAL//////////
#ifndef TYPIC_TIME_RECRUIT	
#define TYPIC_TIME_RECRUIT	100//50//300//1000 //recruitin time
#endif

#ifndef TYPIC_NUM_AGENT	
#define TYPIC_NUM_AGENT	200 //population
#endif

#ifndef TYPIC_NUM_INIT_PREY	
#define TYPIC_NUM_INIT_PREY	5 //food num
#endif

///////////////////////Simulation setting///////////////////////
#ifndef _TimeLimit	
#define _TimeLimit	2000//maximum simulation period[sec]
#endif

#ifndef ATTEMPT
#define ATTEMPT		10	//Number of repetitions for one recruiting period, popoulation, environment
#endif

////////////////////////Environment setting//////////////////////////////
#ifndef RADIUS_F
#define RADIUS_F		1200//Radius of Field[cm]
#endif

#ifndef RADIUS_N
#define RADIUS_N		200//Radius of resting area[cm]
#endif

#ifndef AVG_MASS		
#define AVG_MASS	50//Average of food weight(No use)
#endif

#ifndef SD_MASS
#define SD_MASS		0//Variance of food weight(No use)
#endif

#ifndef MAKE_INITPREY
#define MAKE_INITPREY		true//Make environmental data
#endif

#ifndef MASS_COEFFICIENT
#define MASS_COEFFICIENT		1e-2//No use
#endif

#ifndef MASS_POSITION_VALUE
#define MASS_POSITION_VALUE		(120 * (RADIUS_F + RADIUS_N) / 2)//equivalent to W = (120 * (r_S+r_N + r_N) / 2)
#endif


#ifndef RADIUS_FOOD	
#define RADIUS_FOOD		30//radius of food[cm]
#endif

///////////////////////Agent setting///////////////////////

#ifndef RADIUS_SENSOR
#define RADIUS_SENSOR	50//Sensing range[cm]
#endif

#ifndef RADIUS_GRABBING
#define RADIUS_GRABBING		1e-1//Threshold to grasp food[cm]
#endif

#ifndef RADIUS_AGENT	
#define RADIUS_AGENT		5//Radius of Agent[cm]
#endif

#ifndef MOVING_SPEED	
#define MOVING_SPEED	10//v_H[cm/sec]
#endif

#ifndef RECRUIT_SPEED	
#define RECRUIT_SPEED	5//v_L[cm/sec]
#endif

#ifndef TRANSPORT_SPEED	
#define TRANSPORT_SPEED	5//v_L[cm/sec]
#endif

#ifndef MAXTIME_SEARCH	
#define MAXTIME_SEARCH	500//T_S[sec]
#endif

#ifndef MAXTIME_REST	
#define MAXTIME_REST	700//T_R[sec]
#endif

#ifndef MAXTIME_PUSHINGLEAD	
#define MAXTIME_PUSHINGLEAD	10//T_P[sec]
#endif

#ifndef MEAN_FREE_PATH	
#define MEAN_FREE_PATH	100//mean free path in random walk[cm]
#endif

#ifndef MEAN_FREE_PATH_COLL	
#define MEAN_FREE_PATH_COLL	10//mean free path after collision[cm]
#endif

#ifndef LF_DIST_DESIRE	
#define LF_DIST_DESIRE	(2*RADIUS_AGENT + 10)//Desired distance between leader and follower[cm](No use)
#endif

#ifndef WEIGHT_VEL	
#define WEIGHT_VEL	1.0//0-1. Velocity control weight against position control
#endif

////////////////////////////////////////////////////////////
//State definition
////////////////////////////////////////////////////////////

#ifndef RESTING
#define RESTING		32
#endif

#ifndef SEARCHING
#define SEARCHING	33
#endif

#ifndef PUSHING_F
#define PUSHING_F	34
#endif

#ifndef PUSHING
#define PUSHING	35
#endif

#ifndef TRANSPORTING
#define TRANSPORTING 36
#endif

#ifndef HOMING
#define HOMING 		37
#endif

#ifndef RECRUITING
#define RECRUITING 	38
#endif

#ifndef FOLLOWING
#define FOLLOWING 	39
#endif

#ifndef LEADING
#define LEADING 	40
#endif

#ifndef HOMING_F
#define HOMING_F 	41
#endif

#ifndef RECRUITING_S
#define RECRUITING_S 	42
#endif

#endif