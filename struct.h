#ifndef STRUCT_H_INCLUDED_
#define STRUCT_H_INCLUDED_

#include <vector>

using namespace std;

class xy {
public:
	double x, y;
};

class posori {
public:
	xy pos;
	double ori;
};

class food_detect {
public:
	bool detect;
	int id;
};


//=============================================================//
//name:Agent
//description:All internal states of each robot is memorized
/*!
*/
//==============================================================//
class Agent {
public:
	Agent()
	{
		initialize_agent();
	};

	void initialize_agent(void)//reset
	{
		time_searching = 0;
		time_resting = 0;
		time_pushinglead = 0;
		time_recruiting = 0;
		dist_straight = 0;
	};

	double r0, time_resting0, theta0, ang_r0;//ì«Ç›çûÇÒÇæèâä˙èÛë‘
	posori _posori;
	xy vel;
	xy mem_foodpos;
	int state;						//State
	double time_searching;			//Searching time[sec]
	double time_resting;			//Resting time[sec]
	double time_pushinglead;		//Pushing time[sec]
	double time_recruiting;			//Recruiting time[sec]
	int maxtime_rest;				//MAX_Resting time[sec]
	bool near_food;					//determine behaviro in searching state
	int leaderID;
	double dist_straight;					//Used for mean free path
	int consume_energy;
	int collision;
	int col_foodID;//Food ID with collision
	int mem_foodID;
	food_detect food_info;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		Food
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


//=============================================================//
//name:Prey
//description:äeâaÇ…ä÷Ç∑ÇÈèÓïÒÇãLâØÇ∑ÇÈ
/*!
*	@x,y				Position
*	@mass				Mass
*/
//==============================================================//

class Prey {
public:
	xy pos;
	xy vel;//velocity of Leader is used by follower
	double mass;
	double r0, theta0;//Read initial state
	double mass0;
	bool transport;//Move from initial position or not
	bool inside_nest;//Transport to the resting area or not
	int num_grabbing;
	int collision;//food-food collision		
};




extern vector<Prey> prey;
extern vector<Prey> pastprey;
extern vector<Agent> agent;
extern vector<Agent> pastagent;

extern int num_initprey;
extern int num_agent;
extern int recruit_time;
extern int n_max;

#endif


