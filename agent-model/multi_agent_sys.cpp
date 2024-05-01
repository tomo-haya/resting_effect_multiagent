#include "Prototype.h"
#include "Def-Physics.h"
#include "struct.h"
#include "Struct_Def.h"
#include <stdio.h>
#include <stdlib.h>
#include "glut.h"
#include "mt19937ar.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <vector>
#include <iterator>
#include <Windows.h>			//For Sleep
#include <process.h>			//thread
#include "DEFs.h"
#include <iostream>
#include <sstream>

using namespace std;

FILE* InitAgent_Log;			//Log of initial pose and initial resting time of all robots
FILE* InitPrey_Log;				//Log of initial position and weight of all foods
FILE* Result_Log;				//Log of Result


double sum_transport_mass;
int num_outside_prey;
int state_num[8];//0:Resting, 1:Searching, 2:Pushing, 4:Homing, 5:Recruiting, 6:Leading, 7:Following
bool restart = false;

void init_log()
{
	fprintf(Result_Log, "Case rectime agent initprey n fintime num_finalprey success  obtain_total_mass  consumed_energy Rest     Search     Push     Transport     Homing     Recruit     Lead     Follow    Num.disc_food    Num.Following    Foodweight");
	fprintf(Result_Log, "\n");
}


void make_initpreydata_weight(void)//Determine initial state of food items
{
	double ave_mass = MASS_POSITION_VALUE / (double)num_initprey / ((RADIUS_F + RADIUS_N) / 2);
	double sd_mass = 0;

	InitPrey_Log = fopen("initprey_weight.dat", "w");//Output file name should be different from reading file.	
	double temp_value;
	int tempcount = 0;//For debugging
	prey[num_initprey - 1].r0 = 0;//Even If any file is read, this initialization enables making initpreydata.
	int food_collision;

	while (prey[num_initprey - 1].r0 < (RADIUS_N + RADIUS_FOOD) || prey[num_initprey - 1].r0 > (RADIUS_F- RADIUS_FOOD))//Loop until the final food item is located in the range of searching area.
	{
		food_collision = false;//initialize
		temp_value = 0;//Used for calculation of position of final food item
		for (int i = 0;i < num_initprey;i++)
		{
			double random1 = genrand_real1();
			double random2 = genrand_real1();
			double norm_random = sqrt(-2 * log(random1)) * cos(2 * M_PI * random2);//Box–Muller's method
			prey[i].mass0 = ave_mass + sd_mass * norm_random;//Determine food weight
		}

		for (int i = 0; i < num_initprey - 1;i++)//Determine d_i
		{
			while (1)
			{
				prey[i].r0 = RADIUS_F * (genrand_real1() * ((double)RADIUS_F - 2*RADIUS_FOOD - RADIUS_N) / RADIUS_F + ((double)RADIUS_N + RADIUS_FOOD) / RADIUS_F);//Located randomly in the searching area
				prey[i].theta0 = 2 * M_PI * genrand_real1();
				prey[i].collision = 0;
				for (int j = 0; j < i; j++)//check collision with successfully initialized foods
				{
					if (hypot(prey[i].r0 * cos(prey[i].theta0) - prey[j].r0 * cos(prey[j].theta0), prey[i].r0 * sin(prey[i].theta0) - prey[j].r0 * sin(prey[j].theta0)) < 2 * RADIUS_FOOD)
					{
						prey[i].collision = 1;
					}
				}
				cout << "Food i" << i << endl;
				if (prey[i].collision == 0) break;
			}

			temp_value += prey[i].mass0 * prey[i].r0;
		}


		prey[num_initprey - 1].r0 = (MASS_POSITION_VALUE - temp_value) / prey[num_initprey - 1].mass0;//The position of the final food item is determined based on mass_position_value
		tempcount++;
	}

	while (1)
	{
		prey[num_initprey - 1].theta0 = 2 * M_PI * genrand_real1();
		prey[num_initprey - 1].collision = 0;
		for (int j = 0; j < num_initprey - 1; j++)//check collision with successfully initialized foods
		{
			if (hypot(prey[num_initprey - 1].r0 * cos(prey[num_initprey - 1].theta0) - prey[j].r0 * cos(prey[j].theta0), prey[num_initprey - 1].r0 * sin(prey[num_initprey - 1].theta0) - prey[j].r0 * sin(prey[j].theta0)) < 2 * RADIUS_FOOD)
			{
				prey[num_initprey - 1].collision = 1;
			}
		}
		cout << "Food i" << num_initprey - 1 << endl;
		if (prey[num_initprey - 1].collision == 0) break;
	}

	cout << "Finish Making Init_prey_data" << " Prey:" << num_initprey << endl;

	for (int i = 0;i < num_initprey;i++)
	{
		fprintf(InitPrey_Log, "%10f %10f %10f\n", prey[i].r0, prey[i].theta0, prey[i].mass0);
	}
	fclose(InitPrey_Log);
}

void make_initagentdata(void)
{
	InitAgent_Log = fopen("initagent.dat", "w");//Output file name should be different from reading file.	
	for (int i = 0;i < num_agent;i++)
	{
		agent[i].ang_r0 = 2 * M_PI * genrand_real1();
		agent[i].time_resting0 = MAXTIME_REST * genrand_real1();

		while (1)//determine agent i position without collision
		{			
			agent[i].r0 = (RADIUS_N - RADIUS_AGENT) * genrand_real1();
			agent[i].theta0 = 2 * M_PI * genrand_real1();

			agent[i].collision = 0;
			for (int j = 0; j < i; j++)//check collision with successfully initialized robots
			{
				if (hypot(agent[i].r0 * cos(agent[i].theta0) - agent[j].r0 * cos(agent[j].theta0), agent[i].r0 * sin(agent[i].theta0) - agent[j].r0 * sin(agent[j].theta0)) < 2*RADIUS_AGENT)
				{
					agent[i].collision = 1;
				}
			}
			cout << "Agent i" << i << endl;
			if (agent[i].collision == 0) break;
		}
	}

	for (int i = 0;i < num_agent;i++)
	{
		fprintf(InitAgent_Log, "%10f %10f %10f %20f\n", agent[i].r0, agent[i].theta0, agent[i].ang_r0, agent[i].time_resting0);
	}
	cout << "Finish Making Init_agent_data" << " Agent:" << num_agent << endl;
	fclose(InitAgent_Log);
}

void open_files(int num_prey, int num_agent, int _case)
{
	ostringstream ss;
	ss << num_prey;
	ostringstream ss_a;
	ss_a << num_agent;
	std::string str_controll;
	if (_case == 1 || _case == 2 || _case == 3 || _case == 4 || _case == 100) str_controll = "Proposed_";
	if (_case == 11 || _case == 12 || _case == 13 || _case == 14 || _case == 101) str_controll = "Conventional_";

	std::string str_result = "result.dat";
	std::string str_P = "P";
	std::string str_A = "A";
	str_result = str_controll + str_P + ss.str() + str_A + ss_a.str() + str_result;
	const char* f_resultname = str_result.c_str();
	Result_Log = fopen(f_resultname, "w");
}

Prey update_food_info(vector<Agent> _agent, Prey food_info, int num_food, int num_agent, int food_id)
{
	Prey upd_food = food_info;

	if (upd_food.transport == true && upd_food.inside_nest == false)//Food is transported
	{
		upd_food.pos.x += TRANSPORT_SPEED * DT * cos(atan2(-food_info.pos.y, -food_info.pos.x));	//x-dir
		upd_food.pos.y += TRANSPORT_SPEED * DT * sin(atan2(-food_info.pos.y, -food_info.pos.x));	//y-dir
	}

	upd_food.num_grabbing = 0;//Initialize
	for (int i = 0; i < num_agent; i++)
	{
		if (_agent[i].state == PUSHING_F || _agent[i].state == PUSHING)
		{
			if (_agent[i].food_info.id == food_id) upd_food.num_grabbing++;
		}
	}
	if (hypot(upd_food.pos.x, upd_food.pos.y) > RADIUS_N)//Food is outside the nest
	{
		if (upd_food.num_grabbing >= upd_food.mass)
		{
			upd_food.transport = true;
		}
		else
		{
			upd_food.transport = false;
		}
	}
	else//Food is inside the nest
	{
		upd_food.inside_nest = true;
	}

	return upd_food;
}

vector<Agent> resolve_collision(vector<Agent> _agent, vector<Agent> _pastagent, int num_agent, vector<Prey> _food, vector<Prey> _pastfood, int num_food)
{//Priority: wallblock -> food attack -> robot-robot stop 
	vector<Agent> upd_info = _agent;
	bool collision_occur = true;//initialize to enter while loop
	for (int i = 0; i < num_agent; i++) upd_info[i].collision = 0;//initialize

	while (collision_occur == true)
	{
		collision_occur = false;//initialize
		for (int i = 0; i < num_agent; i++)
		{
			if (upd_info[i].state != PUSHING_F && upd_info[i].state != PUSHING)
			{
				for (int j = 0; j < num_agent; j++)//robot-robot collision
				{
					if (i != j)
					{
						if (hypot(upd_info[i]._posori.pos.x - upd_info[j]._posori.pos.x, upd_info[i]._posori.pos.y - upd_info[j]._posori.pos.y) < (2 * RADIUS_AGENT - 1e-3))
						{
							if ((upd_info[j].state == PUSHING_F || upd_info[j].state == PUSHING))
							{
								if (upd_info[i].collision < 2)//make priority to prevent collide-change oscillation
								{
									collision_occur = true;
									upd_info[i].collision = 2;//robot-robot(transport or collide) collision
									upd_info[i].col_foodID = upd_info[j].food_info.id;
								}
							}
							else if (upd_info[j].collision == 2)//collide with a robot which is not pushing but colliding with food cluster )
							{
								if (upd_info[i].collision < 2)//make priority to prevent collide-change oscillation
								{
									collision_occur = true;
									upd_info[i].collision = 2;//robot-robot(transport or collide) collision
									upd_info[i].col_foodID = upd_info[j].col_foodID;
								}
							}
							else
							{
								if (upd_info[i].collision < 1)//robot-robot collision. make priority to prevent collide-change oscillation
								{
									collision_occur = true;
									upd_info[i].collision = 1;
								}
							}
						}
					}
				}

				for (int j = 0; j < num_food; j++)//robot-food collision
				{
					if (hypot(upd_info[i]._posori.pos.x - _food[j].pos.x, upd_info[i]._posori.pos.y - _food[j].pos.y) < (RADIUS_AGENT + RADIUS_FOOD - 1e-5) && //1e-5[cm] prevents calculation error
						_food[j].inside_nest == false)//food inside nest is removed
					{
						if (upd_info[i].collision < 2)
						{
							collision_occur = true;
							upd_info[i].collision = 2;
							upd_info[i].col_foodID = j;
						}
					}
				}

				if (hypot(upd_info[i]._posori.pos.x, upd_info[i]._posori.pos.y) > (RADIUS_F - RADIUS_AGENT))//robot-wall collision
				{
					if (upd_info[i].collision < 4)
					{
						collision_occur = true;
						upd_info[i].collision = 4;
					}
				}
			}
		}
		for (int i = 0; i < num_agent; i++)
		{
			if (agent[i].state != PUSHING_F && agent[i].state != PUSHING)
			{
				switch (upd_info[i].collision)
				{
				case 1://barerobot-barerobot collision
				case 4://robot-wall collision
					upd_info[i]._posori.pos.x = _pastagent[i]._posori.pos.x;
					upd_info[i]._posori.pos.y = _pastagent[i]._posori.pos.y;
					break;
				case 2://barerobot-foodset collision
					upd_info[i]._posori.pos.x = _pastagent[i]._posori.pos.x + _food[upd_info[i].col_foodID].pos.x - _pastfood[upd_info[i].col_foodID].pos.x;
					upd_info[i]._posori.pos.y = _pastagent[i]._posori.pos.y + _food[upd_info[i].col_foodID].pos.y - _pastfood[upd_info[i].col_foodID].pos.y;
					break;
				}
			}
		}
	}
	return upd_info;
}

void collision_check(vector<Agent> _agent, vector<Agent> _pastagent, int num_agent, vector<Prey> _food, vector<Prey> _pastfood, int num_food)
{
	for (int i = 0; i < num_agent; i++)
	{
		for (int j = i+1; j < num_agent; j++)//robot-robot collision
		{
				if (hypot(_agent[i]._posori.pos.x - _agent[j]._posori.pos.x, _agent[i]._posori.pos.y - _agent[j]._posori.pos.y) < (2 * RADIUS_AGENT - 1e-3))
				{
					//while (1)
					{
						cout << "R-R CollErr" << " state= " << _agent[i].state << " coll= " << _agent[i].collision << " x= " << _agent[i]._posori.pos.x << " y= " << _agent[i]._posori.pos.y << " px= " << _pastagent[i]._posori.pos.x << " py= " << _pastagent[i]._posori.pos.y << " f_id= " << _agent[i].food_info.id << " c_id= " << _agent[i].col_foodID << " state= " << _agent[j].state << " coll= " << _agent[j].collision << " x= " << _agent[j]._posori.pos.x << " y= " << _agent[j]._posori.pos.y << " px= " << _pastagent[j]._posori.pos.x << " py= " << _pastagent[j]._posori.pos.y << " f_id= " << _agent[j].food_info.id << " c_id= " << _agent[j].col_foodID << endl;
						restart = true;
					}
				}
		}

		for (int j = 0; j < num_food; j++)//robot-food collision
		{
			if (hypot(_agent[i]._posori.pos.x - _food[j].pos.x, _agent[i]._posori.pos.y - _food[j].pos.y) < (RADIUS_AGENT + RADIUS_FOOD - 1e-3) && _food[j].inside_nest == false)
			{
				//while (1)
				{
					cout << "R-F CollErr" << " d= " << hypot(_agent[i]._posori.pos.x - _food[j].pos.x, _agent[i]._posori.pos.y - _food[j].pos.y) << " pd= " << hypot(_pastagent[i]._posori.pos.x - _pastfood[j].pos.x, _pastagent[i]._posori.pos.y - _pastfood[j].pos.y) <<  " coll= " << _agent[i].collision << " state= " << _agent[i].state << " coll= " << _agent[i].collision << " x= " << _agent[i]._posori.pos.x << " y= " << _agent[i]._posori.pos.y << " px= " << _pastagent[i]._posori.pos.x << " py= " << _pastagent[i]._posori.pos.y << " f_id= " << _agent[i].food_info.id << " c_id= " << _agent[i].col_foodID  << " f_id= " << j << " x= " << _food[j].pos.x << " y= " << _food[j].pos.y << " px= " << _pastfood[j].pos.x << " py= " << _pastfood[j].pos.y << endl;
					restart = true;
				}
			}
		}
	}
	for (int i = 0; i < num_food; i++)
	{
		for (int j = 0; j < num_food; j++)//food-food collision
		{
			if (i != j)
			{
				if (hypot(_food[i].pos.x - _food[j].pos.x, _food[i].pos.y - _food[j].pos.y) < (2 * RADIUS_FOOD - 1e-3) && _food[i].inside_nest == false && _food[j].inside_nest == false)
				{
					//while (1)
					{
						cout << "F-F CollErr" << " x= " << _food[i].pos.x << " y= " << _food[i].pos.y << " px= " << _pastfood[i].pos.x << " py= " << _pastfood[i].pos.y << " x= " << _food[i].pos.x << " y= " << _food[i].pos.y << " px= " << _pastfood[j].pos.x << " py= " << _pastfood[j].pos.y << endl;
						restart = true;
					}
				}
			}
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//	Substant Main function
///////////////////////////////////////////////////////////////////////////////////////////////////
void idle(void)
{
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// State transition
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	switch (CASE)
	{
	case 100:
		for (int i = 0; i < num_agent; i++)
		{
			agent[i] = agent_state_transition_rule(pastprey, num_initprey, agent[i], pastagent, num_agent);
		}
		break;
	case 101:
		for (int i = 0; i < num_agent; i++)
		{
			agent[i] = agent_conventional_state_transition_rule(pastprey, num_initprey, agent[i], pastagent, num_agent);
		}
		break;
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Behavior of robot in accordance with its state
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < num_agent; i++)
	{
		agent[i] = agent_state_behavior2(pastprey, num_initprey, agent[i], pastagent, num_agent, i);
	}
	for (int i = 0; i < num_initprey; i++)
	{
		prey[i] = update_food_info(agent, prey[i], num_initprey, num_agent, i);//prey update is required before resolve collision
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Consider physical model (Food transport, Collision)
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	agent = resolve_collision(agent, pastagent, num_agent, prey, pastprey, num_initprey);
	collision_check(agent, pastagent, num_agent, prey, pastprey, num_initprey);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Memorize state
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < num_agent; i++)
	{
		agent[i].vel.x = (agent[i]._posori.pos.x - pastagent[i]._posori.pos.x) / DT;
		agent[i].vel.y = (agent[i]._posori.pos.y - pastagent[i]._posori.pos.y) / DT;
	}
	pastagent = agent;
	pastprey = prey;

	for (int i = 0; i < num_agent; i++)//Memorize sum of each state of robot at each timestep
	{
		switch (agent[i].state)
		{
		case RESTING:
			state_num[0]++;
			break;
		case SEARCHING:
			state_num[1]++;
			break;
		case PUSHING_F:
		case PUSHING:
			state_num[2]++;
			break;
		case HOMING:
		case HOMING_F:
			state_num[4]++;
			break;
		case RECRUITING:
		case RECRUITING_S:
			state_num[5]++;
			break;
		case LEADING:
			state_num[6]++;
		break;
		case FOLLOWING:
			state_num[7]++;
			break;
		default:
			break;
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Update step and animation
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	pasttime = step * DT;
	step++;						//Update step
	if (ANIMATION) glutPostRedisplay();		// Update animation

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Simulation end timing Judgement
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	nowtime = step * DT;

	num_outside_prey = 0;//Initialize. Simulation ends when food number in the searching area is 0.
	for (int i = 0; i < num_initprey; i++)
	{
		if (prey[i].inside_nest == false) num_outside_prey++;//巣の外にある
	}

	if (restart == true)//Food-food collision
	{ 
		restart = false;		
		cout << "restart" << endl;
		step = 0;
		init_agent();
		init_prey();
		for (int i = 0; i < 8; i++) state_num[i] = 0;
	}

	if (step * DT >= _TimeLimit || num_outside_prey == 0)//One trial is complete
	{
		cout << "Progress: " << n * 100 / n_max << "%" << " C: " << CASE << " A: " << num_agent << " P: " << num_initprey << " R: " << recruit_time  << " n: " << (n - 1) % ATTEMPT + 1;
		
		fprintf(Result_Log, "%5d %5d %5d %5d %5d %5d %15d", CASE, recruit_time, num_agent, num_initprey, int((n - 1) % ATTEMPT) + 1, int(step * DT), num_outside_prey);

		if (num_outside_prey == 0)//Complete foraging
		{
			cout << " Success! Fintime: " << step * DT << endl;
			fprintf(Result_Log, "        1");
		}
		else if (step * DT >= _TimeLimit)//Timeout
		{
			cout << " Failure! Outside prey: " << num_outside_prey << "/" << num_initprey << endl;
			fprintf(Result_Log, "      0");
		}

		//memorize obtained energy
		sum_transport_mass = 0.0;
		for (int i = 0; i < num_initprey; i++)
		{
			if (hypot(prey[i].pos.x, prey[i].pos.y) <= RADIUS_N) sum_transport_mass += prey[i].mass;
		}
		fprintf(Result_Log, "    %5f", sum_transport_mass);

		//memorize consumed energy
		int total_consume_energy = 0;
		for (int i = 0; i < num_agent; i ++) total_consume_energy += agent[i].consume_energy;
		fprintf(Result_Log, " %15d     ", total_consume_energy);

		//Memorize state of each robot
		for (int i = 0; i < 8; i++) fprintf(Result_Log, " %9d", state_num[i]);
		
		fprintf(Result_Log, "            %lf            %lf", prey[0].mass0, DT);
		fprintf(Result_Log, "\n");//new line
		 
		if (n == n_max) //All trials are complete
		{
			getchar();//Prevent deleting console (For debugging)
			exit(0);
		}
		else //All trials are not yet complete
		{
			if (n % ATTEMPT == 0) //Setting is changed
			{
				switch (CASE)
				{
				case 100:
					if (n % (ATTEMPT * ((MAX_TIME_RECRUIT - MIN_TIME_RECRUIT) / DIFF_TIME_RECRUIT_CHANGE + 1))  == 0)//Simulation for a certain number of prey is complete
					{
						fclose(Result_Log);

						recruit_time = MIN_TIME_RECRUIT;//reset

						if (n % (ATTEMPT * ((MAX_TIME_RECRUIT - MIN_TIME_RECRUIT) / DIFF_TIME_RECRUIT_CHANGE + 1) * ((MAX_NUM_INIT_PREY - MIN_NUM_INIT_PREY) / DIFF_NUM_INIT_PREY_CHANGE + 1)) == 0)//Simulation for a max number of prey is complete
						{
							num_initprey = MIN_NUM_INIT_PREY;//reset
							num_agent += DIFF_NUM_AGENT_CHANGE;//reset
							make_initagentdata();//determine initial position of each robot in accordance with num_agent
							read_agentdata();
							read_preydata_weight(num_initprey);//Read food file
						}
						else // Agent number is same
						{
							num_initprey += DIFF_NUM_INIT_PREY_CHANGE;
							read_preydata_weight(num_initprey);//Read food file
						}

						open_files(num_initprey, num_agent, CASE);
					}
					else recruit_time += DIFF_TIME_RECRUIT_CHANGE;
					break;
				case 101:
					if (n % (ATTEMPT * ((MAX_TIME_RECRUIT - MIN_TIME_RECRUIT) / DIFF_TIME_RECRUIT_CHANGE + 1)) == 0)//Simulation for a certain number of prey is complete
					{
						fclose(Result_Log);

						recruit_time = MIN_TIME_RECRUIT;

						if (n % (ATTEMPT * ((MAX_TIME_RECRUIT - MIN_TIME_RECRUIT) / DIFF_TIME_RECRUIT_CHANGE + 1) * ((MAX_NUM_INIT_PREY - MIN_NUM_INIT_PREY) / DIFF_NUM_INIT_PREY_CHANGE + 1)) == 0)//Simulation for a max number of prey is complete
						{
							num_initprey = MIN_NUM_INIT_PREY;
							num_agent += DIFF_NUM_AGENT_CHANGE;
							make_initagentdata();//determine initial position of each robot in accordance with num_agent
							read_agentdata();
							read_preydata_weight(num_initprey);//Read food file
						}
						else // Agent number is same
						{
							num_initprey += DIFF_NUM_INIT_PREY_CHANGE;
							read_preydata_weight(num_initprey);//Read food file
						}

						open_files(num_initprey, num_agent, CASE);
					}
					else recruit_time += DIFF_TIME_RECRUIT_CHANGE;
					break;
				}
			}

			n++;//Trial num is added

			//Initialize states of robot and food 
			step = 0;
			init_agent();		
			init_prey();
			for (int i=0; i < 8; i ++) state_num[i] = 0;
		}
	}
}

void resize(int w, int h)
{
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30.0, (double)w / (double)h, 1.0, 10000.0);

	glMatrixMode(GL_MODELVIEW);
}


void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 's'://Simulation Start
		cout << "Start!!!" << endl;
		glutIdleFunc(idle);
		break;
	case 'i': //Initial state making
		make_initpreydata_weight();
		make_initagentdata();
		//exit(0);
		break;
	case 'w': //Initial state making about environment
		make_initpreydata_weight();
		break;

	case 'q':			//Forced termination
	case 'Q':
	case '\033':		//'\033': ESC
		exit(0);
		break;
	default:
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//	Main Function
///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	srand((unsigned)time(NULL));
	init_genrand((unsigned long)time(NULL));

	start = clock();
	printf("Hello!!! i: Create Initial agent&prey data   s: start simulation   q: quit simulation\n");
	glutInitWindowPosition(0, 0);								//Beginning point at Window
	glutInitWindowSize(800, 600);								//Size of Window
	glutInit(&argc, argv);										//GLUT initialization
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);	//Display setting
	glutCreateWindow("Agent model");							//Open Window
	glutDisplayFunc(display);									//Illustrate content of display	
	glutReshapeFunc(resize);
	glutKeyboardFunc(keyboard);									//Substant main function(idle) is included
	init();														//Initial setting
	open_files(num_initprey, num_agent, CASE);
	init_log();
	start = clock();

	make_initagentdata();//Making initialized file about agent
	read_agentdata();//read initialized file about agent
	init_agent();//Initialize agent

	if (MAKE_INITPREY == true) make_initpreydata_weight();//Making initialized file about food items
	read_preydata_weight(num_initprey);//read initialized file about food items
	init_prey();//Initialize food items

	glutMainLoop();		//Infinite loop
	printf("end");

	return 0;
}