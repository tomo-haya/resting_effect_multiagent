#include "Prototype.h"
#include "Def-Physics.h"
#include "struct.h"
#include "mt19937ar.h"
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>

posori _follow_leader_vel(xy my_pos, xy target_speed, int my_collision, double speed)
{
	posori upd_posori;

	if (my_collision == 1 || my_collision == 2 || my_collision == 3 || my_collision == 4)
	{
		upd_posori.ori = 2 * M_PI * genrand_real1();
		upd_posori.pos.x = my_pos.x + speed * DT * cos(upd_posori.ori);
		upd_posori.pos.y = my_pos.y + speed * DT * sin(upd_posori.ori);
	}
	else
	{
		upd_posori.ori = atan2(target_speed.y, target_speed.x);
		upd_posori.pos.x = my_pos.x + min(speed * DT, hypot(target_speed.x, target_speed.y) * DT) * cos(upd_posori.ori);
		upd_posori.pos.y = my_pos.y + min(speed * DT, hypot(target_speed.x, target_speed.y) * DT) * sin(upd_posori.ori);
	}
	return upd_posori;
}

Agent _follow_leader_vel2(Agent my_info, xy target_speed, double speed)
{
	Agent upd_info = my_info;

	if (upd_info.collision == 1 || upd_info.collision == 2 || upd_info.collision == 3 || upd_info.collision == 4)
	{
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//orientation is randomly determined
		upd_info.dist_straight = 0;//free path is resetted
	}
	upd_info.dist_straight += speed * DT;

	if (upd_info.dist_straight >= MEAN_FREE_PATH_COLL)
	{
		upd_info._posori.ori = atan2(target_speed.y, target_speed.x);
		upd_info._posori.pos.x += min(speed * DT, hypot(target_speed.x, target_speed.y) * DT) * cos(upd_info._posori.ori);
		upd_info._posori.pos.y += min(speed * DT, hypot(target_speed.x, target_speed.y) * DT) * sin(upd_info._posori.ori);
	}

	return upd_info;
}

food_detect judge_near_food(posori my_posori, vector<Prey> food, int num_food)
{
	food_detect upd_info;

	upd_info.detect = false;
	upd_info.id = -2;

	for (int j = 0; j < num_food; j++)
	{
		if (food[j].transport == false)
		{
			double dist_robot2prey = hypot(food[j].pos.x - my_posori.pos.x, food[j].pos.y - my_posori.pos.y);				

			if (dist_robot2prey < RADIUS_SENSOR + RADIUS_FOOD)
			{
				upd_info.detect = true;
				upd_info.id = j;
			}
		}
	}
	return upd_info;
}

Agent _approach2food(vector<Agent> _agent, Agent my_info, xy food_pos, int my_id, int num_agent, double speed)
{
	Agent upd_info = my_info;

	double ang_robot2prey = atan2(food_pos.y - upd_info._posori.pos.y, food_pos.x - upd_info._posori.pos.x);//calculate robot->food direction w.r.t.ground coord
	double dist2prey = hypot(food_pos.x - upd_info._posori.pos.x, food_pos.y - upd_info._posori.pos.y);//calculate robot-food distance

	double temp_dist = 1e10;//Initialize
	double move_dist = 1e10;//Initialize

	for (int k = 0; k < num_agent; k++)//measure near agent with pushing state
	{
		if (_agent[k].state == (PUSHING) && my_id != k)
		{
			double agent_x = (_agent[k]._posori.pos.x - upd_info._posori.pos.x) * cos(ang_robot2prey) + (_agent[k]._posori.pos.y - upd_info._posori.pos.y) * sin(ang_robot2prey);//agent_k Xpos w.r.t.robot_i
			double agent_y = -(_agent[k]._posori.pos.x - upd_info._posori.pos.x) * sin(ang_robot2prey) + (_agent[k]._posori.pos.y - upd_info._posori.pos.y) * cos(ang_robot2prey);//agent_k Ypos w.r.t.robot_i
			if (agent_x > 0 && //agent_k is in moving direction of agent_i
				fabs(agent_y) < 2 * RADIUS_AGENT) //collision may occur between agent_i and agent_k
			{
				temp_dist = agent_x - pow(4 * pow(RADIUS_AGENT, 2) - pow(agent_y, 2), 0.5);
				if (move_dist > temp_dist)//agent_k is nearest in the moving direction)
				{
					move_dist = temp_dist;
				}
			}
		}
	}

	if (upd_info.collision == 1 || upd_info.collision == 2 || upd_info.collision == 3 || upd_info.collision == 4)
	{
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//orientation is randomly determined
		upd_info.dist_straight = 0;//free path is resetted
	}
	upd_info.dist_straight += speed * DT;

	if (upd_info.dist_straight >= MEAN_FREE_PATH_COLL)
	{
		upd_info._posori.ori = ang_robot2prey;//orient to the position of a food item
		upd_info._posori.pos.x += min({ speed * DT, dist2prey - RADIUS_FOOD - RADIUS_AGENT, move_dist }) * cos(upd_info._posori.ori);	//x-dir. Stop if the robot collides with a food item
		upd_info._posori.pos.y += min({ speed * DT, dist2prey - RADIUS_FOOD - RADIUS_AGENT, move_dist }) * sin(upd_info._posori.ori);	//y-dir. Stop if the robot collides with a food item
	}
	else
	{
		upd_info._posori.pos.x += speed * DT * cos(upd_info._posori.ori);
		upd_info._posori.pos.y += speed * DT * sin(upd_info._posori.ori);
	}

	return upd_info;
}

Agent approach2mem_food(Agent my_info, double speed)
{
	Agent upd_info = my_info;

	double ang_robot2prey = atan2(upd_info.mem_foodpos.y - upd_info._posori.pos.y, upd_info.mem_foodpos.x - upd_info._posori.pos.x);//orient to the position of a food item

	if (upd_info.collision == 1 || upd_info.collision == 2 || upd_info.collision == 3 || upd_info.collision == 4)
	{
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//orientation is randomly determined
		upd_info.dist_straight = 0;//free path is resetted
	}
	upd_info.dist_straight += speed * DT;

	if (upd_info.dist_straight >= MEAN_FREE_PATH_COLL) upd_info._posori.ori = ang_robot2prey;//orient to the position of a food item

	//update position of agent temporarily
	double dist2prey = hypot(upd_info.mem_foodpos.x - upd_info._posori.pos.x, upd_info.mem_foodpos.y - upd_info._posori.pos.y);
	upd_info._posori.pos.x += min(speed * DT, dist2prey - RADIUS_FOOD - RADIUS_AGENT) * cos(upd_info._posori.ori);	//x-dir. Stop if the robot collides with a food item
	upd_info._posori.pos.y += min(speed * DT, dist2prey - RADIUS_FOOD - RADIUS_AGENT) * sin(upd_info._posori.ori);	//y-dir. Stop if the robot collides with a food item

	return upd_info;
}

posori food_transport_agent(xy my_pos, xy food_pos)
{
	posori upd_posori;
	upd_posori.ori = atan2(-food_pos.y, -food_pos.x);	//determine moving direction of food

	upd_posori.pos.x = my_pos.x + TRANSPORT_SPEED * DT * cos(upd_posori.ori);	//x-dir.
	upd_posori.pos.y = my_pos.y + TRANSPORT_SPEED * DT * sin(upd_posori.ori);	//y-dir.

	return upd_posori;
}

Agent _return_nest(Agent my_info, double speed)
{
	Agent upd_info = my_info;

	double ang_robot2prey = atan2(-upd_info._posori.pos.y, -upd_info._posori.pos.x);//orient to the anti-direction to a food item

	if (upd_info.collision == 1 || upd_info.collision == 2 || upd_info.collision == 3 || upd_info.collision == 4)
	{
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//orientation is randomly determined
		upd_info.dist_straight = 0;//free path is resetted
	}
	upd_info.dist_straight += speed * DT;

	if (upd_info.dist_straight >= MEAN_FREE_PATH_COLL) upd_info._posori.ori = ang_robot2prey;//orient to the anti-direction to a food item

	//update position of agent temporarily	
	double dist2nest = hypot(upd_info._posori.pos.x, upd_info._posori.pos.y);
	upd_info._posori.pos.x += min((double)MOVING_SPEED * DT, dist2nest) * cos(upd_info._posori.ori);	//x-dir. Stop at the center of the resting area
	upd_info._posori.pos.y += min((double)MOVING_SPEED * DT, dist2nest) * sin(upd_info._posori.ori);	//y-dir. Stop at the center of the resting area

	return upd_info;
}

Agent _searcharea_random_walk(Agent my_info, double speed)
{
	Agent upd_info = my_info;

	if ((upd_info.dist_straight >= MEAN_FREE_PATH && hypot(upd_info._posori.pos.x, upd_info._posori.pos.y) >= RADIUS_N) ||//move sufficiently at the searching area
		(hypot(upd_info._posori.pos.x, upd_info._posori.pos.y) >= RADIUS_N &&
		hypot(upd_info._posori.pos.x + speed * DT * cos(upd_info._posori.ori), upd_info._posori.pos.y + speed * DT * sin(upd_info._posori.ori)) < RADIUS_N) ||//Enters the resting area in the current orientation
		upd_info.collision == 1 || upd_info.collision == 2 || upd_info.collision == 3 || upd_info.collision == 4)//collision
	{
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//orientation is randomly determined
		upd_info.dist_straight = 0;//free path is resetted
	}
	upd_info.dist_straight += speed * DT;

	if (hypot(upd_info._posori.pos.x, upd_info._posori.pos.y) >= RADIUS_N &&
		hypot(upd_info._posori.pos.x + speed * DT * cos(upd_info._posori.ori), upd_info._posori.pos.y + speed * DT * sin(upd_info._posori.ori)) < RADIUS_N)
	{//If enters the resting area, then position is not updated
		//upd_info._posori.pos = upd_info._posori.pos;
	}
	else
	{//Move in searching area or located in the resting area before moving
		upd_info._posori.pos.x += speed * DT * cos(upd_info._posori.ori);	//x-dir.
		upd_info._posori.pos.y += speed * DT * sin(upd_info._posori.ori);	//y-dir.
	}
	return upd_info;
}

Agent _nest_random_walk(Agent my_info, double speed)
{
	Agent upd_info = my_info;

	if (upd_info.dist_straight >= MEAN_FREE_PATH ||//sufficiently move
		hypot(upd_info._posori.pos.x + speed * DT * cos(upd_info._posori.ori), upd_info._posori.pos.y + speed * DT * sin(upd_info._posori.ori)) >= RADIUS_N ||//enters the searching area by the current orientation
		upd_info.collision == 1 || upd_info.collision == 2 || upd_info.collision == 3 || upd_info.collision == 4)
	{
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//orientation is randomly determined
		upd_info.dist_straight = 0;//free path is resetted
	}
	upd_info.dist_straight += speed * DT;

	if (hypot(upd_info._posori.pos.x + speed * DT * cos(upd_info._posori.ori), upd_info._posori.pos.y + speed * DT * sin(upd_info._posori.ori)) >= RADIUS_N)
	{//If enters the searching area, then position is not updated
		upd_info._posori.pos.x += 0;
		upd_info._posori.pos.y += 0;
	}
	else
	{
		upd_info._posori.pos.x += speed * DT * cos(upd_info._posori.ori);	//x-dir.
		upd_info._posori.pos.y += speed * DT * sin(upd_info._posori.ori);	//y-dir.
	}

	return upd_info;
}
