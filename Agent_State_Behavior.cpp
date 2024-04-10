#include "Prototype.h"
#include "Def-Physics.h"
#include "struct.h"
#include "Struct_Def.h"
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

Agent agent_state_behavior2(vector<Prey> _food, int num_food, Agent my_info, vector<Agent> _agent, int num_agent, int i)
{
	Agent upd_info = my_info;
	switch (my_info.state)
	{
	case RESTING:
		upd_info.time_resting += DT;
		break;
	case SEARCHING:
		upd_info.consume_energy++;
		upd_info.time_searching += DT;
		upd_info.food_info = judge_near_food(my_info._posori, _food, num_food);//_judge_near_food(upd_info, _food, num_food);
		switch (upd_info.food_info.detect)//behavior
		{
		case true:
			upd_info = _approach2food(_agent, upd_info, _food[upd_info.food_info.id].pos, i, num_agent, MOVING_SPEED);
			break;
		case false:
			upd_info = _searcharea_random_walk(upd_info, MOVING_SPEED);
			break;
		}
		break;
	case PUSHING_F:
		upd_info.consume_energy++;
		upd_info.time_pushinglead += DT;
		if (_food[upd_info.food_info.id].transport == true)
		{
			upd_info._posori = food_transport_agent(upd_info._posori.pos, _food[upd_info.food_info.id].pos);
		}
		break;
	case PUSHING:
		upd_info.consume_energy++;
		if (_food[upd_info.food_info.id].transport == true)
		{
			upd_info._posori = food_transport_agent(upd_info._posori.pos, _food[upd_info.food_info.id].pos);
		}
		break;
	case HOMING:
	case HOMING_F:
		upd_info.consume_energy++;
		upd_info = _return_nest(upd_info, MOVING_SPEED);
		break;
	case RECRUITING:
		upd_info.consume_energy++;
		upd_info = _nest_random_walk(upd_info, RECRUIT_SPEED);
		upd_info.time_recruiting += DT;	//Recruiting time is added
		break;
	case FOLLOWING:
		upd_info.consume_energy++;
		upd_info.food_info = judge_near_food(my_info._posori, _food, num_food);
		switch (upd_info.food_info.detect)//behavior
		{
		case true:
			upd_info = _approach2food(_agent, upd_info, _food[upd_info.food_info.id].pos, i, num_agent, RECRUIT_SPEED);
			break;
		case false:
			//upd_info._posori = _follow_leader_vel(upd_info._posori.pos, _agent[upd_info.leaderID].vel, upd_info.collision, MOVING_SPEED);
			upd_info = _follow_leader_vel2(upd_info, _agent[upd_info.leaderID].vel, MOVING_SPEED);
			//upd_info._posori = _follow_leader_posvel(upd_info._posori.pos, _agent[upd_info.leaderID]._posori.pos, _agent[upd_info.leaderID].vel, MOVING_SPEED, upd_info.collision, WEIGHT_VEL);
			break;
		}
		break;
	case LEADING:
		upd_info.consume_energy++;
		upd_info.food_info = judge_near_food(my_info._posori, _food, num_food);
		switch (upd_info.food_info.detect)//behavior
		{
		case true:
			upd_info = _approach2food(_agent, upd_info, _food[upd_info.food_info.id].pos, i, num_agent, RECRUIT_SPEED);
			break;
		case false:
			upd_info = approach2mem_food(upd_info, RECRUIT_SPEED);
			break;
		}
		break;
	case RECRUITING_S:
		upd_info.consume_energy++;
		upd_info = _searcharea_random_walk(upd_info, RECRUIT_SPEED);
		upd_info.time_recruiting += DT;	//Recruiting time is added
		break;
	default:
		break;
	}
	return upd_info;
}
