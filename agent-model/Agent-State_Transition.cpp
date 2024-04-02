#include "Def-Physics.h"
#include "struct.h"
#include "mt19937ar.h"//For random
#define _USE_MATH_DEFINES
#include <math.h>
#include "Prototype.h"

Agent _resting2searching(Agent my_info)
{
	Agent upd_info = my_info;
	if (my_info.time_resting > MAXTIME_REST)
	{
		upd_info.state = SEARCHING;
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.time_searching = 0;//�T�����Ԃ̃��Z�b�g
		upd_info.dist_straight = 0;//���ώ��R�H���̃��Z�b�g
	}
	return upd_info;
}

Agent _resting2following(vector<Agent> _agent, int num_agent, Agent my_info)
{
	Agent upd_info = my_info;
	for (int j = 0; j < num_agent; j++)
	{
		if (_agent[j].state == RECRUITING) //���N���[�^�Ƃ̋����݂̂��v�Z 
		{
			double dist_robot2recruiter = hypot(_agent[j]._posori.pos.x - my_info._posori.pos.x, _agent[j]._posori.pos.y - my_info._posori.pos.y);
			double ang_robot2recruiter = atan2(_agent[j]._posori.pos.y - my_info._posori.pos.y, _agent[j]._posori.pos.x - my_info._posori.pos.x);

			if (dist_robot2recruiter < (RADIUS_SENSOR + RADIUS_AGENT))//���E�������ɓ���
			{
				upd_info.state = FOLLOWING;
				upd_info.leaderID = j;
				break;
			}
		}
	}
	return upd_info;
}

Agent _searching2following(vector<Agent> _agent, int num_agent, Agent my_info)
{
	Agent upd_info = my_info;
	for (int j = 0; j < num_agent; j++)
	{
		if (_agent[j].state == RECRUITING_S) //���N���[�^�Ƃ̋����݂̂��v�Z 
		{
			double dist_robot2recruiter = hypot(_agent[j]._posori.pos.x - my_info._posori.pos.x, _agent[j]._posori.pos.y - my_info._posori.pos.y);
			double ang_robot2recruiter = atan2(_agent[j]._posori.pos.y - my_info._posori.pos.y, _agent[j]._posori.pos.x - my_info._posori.pos.x);

			if (dist_robot2recruiter < (RADIUS_SENSOR + RADIUS_AGENT))//���E�������ɓ���
			{
				upd_info.state = FOLLOWING;
				upd_info.leaderID = j;
				break;
			}
		}
	}
	return upd_info;
}

Agent _searching2pushinglead(vector<Prey> _food, int num_food, Agent my_info)//�a�����ށ��^��
{
	Agent upd_info = my_info;

	for (int j = 0; j < num_food; j++)
	{
		if (_food[j].transport == false)
		{
			double dist_robot2prey = hypot(_food[j].pos.x - my_info._posori.pos.x, _food[j].pos.y - my_info._posori.pos.y);

			if (dist_robot2prey < (RADIUS_AGENT + RADIUS_FOOD + RADIUS_GRABBING))
			{
				upd_info.state = PUSHING_F;
				upd_info.mem_foodpos.x = _food[j].pos.x;
				upd_info.mem_foodpos.y = _food[j].pos.y;
				upd_info.mem_foodID = j;
				upd_info.time_pushinglead = 0;
			}
		}
	}
	return upd_info;
}

Agent _searching2pushing(vector<Prey> _food, int num_food, Agent my_info, vector<Agent> _agent, int num_agent)//�a�����ށ��^��
{
	Agent upd_info = my_info;
	for (int j = 0; j < num_food; j++)
	{
		double dist_robot2prey = hypot(_food[j].pos.x - my_info._posori.pos.x, _food[j].pos.y - my_info._posori.pos.y);

		if (dist_robot2prey < (RADIUS_AGENT + RADIUS_FOOD + RADIUS_GRABBING)&&//contact to food
			_food[j].transport == false)
		{			
			for (int k = 0; k < num_agent; k++)
			{
				double dist_robot2robot = hypot(_agent[k]._posori.pos.x - my_info._posori.pos.x, _agent[k]._posori.pos.y - my_info._posori.pos.y);
				if ((_agent[k].state == PUSHING || _agent[k].state == PUSHING_F) &&
					dist_robot2robot < RADIUS_AGENT + RADIUS_SENSOR)//Any pushing agent is in sensing area
				{
					upd_info.state = PUSHING;
					upd_info.food_info.detect = true;//judge_food��ʂ�Ȃ��ꍇ������
					upd_info.food_info.id = j;
				}
			}
			if (upd_info.state != PUSHING)//No other agent exists
			{
				upd_info.state = PUSHING_F;
				upd_info.mem_foodpos.x = _food[j].pos.x;
				upd_info.mem_foodpos.y = _food[j].pos.y;
				upd_info.mem_foodID = j;
				upd_info.time_pushinglead = 0;
				upd_info.food_info.detect = true;//judge_food��ʂ�Ȃ��ꍇ������
				upd_info.food_info.id = j;
			}
		}

		for (int k = 0; k < num_agent; k++)
		{
			double dist_robot2robot = hypot(_agent[k]._posori.pos.x - my_info._posori.pos.x, _agent[k]._posori.pos.y - my_info._posori.pos.y);
			if ((_agent[k].state == PUSHING) &&
				dist_robot2robot > RADIUS_GRABBING && //exclude myself(dist=0)
				dist_robot2robot < (2 * RADIUS_AGENT + RADIUS_GRABBING))//contact to pushing robot
			{
				if (dist_robot2prey < (RADIUS_FOOD + RADIUS_SENSOR) &&//food is in sensing area
					_food[j].transport == false)
				{
					upd_info.state = PUSHING;
					upd_info.food_info.detect = true;//judge_food��ʂ�Ȃ��ꍇ������
					upd_info.food_info.id = j;
				}
			}
		}
	}
	return upd_info;
}

Agent _following2pushing(vector<Prey> _food, int num_food, Agent my_info, vector<Agent> _agent, int num_agent)
{
	Agent upd_info = my_info;

	for (int j = 0; j < num_food; j++)
	{
		double dist_robot2prey = hypot(_food[j].pos.x - my_info._posori.pos.x, _food[j].pos.y - my_info._posori.pos.y);

		if (dist_robot2prey < (RADIUS_AGENT + RADIUS_FOOD + RADIUS_GRABBING) &&
			_food[j].transport == false)//contact to food
		{
			upd_info.state = PUSHING;
			upd_info.food_info.detect = true;//judge_food��ʂ�Ȃ��ꍇ������
			upd_info.food_info.id = j;
		}

		for (int k = 0; k < num_agent; k++)
		{
			double dist_robot2robot = hypot(_agent[k]._posori.pos.x - my_info._posori.pos.x, _agent[k]._posori.pos.y - my_info._posori.pos.y);
			if (_agent[k].state == PUSHING &&
				dist_robot2robot < (2 * RADIUS_AGENT + RADIUS_GRABBING))//contact to pushing robot
			{
				if (dist_robot2prey < (RADIUS_FOOD + RADIUS_SENSOR) &&//food is in sensing area
					_food[j].transport == false)
				{
					upd_info.state = PUSHING;
					upd_info.food_info.detect = true;//judge_food��ʂ�Ȃ��ꍇ������
					upd_info.food_info.id = j;
				}
			}
		}
	}
	return upd_info;
}

Agent _following2searching(Agent my_info, Agent leader_info)
{
	Agent upd_info = my_info;
	double dist2robot = hypot(leader_info._posori.pos.x - my_info._posori.pos.x, leader_info._posori.pos.y - my_info._posori.pos.y);//calculate robot-robot distance
	if ((leader_info.state == PUSHING_F) && //leader is in pushing
		dist2robot <= (RADIUS_SENSOR + RADIUS_AGENT))//leader is in sensing area
	{
		upd_info.state = SEARCHING;
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.time_searching = 0;//�T�����Ԃ̃��Z�b�g
		upd_info.dist_straight = 0;//���ώ��R�H���̃��Z�b�g
	}

	return upd_info;
}

Agent _following2searching2(Agent my_info, Agent leader_info)
{
	Agent upd_info = my_info;
	double dist2robot = hypot(leader_info._posori.pos.x - my_info._posori.pos.x, leader_info._posori.pos.y - my_info._posori.pos.y);//calculate robot-robot distance
	if ((leader_info.state == PUSHING_F) && //leader is in pushing
		dist2robot <= (RADIUS_SENSOR + RADIUS_AGENT))//leader is in sensing area
	{
		upd_info.state = SEARCHING;
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.time_searching = 0;//�T�����Ԃ̃��Z�b�g
		upd_info.dist_straight = 0;//���ώ��R�H���̃��Z�b�g
	}

	if (((leader_info.state == SEARCHING) && //leader is in searching
		dist2robot <= (RADIUS_SENSOR + RADIUS_AGENT)) || //leader is in sensing area
		dist2robot > (RADIUS_SENSOR + RADIUS_AGENT))//lost leader
	{
		upd_info.state = SEARCHING;
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.time_searching = 0;//�T�����Ԃ̃��Z�b�g
		upd_info.dist_straight = 0;//���ώ��R�H���̃��Z�b�g
	}

	return upd_info;
}

Agent _searching2homing(Agent my_info)
{
	Agent upd_info = my_info;
	if (my_info.time_searching > MAXTIME_SEARCH) upd_info.state = HOMING;
	return upd_info;
}

Agent _homing2resting(Agent my_info)
{
	Agent upd_info = my_info;
	double dist2nest = hypot(my_info._posori.pos.x, my_info._posori.pos.y);

	if (dist2nest < MOVING_SPEED * DT ||//�����S�ł̐U���}��
		(dist2nest < RADIUS_N && //Agent is inside nest
		my_info.collision == 1))//agent-agent collision (Without this, outer-nest-edge aggregation occurs)
	{
		upd_info.state = RESTING;
		upd_info._posori.ori = 2 * M_PI * genrand_real1();	//�p�����΂������
		upd_info.time_resting = 0;			//�x�e���Ԃ̃��Z�b�g
	}
	return upd_info;
}

Agent _recruiting2leading(Agent my_info)
{
	Agent upd_info = my_info;

	if (my_info.time_recruiting >= recruit_time) //�������Ԃ��I�������ꍇ
	{
		upd_info.state = LEADING;
	}
	return upd_info;
}

Agent _following2homing(Agent my_info, Agent leader_info)
{
	Agent upd_info = my_info;
	double dist2robot = hypot(leader_info._posori.pos.x - my_info._posori.pos.x, leader_info._posori.pos.y - my_info._posori.pos.y);//calculate robot-robot distance
	if (((leader_info.state == HOMING || leader_info.state == HOMING_F) && //leader is in homing
		dist2robot <= (RADIUS_SENSOR + RADIUS_AGENT)) || //leader is in sensing area
		dist2robot > (RADIUS_SENSOR + RADIUS_AGENT))//lost leader
	{
		upd_info.state = HOMING;
	}

	return upd_info;
}

Agent _leading2pushinglead(vector<Prey> _food, int num_food, Agent my_info, vector<Agent> _agent, int num_agent)
{
	Agent upd_info = my_info;

	for (int j = 0; j < num_food; j++)
	{
		double dist_robot2prey = hypot(_food[j].pos.x - my_info._posori.pos.x, _food[j].pos.y - my_info._posori.pos.y);

		if (dist_robot2prey < (RADIUS_AGENT + RADIUS_FOOD + RADIUS_GRABBING) &&
			_food[j].transport == false)//contact to food
		{
			upd_info.state = PUSHING_F;
			upd_info.mem_foodpos.x = _food[j].pos.x;
			upd_info.mem_foodpos.y = _food[j].pos.y;
			upd_info.mem_foodID = j;
			upd_info.time_pushinglead = 0;
			upd_info.food_info.detect = true;//judge_food��ʂ�Ȃ��ꍇ������
			upd_info.food_info.id = j;
		}

		for (int k = 0; k < num_agent; k++)
		{
			double dist_robot2robot = hypot(_agent[k]._posori.pos.x - my_info._posori.pos.x, _agent[k]._posori.pos.y - my_info._posori.pos.y);
			if ((_agent[k].state == PUSHING) &&
				dist_robot2robot > RADIUS_GRABBING && //exclude myself(dist=0)
				dist_robot2robot < (2 * RADIUS_AGENT + RADIUS_GRABBING))//contact to pushing robot
			{
				if (dist_robot2prey < (RADIUS_FOOD + RADIUS_SENSOR) &&//food is in sensing area
					_food[j].transport == false)
				{
					upd_info.state = PUSHING_F;
					upd_info.mem_foodpos.x = _food[j].pos.x;
					upd_info.mem_foodpos.y = _food[j].pos.y;
					upd_info.mem_foodID = j;
					upd_info.time_pushinglead = 0;
					upd_info.food_info.detect = true;//judge_food��ʂ�Ȃ��ꍇ������
					upd_info.food_info.id = j;
				}
			}
		}
	}
	return upd_info;
}

Agent _leading2homing(vector<Prey> _food, int num_food, Agent my_info)
{
	Agent upd_info = my_info;
	upd_info.food_info = judge_near_food(my_info._posori, _food, num_food);
	double dist2prey = hypot(my_info.mem_foodpos.x - my_info._posori.pos.x, my_info.mem_foodpos.y - my_info._posori.pos.y);
	if (dist2prey < (RADIUS_SENSOR + RADIUS_FOOD) &&
		upd_info.food_info.detect == false)
	{
		upd_info.state = HOMING;
	}
	return upd_info;
}

Agent _leading2searching(vector<Prey> _food, int num_food, Agent my_info)
{
	Agent upd_info = my_info;
	upd_info.food_info = judge_near_food(my_info._posori, _food, num_food);
	double dist2prey = hypot(my_info.mem_foodpos.x - my_info._posori.pos.x, my_info.mem_foodpos.y - my_info._posori.pos.y);
	if (dist2prey < (RADIUS_SENSOR + RADIUS_FOOD) &&
		upd_info.food_info.detect == false)
	{
		upd_info.state = SEARCHING;
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.time_searching = 0;//�T�����Ԃ̃��Z�b�g
		upd_info.dist_straight = 0;//���ώ��R�H���̃��Z�b�g
	}
	return upd_info;
}

Agent _pushinglead2hominglead(Agent my_info, Prey food)
{
	Agent upd_info = my_info;
	if (my_info.time_pushinglead >= MAXTIME_PUSHINGLEAD &&
		food.transport == false)
	{
		upd_info.state = HOMING_F;
	}
	return upd_info;
}

Agent _hominglead2recruiting(Agent my_info)
{
	Agent upd_info = my_info;
	double dist2nest = hypot(my_info._posori.pos.x, my_info._posori.pos.y);

	if (dist2nest < RADIUS_N)//���ɓ���
	{
		upd_info.state = RECRUITING;
		upd_info.time_recruiting = 0;//���N���[�g���Ԃ̃��Z�b�g
		upd_info.dist_straight = 0;//���ώ��R�H���̃��Z�b�g
	}
	return upd_info;
}

Agent _pushinglead2homing(Agent my_info, Prey food)
{
	Agent upd_info = my_info;
	if (food.inside_nest == true)
	{
		upd_info.state = HOMING;
	}
	return upd_info;
}

Agent _pushinglead2searching(Agent my_info, Prey food)
{
	Agent upd_info = my_info;
	if (food.inside_nest == true)
	{
		upd_info.state = SEARCHING;
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.time_searching = 0;//�T�����Ԃ̃��Z�b�g
		upd_info.dist_straight = 0;//���ώ��R�H���̃��Z�b�g
	}
	return upd_info;
}

Agent _pushinglead2recruiting_out(Agent my_info, Prey food)
{
	Agent upd_info = my_info;
	if (my_info.time_pushinglead >= MAXTIME_PUSHINGLEAD &&
		food.transport == false)
	{
		upd_info.state = RECRUITING_S;
		upd_info.time_recruiting = 0;//���N���[�g���Ԃ̃��Z�b�g
		upd_info.dist_straight = 0;//���ώ��R�H���̃��Z�b�g
	}
	return upd_info;
}

/*****************Transition****************************************/

Agent agent_state_transition_rule(vector<Prey> _food, int num_food, Agent my_info, vector<Agent> _agent, int num_agent)
{
	Agent upd_info = my_info;
	switch (my_info.state)
	{
	case RESTING:
		upd_info = _resting2searching(upd_info);	//resting��searchig�̏��������Ă͂܂邩�`�F�b�N
		upd_info = _resting2following(_agent, num_agent, upd_info);	//resting��followig�̏��������Ă͂܂邩�`�F�b�N
		break;
	case SEARCHING:
		upd_info = _searching2pushing(_food, num_food, upd_info, _agent, num_agent);//pushing:pushing(F) and pushing
		upd_info = _searching2homing(upd_info);
		break;
	case PUSHING_F:
		upd_info = _pushinglead2homing(upd_info, _food[upd_info.food_info.id]);
		upd_info = _pushinglead2hominglead(upd_info, _food[upd_info.food_info.id]);
		break;
	case PUSHING:
		upd_info = _pushinglead2homing(upd_info, _food[upd_info.food_info.id]);
		break;
	case HOMING:
		upd_info = _homing2resting(upd_info);
		break;
	case RECRUITING:
		upd_info = _recruiting2leading(upd_info);
		break;
	case FOLLOWING:
		upd_info = _following2pushing(_food, num_food, upd_info, _agent, num_agent);
		upd_info = _following2searching(upd_info, _agent[upd_info.leaderID]);
		upd_info = _following2homing(upd_info, _agent[upd_info.leaderID]);
		break;
	case LEADING:
		upd_info = _leading2pushinglead(_food, num_food, upd_info, _agent, num_agent);
		upd_info = _leading2homing(_food, num_food, upd_info);
		break;
	case HOMING_F:
		upd_info = _hominglead2recruiting(upd_info);
		break;
	default:
		break;
	}
	return upd_info;
}

Agent agent_conventional_state_transition_rule(vector<Prey> _food, int num_food, Agent my_info, vector<Agent> _agent, int num_agent)
{
	Agent upd_info = my_info;
	switch (my_info.state)
	{
	case SEARCHING:
		upd_info = _searching2pushing(_food, num_food, upd_info, _agent, num_agent);
		upd_info = _searching2following(_agent, num_agent, upd_info);
		break;
	case PUSHING_F:
		upd_info = _pushinglead2searching(upd_info, _food[upd_info.food_info.id]);
		upd_info = _pushinglead2recruiting_out(upd_info, _food[upd_info.food_info.id]);
		break;
	case PUSHING:
		upd_info = _pushinglead2searching(upd_info, _food[upd_info.food_info.id]);
		break;
	case RECRUITING_S:
		upd_info = _recruiting2leading(upd_info);
		break;
	case FOLLOWING:
		upd_info = _following2pushing(_food, num_food, upd_info, _agent, num_agent);
		upd_info = _following2searching2(upd_info, _agent[upd_info.leaderID]);
		break;
	case LEADING:
		upd_info = _leading2pushinglead(_food, num_food, upd_info, _agent, num_agent);
		upd_info = _leading2searching(_food, num_food, upd_info);
		break;
	default:
		break;
	}
	return upd_info;
}