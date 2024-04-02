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
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.dist_straight = 0;//���i�������Z�b�g
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

posori _follow_leader_pos(xy my_pos, xy target_pos, double speed, int my_collision)
{
	posori upd_posori;
	double dist2robot = hypot(target_pos.x - my_pos.x, target_pos.y - my_pos.y);//calculate robot-robot distance

	if (my_collision == 1 || my_collision == 2 || my_collision == 3 || my_collision == 4)
	{
		upd_posori.ori = 2 * M_PI * genrand_real1();
		upd_posori.pos.x = my_pos.x + speed * DT * cos(upd_posori.ori);
		upd_posori.pos.y = my_pos.y + speed * DT * sin(upd_posori.ori);
	}
	else
	{
		upd_posori.ori = atan2(target_pos.y- my_pos.y, target_pos.x - my_pos.x);//rotate to leader-vel direction
		upd_posori.pos.x = my_pos.x + min(speed * DT, dist2robot - LF_DIST_DESIRE) * cos(upd_posori.ori);
		upd_posori.pos.y = my_pos.y + min(speed * DT, dist2robot - LF_DIST_DESIRE) * sin(upd_posori.ori);
	}
	return upd_posori;
}

posori _follow_leader_posvel(xy my_pos, xy target_pos, xy target_vel, double speed, int my_collision, double weight_vel)
{
	posori upd_posori;
	xy component_vel;
	xy component_pos;
	xy component_add;

	if (my_collision == 1 || my_collision == 2 || my_collision == 3 || my_collision == 4)
	{
		upd_posori.ori = 2 * M_PI * genrand_real1();
		upd_posori.pos.x = my_pos.x + speed * DT * cos(upd_posori.ori);
		upd_posori.pos.y = my_pos.y + speed * DT * sin(upd_posori.ori);
	}
	else
	{
		component_vel.x = target_vel.x;
		component_vel.y = target_vel.y;

		double dist2robot = hypot(target_pos.x - my_pos.x, target_pos.y - my_pos.y);//calculate robot-robot distance
		double temp_ori = atan2(target_pos.y - my_pos.y, target_pos.x - my_pos.x);
		component_pos.x = (dist2robot - LF_DIST_DESIRE) * cos(temp_ori);
		component_pos.y = (dist2robot - LF_DIST_DESIRE) * sin(temp_ori);

		component_add.x = component_pos.x * (1.0 - weight_vel) + component_vel.x * weight_vel;
		component_add.y = component_pos.y * (1.0 - weight_vel) + component_vel.y * weight_vel;

		upd_posori.ori = atan2(component_add.y, component_add.x);
		upd_posori.pos.x = my_pos.x + min(speed * DT, hypot(component_add.x, component_add.y) * DT) * cos(upd_posori.ori);
		upd_posori.pos.y = my_pos.y + min(speed * DT, hypot(component_add.x, component_add.y) * DT) * sin(upd_posori.ori);
	}
	return upd_posori;
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
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.dist_straight = 0;//���i�������Z�b�g
	}
	upd_info.dist_straight += speed * DT;

	if (upd_info.dist_straight >= MEAN_FREE_PATH_COLL)
	{
		upd_info._posori.ori = ang_robot2prey;//�a�̕����Ɍ���
		upd_info._posori.pos.x += min({ speed * DT, dist2prey - RADIUS_FOOD - RADIUS_AGENT, move_dist }) * cos(upd_info._posori.ori);	//x����.���{�b�g���aor���{�b�g�ɂԂ�������~�܂�
		upd_info._posori.pos.y += min({ speed * DT, dist2prey - RADIUS_FOOD - RADIUS_AGENT, move_dist }) * sin(upd_info._posori.ori);	//y����.���{�b�g���aor���{�b�g�ɂԂ�������~�܂�
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

	double ang_robot2prey = atan2(upd_info.mem_foodpos.y - upd_info._posori.pos.y, upd_info.mem_foodpos.x - upd_info._posori.pos.x);//�܂��A���̕��������肷��

	if (upd_info.collision == 1 || upd_info.collision == 2 || upd_info.collision == 3 || upd_info.collision == 4)
	{
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.dist_straight = 0;//���i�������Z�b�g
	}
	upd_info.dist_straight += speed * DT;

	if (upd_info.dist_straight >= MEAN_FREE_PATH_COLL) upd_info._posori.ori = ang_robot2prey;//�a�̕����Ɍ���

	//agent�̈ʒu�����肷��
	double dist2prey = hypot(upd_info.mem_foodpos.x - upd_info._posori.pos.x, upd_info.mem_foodpos.y - upd_info._posori.pos.y);
	upd_info._posori.pos.x += min(speed * DT, dist2prey - RADIUS_FOOD - RADIUS_AGENT) * cos(upd_info._posori.ori);	//x����.���{�b�g���a�ɂԂ�������~�܂�
	upd_info._posori.pos.y += min(speed * DT, dist2prey - RADIUS_FOOD - RADIUS_AGENT) * sin(upd_info._posori.ori);	//y����

	return upd_info;
}

posori food_transport_agent(xy my_pos, xy food_pos)
{
	posori upd_posori;
	upd_posori.ori = atan2(-food_pos.y, -food_pos.x);	//determine moving direction of food

	upd_posori.pos.x = my_pos.x + TRANSPORT_SPEED * DT * cos(upd_posori.ori);	//x����.
	upd_posori.pos.y = my_pos.y + TRANSPORT_SPEED * DT * sin(upd_posori.ori);	//y����.

	return upd_posori;
}

Agent _return_nest(Agent my_info, double speed)
{
	Agent upd_info = my_info;

	double ang_robot2prey = atan2(-upd_info._posori.pos.y, -upd_info._posori.pos.x);//�܂��A���̕��������肷��

	if (upd_info.collision == 1 || upd_info.collision == 2 || upd_info.collision == 3 || upd_info.collision == 4)
	{
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.dist_straight = 0;//���i�������Z�b�g
	}
	upd_info.dist_straight += speed * DT;

	if (upd_info.dist_straight >= MEAN_FREE_PATH_COLL) upd_info._posori.ori = ang_robot2prey;//�a�̕����Ɍ���

	//�A���̈ʒu�����肷��	
	double dist2nest = hypot(upd_info._posori.pos.x, upd_info._posori.pos.y);
	upd_info._posori.pos.x += min((double)MOVING_SPEED * DT, dist2nest) * cos(upd_info._posori.ori);	//x����.���̒��S�����s����������~�܂�
	upd_info._posori.pos.y += min((double)MOVING_SPEED * DT, dist2nest) * sin(upd_info._posori.ori);

	return upd_info;
}

Agent _searcharea_random_walk(Agent my_info, double speed)
{
	Agent upd_info = my_info;

	if ((upd_info.dist_straight >= MEAN_FREE_PATH && hypot(upd_info._posori.pos.x, upd_info._posori.pos.y) > RADIUS_N) ||//�T���G���A���\�����ior
		(hypot(upd_info._posori.pos.x, upd_info._posori.pos.y) > RADIUS_N &&
		hypot(upd_info._posori.pos.x + speed * DT * cos(upd_info._posori.ori), upd_info._posori.pos.y + speed * DT * sin(upd_info._posori.ori)) < RADIUS_N) ||//�ړ��O�ɑ��O�ɂ��āA�Ȃ������ݎp���ł͈ړ���ɐ����ɓ���
		upd_info.collision == 1 || upd_info.collision == 2 || upd_info.collision == 3 || upd_info.collision == 4)//collision
	{
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.dist_straight = 0;//���i�������Z�b�g
	}
	upd_info.dist_straight += speed * DT;

	if (hypot(upd_info._posori.pos.x, upd_info._posori.pos.y) > RADIUS_N &&
		hypot(upd_info._posori.pos.x + speed * DT * cos(upd_info._posori.ori), upd_info._posori.pos.y + speed * DT * sin(upd_info._posori.ori)) < RADIUS_N)
	{//�ړ��O�ɑ��O�ɂ��āA�Ȃ����ړ���ɐ����ɓ���
		//upd_info._posori.pos = upd_info._posori.pos;
	}
	else
	{//���O��ʏ�ړ��܂��͈ړ��O�ɑ����ɂ���
		upd_info._posori.pos.x += speed * DT * cos(upd_info._posori.ori);	//�����Ă�������ɐi��(x)
		upd_info._posori.pos.y += speed * DT * sin(upd_info._posori.ori);	//�����Ă�������ɐi��(y)
	}
	return upd_info;
}

Agent _nest_random_walk(Agent my_info, double speed)//�����_���v�f���܂ނ��ߏՓˌ�̃����_���p���ύX�͓���Ă��Ȃ�
{
	Agent upd_info = my_info;

	//�܂��A���̕��������肷��
	if (upd_info.dist_straight >= MEAN_FREE_PATH ||//�\�����ior
		hypot(upd_info._posori.pos.x + speed * DT * cos(upd_info._posori.ori), upd_info._posori.pos.y + speed * DT * sin(upd_info._posori.ori)) > RADIUS_N ||//���ݎp���ł͈ړ���ɑ��̊O�ɏo��
		upd_info.collision == 1 || upd_info.collision == 2 || upd_info.collision == 3 || upd_info.collision == 4)
	{
		upd_info._posori.ori = 2 * M_PI * genrand_real1();		//�����������_���Ɍ���
		upd_info.dist_straight = 0;//���i�������Z�b�g
	}
	upd_info.dist_straight += speed * DT;

	//�A���̈ʒu�����肷��	
	if (hypot(upd_info._posori.pos.x + speed * DT * cos(upd_info._posori.ori), upd_info._posori.pos.y + speed * DT * sin(upd_info._posori.ori)) > RADIUS_N)
	{//�ړ���ɑ��̊O�ɏo��
		upd_info._posori.pos.x += 0;
		upd_info._posori.pos.y += 0;
	}
	else
	{
		upd_info._posori.pos.x += speed * DT * cos(upd_info._posori.ori);	//�����_�������ɐi��(x)
		upd_info._posori.pos.y += speed * DT * sin(upd_info._posori.ori);	//�����_�������ɐi��(y)
	}

	return upd_info;
}