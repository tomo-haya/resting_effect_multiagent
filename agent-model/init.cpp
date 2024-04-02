#include "Prototype.h"
#include "Def-Physics.h"
#include "struct.h"
#include "Struct_Def.h"
#include <stdio.h>
#include <stdlib.h>
#include "glut.h"
#include "mt19937ar.h"
#include <math.h>
#include <time.h>


#include <vector>
#include <iostream>
#include <sstream>

using namespace std;

FILE* Agent_p;
FILE* Object_p;

void read_agentdata(void)
{
	Agent_p = fopen("initagent.dat", "r");//������Ԃ̓������t�@�C���������

	for (int i = 0;i < num_agent;i++)
	{
		fscanf_s(Agent_p, "%lf %lf %lf %lf",&agent[i].r0, &agent[i].theta0, &agent[i].ang_r0, &agent[i].time_resting0);
		//cout << " agent= " << agent[i].rest_distance0 << " theta= " << agent[i].theta0 << " ang_r0= " << agent[i].ang_r0 << " time_resting0= " << agent[i].time_resting0 << endl;
	}
	fclose(Agent_p);
}

void init_agent(void)
{	
	for(int i=0;i< num_agent;i++)
	{
		agent[i].time_resting = agent[i].time_resting0;
		agent[i]._posori.pos.x = agent[i].r0 * cos(agent[i].theta0);
		agent[i]._posori.pos.y = agent[i].r0 * sin(agent[i].theta0);
		agent[i]._posori.ori = agent[i].theta0;
		agent[i].time_pushinglead = 0;
		agent[i].time_searching = 0;
		agent[i].time_recruiting = 0;
		agent[i].leaderID = i;
		agent[i].food_info.detect = false;
		agent[i].food_info.id = -1;
		if (CASE == 1 || CASE == 2 || CASE == 3 || CASE == 4 || CASE == 100) agent[i].state = RESTING;
		if (CASE == 11 || CASE == 14 || CASE == 101) agent[i].state = SEARCHING;
		agent[i].consume_energy = 0;
	}
	pastagent = agent;
}

void read_preydata(int i)
{	
	std::string str = "initprey.dat";
	ostringstream ss;
	ss << i;
	str = ss.str() + str;
	const char *read_name = str.c_str();
	Object_p = fopen(read_name, "r");

	for (int i = 0; i < num_initprey; i++)
	{
		fscanf_s(Object_p, "%lf %lf %lf", &prey[i].r0, &prey[i].theta0, &prey[i].mass0);
	}
	fclose(Object_p);
}

void read_preydata_weight(int i)
{
	std::string str = "initprey_weight.dat";
	ostringstream ss;
	ss << i;
	str = ss.str() + str;
	const char* read_name = str.c_str();
	Object_p = fopen(read_name, "r");

	for (int i = 0; i < num_initprey; i++)
	{
		fscanf_s(Object_p, "%lf %lf %lf", &prey[i].r0, &prey[i].theta0, &prey[i].mass0);
	}
	fclose(Object_p);
}

void init_prey(void)
{
	//��l�ɔz�u
	for(int i = 0 ; i < num_initprey; i ++)
	{
		prey[i].mass = prey[i].mass0;
		prey[i].pos.x = prey[i].r0 * cos(prey[i].theta0);
		prey[i].pos.y = prey[i].r0 * sin(prey[i].theta0);
		prey[i].transport = false;
		prey[i].inside_nest = false;
		prey[i].num_grabbing = 0;
	}
	pastprey = prey;
}

void init_setting(void)
{
	//���Ɣ��˂̏�����
	GLfloat light_position0[] = {0.0,	0.0,	500.0,	0.0};//�����ʒu�i�P�j
	GLfloat light_position1[] = {0.0 , 	0.0,	500.0,	0.0};//�����ʒu�i�Q�j

	//Window��h��Ԃ����̐F�̎w��
	glClearColor(1.0, 1.0, 1.0, 0.0);
	
	//�`��
	glShadeModel(GL_SMOOTH);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

	//��
	glEnable(GL_LIGHTING);						//����������悤�ɂ���
	glEnable(GL_LIGHT0);						//���C�g0��L���ɂ���
	glEnable(GL_LIGHT1);						//���C�g1��L���ɂ���
	glEnable(GL_COLOR_MATERIAL);				//�ގ��p�����[�^�����݂̃J���[�ɒǏ]
	
	//�B�ʏ���
	glEnable(GL_DEPTH_TEST);					//�f�v�X�e�X�g��L���ɂ���
	glDepthFunc(GL_LEQUAL);
	
	//��������
	glEnable(GL_BLEND);							//��������
		
	//�@���̎Z�o
	glEnable(GL_NORMALIZE);						//�@���̐��K��
}



void init(void)
{
	init_setting();//�`��

	switch (CASE) 
	{
	case 1:
		recruit_time = MIN_TIME_RECRUIT;
		num_agent = TYPIC_NUM_AGENT;
		num_initprey = TYPIC_NUM_INIT_PREY;
		n_max = ATTEMPT * ((MAX_TIME_RECRUIT - MIN_TIME_RECRUIT) / DIFF_TIME_RECRUIT_CHANGE + 1);
		break;
	case 2:
		recruit_time = TYPIC_TIME_RECRUIT;
		num_agent = MIN_NUM_AGENT;
		num_initprey = TYPIC_NUM_INIT_PREY;
		n_max = ATTEMPT * ((MAX_NUM_AGENT - MIN_NUM_AGENT) / DIFF_NUM_AGENT_CHANGE + 1);
		break;
	case 3:
		recruit_time = TYPIC_TIME_RECRUIT;
		num_agent = TYPIC_NUM_AGENT;
		num_initprey = MIN_NUM_INIT_PREY;
		n_max = ATTEMPT * ((MAX_NUM_INIT_PREY- MIN_NUM_INIT_PREY) / DIFF_NUM_INIT_PREY_CHANGE + 1);
		break;
	case 4:
		recruit_time = MIN_TIME_RECRUIT;
		num_agent = TYPIC_NUM_AGENT;
		num_initprey = TYPIC_NUM_INIT_PREY;
		n_max = ATTEMPT * ((MAX_TIME_RECRUIT - MIN_TIME_RECRUIT) / DIFF_TIME_RECRUIT_CHANGE + 1);
		break;
	case 11:
		recruit_time = MIN_TIME_RECRUIT;
		num_agent = TYPIC_NUM_AGENT;
		num_initprey = TYPIC_NUM_INIT_PREY;
		n_max = ATTEMPT * ((MAX_TIME_RECRUIT - MIN_TIME_RECRUIT) / DIFF_TIME_RECRUIT_CHANGE + 1);
		break;
	case 14:
		recruit_time = MIN_TIME_RECRUIT;
		num_agent = TYPIC_NUM_AGENT;
		num_initprey = TYPIC_NUM_INIT_PREY;
		n_max = ATTEMPT * ((MAX_TIME_RECRUIT - MIN_TIME_RECRUIT) / DIFF_TIME_RECRUIT_CHANGE + 1);
		break;
	case 100:
		recruit_time = MIN_TIME_RECRUIT;
		num_agent = MIN_NUM_AGENT;
		num_initprey = MIN_NUM_INIT_PREY;
		n_max = ATTEMPT * ((MAX_TIME_RECRUIT - MIN_TIME_RECRUIT) / DIFF_TIME_RECRUIT_CHANGE + 1) * ((MAX_NUM_INIT_PREY - MIN_NUM_INIT_PREY) / DIFF_NUM_INIT_PREY_CHANGE + 1) * ((MAX_NUM_AGENT - MIN_NUM_AGENT) / DIFF_NUM_AGENT_CHANGE + 1);
		break;
	case 101:
		recruit_time = MIN_TIME_RECRUIT;
		num_agent = MIN_NUM_AGENT;
		num_initprey = MIN_NUM_INIT_PREY;
		n_max = ATTEMPT * ((MAX_TIME_RECRUIT - MIN_TIME_RECRUIT) / DIFF_TIME_RECRUIT_CHANGE + 1) * ((MAX_NUM_INIT_PREY - MIN_NUM_INIT_PREY) / DIFF_NUM_INIT_PREY_CHANGE + 1) * ((MAX_NUM_AGENT - MIN_NUM_AGENT) / DIFF_NUM_AGENT_CHANGE + 1);
		break;
	}

	for (int i = 0;i < MAX_NUM_AGENT;i++) agent.push_back(Agent());//MAX_NUM_AGENT�̕����m�ۂ��Ă���
	for (int i = 0;i < MAX_NUM_AGENT;i++) pastagent.push_back(Agent());//MAX_NUM_AGENT�̕����m�ۂ��Ă���
	for (int i = 0;i < MAX_NUM_INIT_PREY;i++) prey.push_back(Prey());//num_initprey�����m�ۂ��Ă���
	for (int i = 0;i < MAX_NUM_INIT_PREY;i++) pastprey.push_back(Prey());//num_initprey�����m�ۂ��Ă���

}