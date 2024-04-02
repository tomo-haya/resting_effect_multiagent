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
//description:�A���̓�����Ԃ����ׂċL������
/*!
*/
//==============================================================//
class Agent {
public:
	Agent()
	{
		initialize_agent();
	};

	void initialize_agent(void)//�e��Ԃ̃��Z�b�g
	{
		time_searching = 0;
		time_resting = 0;
		time_pushinglead = 0;
		time_recruiting = 0;
		dist_straight = 0;
	};

	double r0, time_resting0, theta0, ang_r0;//�ǂݍ��񂾏������
	posori _posori;
	xy vel;
	xy mem_foodpos;
	int state;					//�������		�ǂ��Ȃ��Ă��邩�́uAgent_State_Behavior.cpp�v�Q��
	double time_searching;			//�T������(�a���������Ă��Ȃ�����)[sec]
	double time_resting;			//�x������[sec]
	double time_pushinglead;		//follower���S��pushing�ɏ�ԑJ�ڂ���܂ł̓����p
	double time_recruiting;		//���N���[�g��������
	int maxtime_rest;			//�e���{�b�g���Ƃɍő�x�����Ԃ��`
	bool near_food;				//searching state���̍s��������
	int leaderID;
	double dist_straight;					//���݂̒��i��������(���ώ��R�H�����l�������ۂɎg�p)
	int consume_energy;
	int collision;
	int col_foodID;//Food ID with collision
	int mem_foodID;//Food ID with collision
	food_detect food_info;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		�a�Ɋւ���\����
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


//=============================================================//
//name:Prey
//description:�e�a�Ɋւ�������L������
/*!
*	@x,y				�a�̈ʒu
*	@grid_x,grid_y		�O���b�h�ɂ��\�����a�̈ʒu
*	@dx,dy				�a�̈ʒu�̎��ԍ���
*	@mass				�a�̏d��
*	@state				�a�̏��
*	@internal_nest		���̓������ǂ���
*	@agent				�ǂ̃G�[�W�F���g�ɉ^������Ă��邩
*/
//==============================================================//

class Prey {
public:
	xy pos;
	xy vel;//velocity of Leader is used by follower
	double mass;
	double r0, theta0;//�ǂݍ��񂾏������
	double mass0;
	bool transport;//�����ʒu���瓮�������ǂ���(�������a�̓��{�b�g�ɔ�������Ȃ�)
	bool inside_nest;//�����ɉ^�����ꂽ���ǂ���
	int num_grabbing;
	int collision;//food-food collision
	int num_found;//Searching���{�b�g���甭�����ꂽ��
	int num_recruit;//������
	int transport_time;//��������Ă���^�΂��܂ł̎���
	int find_time;//�ŏ��ɔ������ꂽ����
	double recruit_spending;		//�����������{�b�g�����N���[�g�ɂ�����������
	int recruit_count;			//�����������{�b�g�����N���[�g������
	int recruit_agentnum;			//�����������{�b�g�����N���[�g�������{�b�g��
	int first_agentID;		//�ŏ��ɔ����������{�b�gID
	int firstID_recruit_count;			//�ŏ��ɔ����������{�b�g�����N���[�g������
	int num_recruit_success;//Following���{�b�g���a�ɘA��Ă������{�b�g��
	int recruit_success_ID[100];//Following���{�b�g���a�ɘA��Ă������{�b�gID
};



//�z��i���v�Z�j

extern vector<Prey> prey;
extern vector<Prey> pastprey;
extern vector<Agent> agent;
extern vector<Agent> pastagent;

extern int num_initprey;
extern int num_agent;
extern int recruit_time;
extern int n_max;

#endif


