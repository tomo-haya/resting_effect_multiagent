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
//description:アリの内部状態をすべて記憶する
/*!
*/
//==============================================================//
class Agent {
public:
	Agent()
	{
		initialize_agent();
	};

	void initialize_agent(void)//各状態のリセット
	{
		time_searching = 0;
		time_resting = 0;
		time_pushinglead = 0;
		time_recruiting = 0;
		dist_straight = 0;
	};

	double r0, time_resting0, theta0, ang_r0;//読み込んだ初期状態
	posori _posori;
	xy vel;
	xy mem_foodpos;
	int state;					//内部状態		どうなっているかは「Agent_State_Behavior.cpp」参照
	double time_searching;			//探索時間(餌が見つかっていない期間)[sec]
	double time_resting;			//休息時間[sec]
	double time_pushinglead;		//followerが全てpushingに状態遷移するまでの同期用
	double time_recruiting;		//リクルートした時間
	int maxtime_rest;			//各ロボットごとに最大休息時間を定義
	bool near_food;				//searching state内の行動を決定
	int leaderID;
	double dist_straight;					//現在の直進した距離(平均自由工程を考慮した際に使用)
	int consume_energy;
	int collision;
	int col_foodID;//Food ID with collision
	int mem_foodID;//Food ID with collision
	food_detect food_info;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		餌に関する構造体
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


//=============================================================//
//name:Prey
//description:各餌に関する情報を記憶する
/*!
*	@x,y				餌の位置
*	@grid_x,grid_y		グリッドにより表される餌の位置
*	@dx,dy				餌の位置の時間差分
*	@mass				餌の重量
*	@state				餌の状態
*	@internal_nest		巣の内部かどうか
*	@agent				どのエージェントに運搬されているか
*/
//==============================================================//

class Prey {
public:
	xy pos;
	xy vel;//velocity of Leader is used by follower
	double mass;
	double r0, theta0;//読み込んだ初期状態
	double mass0;
	bool transport;//初期位置から動いたかどうか(動いた餌はロボットに発見されない)
	bool inside_nest;//巣内に運搬されたかどうか
	int num_grabbing;
	int collision;//food-food collision
	int num_found;//Searchingロボットから発見された回数
	int num_recruit;//動員回数
	int transport_time;//発見されてから運ばれるまでの時間
	int find_time;//最初に発見された時間
	double recruit_spending;		//発見したロボットがリクルートにかけた総時間
	int recruit_count;			//発見したロボットがリクルートした回数
	int recruit_agentnum;			//発見したロボットがリクルートしたロボット数
	int first_agentID;		//最初に発見したロボットID
	int firstID_recruit_count;			//最初に発見したロボットがリクルートした回数
	int num_recruit_success;//Followingロボットを餌に連れてきたロボット数
	int recruit_success_ID[100];//Followingロボットを餌に連れてきたロボットID
};



//配列（実計算）

extern vector<Prey> prey;
extern vector<Prey> pastprey;
extern vector<Agent> agent;
extern vector<Agent> pastagent;

extern int num_initprey;
extern int num_agent;
extern int recruit_time;
extern int n_max;

#endif


