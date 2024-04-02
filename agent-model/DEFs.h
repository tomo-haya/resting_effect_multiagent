
#include "Def-Physics.h"

int t;
int step;
int n = 1;
int n_max;
double pasttime;
double nowtime;

int num_initprey;
int num_agent;
int recruit_time;

//配列（実計算）

vector<Agent> agent;
vector<Agent> pastagent;
vector<Prey> prey;
vector<Prey> pastprey;

vector<int> vect{ 1, 2, 3 };

clock_t start,ender;

