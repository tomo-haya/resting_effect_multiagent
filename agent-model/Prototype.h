#include "struct.h"

//Agent
Agent agent_state_transition_rule(vector<Prey> _food, int num_food, Agent my_info, vector<Agent> _agent, int num_agent);	//èÛë‘ëJà⁄ãKë•
Agent agent_conventional_state_transition_rule(vector<Prey> _food, int num_food, Agent my_info, vector<Agent> _agent, int num_agent);
Agent agent_state_behavior2(vector<Prey> _food, int num_food, Agent my_info, vector<Agent> _agent, int num_agent, int i);

//Behavior
posori _follow_leader_vel(xy my_pos, xy target_speed, int my_collision, double speed);
Agent _follow_leader_vel2(Agent my_info, xy target_speed, double speed);
posori _follow_leader_pos(xy my_pos, xy target_pos, double speed, int my_collision);
posori _follow_leader_posvel(xy my_pos, xy target_pos, xy target_vel, double speed, int my_collision, double weight_vel);
food_detect judge_near_food(posori my_posori, vector<Prey> food, int num_food);
posori food_transport_agent(xy my_pos, xy food_pos);
Agent _return_nest(Agent my_info, double speed);
Agent _approach2food(vector<Agent> _agent, Agent my_info, xy food_pos, int my_id, int num_agent, double speed);
Agent _searcharea_random_walk(Agent my_info, double speed);
Agent _nest_random_walk(Agent my_info, double speed);
Agent approach2mem_food(Agent my_info, double speed);
food_detect judge_near_food(posori my_posori, vector<Prey> food, int num_food);

//Initial
void init(void);
void init_agent(void);
void init_prey(void);
void read_preydata(int i);
void read_agentdata(void);
void read_preydata_weight(int i);

//Display
void display(void);

// Others
void init_genrand(unsigned long s);