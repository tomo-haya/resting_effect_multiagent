#include "Prototype.h"
#include "Def-Physics.h"
#include "Struct_Def.h"
#include "struct.h"
#include <stdlib.h>
#include "glut.h"
#include <stdio.h>
#include <math.h>
#include <vector>
#include <sstream>
#define _USE_MATH_DEFINES
#include <math.h>

void draw_field2(void)
{
	glDisable(GL_LIGHTING);//glColor3d()で色をつけるため，ライティング処理をオフにする(不要かも)
	glColor4d(0.0, 0.0, 0.0, 1);//Black
	glTranslated(RADIUS_F ,RADIUS_F, 0);	
	
	glBegin(GL_LINE_LOOP);
	int edgenum = 50;
	for(int d = 0; d <= edgenum; d++) glVertex2d( RADIUS_F * cos((double)d*2.0*M_PI/edgenum), RADIUS_F * sin((double)d * 2.0 * M_PI / edgenum));//draw circle as a field

	glEnd();
	glEnable(GL_LIGHTING);//ライティング処理をオンにする(不要かも)
}


void draw_nest3(void)
{
	glDisable(GL_LIGHTING);//glColor3d()で色をつけるため，ライティング処理をオフにする(不要かも)
	glColor4d(0.0, 0.0, 0.0, 1);//Black
	glTranslated(0, 0, 0);	

	glBegin(GL_LINE_LOOP);
	int edgenum = 50;
	for(int d = 0; d <= edgenum; d++) glVertex2d( RADIUS_N * cos((double)d*2.0*M_PI/edgenum), RADIUS_N * sin((double)d * 2.0 * M_PI / edgenum));//draw circle as a nest

	glEnd();
	glEnable(GL_LIGHTING);//ライティング処理をオンにする(不要かも)
}



void draw_agent(void)
{
	int flag_show = 0;

	for (int i = 0;i < num_agent;i++)
	{
		switch (agent[i].state)
		{
		case RESTING:
			glColor4d(0.0, 1.0, 0.0, 1.0);//GREEN
			flag_show = 1;
			break;
		case SEARCHING:
			glColor4d(1.0, 0.0, 0.0, 1.0);//RED
			flag_show = 1;
			break;
		case PUSHING_F:
			glColor4d(0.0, 0.0, 1.0, 1.0);//BLUE
			flag_show = 1;
			break;
		case PUSHING:
			glColor4d(0.0, 0.0, 1.0, 1.0);//BLUE
			//glColor4d(0.7, 0.15, 0.15, 1.0);//BROWN
			flag_show = 1;
			break;
		case TRANSPORTING:
			//glColor4d(0.7, 0.15, 0.15, 1.0);//BROWN
			glColor4d(0.5, 0.0, 1.0, 1.0);//PURPLE
			flag_show = 1;
			break;
		case HOMING:
			glColor4d(0.0, 1.0, 1.0, 1.0);//Cyan
			flag_show = 1;
			break;
		case RECRUITING:
			glColor4d(0.5, 0.0, 1.0, 1.0);//PURPLE
			flag_show = 1;
			break;
		case RECRUITING_S:
			glColor4d(0.5, 0.0, 1.0, 1.0);//PURPLE
			flag_show = 1;
			break;
		case HOMING_F:
			glColor4d(0.0, 1.0, 1.0, 1.0);//Cyan
			flag_show = 1;
			break;
		case FOLLOWING://RECRUITINGやLEADINGと重なるので表示しない
			glColor4d(0.0, 0.4, 0.0, 1.0);//DARK GREEN
			flag_show = 1;
			break;
		case LEADING:
			glColor4d(1.0, 0.5, 0.0, 1.0);//ORANGE
			flag_show = 1;
			break;
		}

		if (flag_show)
		{
			glPushMatrix();//巣中心位置などのモデルビュー変換行列の保存
			//glTranslated(agent[i].x, agent[i].y, 0);	//position	
			glTranslated(agent[i]._posori.pos.x, agent[i]._posori.pos.y, 0);	//position	
			//glRotated(agent[i].ang_d, 0.0, 0.0, 1.0);	//posture(yaw)	
			glScaled(RADIUS_AGENT, RADIUS_AGENT, 1);//size
			//glEnable(GL_NORMALIZE);

			glBegin(GL_POLYGON);
			int edgenum = 40;
			for (int d = 0; d <= edgenum; d++) glVertex2d((double)cos((double)d * 2.0 * M_PI / edgenum), (double)sin((double)d * 2.0 * M_PI / edgenum));//draw circle as an agent
			glEnd();
			//glDisable(GL_NORMALIZE);
			glPopMatrix();//巣中心位置などのモデルビュー変換行列の取出し
		}
	}
}

void draw_prey(void)
{
	for (int p_ID=0;p_ID< num_initprey;p_ID++)
	{
		double scale_prey = 1;

		//if (prey[p_ID].transport == false)
		if (prey[p_ID].inside_nest == false)
		{
			glColor4d(1.0, 0.0, 1.0, 1.0);//R,G,B,alpha

			glPushMatrix();//巣中心位置などのモデルビュー変換行列の保存
			glTranslated(prey[p_ID].pos.x, prey[p_ID].pos.y, 0);
			glScaled(RADIUS_FOOD, RADIUS_FOOD, 1);
			//glEnable(GL_NORMALIZE);

			glBegin(GL_POLYGON);//点集合から、凸多角形を構築
			int edgenum = 40;
			for (int d = 0; d <= edgenum; d++) glVertex2d((double)cos((double)d * 2.0 * M_PI / edgenum), (double)sin((double)d * 2.0 * M_PI / edgenum));
			glEnd();
			//glDisable(GL_NORMALIZE);
			glPopMatrix();//巣中心位置などのモデルビュー変換行列の取出し
		}
		//else glColor4d(0.0, 0.0, 0.0, 1.0);//表示しない
	}
}


/*!
 * 文字列描画
 * @param[in] str 文字列
 * @param[in] w,h ウィンドウサイズ
 * @param[in] x0,y0 文字列の位置(左上原点のスクリーン座標系,文字列の左下がこの位置になる)
 */
static void DrawString(string str, int w, int h, int x0, int y0)
{
    glDisable(GL_LIGHTING);
    // 平行投影にする
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, w, h, 0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // 画面上にテキスト描画
    glRasterPos2f(x0, y0);
    int size = (int)str.size();
    for(int i = 0; i < size; ++i){
        char ic = str[i];
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ic);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void draw_nowtime(void)
{
	glColor3d(0.0, 0.0, 0.0);
	string str = "t=";
    ostringstream ss;
    ss << nowtime;
    str = str + ss.str();
	string sec = "[sec]";
	str = str + sec;
	DrawString(str, 800, 600, 650, 550);
}

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(RADIUS_F, RADIUS_F, RADIUS_F*4, //カメラの位置（視点）のX位置,Y位置,Z位置(各々の最大値は10000)
	RADIUS_F, RADIUS_F, 0.0,  //カメラが見ているところ（注視点）のX位置,Y位置,Z位置
	0.0, 1.0, 0.0);//カメラの上方向がX軸,Y位置,Z位置に対してどれくらいか

	draw_field2();		
	
	draw_prey();

	draw_agent();

	draw_nest3();//ロボットよりも前面に表示するため，draw_agentよりも後

	draw_nowtime();

	glutSwapBuffers();		
}