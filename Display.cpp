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
	glDisable(GL_LIGHTING);//LIGHTING is off to make color by glColor3d()
	glColor4d(0.0, 0.0, 0.0, 1);//Black
	glTranslated(RADIUS_F ,RADIUS_F, 0);	
	
	glBegin(GL_LINE_LOOP);
	int edgenum = 50;
	for(int d = 0; d <= edgenum; d++) glVertex2d( RADIUS_F * cos((double)d*2.0*M_PI/edgenum), RADIUS_F * sin((double)d * 2.0 * M_PI / edgenum));//draw circle as a field

	glEnd();
	glEnable(GL_LIGHTING);//LIGHTING is on
}


void draw_nest3(void)
{
	glDisable(GL_LIGHTING);//LIGHTING is off to make color by glColor3d()
	glColor4d(0.0, 0.0, 0.0, 1);//Black
	glTranslated(0, 0, 0);	

	glBegin(GL_LINE_LOOP);
	int edgenum = 50;
	for(int d = 0; d <= edgenum; d++) glVertex2d( RADIUS_N * cos((double)d*2.0*M_PI/edgenum), RADIUS_N * sin((double)d * 2.0 * M_PI / edgenum));//draw circle as a nest

	glEnd();
	glEnable(GL_LIGHTING);//LIGHTING is on
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
		case FOLLOWING:
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
			glPushMatrix();//model view transformation matrix is memorized
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
			glPopMatrix();//model view transformation matrix is outputted
		}
	}
}

void draw_prey(void)
{
	for (int p_ID=0;p_ID< num_initprey;p_ID++)
	{
		double scale_prey = 1;

		if (prey[p_ID].inside_nest == false)
		{
			glColor4d(1.0, 0.0, 1.0, 1.0);//R,G,B,alpha

			glPushMatrix();//model view transformation matrix is memorized
			glTranslated(prey[p_ID].pos.x, prey[p_ID].pos.y, 0);
			glScaled(RADIUS_FOOD, RADIUS_FOOD, 1);
			//glEnable(GL_NORMALIZE);

			glBegin(GL_POLYGON);//convex polygon is constructed by a set of points
			int edgenum = 40;
			for (int d = 0; d <= edgenum; d++) glVertex2d((double)cos((double)d * 2.0 * M_PI / edgenum), (double)sin((double)d * 2.0 * M_PI / edgenum));
			glEnd();
			//glDisable(GL_NORMALIZE);
			glPopMatrix();//model view transformation matrix is outputted
		}
		//else glColor4d(0.0, 0.0, 0.0, 1.0);//Not shown
	}
}


/*!
 * String is shown
 * @param[in] str: string
 * @param[in] w,h: window size
 * @param[in] x0,y0: position of string(Screen coordinate whose origin is left-top. left-below corresponds this position)
 */
static void DrawString(string str, int w, int h, int x0, int y0)
{
    glDisable(GL_LIGHTING);
    // parallel projection
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, w, h, 0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // text is written on the screen
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
	gluLookAt(RADIUS_F, RADIUS_F, RADIUS_F*4, //Camera position(X,Y,Z) whose maximum value is 10000
	RADIUS_F, RADIUS_F, 0.0,  //Camera viewing position(X,Y,Z)
	0.0, 1.0, 0.0);//Rate of Camera upper direction compared to X,Y,Z-axes

	draw_field2();		
	
	draw_prey();

	draw_agent();

	draw_nest3();//this function is called after draw_agent to show nest in front of robots

	draw_nowtime();

	glutSwapBuffers();		
}