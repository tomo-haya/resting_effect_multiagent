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
	glDisable(GL_LIGHTING);//glColor3d()�ŐF�����邽�߁C���C�e�B���O�������I�t�ɂ���(�s�v����)
	glColor4d(0.0, 0.0, 0.0, 1);//Black
	glTranslated(RADIUS_F ,RADIUS_F, 0);	
	
	glBegin(GL_LINE_LOOP);
	int edgenum = 50;
	for(int d = 0; d <= edgenum; d++) glVertex2d( RADIUS_F * cos((double)d*2.0*M_PI/edgenum), RADIUS_F * sin((double)d * 2.0 * M_PI / edgenum));//draw circle as a field

	glEnd();
	glEnable(GL_LIGHTING);//���C�e�B���O�������I���ɂ���(�s�v����)
}


void draw_nest3(void)
{
	glDisable(GL_LIGHTING);//glColor3d()�ŐF�����邽�߁C���C�e�B���O�������I�t�ɂ���(�s�v����)
	glColor4d(0.0, 0.0, 0.0, 1);//Black
	glTranslated(0, 0, 0);	

	glBegin(GL_LINE_LOOP);
	int edgenum = 50;
	for(int d = 0; d <= edgenum; d++) glVertex2d( RADIUS_N * cos((double)d*2.0*M_PI/edgenum), RADIUS_N * sin((double)d * 2.0 * M_PI / edgenum));//draw circle as a nest

	glEnd();
	glEnable(GL_LIGHTING);//���C�e�B���O�������I���ɂ���(�s�v����)
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
		case FOLLOWING://RECRUITING��LEADING�Əd�Ȃ�̂ŕ\�����Ȃ�
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
			glPushMatrix();//�����S�ʒu�Ȃǂ̃��f���r���[�ϊ��s��̕ۑ�
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
			glPopMatrix();//�����S�ʒu�Ȃǂ̃��f���r���[�ϊ��s��̎�o��
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

			glPushMatrix();//�����S�ʒu�Ȃǂ̃��f���r���[�ϊ��s��̕ۑ�
			glTranslated(prey[p_ID].pos.x, prey[p_ID].pos.y, 0);
			glScaled(RADIUS_FOOD, RADIUS_FOOD, 1);
			//glEnable(GL_NORMALIZE);

			glBegin(GL_POLYGON);//�_�W������A�ʑ��p�`���\�z
			int edgenum = 40;
			for (int d = 0; d <= edgenum; d++) glVertex2d((double)cos((double)d * 2.0 * M_PI / edgenum), (double)sin((double)d * 2.0 * M_PI / edgenum));
			glEnd();
			//glDisable(GL_NORMALIZE);
			glPopMatrix();//�����S�ʒu�Ȃǂ̃��f���r���[�ϊ��s��̎�o��
		}
		//else glColor4d(0.0, 0.0, 0.0, 1.0);//�\�����Ȃ�
	}
}


/*!
 * ������`��
 * @param[in] str ������
 * @param[in] w,h �E�B���h�E�T�C�Y
 * @param[in] x0,y0 ������̈ʒu(���㌴�_�̃X�N���[�����W�n,������̍��������̈ʒu�ɂȂ�)
 */
static void DrawString(string str, int w, int h, int x0, int y0)
{
    glDisable(GL_LIGHTING);
    // ���s���e�ɂ���
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, w, h, 0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // ��ʏ�Ƀe�L�X�g�`��
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
	gluLookAt(RADIUS_F, RADIUS_F, RADIUS_F*4, //�J�����̈ʒu�i���_�j��X�ʒu,Y�ʒu,Z�ʒu(�e�X�̍ő�l��10000)
	RADIUS_F, RADIUS_F, 0.0,  //�J���������Ă���Ƃ���i�����_�j��X�ʒu,Y�ʒu,Z�ʒu
	0.0, 1.0, 0.0);//�J�����̏������X��,Y�ʒu,Z�ʒu�ɑ΂��Ăǂꂭ�炢��

	draw_field2();		
	
	draw_prey();

	draw_agent();

	draw_nest3();//���{�b�g�����O�ʂɕ\�����邽�߁Cdraw_agent������

	draw_nowtime();

	glutSwapBuffers();		
}