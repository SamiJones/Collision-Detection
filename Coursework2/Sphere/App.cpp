#include <gl/glut.h>
#include "App.h"

Application::Application()
{
	timeInterval = 16.66f;
}

void Application::initGraphics()
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void Application::display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void Application::update()
{
	glutPostRedisplay();
}

void Application::resize(int width, int height)
{
	GLfloat nRange = 100.0f;
	GLfloat aspectRatio = (GLfloat)width / (GLfloat)height;

	if (height == 0)
		height = 1;

	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	if (width <= height)
	{
		Application::width = nRange;
		Application::height = nRange / aspectRatio;
		glOrtho(-nRange, nRange, -nRange / aspectRatio, nRange / aspectRatio, -nRange * 2.0f, nRange * 2.0f);
	}
	else
	{
		Application::width = nRange * aspectRatio;
		Application::height = nRange;
		glOrtho(-nRange*aspectRatio, nRange*aspectRatio, -nRange, nRange, -nRange * 2.0f, nRange * 2.0f);
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

GLfloat Application::getTimeInterval()
{
	return timeInterval;
}
