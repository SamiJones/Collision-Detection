#include <gl/glut.h>
#include "App.h"

extern Application* getApplication();
Application* app;

void display(void)
{
	app->display();
}

void createWindow(const char* title)
{
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(600, 600);
	glutCreateWindow(title);
}

void resize(int width, int height)
{
	app->resize(width, height);
}

void TimerFunc(int value)
{
	app->update();
	glutTimerFunc(app->getTimeInterval(), TimerFunc, 1);
}

int main(int argc, char* argv[])
{
	glutInit(&argc, argv);

	app = getApplication();
	
	createWindow("Sphere");
	
	glutReshapeFunc(resize);
	glutDisplayFunc(display);
	glutTimerFunc(app->getTimeInterval(), TimerFunc, 1);
	
	app->initGraphics();
	glutMainLoop();
	
	delete app;
	
	return 0;
}