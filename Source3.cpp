#define _USE_MATH_DEFINES 

#include <cmath>
#include <iostream>

#include <GL/glew.h>
#include <GL/freeglut.h>

static float z = 0.0;
static float xx = 0.0;

const int numPoints = 4;

static GLfloat controlPoints[numPoints][numPoints][3] = {
    {{-5.0, -5.0, 0.0}, {-1.67, -5.0, 0.0}, {1.67, -5.0, 0.0}, {5.0, -5.0, 0.0}},
    {{-5.0, -1.67, 0.0}, {-1.67, -1.67, 0.0}, {1.67, -1.67, 0.0}, {5.0, -1.67, 0.0} },
    {{-5.0, 1.67, 0.0}, {-1.67, 1.67, 0.0}, {1.67, 1.67, 0.0}, {5.0, 1.67, 0.0}},
    {{-5.0, 5.0, 0.0}, {-1.67, 5.0, 0.0}, {1.67, 5.0, 0.0}, {5.0, 5.0, 0.0}}
};

// Initialization routine.
void setup(void)
{
    glClearColor(1.0, 1.0, 1.0, 0.0);
}

// Drawing routine.
void drawScene(void)
{
    int  i, j;

    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(0.0, 0.0, 0.0);

    glLoadIdentity();

    if (z >= 1.5 * M_PI) z -= 2 * M_PI;
    if (z <= -1.5 * M_PI) z += 2 * M_PI;

    float upY = (z > M_PI/2 || z < -M_PI/2) ? -1.0f : 1.0f;

    
    gluLookAt(10.0 * cos(z) * sin(xx), 10.0 * sin(z), 10.0 * cos(z) * cos(xx) , 0.0, 0.0, 0.0, 0.0, upY, 0.0);
    //glFrustum(0.0, 100.0, 0.0, 100.0, -5.0, 100.0);
    

    //glColor3f(1.0, 0.0, 0.0);

    float part = 10.0 / 100.0;

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    
    glBegin(GL_QUADS);
    
    for (float i = -5.0; i <= 5.0; i += part) {


        for (float j = -5.0; j <= 5.0; j += part) {

            glVertex3f(i, i, 0.0);
            glVertex3f(i, j, 0.0);
            glVertex3f(j, j, 0.0);
            glVertex3f(j, i, 0.0);
        }
    }
    glEnd();

    glPointSize(5.0);

    glBegin(GL_POINTS);

    
    for (int i = 0; i < 4; i++) {  
        for (int j = 0; j < 4; j++) {  
            glVertex3f(controlPoints[i][j][0], controlPoints[i][j][1], controlPoints[i][j][2]);
        }
    }

    glEnd();

    //glutWireTeapot(5);

    glFlush();
}

// OpenGL window reshape routine.
void resize(int w, int h)
{

    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //gluPerspective(60.0, (float)w / (float)h, 1.0, 20.0);
    //gluLookAt(0.0, 0.0, z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    glFrustum(-7.0, 7.0, -7.0, 7.0, 7.0, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

// Keyboard input processing routine.
void keyInput(unsigned char key, int x, int y)
{

    switch (key)
    {
    case 27:
        exit(0);
        break;
    case 'w':
        z += 0.1;
        glutPostRedisplay();
        break;
    case 's':
        z -= 0.1;
        glutPostRedisplay();
        break;
    case 'd':
        xx -= 0.1;
        glutPostRedisplay();
        break;
    case 'a':
        xx += 0.1;
        glutPostRedisplay();
        break;
    default:
        break;
    }
}

// Routine to output interaction instructions to the C++ window.
void printInteraction(void)
{
    std::cout << "Interaction:" << std::endl;
    std::cout << "Press wasd to rotate" << std::endl;
}

// Main routine.
int main(int argc, char** argv)
{
    printInteraction();
    glutInit(&argc, argv);

    glutInitContextVersion(4, 3);
    glutInitContextProfile(GLUT_COMPATIBILITY_PROFILE);

    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA);
    glutInitWindowSize(500, 500);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("surface.cpp");
    glutDisplayFunc(drawScene);
    glutReshapeFunc(resize);
    glutKeyboardFunc(keyInput);

    glewExperimental = GL_TRUE;
    glewInit();

    setup();

    glutMainLoop();
}