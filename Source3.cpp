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
    {{-5.0, -1.67, 0.0}, {-1.67, -1.67, 0.0}, {1.67, -1.67, 0.0}, {5.0, -1.67, 0.0}},
    {{-5.0, 1.67, 0.0}, {-1.67, 1.67, 0.0}, {1.67, 1.67, 0.0}, {5.0, 1.67, 0.0}},
    {{-5.0, 5.0, 0.0}, {-1.67, 5.0, 0.0}, {1.67, 5.0, 0.0}, {5.0, 5.0, 0.0}}
};

float Bernstein(int i, int n, float t) {
    // Binomial coefficient
    float binomial = 1;
    for (int k = 0; k < i; ++k)
        binomial *= (float)(n - k) / (k + 1);

    return binomial * pow(t, i) * pow(1 - t, n - i);
}

// Function to evaluate the Bézier surface at (u, v)
void evaluateBezierSurface(float u, float v, float* point) {
    point[0] = point[1] = point[2] = 0.0f;

    for (int i = 0; i < numPoints; ++i) {
        for (int j = 0; j < numPoints; ++j) {
            float bu = Bernstein(i, numPoints - 1, u);
            float bv = Bernstein(j, numPoints - 1, v);

            point[0] += bu * bv * controlPoints[i][j][0];
            point[1] += bu * bv * controlPoints[i][j][1];
            point[2] += bu * bv * controlPoints[i][j][2];
        }
    }
}

// Function to draw the Bézier surface as a mesh of quads
void drawBezierSurface(int resolution) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glBegin(GL_QUADS);
    for (int i = 0; i < resolution-1; ++i) {
        float u = (float)i / (resolution - 1);
        float uNext = (float)(i + 1) / (resolution - 1);

        for (int j = 0; j < resolution-1; ++j) {
            float v = (float)j / (resolution - 1);
            float vNext = (float)(j + 1) / (resolution - 1);

            // Evaluate 4 corners of the quad
            float p1[3], p2[3], p3[3], p4[3];
            evaluateBezierSurface(u, v, p1);
            evaluateBezierSurface(uNext, v, p2);
            evaluateBezierSurface(uNext, vNext, p3);
            evaluateBezierSurface(u, vNext, p4);

            // Draw the quad
            glVertex3fv(p1);
            glVertex3fv(p2);
            glVertex3fv(p3);
            glVertex3fv(p4);
        }
    }
    glEnd();
}

// Track the selected control point
static int rowCount = 0, columnCount = 0;

// Initialization routine.
void setup(void)
{
    glClearColor(1.0, 1.0, 1.0, 0.0);
}

// Drawing routine.
void drawScene(void)
{
    int i, j;

    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(0.0, 0.0, 0.0);
    glLoadIdentity();

    if (z >= 1.5 * M_PI) z -= 2 * M_PI;
    if (z <= -1.5 * M_PI) z += 2 * M_PI;

    float upY = (z > M_PI / 2 || z < -M_PI / 2) ? -1.0f : 1.0f;
    gluLookAt(10.0 * cos(z) * sin(xx), 10.0 * sin(z), 10.0 * cos(z) * cos(xx), 0.0, 0.0, 0.0, 0.0, upY, 0.0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    
    /*
    glBegin(GL_QUADS);
    float part = 10.0 / 30.0;
    for (float i = -5.0; i <= 5.0; i += part) {
        for (float j = -5.0; j <= 5.0; j += part) {
            glVertex3f(i, i, 0.0);
            glVertex3f(i, j, 0.0);
            glVertex3f(j, j, 0.0);
            glVertex3f(j, i, 0.0);
        }
    }
    glEnd();*/

    drawBezierSurface(30);


    glBegin(GL_QUADS);
    glColor3f(1.0, 0.0, 0.0);
    for (int i = 0; i < numPoints-1; i += 1) {
        for (int j = 0; j < numPoints-1; j += 1) {
            glVertex3f(controlPoints[i][j][0], controlPoints[i][j][1], controlPoints[i][j][2]);
            glVertex3f(controlPoints[i+1][j][0], controlPoints[i+1][j][1], controlPoints[i+1][j][2]);
            glVertex3f(controlPoints[i+1][j+1][0], controlPoints[i + 1][j + 1][1], controlPoints[i + 1][j + 1][2]);
            glVertex3f(controlPoints[i][j + 1][0], controlPoints[i][j + 1][1], controlPoints[i][j + 1][2]);
        }
    }
    glEnd();

    glPointSize(5.0);
    glBegin(GL_POINTS);
    for (int i = 0; i < numPoints; i++) {
        for (int j = 0; j < numPoints; j++) {
            if (rowCount == i && columnCount == j) glColor3f(1.0, 0.0, 0.0);
            else glColor3f(0.0, 0.0, 0.0);
           
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
    glFrustum(-3.0, 3.0, -3.0, 3.0, 3.0, 20.0);
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
    case 9:
    {
        if (rowCount < 3) rowCount++;
        else rowCount = 0;
        glutPostRedisplay();
        break;
    }
    
    case ' ':
    {
        if (columnCount < 3) columnCount++;
        else columnCount = 0;
        glutPostRedisplay();
        break;
    }
    case 'X':
        controlPoints[rowCount][columnCount][0] -= 0.1;
        glutPostRedisplay();
        break;
    case 'x':
        controlPoints[rowCount][columnCount][0] += 0.1;
        glutPostRedisplay();
        break;
    case 'Y':
        controlPoints[rowCount][columnCount][1] -= 0.1;
        glutPostRedisplay();
        break;
    case 'y':
        controlPoints[rowCount][columnCount][1] += 0.1;
        glutPostRedisplay();
        break;
    case 'Z':
        controlPoints[rowCount][columnCount][2] -= 0.1;
        glutPostRedisplay();
        break;
    case 'z':
        controlPoints[rowCount][columnCount][2] += 0.1;
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
    std::cout << "Press WASD to rotate" << std::endl;
    std::cout << "Use x/X, y/Y, z/Z keys to move the selected control point" << std::endl;
    std::cout << "Use space and tab to swap the selected control point" << std::endl;
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
    glutCreateWindow("Control Points Movement");
    glutDisplayFunc(drawScene);
    glutReshapeFunc(resize);
    glutKeyboardFunc(keyInput);


    glewExperimental = GL_TRUE;
    glewInit();

    setup();

    glutMainLoop();
}