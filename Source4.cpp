#define _USE_MATH_DEFINES 

#include <cmath>
#include <iostream>
#include <vector>

#include <GL/glew.h>
#include <GL/freeglut.h>

static float z = 0.0;
static float xx = 0.0;

static int numPoints = 4;

static int mode = 0;

bool showWireframe = false;


// Track the selected control point
static int rowCount = 0, columnCount = 0;

std::vector<std::vector<std::vector<GLfloat>>> controlPoints(numPoints, std::vector<std::vector<GLfloat>>(numPoints, std::vector<GLfloat>(4))); // x, y, z, weight

std::vector<float> knotVectorU;
std::vector<float> knotVectorV;




float Bernstein(int i, int n, float t) {
    // Binomial coefficient
    float binomial = 1;
    for (int k = 0; k < i; ++k)
        binomial *= (float)(n - k) / (k + 1);

    return binomial * pow(t, i) * pow(1 - t, n - i);
}

void drawAxes() {
    glLineWidth(2.0f); // Set line width for axes
    glColor3f(0.0f, 0.0f, 0.0f); // Set color to black

    glBegin(GL_LINES);

    // X-axis
    glVertex3f(-5.0f, 0.0f, 0.0f);
    glVertex3f(5.0f, 0.0f, 0.0f);

    // Y-axis
    glVertex3f(0.0f, -5.0f, 0.0f);
    glVertex3f(0.0f, 5.0f, 0.0f);

    // Z-axis
    glVertex3f(0.0f, 0.0f, -5.0f);
    glVertex3f(0.0f, 0.0f, 5.0f);

    glEnd();
    glLineWidth(1.0f); // Reset line width
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
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
    //glColor3f(0.0, 1.0, 0.0);

    glBegin(GL_QUADS);
    //glColor3f(0.0, 1.0, 0.0);
    for (int i = 0; i < resolution - 1; ++i) {
        float u = (float)i / (resolution - 1);
        float uNext = (float)(i + 1) / (resolution - 1);

        for (int j = 0; j < resolution - 1; ++j) {
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

void changeControlPoints()
{
    controlPoints.resize(numPoints);

    // Resize each row to have 3 columns
    for (int i = 0; i < numPoints; ++i) {
        controlPoints[i].resize(numPoints);
    }

    GLfloat part = 10.0 / (numPoints - 1);
    for (int i = 0; i < numPoints; ++i) {
        for (int j = 0; j < numPoints; ++j) {
            controlPoints[i][j] = { -5 + (i * part), -5 + (j * part), 0.0, 1.0 }; // x, y, z, w
        }
    }
}



std::vector<float> generateKnotVector(int degree) {
    int numControlPoints = numPoints;
    int numKnots = numControlPoints + degree + 1;
    std::vector<float> knotVector(numKnots);

    // First and last knots are clamped
    for (int i = 0; i <= degree; ++i) {
        knotVector[i] = 0.0f;  // Clamped at the beginning
        knotVector[numKnots - 1 - i] = 1.0f;  // Clamped at the end
    }

    // Interior knots are uniformly spaced
    float step = 1.0f / (numControlPoints - degree);
    for (int i = degree + 1; i < numControlPoints; ++i) {
        knotVector[i] = (i - degree) * step;
    }

    
    return knotVector;
}



float N(int i, int p, float u, const std::vector<float>& knotVector) {
    if (p == 0) {
        return (knotVector[i] <= u && u < knotVector[i + 1]) ? 1.0f : 0.0f;
    }
    else {
        float leftTerm = 0.0f;
        float rightTerm = 0.0f;

        float denominator1 = knotVector[i + p] - knotVector[i];
        float denominator2 = knotVector[i + p + 1] - knotVector[i + 1];

        // Only compute the left term if the denominator is not zero
        if (denominator1 != 0.0f) {
            leftTerm = (u - knotVector[i]) / denominator1 * N(i, p - 1, u, knotVector);
        }

        // Only compute the right term if the denominator is not zero
        if (denominator2 != 0.0f) {
            rightTerm = (knotVector[i + p + 1] - u) / denominator2 * N(i + 1, p - 1, u, knotVector);
        }

        return leftTerm + rightTerm;
    }
}

std::vector<GLfloat> evaluateNURBSSurface(float u, float v, const std::vector<float>& knotVectorU, const std::vector<float>& knotVectorV, int p, int q) {
    int n = controlPoints.size() - 1;
    int m = controlPoints[0].size() - 1;
    std::vector<GLfloat> surfacePoint(3, 0.0f);
    float wSum = 0.0f;

    for (int i = 0; i <= n; ++i) {
        for (int j = 0; j <= m; ++j) {
            float Ni = N(i, p, u, knotVectorU);
            float Nj = N(j, q, v, knotVectorV);
            //float weight = controlPoints[i][j][3]; // Access the weight
            float weight = 2; // Access the weight
            float basis = Ni * Nj * weight;

            surfacePoint[0] += basis * controlPoints[i][j][0];
            surfacePoint[1] += basis * controlPoints[i][j][1];
            surfacePoint[2] += basis * controlPoints[i][j][2];
            wSum += basis;
        }
    }

    // Divide by the weight sum
    surfacePoint[0] /= wSum;
    surfacePoint[1] /= wSum;
    surfacePoint[2] /= wSum;

    return surfacePoint;
    if (wSum != 0.0f) {
        surfacePoint[0] /= wSum;
        surfacePoint[1] /= wSum;
        surfacePoint[2] /= wSum;
    }
    else {
        std::cerr << "Warning: Weight sum is zero!" << std::endl;
    }
}

std::vector<GLfloat> evaluateSurface(float u, float v, const std::vector<float>& knotVectorU, const std::vector<float>& knotVectorV, int p, int q) {
    int n = controlPoints.size() - 1;
    int m = controlPoints[0].size() - 1;
    std::vector<GLfloat> surfacePoint(3, 0.0f);;

    for (int i = 0; i <= n; ++i) {
        for (int j = 0; j <= m; ++j) {
            float Ni = N(i, p, u, knotVectorU);
            float Nj = N(j, q, v, knotVectorV);
            surfacePoint[0] += Ni * Nj * controlPoints[i][j][0];
            surfacePoint[1] += Ni * Nj * controlPoints[i][j][1];
            surfacePoint[2] += Ni * Nj * controlPoints[i][j][2];
        }
    }
    return surfacePoint;
}

void renderNURBSSurface(const std::vector<float>& knotVectorU, const std::vector<float>& knotVectorV, int p, int q, int resolution) {
    glBegin(GL_QUADS);
    //glColor3f(0.0, 1.0, 0.0);
    for (int i = 0; i < resolution - 1; ++i) {
        float u1 = std::min((float)i / (resolution - 1), 0.999f);
        float u2 = std::min((float)(i + 1) / (resolution - 1), 0.999f);

        for (int j = 0; j < resolution - 1; ++j) {
            float v1 = std::min((float)j / (resolution - 1), 0.999f);
            float v2 = std::min((float)(j + 1) / (resolution - 1), 0.999f);

            // Evaluate four corner points of the current quad
            std::vector<GLfloat> p1 = evaluateNURBSSurface(u1, v1, knotVectorU, knotVectorV, p, q);
            std::vector<GLfloat> p2 = evaluateNURBSSurface(u2, v1, knotVectorU, knotVectorV, p, q);
            std::vector<GLfloat> p3 = evaluateNURBSSurface(u2, v2, knotVectorU, knotVectorV, p, q);
            std::vector<GLfloat> p4 = evaluateNURBSSurface(u1, v2, knotVectorU, knotVectorV, p, q);

            // Draw the quad
            glVertex3f(p1[0], p1[1], p1[2]);
            glVertex3f(p2[0], p2[1], p2[2]);
            glVertex3f(p3[0], p3[1], p3[2]);
            glVertex3f(p4[0], p4[1], p4[2]);
        }
    }
    glEnd();
}

void renderBSplineSurface(const std::vector<float>& knotVectorU, const std::vector<float>& knotVectorV, int p, int q, int resolution) {

    glBegin(GL_QUADS);
    //glColor3f(0.0, 1.0, 0.0);
    for (int i = 0; i < resolution - 1; ++i) {
        float u1 = std::min((float)i / (resolution - 1), 0.999f);
        float u2 = std::min((float)(i + 1) / (resolution - 1), 0.999f);


        for (int j = 0; j < resolution - 1; ++j) {
            float v1 = std::min((float)j / (resolution - 1), 0.999f);
            float v2 = std::min((float)(j + 1) / (resolution - 1), 0.999f);

            // Evaluate four corner points of the current quad
            std::vector<GLfloat> p1 = evaluateSurface(u1, v1, knotVectorU, knotVectorV, p, q);
            std::vector<GLfloat> p2 = evaluateSurface(u2, v1, knotVectorU, knotVectorV, p, q);
            std::vector<GLfloat> p3 = evaluateSurface(u2, v2, knotVectorU, knotVectorV, p, q);
            std::vector<GLfloat> p4 = evaluateSurface(u1, v2, knotVectorU, knotVectorV, p, q);

            // Draw the quad
            glVertex3f(p1[0], p1[1], p1[2]);
            glVertex3f(p2[0], p2[1], p2[2]);
            glVertex3f(p3[0], p3[1], p3[2]);
            glVertex3f(p4[0], p4[1], p4[2]);
            //std::cout << "P1: " << p1[0] << ", " << p1[1] << ", " << p1[2] << std::endl;


        }
    }
    glEnd();
    //for (auto& kv : knotVectorU) std::cout << kv << " ";

}

void setup(void)
{
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glEnable(GL_DEPTH_TEST); // Enable depth testing
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 }; // Directional light
    GLfloat light_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };  // Diffuse color
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 }; // Specular color
    GLfloat light_ambient[] = { 0.2, 0.2, 0.2, 1.0 };  // Ambient color

    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    

    changeControlPoints(); // Ensure control points are initialized
}


void drawScene(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    if (z >= 1.5 * M_PI) z -= 2 * M_PI;
    if (z <= -1.5 * M_PI) z += 2 * M_PI;

    float upY = (z > M_PI / 2 || z < -M_PI / 2) ? -1.0f : 1.0f;
    gluLookAt(10.0 * cos(z) * sin(xx), 10.0 * sin(z), 10.0 * cos(z) * cos(xx), 0.0, 0.0, 0.0, 0.0, upY, 0.0);

    drawAxes(); // Draw the axes.

    if (!showWireframe) {
        // Set material for the surface
        GLfloat mat_ambient[] = { 0.1f, 0.3f, 0.1f, 1.0f };  // Dark green ambient color
        GLfloat mat_diffuse[] = { 0.2f, 0.7f, 0.2f, 1.0f };  // Bright green diffuse color
        GLfloat mat_specular[] = { 0.8f, 0.8f, 0.8f, 1.0f }; // White specular highlights
        GLfloat mat_shininess[] = { 50.0f };                 // Shininess factor

        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
    }

    // Draw the surface
    if (mode == 0) {
        drawBezierSurface(30);
    }
    else if (mode == 1) {
        knotVectorU = generateKnotVector(3);
        knotVectorV = generateKnotVector(3);
        renderBSplineSurface(knotVectorU, knotVectorV, 3, 3, 30);
    }
    else if (mode == 2) {
        knotVectorU = generateKnotVector(3);
        knotVectorV = generateKnotVector(3);
        renderNURBSSurface(knotVectorU, knotVectorV, 3, 3, 30);
    }

    // Draw control points and polygons if needed
    glDisable(GL_LIGHTING);  // Always disable lighting for control points
     // Draw control points
    glPointSize(5.0);
    glBegin(GL_POINTS);
    for (int i = 0; i < numPoints; i++) {
        for (int j = 0; j < numPoints; j++) {
            if (rowCount == i && columnCount == j)
                glColor3f(1.0, 0.0, 0.0); // Highlight the selected control point (red)
            else
                glColor3f(0.0, 0.0, 0.0); // Blue for other control points

            glVertex3f(controlPoints[i][j][0], controlPoints[i][j][1], controlPoints[i][j][2]);
        }
    }
    glEnd();

    // Draw control polygon (lines connecting control points)
    glLineWidth(1.5);
    glColor3f(0.0f, 0.0f, 1.0f); // Black for lines
    glBegin(GL_LINES);
    for (int i = 0; i < numPoints; ++i) {
        for (int j = 0; j < numPoints - 1; ++j) {
            glVertex3f(controlPoints[i][j][0], controlPoints[i][j][1], controlPoints[i][j][2]);
            glVertex3f(controlPoints[i][j + 1][0], controlPoints[i][j + 1][1], controlPoints[i][j + 1][2]);
        }
    }
    for (int j = 0; j < numPoints; ++j) {
        for (int i = 0; i < numPoints - 1; ++i) {
            glVertex3f(controlPoints[i][j][0], controlPoints[i][j][1], controlPoints[i][j][2]);
            glVertex3f(controlPoints[i + 1][j][0], controlPoints[i + 1][j][1], controlPoints[i + 1][j][2]);
        }
    }
    glEnd();
    glEnable(GL_LIGHTING);   // Re-enable lighting for other elements if needed
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
        if (rowCount < numPoints - 1) rowCount++;
        else rowCount = 0;
        glutPostRedisplay();
        break;
    }

    case ' ':
    {
        if (columnCount < numPoints - 1) columnCount++;
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
    case 'j':
        if (numPoints < 10) {
            numPoints += 1;
            columnCount = 0;
            rowCount = 0;
            changeControlPoints();
        }
        glutPostRedisplay();
        break;
    case 'k':
        if (numPoints > 4) {
            numPoints -= 1;
            columnCount = 0;
            rowCount = 0;
            changeControlPoints();
        }
        glutPostRedisplay();
        break;
    case 'h':
        mode = (mode + 1) % 3; // Cycle through 0, 1, and 2
        glutPostRedisplay();
        break;

    case 'g':  // Toggle shading and wireframe
        showWireframe = !showWireframe;
        if (showWireframe) {
            glDisable(GL_LIGHTING);                 // Turn off lighting
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // Render as wireframe
        }
        else {
            glEnable(GL_LIGHTING);                  // Turn lighting back on
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Render as solid surface
        }
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
    std::cout << "Use h to cycle through the modes" << std::endl;
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