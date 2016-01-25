#include "gridmap.h"

#include <QMouseEvent>
#include <math.h>

Gridmap::Gridmap(QWidget *parent) : QOpenGLWidget(parent)
{
    for (int i = 0; i < 200000; ++i)
        kernel.addPoint(rand() % kernel.mpsz, rand() % kernel.mpsz, rand() % kernel.mpsz);
}

Gridmap::~Gridmap() {}

void Gridmap::drawBox(int x, int y, int z, float a, QColor c)
{
    const static int scale = 1;
    float xx = x * scale * 2;
    float yy = y * scale * 2;
    float zz = z * scale * 2;

    // glColor4f(c.redF(), c.greenF(), c.blueF(), a);
    glVertex3f(-1 * scale + xx, -1 * scale + yy, 01 * scale + zz);
    glVertex3f(-1 * scale + xx, -1 * scale + yy, -1 * scale + zz);
    glVertex3f(01 * scale + xx, -1 * scale + yy, -1 * scale + zz);
    glVertex3f(01 * scale + xx, -1 * scale + yy, 01 * scale + zz);

    glVertex3f(01 * scale + xx, -1 * scale + yy, 01 * scale + zz);
    glVertex3f(01 * scale + xx, -1 * scale + yy, -1 * scale + zz);
    glVertex3f(01 * scale + xx, 01 * scale + yy, -1 * scale + zz);
    glVertex3f(01 * scale + xx, 01 * scale + yy, 01 * scale + zz);

    glVertex3f(01 * scale + xx, 01 * scale + yy, 01 * scale + zz);
    glVertex3f(01 * scale + xx, 01 * scale + yy, -1 * scale + zz);
    glVertex3f(-1 * scale + xx, 01 * scale + yy, -1 * scale + zz);
    glVertex3f(-1 * scale + xx, 01 * scale + yy, 01 * scale + zz);

    glVertex3f(-1 * scale + xx, 01 * scale + yy, 01 * scale + zz);
    glVertex3f(-1 * scale + xx, 01 * scale + yy, -1 * scale + zz);
    glVertex3f(-1 * scale + xx, -1 * scale + yy, -1 * scale + zz);
    glVertex3f(-1 * scale + xx, -1 * scale + yy, 01 * scale + zz);

    // glColor4f(0.5, 0.5, 1.0, a);
    glVertex3f(-1 * scale + xx, -1 * scale + yy, 01 * scale + zz);
    glVertex3f(01 * scale + xx, -1 * scale + yy, 01 * scale + zz);
    glVertex3f(01 * scale + xx, 01 * scale + yy, 01 * scale + zz);
    glVertex3f(-1 * scale + xx, 01 * scale + yy, 01 * scale + zz);
    // glColor4f(0.5, 1.0, 0.5, a);
    glVertex3f(-1 * scale + xx, -1 * scale + yy, -1 * scale + zz);
    glVertex3f(01 * scale + xx, -1 * scale + yy, -1 * scale + zz);
    glVertex3f(01 * scale + xx, 01 * scale + yy, -1 * scale + zz);
    glVertex3f(-1 * scale + xx, 01 * scale + yy, -1 * scale + zz);
}

void Gridmap::mousePressEvent(QMouseEvent *e) { QOpenGLWidget::mousePressEvent(e); }

void Gridmap::mouseReleaseEvent(QMouseEvent *e) { QOpenGLWidget::mouseReleaseEvent(e); }

void Gridmap::mouseMoveEvent(QMouseEvent *e) { QOpenGLWidget::mouseMoveEvent(e); }

void Gridmap::wheelEvent(QWheelEvent *e) { QOpenGLWidget::wheelEvent(e); }

void Gridmap::timerEvent(QTimerEvent *)
{
    rotate += 1;
    update();
}

void Gridmap::initializeGL()
{
    glShadeModel(GL_SMOOTH);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearDepth(1.0);
    glEnable(GL_BLEND);
    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);

    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    timer.start(12, this);
}

void Gridmap::resizeGL(int width, int height)
{
    if (height == 0)
    {
        height = 1;
    }
    glViewport(0, 0, (GLint)width, (GLint)height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLfloat zNear = 0.1;
    GLfloat zFar = 10000.0;
    GLfloat aspect = (GLfloat)width / (GLfloat)height;
    GLfloat fH = tan(GLfloat(90.0 / 360.0 * 3.14159)) * zNear;
    GLfloat fW = fH * aspect;
    glFrustum(-fW, fW, -fH, fH, zNear, zFar);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void Gridmap::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -45.0);
    glRotatef(rotate, 10.0f, 10.0f, 0.0f);
    //! @todo optimize

    glColor4f(0.5, 1.0, 0.5, 0.8);
    glBegin(GL_QUADS);
    DJI::vision::GridmapKernel::Point *v = kernel.getView();
    for (size_t i = 0; i < DJI::vision::GridmapKernel::mpsz; ++i)
    {
//        glColor4f(0.5 + (0.5 * i / DJI::vision::GridmapKernel::mpsz),
//                  0.5 + (0.5 * i / DJI::vision::GridmapKernel::mpsz),
//                  0.5 + (0.5 * i / DJI::vision::GridmapKernel::mpsz), 0.8);
        for (size_t j = 0; j < DJI::vision::GridmapKernel::mpsz; ++j)
            for (size_t k = 0; k < DJI::vision::GridmapKernel::mpsz; ++k)
            {
                if (v[i * DJI::vision::GridmapKernel::mpsz * DJI::vision::GridmapKernel::mpsz +
                      j * DJI::vision::GridmapKernel::mpsz + k] == true)
                    drawBox(i, j, k, 0.8, QColor(255, 255, 0));
            }
    }
    glEnd();
}
