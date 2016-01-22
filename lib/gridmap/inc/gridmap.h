#ifndef GRIDMAP_H
#define GRIDMAP_H

#include "gridmapKernel.h"

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
class Gridmap : public QOpenGLWidget
{
    Q_OBJECT

  public:
    explicit Gridmap(QWidget *parent = 0);
    ~Gridmap();

  public:
    void drawBox(int x, int y, int z, float a, QColor c);

  protected:
    void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void wheelEvent(QWheelEvent *e) Q_DECL_OVERRIDE;
    void timerEvent(QTimerEvent *e) Q_DECL_OVERRIDE;

    void initializeGL() Q_DECL_OVERRIDE;
    void resizeGL(int width, int height) Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;

  private:
    DJI::vision::GridmapKernel kernel;
    QBasicTimer timer;
    float rotate;
};

#endif // GRIDMAP_H
