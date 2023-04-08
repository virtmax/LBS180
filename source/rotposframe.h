#ifndef ROTPOSFRAME_H
#define ROTPOSFRAME_H

#include <QFrame>
#include <QPainter>


class RotPosFrame : public QFrame
{
    Q_OBJECT
public:
    RotPosFrame(QWidget *parent);
    void paintEvent(QPaintEvent *e);

    double rotationAngle {0.0};
};

#endif
