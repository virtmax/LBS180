#include "rotposframe.h"

RotPosFrame::RotPosFrame(QWidget *parent)  : QFrame(parent)
{

}

void RotPosFrame::paintEvent(QPaintEvent *e)
{
    QPoint minuteHand[3] = {
            QPoint(7, 8),
            QPoint(-7, 8),
            QPoint(0, -96)
        };

    QColor hourColor(127, 0, 127);

    // draw marks on left clock
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.translate(width() / 4, height() / 2);
        painter.scale(1, 1);

        painter.setPen(hourColor);

        painter.save();
        for (int i = 0; i < 72; ++i) {
            painter.drawLine(100, 0, 110, 0);
            painter.rotate(5.0);
        }
        painter.restore();

        painter.save();
        for (int i = 0; i < 8; ++i) {
            painter.drawLine(100, 0, 120, 0);
            painter.rotate(45);
        }
        painter.restore();

        painter.save();
        painter.rotate(rotationAngle);
        painter.drawConvexPolygon(minuteHand, 3);
        painter.restore();

        painter.end();
    }

    // draw marks on right clock
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.translate(width() * (3.0/4.0), height() / 2);
        painter.scale(1, 1);

        //painter.setPen(hourColor);

        painter.save();
        painter.rotate(-90-90);
        for (int i = 0; i < 19; ++i) {
            painter.drawLine(100, 0, 120, 0);
            painter.rotate(10);
        }
        painter.restore();

        painter.save();
        painter.rotate(-90-90);
        for (int i = 0; i < 36; ++i) {
            painter.drawLine(100, 0, 110, 0);
            painter.rotate(5);
        }
        painter.restore();

        double newAngle = 0;
        if(rotationAngle >= 0 && rotationAngle <= 9)
            newAngle = rotationAngle*10;
        else if(rotationAngle >= 351 && rotationAngle <= 360)
            newAngle = 360 - (360 - rotationAngle)*10;
        else
            newAngle = 180;

        painter.save();
        painter.rotate(newAngle);
        painter.drawConvexPolygon(minuteHand, 3);
        painter.restore();

        painter.end();
    }

    QFrame::paintEvent(e);
}
