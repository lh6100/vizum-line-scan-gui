#ifndef MOTIONTRACEPLOTWIDGET_H
#define MOTIONTRACEPLOTWIDGET_H

#include <QColor>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QWidget>

class QPainter;

class MotionTracePlotWidget : public QWidget {
public:
    explicit MotionTracePlotWidget(const QString& title, const QString& yUnit, QWidget* parent = nullptr);

    void setSeries(const QVector<double>& time,
                   const QVector<double>& x,
                   const QVector<double>& y,
                   const QVector<double>& z);
    void clearSeries();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawCurve(QPainter& painter,
                   const QRectF& plotRect,
                   const QVector<double>& values,
                   const QColor& color,
                   double minTime,
                   double maxTime,
                   double minValue,
                   double maxValue) const;

    QString m_title;
    QString m_yUnit;
    QVector<double> m_time;
    QVector<double> m_x;
    QVector<double> m_y;
    QVector<double> m_z;
};

#endif // MOTIONTRACEPLOTWIDGET_H
