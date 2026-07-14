#include "MotionTracePlotWidget.h"

#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QPolygonF>
#include <QSizePolicy>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

bool finiteValue(double value) {
    return std::isfinite(value);
}

void updateRange(const QVector<double>& values, double* minValue, double* maxValue) {
    for (double value : values) {
        if (!finiteValue(value)) {
            continue;
        }
        *minValue = std::min(*minValue, value);
        *maxValue = std::max(*maxValue, value);
    }
}

QString formatAxisValue(double value) {
    const double absValue = std::fabs(value);
    if (absValue >= 1000.0 || (absValue > 0.0 && absValue < 0.01)) {
        return QString::number(value, 'e', 1);
    }
    if (absValue >= 100.0) {
        return QString::number(value, 'f', 0);
    }
    if (absValue >= 10.0) {
        return QString::number(value, 'f', 1);
    }
    return QString::number(value, 'f', 2);
}

} // namespace

MotionTracePlotWidget::MotionTracePlotWidget(const QString& title, const QString& yUnit, QWidget* parent)
    : QWidget(parent),
      m_title(title),
      m_yUnit(yUnit) {
    setMinimumHeight(180);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void MotionTracePlotWidget::setSeries(const QVector<double>& time,
                                      const QVector<double>& x,
                                      const QVector<double>& y,
                                      const QVector<double>& z) {
    m_time = time;
    m_x = x;
    m_y = y;
    m_z = z;
    update();
}

void MotionTracePlotWidget::clearSeries() {
    m_time.clear();
    m_x.clear();
    m_y.clear();
    m_z.clear();
    update();
}

void MotionTracePlotWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(250, 250, 250));

    const QRectF outer = rect().adjusted(8, 8, -8, -8);
    painter.setPen(QPen(QColor(205, 210, 216)));
    painter.setBrush(Qt::NoBrush);
    painter.drawRoundedRect(outer, 4, 4);

    painter.setPen(QColor(36, 42, 49));
    QFont titleFont = painter.font();
    titleFont.setBold(true);
    painter.setFont(titleFont);
    painter.drawText(QRectF(outer.left() + 10, outer.top() + 4, outer.width() - 20, 22),
                     Qt::AlignLeft | Qt::AlignVCenter, m_title);

    QFont smallFont = painter.font();
    smallFont.setBold(false);
    smallFont.setPointSize(std::max(8, smallFont.pointSize() - 1));
    painter.setFont(smallFont);

    const QRectF plotRect = outer.adjusted(58, 34, -18, -34);
    if (m_time.size() < 2 || m_x.size() != m_time.size() ||
        m_y.size() != m_time.size() || m_z.size() != m_time.size()) {
        painter.setPen(QColor(110, 118, 128));
        painter.drawText(plotRect, Qt::AlignCenter, QStringLiteral("暂无真实运动采样数据"));
        return;
    }

    double minTime = std::numeric_limits<double>::infinity();
    double maxTime = -std::numeric_limits<double>::infinity();
    updateRange(m_time, &minTime, &maxTime);
    if (!finiteValue(minTime) || !finiteValue(maxTime) || maxTime <= minTime) {
        maxTime = minTime + 1.0;
    }

    double minValue = 0.0;
    double maxValue = 0.0;
    updateRange(m_x, &minValue, &maxValue);
    updateRange(m_y, &minValue, &maxValue);
    updateRange(m_z, &minValue, &maxValue);
    if (maxValue <= minValue) {
        maxValue += 1.0;
        minValue -= 1.0;
    } else {
        const double pad = (maxValue - minValue) * 0.08;
        maxValue += pad;
        minValue -= pad;
    }

    painter.setPen(QPen(QColor(225, 229, 233), 1));
    for (int i = 0; i <= 4; ++i) {
        const double x = plotRect.left() + plotRect.width() * i / 4.0;
        painter.drawLine(QPointF(x, plotRect.top()), QPointF(x, plotRect.bottom()));
        const double t = minTime + (maxTime - minTime) * i / 4.0;
        painter.setPen(QColor(90, 96, 105));
        painter.drawText(QRectF(x - 26, plotRect.bottom() + 4, 52, 18),
                         Qt::AlignCenter, formatAxisValue(t));
        painter.setPen(QPen(QColor(225, 229, 233), 1));
    }
    for (int i = 0; i <= 4; ++i) {
        const double y = plotRect.bottom() - plotRect.height() * i / 4.0;
        painter.drawLine(QPointF(plotRect.left(), y), QPointF(plotRect.right(), y));
        const double value = minValue + (maxValue - minValue) * i / 4.0;
        painter.setPen(QColor(90, 96, 105));
        painter.drawText(QRectF(outer.left() + 6, y - 9, 48, 18),
                         Qt::AlignRight | Qt::AlignVCenter, formatAxisValue(value));
        painter.setPen(QPen(QColor(225, 229, 233), 1));
    }

    painter.setPen(QPen(QColor(82, 90, 100), 1.2));
    painter.drawRect(plotRect);
    painter.setPen(QColor(82, 90, 100));
    painter.drawText(QRectF(plotRect.left(), plotRect.bottom() + 17, plotRect.width(), 16),
                     Qt::AlignCenter, QStringLiteral("时间(s)"));
    painter.save();
    painter.translate(outer.left() + 12, plotRect.center().y());
    painter.rotate(-90.0);
    painter.drawText(QRectF(-plotRect.height() / 2.0, 0, plotRect.height(), 16),
                     Qt::AlignCenter, m_yUnit);
    painter.restore();

    drawCurve(painter, plotRect, m_x, QColor(209, 64, 64), minTime, maxTime, minValue, maxValue);
    drawCurve(painter, plotRect, m_y, QColor(37, 142, 89), minTime, maxTime, minValue, maxValue);
    drawCurve(painter, plotRect, m_z, QColor(53, 106, 202), minTime, maxTime, minValue, maxValue);

    const int legendY = static_cast<int>(outer.top() + 8);
    const int legendX = static_cast<int>(outer.right() - 150);
    const QColor colors[3] = {QColor(209, 64, 64), QColor(37, 142, 89), QColor(53, 106, 202)};
    const QString labels[3] = {QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")};
    for (int i = 0; i < 3; ++i) {
        const int x = legendX + i * 46;
        painter.setPen(QPen(colors[i], 2));
        painter.drawLine(x, legendY + 10, x + 18, legendY + 10);
        painter.setPen(QColor(45, 51, 58));
        painter.drawText(QRectF(x + 22, legendY, 20, 20), Qt::AlignLeft | Qt::AlignVCenter, labels[i]);
    }
}

void MotionTracePlotWidget::drawCurve(QPainter& painter,
                                      const QRectF& plotRect,
                                      const QVector<double>& values,
                                      const QColor& color,
                                      double minTime,
                                      double maxTime,
                                      double minValue,
                                      double maxValue) const {
    QPolygonF points;
    points.reserve(std::min(m_time.size(), values.size()));
    const double timeSpan = maxTime - minTime;
    const double valueSpan = maxValue - minValue;
    for (int i = 0; i < m_time.size() && i < values.size(); ++i) {
        if (!finiteValue(m_time[i]) || !finiteValue(values[i])) {
            continue;
        }
        const double x = plotRect.left() + (m_time[i] - minTime) / timeSpan * plotRect.width();
        const double y = plotRect.bottom() - (values[i] - minValue) / valueSpan * plotRect.height();
        points << QPointF(x, y);
    }
    if (points.size() < 2) {
        return;
    }
    painter.setPen(QPen(color, 1.8));
    painter.drawPolyline(points);
}
