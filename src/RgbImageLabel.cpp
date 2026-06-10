#include "RgbImageLabel.h"

#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

RgbImageLabel::RgbImageLabel(QWidget* parent) : QLabel(parent) {
    setAlignment(Qt::AlignCenter);
    setMouseTracking(true);
    setFocusPolicy(Qt::WheelFocus);
    setStyleSheet("background:#111; color:#bbb; border:1px solid #444;");
}

void RgbImageLabel::setImage(const QImage& image) {
    m_image = image;
    m_hasLine = false;
    m_drawing = false;
    m_panning = false;
    m_viewInitialized = false;
    fitImageToView();
    update();
}

void RgbImageLabel::resetView() {
    fitImageToView();
    update();
}

double RgbImageLabel::fitScale() const {
    if (m_image.isNull() || width() <= 0 || height() <= 0) {
        return 1.0;
    }
    const double sx = static_cast<double>(width()) / static_cast<double>(m_image.width());
    const double sy = static_cast<double>(height()) / static_cast<double>(m_image.height());
    return std::max(0.001, std::min(sx, sy));
}

void RgbImageLabel::fitImageToView() {
    if (m_image.isNull()) {
        m_scale = 1.0;
        m_offset = QPointF(0.0, 0.0);
        m_viewInitialized = false;
        return;
    }

    m_scale = fitScale();
    const double w = m_image.width() * m_scale;
    const double h = m_image.height() * m_scale;
    m_offset = QPointF((width() - w) * 0.5, (height() - h) * 0.5);
    m_viewInitialized = true;
}

QRectF RgbImageLabel::imageRect() const {
    if (m_image.isNull()) {
        return QRectF(rect());
    }
    return QRectF(m_offset.x(),
                  m_offset.y(),
                  m_image.width() * m_scale,
                  m_image.height() * m_scale);
}

QPoint RgbImageLabel::widgetToImage(const QPoint& point) const {
    if (m_image.isNull()) {
        return {};
    }
    if (m_scale <= 0.0) {
        return {};
    }
    QPoint imagePoint(static_cast<int>((point.x() - m_offset.x()) / m_scale + 0.5),
                      static_cast<int>((point.y() - m_offset.y()) / m_scale + 0.5));
    return clampImagePoint(imagePoint);
}

QPoint RgbImageLabel::imageToWidget(const QPoint& point) const {
    if (m_image.isNull()) {
        return {};
    }
    return QPoint(static_cast<int>(m_offset.x() + point.x() * m_scale + 0.5),
                  static_cast<int>(m_offset.y() + point.y() * m_scale + 0.5));
}

QPoint RgbImageLabel::clampImagePoint(const QPoint& point) const {
    if (m_image.isNull()) {
        return {};
    }
    return QPoint(std::max(0, std::min(m_image.width() - 1, point.x())),
                  std::max(0, std::min(m_image.height() - 1, point.y())));
}

void RgbImageLabel::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.fillRect(rect(), QColor(17, 17, 17));

    if (m_image.isNull()) {
        painter.setPen(QColor(190, 190, 190));
        painter.drawText(rect(), Qt::AlignCenter, text());
        return;
    }

    if (!m_viewInitialized) {
        fitImageToView();
    }
    const QRectF r = imageRect();
    painter.drawImage(r, m_image);

    painter.setPen(QColor(230, 230, 230));
    painter.drawText(QRect(10, 10, width() - 20, 22),
                     Qt::AlignLeft | Qt::AlignVCenter,
                     QStringLiteral("左键画线，滚轮缩放，右键拖动平移"));

    if (!m_hasLine) {
        return;
    }

    const QPoint p1 = imageToWidget(m_lineStart);
    const QPoint p2 = imageToWidget(m_lineEnd);
    painter.setRenderHint(QPainter::Antialiasing, true);
    QPen pen(QColor(255, 55, 65), 4);
    pen.setCapStyle(Qt::RoundCap);
    painter.setPen(pen);
    painter.drawLine(p1, p2);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(0, 220, 80));
    painter.drawEllipse(p1, 6, 6);
    painter.setBrush(QColor(255, 85, 40));
    painter.drawEllipse(p2, 6, 6);
}

void RgbImageLabel::mousePressEvent(QMouseEvent* event) {
    if (m_image.isNull()) {
        QLabel::mousePressEvent(event);
        return;
    }

    if (event->button() == Qt::RightButton) {
        m_panning = true;
        m_lastMousePos = event->pos();
        setCursor(Qt::ClosedHandCursor);
        return;
    }

    if (event->button() != Qt::LeftButton) {
        QLabel::mousePressEvent(event);
        return;
    }

    m_drawing = true;
    m_hasLine = true;
    m_lineStart = widgetToImage(event->pos());
    m_lineEnd = m_lineStart;
    update();
}

void RgbImageLabel::mouseMoveEvent(QMouseEvent* event) {
    if (m_panning && !m_image.isNull()) {
        const QPoint delta = event->pos() - m_lastMousePos;
        m_lastMousePos = event->pos();
        m_offset += QPointF(delta.x(), delta.y());
        update();
        return;
    }

    if (!m_drawing || m_image.isNull()) {
        QLabel::mouseMoveEvent(event);
        return;
    }
    m_lineEnd = widgetToImage(event->pos());
    update();
}

void RgbImageLabel::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton && m_panning) {
        m_panning = false;
        unsetCursor();
        return;
    }

    if (event->button() != Qt::LeftButton || !m_drawing || m_image.isNull()) {
        QLabel::mouseReleaseEvent(event);
        return;
    }
    m_lineEnd = widgetToImage(event->pos());
    m_drawing = false;
    update();
    if ((m_lineEnd - m_lineStart).manhattanLength() >= 3) {
        emit lineSelected(m_lineStart, m_lineEnd);
    }
}

void RgbImageLabel::wheelEvent(QWheelEvent* event) {
    if (m_image.isNull()) {
        QLabel::wheelEvent(event);
        return;
    }

    const double factor = event->angleDelta().y() > 0 ? 1.25 : 0.8;
    const double minScale = fitScale() * 0.2;
    const double maxScale = fitScale() * 30.0;
    const double newScale = std::max(minScale, std::min(maxScale, m_scale * factor));
    if (std::abs(newScale - m_scale) < 1e-9) {
        event->accept();
        return;
    }

    const QPointF cursor = event->position();
    const QPointF imageUnderCursor((cursor.x() - m_offset.x()) / m_scale,
                                   (cursor.y() - m_offset.y()) / m_scale);
    m_scale = newScale;
    m_offset = QPointF(cursor.x() - imageUnderCursor.x() * m_scale,
                       cursor.y() - imageUnderCursor.y() * m_scale);
    update();
    event->accept();
}

void RgbImageLabel::resizeEvent(QResizeEvent* event) {
    QLabel::resizeEvent(event);
    if (!m_image.isNull() && !m_viewInitialized) {
        fitImageToView();
    }
}
