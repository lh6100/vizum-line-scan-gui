#ifndef RGBIMAGELABEL_H
#define RGBIMAGELABEL_H

#include <QImage>
#include <QLabel>
#include <QPoint>
#include <QPointF>
#include <QRectF>

class RgbImageLabel : public QLabel {
    Q_OBJECT
public:
    explicit RgbImageLabel(QWidget* parent = nullptr);

    void setImage(const QImage& image);
    bool hasImage() const { return !m_image.isNull(); }
    void resetView();

signals:
    void lineSelected(QPoint start, QPoint end);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    QRectF imageRect() const;
    QPoint widgetToImage(const QPoint& point) const;
    QPoint imageToWidget(const QPoint& point) const;
    QPoint clampImagePoint(const QPoint& point) const;
    void fitImageToView();
    double fitScale() const;

    QImage m_image;
    QPoint m_lineStart;
    QPoint m_lineEnd;
    QPoint m_lastMousePos;
    QPointF m_offset{0.0, 0.0};
    double m_scale{1.0};
    bool m_hasLine{false};
    bool m_drawing{false};
    bool m_panning{false};
    bool m_viewInitialized{false};
};

#endif // RGBIMAGELABEL_H
