#ifndef MYLINE_HIK_IMAGE_VIEW_H
#define MYLINE_HIK_IMAGE_VIEW_H

#include <QImage>
#include <QPoint>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QWidget>

class QMouseEvent;
class QPaintEvent;
class QResizeEvent;
class QWheelEvent;

class ImageView : public QWidget {
public:
    explicit ImageView(QWidget* parent = nullptr);

    // QImage::copy() is used so the view never retains camera-owned memory.
    void setImage(const QImage& image);
    void setImage(const QImage& image, bool keepCurrentView);
    void clearImage(const QString& emptyText = QString());

    bool hasImage() const;
    QImage image() const;

    void setKeepViewOnImageUpdate(bool keep);
    bool keepViewOnImageUpdate() const;

    void setEmptyText(const QString& text);
    QString emptyText() const;

    void resetView();

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;

private:
    double fittedScale() const;
    void fitImageToWindow();
    QRectF displayedImageRect() const;

    QImage image_;
    QString emptyText_{QStringLiteral("暂无图像")};
    QPointF imageOffset_;
    QPoint lastPanPosition_;
    double imageScale_{1.0};
    bool keepViewOnImageUpdate_{false};
    bool viewInitialized_{false};
    bool fitToWindow_{true};
    bool panning_{false};
};

#endif // MYLINE_HIK_IMAGE_VIEW_H
