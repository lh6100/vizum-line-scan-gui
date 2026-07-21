#include "ImageView.h"

#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace {

const double kMinimumScaleFactor = 0.05;
const double kMaximumScaleFactor = 100.0;
const double kZoomInFactor = 1.25;
const double kZoomOutFactor = 0.8;

} // namespace

ImageView::ImageView(QWidget* parent)
    : QWidget(parent) {
    setMouseTracking(true);
    setFocusPolicy(Qt::WheelFocus);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setCursor(Qt::ArrowCursor);
}

void ImageView::setImage(const QImage& image) {
    setImage(image, keepViewOnImageUpdate_);
}

void ImageView::setImage(const QImage& image, bool keepCurrentView) {
    if (image.isNull()) {
        clearImage();
        return;
    }

    const bool canKeepView = keepCurrentView && !image_.isNull() &&
                             viewInitialized_ && std::isfinite(imageScale_) &&
                             imageScale_ > 0.0;

    // Camera SDK buffers are commonly recycled immediately after a callback.
    // An explicit deep copy keeps all painting on Qt-owned memory.
    image_ = image.copy();

    if (!canKeepView || fitToWindow_) {
        fitImageToWindow();
    }
    update();
}

void ImageView::clearImage(const QString& emptyText) {
    image_ = QImage();
    if (!emptyText.isNull()) {
        emptyText_ = emptyText.isEmpty() ? QStringLiteral("暂无图像") : emptyText;
    }
    imageOffset_ = QPointF();
    imageScale_ = 1.0;
    viewInitialized_ = false;
    fitToWindow_ = true;
    panning_ = false;
    unsetCursor();
    update();
}

bool ImageView::hasImage() const {
    return !image_.isNull();
}

QImage ImageView::image() const {
    // Returning by value is safe because image_ itself already owns a deep
    // copy. Qt's implicit sharing avoids another copy until it is modified.
    return image_;
}

void ImageView::setKeepViewOnImageUpdate(bool keep) {
    keepViewOnImageUpdate_ = keep;
}

bool ImageView::keepViewOnImageUpdate() const {
    return keepViewOnImageUpdate_;
}

void ImageView::setEmptyText(const QString& text) {
    emptyText_ = text.isEmpty() ? QStringLiteral("暂无图像") : text;
    if (image_.isNull()) {
        update();
    }
}

QString ImageView::emptyText() const {
    return emptyText_;
}

void ImageView::resetView() {
    panning_ = false;
    unsetCursor();
    fitToWindow_ = true;
    fitImageToWindow();
    update();
}

QSize ImageView::sizeHint() const {
    return QSize(640, 480);
}

QSize ImageView::minimumSizeHint() const {
    return QSize(160, 120);
}

double ImageView::fittedScale() const {
    if (image_.isNull() || image_.width() <= 0 || image_.height() <= 0) {
        return 1.0;
    }

    const QRect area = contentsRect().adjusted(1, 1, -1, -1);
    if (area.width() <= 0 || area.height() <= 0) {
        return 1.0;
    }

    const double scaleX = static_cast<double>(area.width()) /
                          static_cast<double>(image_.width());
    const double scaleY = static_cast<double>(area.height()) /
                          static_cast<double>(image_.height());
    return std::max(1.0e-9, std::min(scaleX, scaleY));
}

void ImageView::fitImageToWindow() {
    if (image_.isNull()) {
        imageOffset_ = QPointF();
        imageScale_ = 1.0;
        viewInitialized_ = false;
        return;
    }

    fitToWindow_ = true;
    imageScale_ = fittedScale();
    const double displayedWidth = image_.width() * imageScale_;
    const double displayedHeight = image_.height() * imageScale_;
    const QRectF area(contentsRect());
    imageOffset_ = QPointF(
        area.left() + (area.width() - displayedWidth) * 0.5,
        area.top() + (area.height() - displayedHeight) * 0.5);
    viewInitialized_ = true;
}

QRectF ImageView::displayedImageRect() const {
    if (image_.isNull()) {
        return QRectF();
    }
    return QRectF(
        imageOffset_.x(),
        imageOffset_.y(),
        image_.width() * imageScale_,
        image_.height() * imageScale_);
}

void ImageView::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter painter(this);
    painter.fillRect(rect(), QColor(17, 17, 17));

    if (image_.isNull()) {
        painter.setPen(QColor(190, 190, 190));
        painter.drawText(contentsRect().adjusted(16, 16, -16, -16),
                         Qt::AlignCenter | Qt::TextWordWrap,
                         emptyText_.isEmpty() ? QStringLiteral("暂无图像") : emptyText_);
        painter.setPen(QColor(68, 68, 68));
        painter.drawRect(rect().adjusted(0, 0, -1, -1));
        return;
    }

    if (!viewInitialized_) {
        fitImageToWindow();
    }

    painter.setRenderHint(QPainter::SmoothPixmapTransform, imageScale_ < 1.0);
    painter.drawImage(displayedImageRect(), image_);
    painter.setPen(QColor(68, 68, 68));
    painter.drawRect(rect().adjusted(0, 0, -1, -1));
}

void ImageView::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    if (!image_.isNull() && fitToWindow_) {
        fitImageToWindow();
        update();
    }
}

void ImageView::wheelEvent(QWheelEvent* event) {
    if (image_.isNull() || event->angleDelta().y() == 0) {
        QWidget::wheelEvent(event);
        return;
    }

    if (!viewInitialized_) {
        fitImageToWindow();
    }

    const double fit = fittedScale();
    const double minimumScale = fit * kMinimumScaleFactor;
    const double maximumScale = fit * kMaximumScaleFactor;
    const double zoomFactor = event->angleDelta().y() > 0
        ? kZoomInFactor
        : kZoomOutFactor;
    const double newScale = std::max(
        minimumScale,
        std::min(maximumScale, imageScale_ * zoomFactor));

    if (std::fabs(newScale - imageScale_) <= 1.0e-12) {
        event->accept();
        return;
    }

#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
    const QPointF cursorPosition = event->position();
#else
    const QPointF cursorPosition = event->posF();
#endif
    const QPointF imagePoint(
        (cursorPosition.x() - imageOffset_.x()) / imageScale_,
        (cursorPosition.y() - imageOffset_.y()) / imageScale_);

    imageScale_ = newScale;
    imageOffset_ = QPointF(
        cursorPosition.x() - imagePoint.x() * imageScale_,
        cursorPosition.y() - imagePoint.y() * imageScale_);
    fitToWindow_ = false;
    update();
    event->accept();
}

void ImageView::mousePressEvent(QMouseEvent* event) {
    if (!image_.isNull() && event->button() == Qt::RightButton) {
        panning_ = true;
        fitToWindow_ = false;
        lastPanPosition_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void ImageView::mouseMoveEvent(QMouseEvent* event) {
    if (panning_ && !image_.isNull()) {
        const QPoint delta = event->pos() - lastPanPosition_;
        lastPanPosition_ = event->pos();
        imageOffset_ += QPointF(delta.x(), delta.y());
        update();
        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void ImageView::mouseReleaseEvent(QMouseEvent* event) {
    if (panning_ && event->button() == Qt::RightButton) {
        panning_ = false;
        unsetCursor();
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

void ImageView::mouseDoubleClickEvent(QMouseEvent* event) {
    if (!image_.isNull()) {
        resetView();
        event->accept();
        return;
    }
    QWidget::mouseDoubleClickEvent(event);
}
