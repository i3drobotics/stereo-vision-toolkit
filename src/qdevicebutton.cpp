#include "qdevicebutton.h"

QDeviceButton::QDeviceButton(QWidget* parent) : QPushButton(parent)
{
}

QDeviceButton::~QDeviceButton()
{
}

QSize QDeviceButton::sizeHint() const
{
    const auto parentHint = QPushButton::sizeHint();
    // add margins here if needed
    return QSize(parentHint.width() + m_pixmap.width() + x_margin,
                 std::max(parentHint.height(), m_pixmap.height()) + y_margin);
}

void QDeviceButton::setText(QString text){
    m_text = text;
}

void QDeviceButton::setPixmap(const QPixmap& pixmap)
{
    m_pixmap = pixmap;
}

void QDeviceButton::paintEvent(QPaintEvent* e)
{
    QPushButton::paintEvent(e);

    if (!m_pixmap.isNull())
    {
        const int y = ((height() - m_pixmap.height()) / 2) - (y_margin/2);
        const int x = ((width() - m_pixmap.width()) / 2)  - (x_margin/2);
        QPainter painter(this);
        painter.drawPixmap(x, y, m_pixmap); // hardcoded horizontal margin
        painter.drawText(QRectF(x, y_margin - 5, m_pixmap.width(), m_pixmap.height()),Qt::AlignCenter | Qt::AlignBottom,m_text);
    }
}
