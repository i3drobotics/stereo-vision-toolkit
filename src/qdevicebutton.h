#ifndef QDEVICEBUTTON_H
#define QDEVICEBUTTON_H

#include <QPushButton>
#include <QPainter>

class QDeviceButton : public QPushButton
{
public:
    explicit QDeviceButton(QWidget* parent = nullptr);
    virtual ~QDeviceButton();

    void setText(QString text);
    void setPixmap(const QPixmap& pixmap);

    virtual QSize sizeHint() const override;

protected:
    virtual void paintEvent(QPaintEvent* e) override;

private:
    QPixmap m_pixmap;
    QString m_text;
    int x_margin = 0;
    int y_margin = 20;
};

#endif // QDEVICEBUTTON_H
