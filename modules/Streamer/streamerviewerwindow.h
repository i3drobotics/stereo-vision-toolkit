#ifndef STREAMER_VIEWER_WINDOW_H
#define STREAMER_VIEWER_WINDOW_H

#include <QMainWindow>

namespace Ui {
class StreamerViewerWindow;
}

class StreamerViewerWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit StreamerViewerWindow(QWidget *parent = nullptr);
    ~StreamerViewerWindow();

private:
    Ui::StreamerViewerWindow *ui;
};

#endif // STREAMER_VIEWER_WINDOW_H
