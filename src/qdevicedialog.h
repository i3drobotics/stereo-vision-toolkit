#ifndef QDEVICEDIALOG_H
#define QDEVICEDIALOG_H


#include <QDialog>
#include <QtGui>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>

class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;

class QDeviceDialog : public QDialog
{
    Q_OBJECT
public:
    QDeviceDialog(QWidget *parent = 0);

signals:
    void findNext(const QString &str, Qt::CaseSensitivity cs);
    void findPrevious(const QString &str, Qt::CaseSensitivity cs);


private slots:
    void findClicked();
    void enableFindButton(const QString &text);
private:
    QLabel *label;
    QLineEdit *lineEdit;
    QCheckBox *caseCheckBox;
    QCheckBox *backwardCheckBox;
    QPushButton *findButton;
    QPushButton *closeButton;
};

#endif // QDEVICEDIALOG_H
