/*
* Copyright I3D Robotics Ltd, 2020
* Author: Josh Veitch-Michaelis, Ben Knight (bknight@i3drobotics.com)
*/

#ifndef MATCHERWIDGET_H
#define MATCHERWIDGET_H

#include <QWidget>
#include <abstractstereomatcher.h>

//!  Matcher QT Widget
/*!
  Base class for stereo matcher QT widgets
*/

class MatcherWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MatcherWidget(QWidget *parent = 0) : QWidget(parent) {}
    virtual AbstractStereoMatcher* getMatcher(void) = 0;

signals:

public slots:
};

#endif // MATCHERWIDGET_H
