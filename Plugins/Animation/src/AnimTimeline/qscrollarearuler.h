#ifndef QSCROLLAREARULER_H
#define QSCROLLAREARULER_H

#include <AnimTimeline/animtimeline.h>

#include <AnimTimeline/animtimeline.h>
#include <AnimTimeline/qwidgetruler.h>
#include <AnimTimeline/qdoublespinboxsmart.h>
#include <AnimTimeline/qframeselector.h>

#include <QObject>
#include <QScrollArea>
#include <QWidget>

class QScrollAreaRuler : public QScrollArea {
    Q_OBJECT
public:
    explicit QScrollAreaRuler(QWidget* parent = nullptr);

    bool* getShiftDown();
    void setRuler(QWidgetRuler* value);
    void setPlayPause(QToolButtonPlayPause* value);
    void setAnimTimeline(AnimTimeline* value);
    void setSpinDuration(QDoubleSpinBoxSmart* value);
    bool* getCtrlDown();
    void setSelector(QFrameSelector* value);
    bool* getMidMouseDown();
    void setCursorSpin(QDoubleSpinBoxSmart* value);

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

    void wheelEvent(QWheelEvent* event) override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;

signals:
    void keyPoseOnMouseAdded();
    void removeKeyPose();
    void previousKeyPose();
    void nextKeyPose();
    void durationChanged(double time);
    void addKeyPose();

    void undo();
    void redo();

public slots:
    void onKeyPress(QKeyEvent* event);
    void onKeyRelease(QKeyEvent* event);

private:
    bool ctrlDown = false;
    bool shiftDown = false;
    bool midMouseDown = false;
    bool align[5];

    int mousePosX;
    int sliderPos;

    QWidgetRuler* ruler;
    QToolButtonPlayPause* playPause;
    AnimTimeline* animTimeline;
    QDoubleSpinBoxSmart* spinDuration;

    QDoubleSpinBoxSmart* cursorSpin;

    QFrameSelector* selector;
};

#endif // QSCROLLAREARULER_H
