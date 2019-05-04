#ifndef QLABELSLIDER_H
#define QLABELSLIDER_H

#include <QLabel>

class QLabelSlider : public QLabel {
    Q_OBJECT
public:
    explicit QLabelSlider(QWidget* parent = nullptr);

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;

signals:
    void slide(int deltaX);
    void slideRelease();

public slots:

private:
    bool clicked = false;
    bool align[3];
    int x;
};

#endif // QLABELSLIDER_H
