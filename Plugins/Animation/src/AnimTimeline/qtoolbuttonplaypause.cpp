#include <AnimTimeline/qtoolbuttonplaypause.h>

#include <QMouseEvent>

QToolButtonPlayPause::QToolButtonPlayPause( QWidget* parent ) : QToolButton( parent ) {

    playIcon = new QIcon();
    playIcon->addPixmap( QPixmap( ":/images/play.png" ) );
    pauseIcon = new QIcon();
    pauseIcon->addPixmap( QPixmap( ":/images/pause.png" ) );

    this->setIcon( *playIcon );
}

QToolButtonPlayPause::~QToolButtonPlayPause() {
    delete playIcon;
    delete pauseIcon;
}

void QToolButtonPlayPause::mousePressEvent( QMouseEvent* event ) {
    if ( event->button() == Qt::LeftButton )
    {
        onChangeMode();
        event->accept();
    }
}

// EXTERNAL SLOT
void QToolButtonPlayPause::onPlayMode() {
    if ( play ) { return; }

    this->setIcon( *pauseIcon );
    play = true;
}

// EXTERNAL SLOT
void QToolButtonPlayPause::onPauseMode() {
    if ( !play ) { return; }

    this->setIcon( *playIcon );
    play = false;
}

void QToolButtonPlayPause::onChangeMode() {
    if ( play )
    {
        onPauseMode();
        emit pauseClicked();
    }
    else
    {
        onPlayMode();
        emit playClicked();
    }
}

bool* QToolButtonPlayPause::getPlay() {
    return &play;
}
