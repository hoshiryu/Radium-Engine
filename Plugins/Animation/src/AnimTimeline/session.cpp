#include "configurations.h"
#include <AnimTimeline/configurations.h>
#include <AnimTimeline/session.h>

#include <QTimer>
#include <set>
//#include <boost/circular_buffer.hpp> // need boost lib

Session::Session( QObject* parent ) : QObject( parent ) {
    saveDelay = new QTimer( this );
    connect( saveDelay, &QTimer::timeout, this, &Session::envSaved );
    saveDelay->setSingleShot( true );
}

Session::~Session() {
    onClearSession();
    delete saveDelay;
}

// timeline changed, save timeline and anim environment
void Session::onChangeEnv() {
    saveDelay->start( DELAY_AUTO_SAVE );
}

void Session::onClearSession() {
    while ( !undo.empty() )
    {
        emit renderDeleted( undo.back().anim );
        undo.pop_back();
    }
    while ( !redoHeap.empty() )
    {
        emit renderDeleted( redoHeap.top().anim );
        redoHeap.pop();
    }

    size = 0;

    onChangeEnv();
}

void Session::onUndo() {
    if ( undo.empty() ) { return; }

    if ( undo.size() > 1 )
    {
        redoHeap.push( undo.back() );
        undo.pop_back();

        setEnv( undo.back() );
    }
}

void Session::onRedo() {
    if ( redoHeap.empty() ) { return; }

    undo.emplace_back( redoHeap.top() );
    redoHeap.pop();

    setEnv( undo.back() );
}

void Session::onSaveRendering( void* anim, size_t bytes ) {

    while ( !redoHeap.empty() )
    {
        emit renderDeleted( redoHeap.top().anim );
        size -= redoHeap.top().bytes;
        redoHeap.pop();
    }

    undo.emplace_back( Env{*start, *end, *cursor, *duration, *keyPoses, anim, bytes} );
    size += bytes;

    while ( size > BUFFER_SESSION_MAX_SIZE )
    {
        emit renderDeleted( undo.front().anim );
        size -= undo.front().bytes;
        undo.pop_front();
    }
}

void Session::setEnv( Env env ) {
    *start = env.start;
    selector->updateStartSpin();

    *end = env.end;
    selector->updateEndSpin();

    *keyPoses = env.keyPoses;
    nbKeyPosesSpin->setValue( static_cast<int>( keyPoses->size() ) );

    selector->updateCursorSpin();

    *duration = env.duration;
    selector->updateDurationSpin();

    selector->redrawPlayZone();
    ruler->onDrawRuler( ruler->width() );

    emit rendered( env.anim );
}

void Session::setKeyPoses( std::set<double>* value ) {
    keyPoses = value;
}

void Session::setDuration( double* value ) {
    duration = value;
}

void Session::setCursor( double* value ) {
    cursor = value;
}

void Session::setEnd( double* value ) {
    end = value;
}

void Session::setStart( double* value ) {
    start = value;
}

void Session::setDurationSpin( QDoubleSpinBoxSmart* value ) {
    durationSpin = value;
}

void Session::setCursorSpin( QDoubleSpinBoxSmart* value ) {
    cursorSpin = value;
}

void Session::setEndSpin( QDoubleSpinBoxSmart* value ) {
    endSpin = value;
}

void Session::setStartSpin( QDoubleSpinBoxSmart* value ) {
    startSpin = value;
}

void Session::setNbKeyPosesSpin( QSpinBoxSmart* value ) {
    nbKeyPosesSpin = value;
}

void Session::setSelector( QFrameSelector* value ) {
    selector = value;
}

void Session::setRuler( QWidgetRuler* value ) {
    ruler = value;
}

void Session::setPlayButton( QToolButtonPlayPause* value ) {
    playButton = value;
}
