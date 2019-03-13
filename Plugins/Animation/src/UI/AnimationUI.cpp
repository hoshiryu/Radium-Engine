#include "AnimationUI.h"
#include "ui_AnimationUI.h"

#include <QDebug>
#include <QFileDialog>
#include <QInputDialog>
#include <QSettings>
#include <iostream>

AnimationUI::AnimationUI( QWidget* parent ) : QFrame( parent ), ui( new Ui::AnimationUI ) {
    ui->setupUi( this );
    ui->m_play->setProperty( "pressed", false );
    ui->m_play->style()->unpolish( ui->m_play );
    ui->m_play->style()->polish( ui->m_play );
    ui->m_play->update();
    connect( ui->actionXray, &QAction::toggled, this, &AnimationUI::on_m_xray_clicked );
    connect( ui->actionXray, &QAction::toggled, this, &AnimationUI::on_m_showSkeleton_toggled );
    connect( ui->actionPlay, &QAction::toggled, this, &AnimationUI::on_m_play_clicked );
    connect( ui->actionStep, &QAction::triggered, this, &AnimationUI::on_m_step_clicked );
    connect( ui->actionStop, &QAction::triggered, this, &AnimationUI::on_m_reset_clicked );

    animTimeline = new AnimTimeline( this );
    connect( animTimeline, &AnimTimeline::cursorChanged, this, &AnimationUI::cursorChanged );
    connect( animTimeline, &AnimTimeline::startChanged, this, &AnimationUI::startChanged );
    connect( animTimeline, &AnimTimeline::endChanged, this, &AnimationUI::endChanged );
    connect( animTimeline, &AnimTimeline::keyPoseAdded, this, &AnimationUI::keyPoseAdded );
    connect( animTimeline, &AnimTimeline::keyPoseDeleted, this, &AnimationUI::keyPoseDeleted );
    connect( animTimeline, &AnimTimeline::keyPoseChanged, this, &AnimationUI::keyPoseChanged );
    connect( animTimeline, &AnimTimeline::keyPosesChanged, this, &AnimationUI::keyPosesChanged );
}

AnimationUI::~AnimationUI() {
    delete ui;
    delete animTimeline;
}

void AnimationUI::showEvent( QShowEvent* event ) {
    animTimeline->show();
}

void AnimationUI::hideEvent( QHideEvent* event ) {
    animTimeline->hide();
}

void AnimationUI::on_m_xray_clicked( bool checked ) {
    emit toggleXray( checked );
}

void AnimationUI::on_m_showSkeleton_toggled( bool checked ) {
    emit showSkeleton( checked );
}

void AnimationUI::on_m_play_clicked( bool checked ) {
    if ( checked )
    {
        ui->m_play->setChecked( true );

        ui->m_play->setProperty( "pressed", true );
        ui->m_play->style()->unpolish( ui->m_play );
        ui->m_play->style()->polish( ui->m_play );
        ui->m_play->update();

        emit play();
    } else
    {
        ui->m_play->setChecked( false );

        ui->m_play->setProperty( "pressed", false );
        ui->m_play->style()->unpolish( ui->m_play );
        ui->m_play->style()->polish( ui->m_play );
        ui->m_play->update();

        emit pause();
    }
}

void AnimationUI::on_m_step_clicked() {
    ui->m_play->setChecked( false );

    ui->m_play->setProperty( "pressed", false );
    ui->m_play->style()->unpolish( ui->m_play );
    ui->m_play->style()->polish( ui->m_play );
    ui->m_play->update();

    emit step();
}

void AnimationUI::on_m_reset_clicked() {
    ui->m_play->setChecked( false );

    ui->m_play->setProperty( "pressed", false );
    ui->m_play->style()->unpolish( ui->m_play );
    ui->m_play->style()->polish( ui->m_play );
    ui->m_play->update();

    emit stop();
}

void AnimationUI::on_m_timeStep_currentIndexChanged( int index ) {
    emit toggleAnimationTimeStep( ( index == 0 ) );
}

void AnimationUI::on_m_speed_valueChanged( double arg1 ) {
    emit animationSpeed( arg1 );
}

void AnimationUI::on_m_slowMo_toggled( bool checked ) {
    emit toggleSlowMotion( checked );
}

void AnimationUI::on_m_cacheFrame_clicked() {
    emit cacheFrame();
}

void AnimationUI::on_m_restoreFrame_clicked() {
    emit restoreFrame( ui->m_loadedFrame->value() );
}

void AnimationUI::frameLoaded( int f ) {
    ui->m_currentFrame->setText( QString::number( f ) );
}

void AnimationUI::setMaxFrame( int f ) {
    ui->m_loadedFrame->setMaximum( f );
}

void AnimationUI::on_m_saveDir_clicked() {
    emit changeDataDir();
}

void AnimationUI::updateTime( float t ) {
    ui->m_animationTimeDisplay->setText( QString::number( t ) );
}

void AnimationUI::updateFrame( int f ) {
    ui->m_currentFrame->setText( QString::number( f ) );
}

void AnimationUI::on_comboBox_currentPlayZone_currentIndexChanged( int index ) {
    emit playZoneID( index );
}

void AnimationUI::on_pushButton_newPlayZone_clicked() {
    bool ok;
    QString text = QInputDialog::getText( this, tr( "Adding PlayZone" ), tr( "PlayZone name :" ),
                                          QLineEdit::Normal, "", &ok );

    if ( ok && !text.isEmpty() )
    {
        ui->comboBox_currentPlayZone->addItem( text );
        emit newPlayzone();
    }
}

void AnimationUI::on_pushButton_removePlayZone_clicked() {
    int removeIndex = ui->comboBox_currentPlayZone->currentIndex();
    ui->comboBox_currentPlayZone->removeItem( removeIndex );

    emit removePlayzone( removeIndex );
}

void AnimationUI::on_comboBox_currentAnimation_currentIndexChanged( int index ) {
    emit animationID( index );
}

void AnimationUI::on_pushButton_newAnimation_clicked() {
    bool ok;
    QString text = QInputDialog::getText( this, tr( "Adding Animation" ), tr( "Animation name :" ),
                                          QLineEdit::Normal, "", &ok );

    if ( ok && !text.isEmpty() )
    {
        ui->comboBox_currentAnimation->addItem( text );
        emit newAnimation();
    }
}

void AnimationUI::on_pushButton_removeAnimation_clicked() {
    int removeIndex = ui->comboBox_currentAnimation->currentIndex();
    ui->comboBox_currentAnimation->removeItem( removeIndex );

    emit removeAnimation( removeIndex );
}

void AnimationUI::on_pushButton_loadRdmaFile_clicked() {
    QSettings settings;
    QString path = settings.value( "files/load", QDir::homePath() ).toString();
    QString filename = QFileDialog::getOpenFileName( this, "Open RDMA file", path,
                                                     "Radium Animation File (*.rdma)" );

    if ( !filename.isEmpty() )
    {
        ui->label_currentRDMA->setText( filename );
        emit loadRDMA( filename.toStdString() );
    }
}

void AnimationUI::on_pushButton_saveRdma_clicked() {
    emit saveRDMA( ui->label_currentRDMA->text().toStdString() );
}

void AnimationUI::on_pushButton_newRdmaFile_clicked() {
    QSettings settings;
    QString path = settings.value( "files/load", QDir::homePath() ).toString();
    QString filename = QFileDialog::getSaveFileName( this, "Open RDMA file", path,
                                                     "Radium Animation File (*.rdma)" );

    if ( !filename.isEmpty() )
    {
        ui->label_currentRDMA->setText( filename );
        emit newRDMA( filename.toStdString() );
    }
}
