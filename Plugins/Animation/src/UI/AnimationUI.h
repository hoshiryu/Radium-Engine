#ifndef ANIMATIONUI_H
#define ANIMATIONUI_H

#include <AnimTimeline/animtimeline.h>
#include <QFrame>

namespace UI {
class AnimationUi;
}

namespace AnimationPlugin {
class AnimationPluginC;
}

namespace Ui {
class AnimationUI;
}

class AnimationUI : public QFrame {
    Q_OBJECT

    friend class AnimationPlugin::AnimationPluginC;

  public:
    explicit AnimationUI( QWidget* parent = 0 );
    ~AnimationUI();

    void setAnimationComboBox( int count, int nonEditableCount );
    void setPlayzoneComboBox( const std::vector<std::string> labels );
    void setKeyPoses( std::vector<double> timestamps );
    void switchToPlayButton();
    void switchToPauseButton();
    void removeAnimItem( int index );
    void showTimeline();

  protected:
    void showEvent( QShowEvent* event ) override;
    void hideEvent( QHideEvent* event ) override;

  signals:
    void toggleXray( bool );
    void showSkeleton( bool );
    void play();
    void pause();
    void step();
    void stop();
    void animationID( int );
    void toggleAnimationTimeStep( bool );
    void animationSpeed( double );
    void toggleSlowMotion( bool );
    void cacheFrame();
    void restoreFrame( int );
    void changeDataDir();

    void playZoneID( int );
    void newPlayzone( const std::string& );
    void removePlayzone( int );
    void newAnimation();
    void removeAnimation( int );
    void loadRDMA( const std::string& filename );
    void saveRDMA( const std::string& filename );

    /// Timeline signals
    void durationChanged( double );
    void cursorChanged( double );
    void startChanged( double );
    void endChanged( double );
    void keyPoseAdded( double );
    void keyPoseDeleted( int );
    void keyPoseChanged( int, double );
    void keyPosesChanged( double );

    void changeCursor( double );
    void addKeyPose( double );
    void clearKeyPoses();
    void changeStart( double );
    void changeEnd( double );
    void changeDuration( double );

  private slots:
    void on_m_xray_clicked( bool checked );
    void on_m_showSkeleton_toggled( bool checked );
    void on_m_play_clicked();
    void on_m_step_clicked();
    void on_m_reset_clicked();
    void on_m_timeStep_currentIndexChanged( int index );
    void on_m_speed_valueChanged( double arg1 );
    void on_m_slowMo_toggled( bool checked );
    void on_m_cacheFrame_clicked();
    void on_m_restoreFrame_clicked();
    void frameLoaded( int f );
    void setMaxFrame( int f );
    void on_m_saveDir_clicked();

    void on_comboBox_currentPlayZone_currentIndexChanged( int index );
    void on_pushButton_newPlayZone_clicked();
    void on_pushButton_removePlayZone_clicked();
    void on_comboBox_currentAnimation_currentIndexChanged( int index );
    void on_pushButton_newAnimation_clicked();
    void on_pushButton_removeAnimation_clicked();
    void on_pushButton_loadRdmaFile_clicked();
    void on_pushButton_saveRdmaFile_clicked();
    void on_pushButton_newRdmaFile_clicked();

  private:
    Ui::AnimationUI* ui;
    AnimTimeline* animTimeline;

    void updateTime( float t );
    void updateFrame( int f );
};

#endif // ANIMATIONUI_H
