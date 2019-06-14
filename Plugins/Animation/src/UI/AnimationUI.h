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

class AnimationUI : public QFrame
{
    Q_OBJECT

    friend class AnimationPlugin::AnimationPluginC;

  public:
    explicit AnimationUI( QWidget* parent = nullptr );
    ~AnimationUI();

    /// Set the animation combo box with count elements and add a prefix to the nonEditableCount
    /// firsts.
    void setAnimationComboBox( int count, int nonEditableCount );
    /// Set the playzone combo box with the labels provided.
    void setPlayzoneComboBox( const std::vector<std::string> labels );
    /// Add marks on the timeline at the given timestamps.
    void setKeyPoses( std::vector<double> timestamps );
    /// Set the main button to the play button.
    void switchToPlayButton();
    /// Set the main button to the pause button.
    void switchToPauseButton();
    /// Remove an item from the animation combo box.
    void removeAnimItem( int index );
    /// Show the timeline.
    void showTimeline();

  protected:
    void showEvent( QShowEvent* event ) override;
    void hideEvent( QHideEvent* event ) override;

  signals:
    void toggleXray( bool );
    void showSkeleton( bool );
    void enableIK( bool );
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

    /// \brief Emitted by the UI for the plugin when selecting a playzone.
    /// \param the ID of the playzone selected.
    void playzoneID( int );
    /// \brief Emitted by the UI for the plugin when creating a playzone.
    /// \param the label of the playzone.
    void newPlayzone( const std::string& );
    /// \brief Emitted by the UI for the plugin when deleting a playzone.
    /// \param the ID of the playzone removed.
    void removePlayzone( int );
    /// \brief Emitted by the UI for the plugin when creating an animation.
    void newAnimation();
    /// \brief Emitted by the UI for the plugin when selecting a playzone.
    /// \param the ID of the playzone selected.
    void removeAnimation( int );
    /// \brief Emitted by the UI for the plugin when loading an RDMA file.
    /// \param filename: the name of the file to load from.
    void loadRDMA( const std::string& filename );
    /// \brief Emitted by the UI for the plugin when saving to an RDMA file.
    /// \param filename: the name of the file to write in.
    void saveRDMA( const std::string& filename );

    //// Timeline signals ////

    /// \brief Forwarded from the timeline when the cursor is moved.
    /// \param the timestamp of the cursor.
    void cursorChanged( double );

    /// \brief Forwarded from the timeline when the start of the playzone is moved.
    /// \param the timestamp of the playzone start.
    void startChanged( double );

    /// \brief Forwarded from the timeline when the end of the playzone is moved.
    /// \param the timestamp of the playzone end.
    void endChanged( double );

    /// \brief Forwarded from the timeline when a key pose is added.
    /// \param the index of the pose to delete.
    void keyPoseAdded( double );

    /// \brief Forwarded from the timeline when a key pose is deleted.
    /// \param the index of the pose to delete.
    void keyPoseDeleted( size_t );

    /// \brief Forwarded from the timeline when a key pose is replaced.
    /// \param the index of the pose to replace.
    void keyPoseChanged( size_t );

    /// \brief Forwarded from the timeline when an offset should be added to every pose after the
    /// index specified (included). \param offset: the offset to add to the poses. \param index: the
    /// index of the first pose to offset.
    void keyPosesMoved( double offset, size_t index );

    /// \brief Forwarded from the timeline when an offset should be added to a pose.
    /// \param the index of the pose to move.
    /// \param the new timestamp of the pose.
    void keyPoseMoved( size_t, double );

    /// \brief Save current environment (minimal just Animation and Playzone)
    /// allow future rendering when signals session comming (undo/redo)
    void envSaved();

    /// \brief Render previous saved environment
    /// \param generic parameter to rendering (use void * because Q_OBJECT unauthorize template
    /// class)
    void rendered( void* anim );

    /// \brief Delete previous instance of environment
    /// \param generic parameter to delete
    void renderDeleted( void* anim );

    /// \brief Forwarded from the plugin when the animation time change when playing.
    /// \param the timestamp where to put the cursor.
    void changeCursor( double );

    /// \brief Forwarded from the plugin when loading new poses.
    /// \param the timestamp where to add the pose.
    void addKeyPose( double );

    /// \brief Forwarded from the plugin when a new animation or playzone is selected.
    void clearKeyPoses();

    /// \brief Forwarded from the plugin when loading new poses.
    /// \param the timestamp where to add the pose.
    void changeStart( double );

    /// \brief Forwarded from the plugin when selecting a playzone.
    /// \param the timestamp where to set the end.
    void changeEnd( double );

    /// \brief Forwarded from the plugin when selecting a playzone.
    /// \param the duration to show on the timeline.
    void changeDuration( double );

  private slots:
    // Rig
    void on_m_xray_clicked( bool checked );
    void on_m_showSkeleton_toggled( bool checked );
    void on_m_enableIK_toggled( bool checked );

    // Player
    void on_m_play_clicked( bool checked );
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

    // edition
    void on_m_showTimeLine_clicked();
    void on_m_currentAnimation_currentIndexChanged( int index );
    void on_m_newAnimation_clicked();
    void on_m_removeAnimation_clicked();
    void on_m_currentPlayZone_currentIndexChanged( int index );
    void on_m_newPlayZone_clicked();
    void on_m_removePlayZone_clicked();
    void on_m_loadRdmaFile_clicked();
    void on_m_saveRdmaFile_clicked();
    void on_m_newRdmaFile_clicked();

    // timeline
    void on_saveRendering( void* anim, size_t bytes ); // undo/redo session

  private:
    Ui::AnimationUI* ui;
    AnimTimelineWithSession* timeline; // for undo/redo

    void updateTime( float t );
    void updateFrame( int f );
};

#endif // ANIMATIONUI_H
