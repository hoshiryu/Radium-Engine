#ifndef CAMERAMANIPPLUGIN_HPP_
#define CAMERAMANIPPLUGIN_HPP_

#include <Core/CoreMacros.hpp>
#include <PluginBase/RadiumPluginInterface.hpp>
#include <QObject>

#include <map>

#include <CameraManipPluginMacros.hpp>
#include <UI/CameraManipUI.h>

namespace Ra {
namespace Engine {
class RadiumEngine;
class Entity;
} // namespace Engine
} // namespace Ra

namespace CameraManipPlugin {

class CameraManipSystem;

// Due to an ambigous name while compiling with Clang, must differentiate plugin class from plugin
// namespace
/// The CameraManipPlugin allows to change the current camera used for display,
/// create a copy of the current camera and save the current camera data to a file.
class CameraManipPluginC : public QObject, Ra::Plugins::RadiumPluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA( IID "RadiumEngine.PluginInterface" )
    Q_INTERFACES( Ra::Plugins::RadiumPluginInterface )

  public:
    CameraManipPluginC();
    ~CameraManipPluginC();

    void registerPlugin( const Ra::Plugins::Context& context ) override;

    bool doAddWidget( QString& name ) override;
    QWidget* getWidget() override;

    bool doAddMenu() override;
    QMenu* getMenu() override;

    bool doAddAction( int& nb ) override;
    QAction* getAction( int id ) override;

  public slots:
    void useSelectedCamera();
    void saveCamera();
    void createCamera();
    void addKeyFrame();
    void showFrames( bool show );
    void playPath( bool play );
    void resetPath();
    void advance();
    void savePath() const;
    void loadPath();
    void onCurrentChanged( const QModelIndex& current, const QModelIndex& prev );

  signals:
    void setContinuousUpdate( bool on );
    void askForUpdate();

  private:
    CameraManipUI* m_widget;
    CameraManipSystem* m_system;

    Ra::Engine::RadiumEngine* m_engine;
    Ra::GuiBase::SelectionManager* m_selectionManager;
    Ra::Gui::Viewer* m_viewer;

    std::map<Scalar, Ra::Engine::Camera*> m_keyFrames;
    Scalar m_time{0_ra};
    bool m_isPlaying{false};
    bool m_isReset{true};
};

} // namespace CameraManipPlugin

#endif // CAMERAMANIPPLUGIN_HPP_
