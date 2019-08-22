#ifndef CAMERAMANIPSYSTEM_HPP_
#define CAMERAMANIPSYSTEM_HPP_

#include <CameraManipPluginMacros.hpp>

#include <Core/Utils/Color.hpp>

#include <Engine/Renderer/Renderer.hpp>
#include <Engine/System/System.hpp>

#include <CameraManipPlugin.hpp>

namespace Ra {
namespace Engine {
class Entity;
class RenderTechnique;
class Component;
} // namespace Engine
} // namespace Ra

namespace CameraManipPlugin {
class CAMERA_MANIP_PLUGIN_API CameraManipSystem : public Ra::Engine::System
{
  public:
    CameraManipSystem( CameraManipPluginC* plugin ) : Ra::Engine::System(), m_plugin( plugin ) {}
    ~CameraManipSystem() {}

    void generateTasks( Ra::Core::TaskQueue* taskQueue,
                        const Ra::Engine::FrameInfo& frameInfo ) override {
        m_plugin->advance();
    }

    CameraManipPluginC* m_plugin;
};

} // namespace CameraManipPlugin

#endif // CAMERAMANIPSYSTEM_HPP_
