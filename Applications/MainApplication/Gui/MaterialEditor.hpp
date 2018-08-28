#ifndef RADIUMENGINE_MATERIALEDITOR_HPP
#define RADIUMENGINE_MATERIALEDITOR_HPP

#include <QWidget>

#include <memory>

#include <Core/Index/Index.hpp>

#include <ui_MaterialEditor.h>

class QCloseEvent;
class QShowEvent;

namespace Ra {
namespace Engine {
class RadiumEngine;
class RenderObjectManager;
class RenderObject;
class Material;
class BlinnPhongMaterial;
} // namespace Engine
} // namespace Ra

namespace Ra {
namespace Gui {
/// The MaterialEditor is a simple Widget allowing the modification of the material parameters of a
/// RenderObject. For now, it only handles `BlinnPhongMaterial` parameters.
class MaterialEditor : public QWidget, private Ui::MaterialEditor {
    Q_OBJECT

  public:
    MaterialEditor( QWidget* parent = nullptr );

    /// Change the RenderObject to edit.
    void changeRenderObject( Ra::Core::Index roIdx );

  private slots:
    /// Updates the preview of the material when changing any parameter.
    void updateMaterialViz();

    /// Slot for the user changing the Kd parameter.
    void onKdColorChanged( int );

    /// Slot for the user changing the Ks parameter.
    void onKsColorChanged( int );

    /// Slot for the user changing the Exp parameter.
    void onExpChanged( double );

    /// Slot for the user picking the new Kd color.
    void newKdColor( const QColor& color );

    /// Slot for the user picking the new Ks color.
    void newKsColor( const QColor& color );

  protected:
    void showEvent( QShowEvent* e ) override;
    void closeEvent( QCloseEvent* e ) override;

  private:
    /// Whether this widget is showing or not.
    bool m_visible;

    /// The Engine RenderObjects belong to.
    Engine::RadiumEngine* m_engine;

    /// The RenderObjectManager of the Engine.
    Engine::RenderObjectManager* m_roMgr;

    /// The current RenderObject index.
    Core::Index m_roIdx;

    /// The current RenderObject.
    std::shared_ptr<Engine::RenderObject> m_renderObject;

    // TODO generalize material editor to others materials
    /// Whether we can edit the current material or not.
    bool m_usable;

    /// The current material to be edited.
    Ra::Engine::BlinnPhongMaterial* m_material;

  private:
    // FIXME (florian): unused?
    enum {
        OUTPUT_FINAL = 0,
        OUTPUT_DIFFUSE = 1,
        OUTPUT_SPECULAR = 2,
        OUTPUT_NORMAL = 3,
    };
};
} // namespace Gui
} // namespace Ra

#endif // RADIUMENGINE_MATERIALEDITOR_HPP
