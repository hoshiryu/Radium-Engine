#ifndef RADIUMENGINE_SHADERCONFIGFACTORY_HPP
#define RADIUMENGINE_SHADERCONFIGFACTORY_HPP

#include <string>

#include <Engine/Renderer/RenderTechnique/ShaderConfiguration.hpp>

namespace Ra {
namespace Engine {

/**
 * Factory that manage the set of named shader configurations.
 * Usefull for the management of shader libraries where a shader configuration
 * (source code, defines, properties) are defined once and re-used multiple times.
 */
namespace ShaderConfigurationFactory {
/**
 * Add a configuration to the factory.
 * \note In case of name collision, the configuration is not added to the factory.
 */
RA_ENGINE_API void addConfiguration( const ShaderConfiguration& config );

/**
 * Get a configuration from the factory.
 * \param name The configuration to get.
 * \return The configuration or Default.
 * \note If the requested configuration is not in the factory,
 *       returns the default shader configuration.
 */
RA_ENGINE_API ShaderConfiguration getConfiguration( const std::string& name );
} // namespace ShaderConfigurationFactory

} // namespace Engine
} // namespace Ra

#endif // RADIUMENGINE_SHADERCONFIGFACTORY_HPP
