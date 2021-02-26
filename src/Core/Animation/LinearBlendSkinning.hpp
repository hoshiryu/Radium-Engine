#pragma once

#include <Core/Animation/HandleWeight.hpp>
#include <Core/Animation/Pose.hpp>
#include <Core/Animation/SkinningData.hpp>
#include <Core/Containers/VectorArray.hpp>
#include <Core/Types.hpp>

namespace Ra {
namespace Core {
namespace Animation {

void RA_CORE_API linearBlendSkinning( const Vector3Array& inMesh,
                                      const Pose& pose,
                                      const WeightMatrix& weight,
                                      Vector3Array& outMesh );

/// \brief Computes the LBS using the paper:
/// http://vcg.isti.cnr.it/Publications/2014/TPS14/skin_light.pdf
void RA_CORE_API accurateLightningLBS( const Skinning::RefData& refData,
                                       const Vector3Array& tangents,
                                       const Vector3Array& bitangents,
                                       Skinning::FrameData& frameData );

} // namespace Animation
} // namespace Core
} // namespace Ra
