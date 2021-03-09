#pragma once

#include <Core/Animation/Pose.hpp>
#include <Core/Animation/SkinningData.hpp>
#include <Core/Containers/AlignedStdVector.hpp>
#include <Core/Containers/VectorArray.hpp>
#include <Core/Math/DualQuaternion.hpp>

namespace Ra {
namespace Core {
namespace Animation {

using DQList = AlignedStdVector<DualQuaternion>;

/*
 * computeDQ computes the dual quaternions from a given pose and a given set of skinning weights.
 *
 * WARNING : in Debug the function will assert if pose and weight size mismatch. In Release will
 * simply crash.
 */
void RA_CORE_API computeDQ( const Pose& pose, const Sparse& weight, DQList& DQ );

// Same version, without the parallelism for reference purposes (see github issue #118)
void RA_CORE_API computeDQ_naive( const Pose& pose, const Sparse& weight, DQList& DQ );

/*
 * DualQuaternionSkinning applies a set of dual quaternions to a given input set of vertices and
 * returns the resulting transformed vertices.
 *
 * WARNING : in Debug the function will assert if input and DQ size mismatch. In Release will simply
 * crash.
 */
void RA_CORE_API dualQuaternionSkinning( const Ra::Core::Vector3Array& input,
                                         const DQList& DQ,
                                         Ra::Core::Vector3Array& output );

void RA_CORE_API accurateLightingDQS( const Skinning::RefData& refData,
                                      const DQList& DQ,
                                      const Vector3Array& tangents,
                                      const Vector3Array& bitangents,
                                      Skinning::FrameData& frameData );

} // namespace Animation
} // namespace Core
} // namespace Ra
