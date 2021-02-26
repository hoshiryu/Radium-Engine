#pragma once

#include <Core/Animation/HandleWeight.hpp>
#include <Core/Animation/Pose.hpp>
#include <Core/Animation/Skeleton.hpp>
#include <Core/Geometry/TriangleMesh.hpp>
#include <Core/RaCore.hpp>

namespace Ra {
namespace Core {
namespace Skinning {

/// Skinning data that gets set at startup including the "reference state"
struct RefData {
    /// Skeleton
    Animation::Skeleton m_skeleton;

    /// Mesh in reference position
    Geometry::TriangleMesh m_referenceMesh;

    /// Reference pose
    Animation::Pose m_refPose;

    /// Skinning weights.
    Animation::WeightMatrix m_weights;

    /// The per-bone bind matrices.
    std::map<uint, Transform> m_bindMatrices;

    /// The per-vertex alpha-beta deform factors.
    std::vector<std::vector<std::tuple<uint,Scalar,Scalar>>> m_alphaBeta;

    /// Optionnal centers of rotations for CoR skinning.
    Vector3Array m_CoR;
};

/// Pose data of one frame. Poses are in model space
struct FrameData {
    /// Pose of the previous frame.
    Animation::Pose m_previousPose;

    /// Pose of the current frame.
    Animation::Pose m_currentPose;

    /// Relative pose from previous to current
    Animation::Pose m_prevToCurrentRelPose;

    /// Relative pose from reference pose to current.
    Animation::Pose m_refToCurrentRelPose;

    /// Previous position of the vertices
    Vector3Array m_previousPos;

    /// Current position of the vertices
    Vector3Array m_currentPos;

    /// Current vertex normals
    Vector3Array m_currentNormal;

    /// Current vertex tangent vector
    Vector3Array m_currentTangent;

    /// Current vertex bitangent vector
    Vector3Array m_currentBitangent;

    /// Number of animation frames
    uint m_frameCounter;

    /// Indicator whether skinning must be processed.
    /// It is set to true if the current pose is different from previous.
    bool m_doSkinning;

    /// Indicator whether the skin must be reset to its initial reference
    /// configuration.
    bool m_doReset;
};

} // namespace Skinning
} // namespace Core
} // namespace Ra
