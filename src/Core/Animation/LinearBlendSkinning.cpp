#include <Core/Animation/LinearBlendSkinning.hpp>

namespace Ra {
namespace Core {
namespace Animation {

void linearBlendSkinning( const Vector3Array& inMesh,
                          const Pose& pose,
                          const WeightMatrix& weight,
                          Vector3Array& outMesh ) {
    outMesh.clear();
    outMesh.resize( inMesh.size(), Vector3::Zero() );
    for ( int k = 0; k < weight.outerSize(); ++k )
    {
        const int nonZero = weight.col( k ).nonZeros();
        WeightMatrix::InnerIterator it0( weight, k );
#pragma omp parallel for
        for ( int nz = 0; nz < nonZero; ++nz )
        {
            WeightMatrix::InnerIterator it = it0 + Eigen::Index( nz );
            const uint i                   = it.row();
            const uint j                   = it.col();
            const Scalar w                 = it.value();
            outMesh[i] += w * ( pose[j] * inMesh[i] );
        }
    }
}

void RA_CORE_API accurateLightingLBS( const Skinning::RefData& refData,
                                      const Vector3Array& tangents,
                                      const Vector3Array& bitangents,
                                      Skinning::FrameData& frameData ) {
    const auto& vertices = refData.m_referenceMesh.vertices();
    auto cap = []( Eigen::Ref<Vector3> p ) {
        Scalar div = 1 / p.norm();
        p *= std::min( 1_ra, div*0.9_ra );
    };
//#pragma omp parallel for
    for ( int v = 0; v < frameData.m_currentPos.size(); ++v )
    {
        const Vector3& V = vertices[v];
        // compute Q0
        const auto& [s,a,b] = refData.m_alphaBeta[v][0];
        const auto Q0 = frameData.m_currentPose[s] * V;
        // sum up T, p, t and b
        Matrix3 T = Matrix3::Zero();
        frameData.m_currentPos[v] = Q0;
        frameData.m_currentTangent[v] = Vector3::Zero();
        frameData.m_currentBitangent[v] = Vector3::Zero();
        for ( const auto& [s,a,b] : refData.m_alphaBeta[v] )
        {
            const Scalar& w = refData.m_weights.coeff( v, s );
            const Vector3 Q = frameData.m_currentPose[s] * V - Q0;
            T += w * frameData.m_currentPose[s].linear();
            frameData.m_currentPos[v] += w * Q;
            std::cout << v << " " << a << " " << Q.transpose() << std::endl;
            frameData.m_currentTangent[v] += a * Q;
            frameData.m_currentBitangent[v] += b * Q;
        }
        cap( frameData.m_currentTangent[v] );
        cap( frameData.m_currentBitangent[v] );
        std::cout << frameData.m_currentTangent[v].transpose() << std::endl;
        std::cout << (T * tangents[v]).transpose() << std::endl;
        frameData.m_currentTangent[v] += T * tangents[v];
        frameData.m_currentTangent[v].normalize();
        frameData.m_currentBitangent[v] += T * bitangents[v];
        frameData.m_currentBitangent[v].normalize();
        // compute n
        frameData.m_currentNormal[v] = frameData.m_currentTangent[v].cross(
                    frameData.m_currentBitangent[v] );

    }
}

} // namespace Animation
} // namespace Core
} // namespace Ra
