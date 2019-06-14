#ifndef ANIMATION_HPP
#define ANIMATION_HPP

#include <Core/Animation/Pose.hpp>
#include <vector>

namespace Ra {
namespace Core {
namespace Animation {

class RA_CORE_API Animation
{
  public:
    using MyKeyPose = std::pair<Scalar, Pose>;
    // Add the key pose after the previous ones.
    // Call normalize after all the key poses have been added.
    // timestamp must be given in seconds.
    void addKeyPose( const Pose& pose, Scalar timestamp );
    void addKeyPose( const MyKeyPose& keyPose );

    // Remove all the key poses.
    void clear();

    bool isEmpty() const;

    // Re-order the poses by chronological order.
    void normalize();

    // Get the pose corresponding to the given timestamp.
    // timestamp must be given in seconds.
    Pose getPose( Scalar timestamp ) const;

    // Get the internal animation time from a timestamp.
    // Guaranteed to be between 0 and the animation last time
    Scalar getTime( Scalar timestamp ) const;

    // Get the animation last time.
    Scalar getDuration() const;

    /// \brief Remove the i-th key pose from m_keys.
    /// \param i: the index of the key pose to remove.
    void removeKeyPose( int i );

    /// \brief Set the i-th key pose time to timestamp.
    /// \param i: the index of the key pose to remove.
    /// \param timestamp: the timestamp to set the key pose at.
    void setKeyPoseTime( int i, Scalar timestamp );

    /// \brief Move the keyposes after first (included).
    /// \param offset: the offset to add to the key poses.
    /// \param first: the index of the first key pose to move.
    void offsetKeyPoses( Scalar offset, int first );

    /// \brief Replace the i-th key pose pose with pose.
    /// \param i: the index of the pose to replace.
    /// \param pose: the pose to replace the current one.
    void replacePose( int i, const Pose& pose );

    /// \brief Getter for the size of m_keys.
    /// \return the size of m_keys.
    int size() const;

    /// \brief Getter for the i-th key pose.
    /// return the i-th key pose.
    const MyKeyPose& keyPose( int i ) const;

  private:
    std::vector<MyKeyPose> m_keys;
};

} // namespace Animation
} // namespace Core
} // namespace Ra

#endif // ANIMATION_HPP
