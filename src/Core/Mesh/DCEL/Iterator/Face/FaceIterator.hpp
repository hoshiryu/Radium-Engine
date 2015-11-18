#ifndef RADIUMENGINE_DCEL_FACE_ITERATOR_HPP
#define RADIUMENGINE_DCEL_FACE_ITERATOR_HPP

#include <Core/Mesh/DCEL/Iterator/Iterator.hpp>

namespace Ra {
namespace Core {

template < typename OBJECT >
class FIterator : public Iterator< OBJECT > {
public:
    /// CONSTRUCTOR
    FIterator( const Face& f );
    FIterator( const FIterator& it ) = default;

    /// DESTRUCTOR
    ~FIterator();

    /// SIZE
    inline uint size() const override;

    /// RESET
    inline void reset() override;

    /// OPERATOR
    inline FIterator& operator= ( const FIterator& it );
    inline FIterator& operator++();
    inline FIterator& operator--();
    inline bool       operator==( const FIterator& it ) const;

protected:
    /// VARIABLE
    Face_ptr   m_f;
};

} // namespace Core
} // namespace Ra

#include <Core/Mesh/DCEL/Iterator/Face/FaceIterator.inl>

#endif // RADIUMENGINE_DCEL_FACE_ITERATOR_HPP
