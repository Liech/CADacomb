#pragma once

#include <array>
#include <bit>
#include <ostream>
#include <vector>

namespace CADacomb
{
    // Streams trivially copyable values in their native byte layout. Vectors are written element-wise without a length prefix.
    class BinaryWriter
    {
      public:
        explicit BinaryWriter(std::ostream& stream);
        virtual ~BinaryWriter();

        template<typename T>
        BinaryWriter& operator<<(const T& value);
        template<typename T>
        BinaryWriter& operator<<(const std::vector<T>& values);

      private:
        std::ostream& stream;
    };

    template<typename T>
    BinaryWriter& BinaryWriter::operator<<(const T& value)
    {
        std::array<char, sizeof(T)> bytes = std::bit_cast<std::array<char, sizeof(T)>>(value);
        stream.write(bytes.data(), bytes.size());
        return *this;
    }

    template<typename T>
    BinaryWriter& BinaryWriter::operator<<(const std::vector<T>& values)
    {
        for (const T& value : values)
        {
            *this << value;
        }
        return *this;
    }
}
