#pragma once

#include <array>
#include <bit>
#include <istream>
#include <vector>

namespace CADacomb
{
    // Reads trivially copyable values in their native byte layout. Vectors are filled element-wise up to their current size.
    class BinaryReader
    {
      public:
        explicit BinaryReader(std::istream& stream);
        virtual ~BinaryReader();

        template<typename T>
        BinaryReader& operator>>(T& value);
        template<typename T>
        BinaryReader& operator>>(std::vector<T>& values);

      private:
        std::istream& stream;
    };

    template<typename T>
    BinaryReader& BinaryReader::operator>>(T& value)
    {
        std::array<char, sizeof(T)> bytes{};
        stream.read(bytes.data(), bytes.size());
        value = std::bit_cast<T>(bytes);
        return *this;
    }

    template<typename T>
    BinaryReader& BinaryReader::operator>>(std::vector<T>& values)
    {
        for (T& value : values)
        {
            *this >> value;
        }
        return *this;
    }
}
