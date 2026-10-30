#include "CADacomb/Volume/MagicaVox/IO/Reader.h"
#include "CADacomb/Volume/MagicaVox/Chunk/Chunk.h"
#include "CADacomb/Volume/MagicaVox/Chunk/ChunkFactory.h"
#include <stdexcept>

namespace MagicaVoxImporter
{
    Reader::Reader() {}

    Reader::Reader(const std::vector<unsigned char>& buffer)
    {
        content = buffer;
    }

    Reader::~Reader() {}

    bool Reader::endOfBufferReached()
    {
        return content.size() <= currentPosition;
    }

    void Reader::assertCanRead()
    {
        if (endOfBufferReached())
            throw std::runtime_error("End of Buffer Reached!");
    }

    Reader Reader::read(size_t numberOfBytes)
    {
        return Reader(readChars(numberOfBytes));
    }

    std::vector<unsigned char> Reader::readChars(size_t numberOfBytes)
    {
        if (numberOfBytes == 0)
            return {};
        assertCanRead();
        std::vector<unsigned char> subVector(content.begin() + currentPosition, content.begin() + currentPosition + numberOfBytes);
        currentPosition += numberOfBytes;
        return subVector;
    }

    std::string Reader::readChunkID()
    {
        assertCanRead();
        std::string id(content.begin() + currentPosition, content.begin() + currentPosition + 4);
        currentPosition += 4;
        return id;
    }

    int Reader::readInt()
    {
        std::vector<unsigned char> bytes = readChars(4);

        int result = int((unsigned char)(bytes[0]) | (unsigned char)(bytes[1]) << 8 | (unsigned char)(bytes[2]) << 16 | (unsigned char)(bytes[3] << 24));
        return result;
    }

    std::unique_ptr<Chunk> Reader::readChunk()
    {
        std::string id           = readChunkID();
        int         ownContent   = readInt();
        int         childContent = readInt();
        Reader      own;
        if (ownContent > 0)
            own = read(ownContent);
        Reader child;
        if (childContent > 0)
            child = read(childContent);
        std::unique_ptr<Chunk> result = ChunkFactory::make(id);
        result->read(own, child);
        return result;
    }

    std::string Reader::readString()
    {
        int                        size = readInt();
        std::vector<unsigned char> msg  = readChars(size);
        return std::string(msg.begin(), msg.end());
    }

    std::map<std::string, std::string> Reader::readDict()
    {
        int                                numberOfPairs = readInt();
        std::map<std::string, std::string> result;

        for (int i = 0; i < numberOfPairs; i++)
        {
            std::string key   = readString();
            std::string value = readString();
            result[key]       = value;
        }
        return result;
    }

    Rotation Reader::readRotation()
    {
        Rotation      result;
        unsigned char raw = readChars(1)[0];

        int  firstRowPos   = (0b11000000 & raw) >> 6;
        int  secondRowPos  = (0b00110000 & raw) >> 4;
        bool signFirstRow  = (0b00001000 & raw) != 0;
        bool signSecondRow = (0b00000100 & raw) != 0;
        bool signThirdRow  = (0b00000010 & raw) != 0;

        if (firstRowPos == 0)
            result.v00 = signFirstRow ? -1 : 1;
        if (firstRowPos == 1)
            result.v10 = signFirstRow ? -1 : 1;
        if (firstRowPos == 2)
            result.v20 = signFirstRow ? -1 : 1;
        if (secondRowPos == 0)
            result.v01 = signSecondRow ? -1 : 1;
        if (secondRowPos == 1)
            result.v11 = signSecondRow ? -1 : 1;
        if (secondRowPos == 2)
            result.v21 = signSecondRow ? -1 : 1;

        if (firstRowPos + secondRowPos == 3)
            result.v02 = signThirdRow ? -1 : 1;
        if (firstRowPos + secondRowPos == 2)
            result.v12 = signThirdRow ? -1 : 1;
        if (firstRowPos + secondRowPos == 1)
            result.v22 = signThirdRow ? -1 : 1;
        return result;
    }
}
