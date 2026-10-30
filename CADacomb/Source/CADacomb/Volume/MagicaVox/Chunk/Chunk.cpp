#include "Chunk.h"

#include "CADacomb/Volume/MagicaVox/IO/Reader.h"
#include "ChunkFactory.h"

#include <iostream>

namespace MagicaVoxImporter
{
    Chunk::~Chunk() {}

    void Chunk::read(Reader own, Reader child)
    {
        while (!child.endOfBufferReached())
        {
            auto chunk = child.readChunk();
            if (chunk->getID() != "????")
                addChild(std::move(chunk));
        }
    }

    void Chunk::print(int indentation)
    {
        std::cout << std::string(indentation, ' ') << getID() << std::endl;
        for (int i = 0; i < childs.size(); i++)
            childs[i]->print(indentation + 1);
    }

    void Chunk::write(std::vector<unsigned char>& file) const {}

    size_t Chunk::numberOfChilds() const
    {
        return childs.size();
    }

    const Chunk& Chunk::getChild(size_t i) const
    {
        return *childs[i];
    }

    void Chunk::addChild(std::unique_ptr<Chunk> newchild)
    {
        childs.push_back(std::move(newchild));
    }
}