#pragma once

#include <memory>
#include <string>
#include <vector>

namespace MagicaVoxImporter
{
    class Reader;

    class Chunk
    {
      public:
        virtual ~Chunk();

        virtual std::string getID() const = 0;
        virtual void        read(Reader own, Reader child);
        virtual void        print(int indentation = 0);
        virtual void        write(std::vector<unsigned char>&) const;

        size_t       numberOfChilds() const;
        const Chunk& getChild(size_t i) const;
        void         addChild(std::unique_ptr<Chunk> newchild);

        template<typename T>
        bool childIsType(size_t i);

      private:
        std::vector<std::unique_ptr<Chunk>> childs;
    };

    template<typename T>
    bool Chunk::childIsType(size_t i)
    {
        if (i >= childs.size())
            return false;
        return dynamic_cast<T*>(childs[i].get()) != nullptr;
    }
}