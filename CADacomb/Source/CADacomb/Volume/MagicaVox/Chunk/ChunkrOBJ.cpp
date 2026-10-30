#include "ChunkrOBJ.h"

#include "CADacomb/Volume/MagicaVox/IO/Reader.h"

namespace MagicaVoxImporter {
  void ChunkrOBJ::read(Reader own, Reader child) {
    Chunk::read(own, child);
  }
}