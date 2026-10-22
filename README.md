# CADacomb

Spatial data structures and CAD modeling core library.

## Dependencies

Managed via [vcpkg](https://github.com/microsoft/vcpkg):
* `glm`
* `nlohmann-json`
* `catch2`

## Building

```bash
cmake -B out -S . -DCMAKE_TOOLCHAIN_FILE=[vcpkg-root]/scripts/buildsystems/vcpkg.cmake
cmake --build out
```
