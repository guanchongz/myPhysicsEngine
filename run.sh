rm ./build/demo
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug -B build
cmake  --build build --config Debug
./build/demo
