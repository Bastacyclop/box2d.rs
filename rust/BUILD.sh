mkdir -p lib
mkdir -p bin
rustc src/box2d/lib.rs --cfg main_test -o bin/test -L ../c/lib/
rustc src/box2d/lib.rs --crate-type lib --out-dir lib -L ../c/lib/
