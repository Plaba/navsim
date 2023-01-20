#!/bin/bash

# This script is used to build colcon packages.

for arg in "$@"
do
    case $arg in
        -c|--clean)
        CLEAN=1
        shift
        ;;
        *)
        PKG_NAME=$arg
        shift
        ;;
    esac
done

COLCON_ARGS=""

if [ -n "$PKG_NAME" ]; then
    COLCON_ARGS="$COLCON_ARGS --packages-select $PKG_NAME"
fi

if [ -n "$CLEAN" ]; then
    COLCON_ARGS="$COLCON_ARGS --cmake-clean-first"
fi

if [ -n "$IGNORED_PACKAGES" ]; then
    COLCON_ARGS="$COLCON_ARGS --packages-ignore $IGNORED_PACKAGES"
fi

colcon build $COLCON_ARGS --cmake-args\
 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON\
 -DBoost_LIBRARY_DIR_RELEASE=/usr/lib/x86_64-linux-gnu