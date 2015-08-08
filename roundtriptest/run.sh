#!/bin/bash

set -e

BASEDIR=`dirname "$0"`

make -C "$BASEDIR/.."

FILES="$BASEDIR/tibby.264 $BASEDIR/black.264 $BASEDIR/../res/BA1_FT_C.264"

for f in $FILES; do
    rm -f /tmp/a.pip* /tmp/a.264
    echo Running $f ================
    ./h264dec $f /tmp/a.pip
    ./h264dec /tmp/a.pip /tmp/a.264
    diff /tmp/a.264 $f
done
