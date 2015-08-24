#!/bin/bash

set -e

BASEDIR=`dirname "$0"`

#make -C "$BASEDIR/.."
pushd "$BASEDIR/.."
./piedpiper_make
popd

if [ "$#" -ge 1 ]
then
    FILES=()
    for f in "$@"; do
        FILES+=("$f")
    done
else
    FILES=($BASEDIR/tibby.264 $BASEDIR/black.264 $BASEDIR/../res/BAMQ2_JVC_C.264 $BASEDIR/../res/BA1_FT_C.264 $BASEDIR/walk.264)
fi

IFS=""

REPORT=/tmp/report
rm -f $REPORT
for f in ${FILES[@]}; do
    rm -f /tmp/a.pip* /tmp/a.264
    bill=`mktemp`
    echo "============== $f ================" |tee -a $bill
    echo "    ./h264dec $f /tmp/a.pip" | tee -a $bill
    ./h264dec "$f" /tmp/a.pip | tee -a $bill
    python analyze_billing.py $bill
    cat $bill >> $REPORT
    rm $bill
    echo "    ./h264dec /tmp/a.pip /tmp/a.264"
    ./h264dec /tmp/a.pip /tmp/a.264
    diff /tmp/a.264 "$f"
done
echo "FULL report in $REPORT"