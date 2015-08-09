#!/bin/sh
infile="$1"
outfile="$2"
shift
shift
ffmpeg -i "$infile" -vcodec copy -an -vbsf h264_mp4toannexb "$@" "$outfile"
