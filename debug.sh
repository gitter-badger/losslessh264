#!/bin/bash
f=${1}
echo "File" ${f}
grep ^R "${f}" >> "${f}_R"
grep ^W "${f}" >> "${f}_W"
cat "${f}_R" | tr R W > "${f}_RW"
echo "Run 'diff ${f}_W ${f}_R'"
