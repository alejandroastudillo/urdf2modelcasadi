#!/bin/sh
#for i in `find . -name '*.STL'`; do
#    meshlabserver -i $i -o `pwd`/$(echo `basename $i` | sed 's|.wrl$|.dae|')
#done
for file in *.STL; do
    meshlabserver -i "$file" -o "${file%.*}.dae" # -om vc
done
