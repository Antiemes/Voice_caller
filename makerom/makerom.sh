rm -rf dummy.raw pwav raw
mkdir pwav raw
#flashsize=16777216
#wsize=0
for x in swav/*.wav
  do
    bfile=$(basename $x)
    ofile=pwav/$bfile
    rfile=raw/$(echo $bfile | sed 's,wav$,raw,')
    echo $bfile
    mpv --af=lavfi="[agate=1:0:0.1,alimiter=20:1:1:5:50]" $x -ao pcm --ao-pcm-file=$ofile
    sox $ofile -c 1 -b 8 -e unsigned -r 32k $rfile
    #size=$(ls -l $rfile | awk '{print $5}')
    #((wsize+=size))
  done
cat raw/*.raw > out.raw
truncate -s 16m out.raw
