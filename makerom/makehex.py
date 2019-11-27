#!/usr/bin/python3
import sys

addr=0x000000
chunksize=128
blocksize=4096
blockfree=0
with open(sys.argv[1], "rb") as f:
    while True:
      chunk = f.read(128)
      if chunk:
        if (blockfree==0):
          print("E", end='')
          print('{:08x}'.format(addr), end='')
          print('')
          blockfree=blocksize
        print("W", end='')
        print('{:08x}'.format(addr), end='')
        print('{:08x}'.format(128), end='')
        for b in chunk:
          print('{:02x}'.format(b), end='')
        print('')
        addr+=chunksize
        blockfree-=chunksize
      else:
        break
