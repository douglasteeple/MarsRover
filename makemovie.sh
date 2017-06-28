#!/bin/bash
ffmpeg -pattern_type sequence -i *.jpg -pix_fmt yuv420p $1.mp4
