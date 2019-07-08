# image-stitching

This repo contains simple stitching application which is capable to stitch
images or videos into panoramas.

## Main ideas

As it described in [OpenCV Stitching pipeline][opencv-stitching-pipeline], it
consists of two main parts: registration and compositing.

[opencv-stitching-pipeline]: https://docs.opencv.org/2.4/modules/stitching/doc/introduction.html "OpenCV Stitching pipeline"

In this repo, these two stages are represented by [calibrate](/tools/calibrate)
and [stitch](/tools/stitch) tools correspondingly.

This project doesn't use any existing feature-detection algorithms and methods
like SURF, SIFT, ORB, etc. Instead, to stitch images they must contain
chessborad on them - its corners will be used as special points to estimate
camera parameters and to understand how to stitch images.

## How to build

TBD

## How to use

TBD
