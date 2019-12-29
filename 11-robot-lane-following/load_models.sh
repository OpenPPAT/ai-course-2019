#!/bin/bash

CURR_PATH=${PWD}

cd nano_ws/src/imitation_following/models

gdown https://drive.google.com/uc?id=18iZQHZ31YiNXeMvIhyygpPpRtLRDJD3Q -O models.zip

unzip models.zip

rm models.zip

cd $CURR_PATH
