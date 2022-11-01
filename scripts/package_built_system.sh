#!/usr/bin/env bash

OUTPUT_FOLDER=system_compilation
if [ ! -d $OUTPUT_FOLDER ]; then  
    mkdir $OUTPUT_FOLDER; 
    mkdir $PWD/$OUTPUT_FOLDER/maps
fi


UPDATE_CMD="rsync -au --delete "

eval $UPDATE_CMD $PWD/build/bin $OUTPUT_FOLDER && \
eval $UPDATE_CMD $PWD/scripts/* $OUTPUT_FOLDER && \
eval $UPDATE_CMD $PWD/mbot/visualizers/vx $OUTPUT_FOLDER && \
mkdir $PWD/$OUTPUT_FOLDER/python/mbot_lcm_msgs && \
eval $UPDATE_CMD $PWD/build/mbot/mbot_lcm_msgs/mbot_lcm_msgs/*.py $PWD/$OUTPUT_FOLDER/python/mbot_lcm_msgs/ && \
eval $UPDATE_CMD $PWD/data $PWD/$OUTPUT_FOLDER/