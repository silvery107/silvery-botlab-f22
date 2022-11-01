#!/usr/bin/env bash

VX_SHADERS_DIR=$(find . -regex '.*vx\/shaders' | head -n 1)
export VX_SHADER_PATH=$PWD/$VX_SHADERS_DIR
echo "VX shades location: {$VX_SHADERS_DIR}"

VX_FONTS_DIR=$(find . -regex '.*vx\/fonts' | head -n 1)
export VX_FONT_PATH=$PWD/$VX_FONTS_DIR
echo "VX fonts location: {$VX_FONTS_DIR}"

LCM_JAVA_FILE_LOCATION=$(find ../ -regex '.*lcm.*\.jar' | head -n 1)
export CLASSPATH=$CLASSPATH:$PWD/$LCM_JAVA_FILE_LOCATION
echo "Jar file location: {$LCM_JAVA_FILE_LOCATION}"
