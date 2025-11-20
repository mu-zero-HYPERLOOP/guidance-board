#!/usr/bin/env sh

######################################################################
# @author      : kistenklaus (karlsasssie@gmail.com)
# @file        : run_all
# @created     : Sonntag Jun 23, 2024 14:12:37 CEST
#
# @description : 
######################################################################


canzero gen guidance_board_front src/canzero
cmake -Bbuild
cmake --build build -j8
alacritty -e ./build/guidance-board&

canzero gen guidance_board_back src/canzero
cmake -Bbuild
cmake --build build -j8
alacritty -e ./build/guidance-board&


