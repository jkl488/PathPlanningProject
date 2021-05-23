#!/bin/sh
set -e
if test "$CONFIGURATION" = "Debug"; then :
  cd "/Users/jakobklein/Desktop/Desktop/Udacity - Nanodegree Sefl driving cars engineer/Term_2/Planning/PathPlanningProject/build"
  make -f /Users/jakobklein/Desktop/Desktop/Udacity\ -\ Nanodegree\ Sefl\ driving\ cars\ engineer/Term_2/Planning/PathPlanningProject/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "Release"; then :
  cd "/Users/jakobklein/Desktop/Desktop/Udacity - Nanodegree Sefl driving cars engineer/Term_2/Planning/PathPlanningProject/build"
  make -f /Users/jakobklein/Desktop/Desktop/Udacity\ -\ Nanodegree\ Sefl\ driving\ cars\ engineer/Term_2/Planning/PathPlanningProject/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "MinSizeRel"; then :
  cd "/Users/jakobklein/Desktop/Desktop/Udacity - Nanodegree Sefl driving cars engineer/Term_2/Planning/PathPlanningProject/build"
  make -f /Users/jakobklein/Desktop/Desktop/Udacity\ -\ Nanodegree\ Sefl\ driving\ cars\ engineer/Term_2/Planning/PathPlanningProject/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "RelWithDebInfo"; then :
  cd "/Users/jakobklein/Desktop/Desktop/Udacity - Nanodegree Sefl driving cars engineer/Term_2/Planning/PathPlanningProject/build"
  make -f /Users/jakobklein/Desktop/Desktop/Udacity\ -\ Nanodegree\ Sefl\ driving\ cars\ engineer/Term_2/Planning/PathPlanningProject/build/CMakeScripts/ReRunCMake.make
fi

