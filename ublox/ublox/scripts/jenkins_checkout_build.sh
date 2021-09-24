#!/bin/sh

wstool_conf_url="http://gitlab.is4s-auburn.com/common/install_manifests/raw/master/gps/ublox.install"

# source ros setup file to set up environment for catkin
. /usr/local/setup.sh

if [ "$1" ]; then
	catkinws="$1"
else
	catkinws=`pwd`
fi
echo "using catkin workspace in $catkinws" >&2

mkdir -p $catkinws/src

# UPDATE CATKIN WORKSPACE
cd "$catkinws/src"

if [ ! -f "./.rosinstall" ]; then
	echo "create new wstool workspace" >&2
	wstool init .
	catkin_init_workspace
fi
 
 
# stash local changes (just to make sure the next step always works)
for d in *; do (if [ -d "$d/.git" ]; then echo $d; cd $d; git stash; fi); done
 
# get the latest successful rosinstall files from jenkins ()
wstool merge -r -y "$wstool_conf_url"

# get everything from the git server
wstool up --delete-changed-uris
 
# run catkin_make
catkin_make -C .. clean
catkin_make -C ..