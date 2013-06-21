#!/bin/sh

# This script tags the current HEAD and then creates a patch of the application
# given as parameter to this script.

# It finds all modified files from HEAD and adds these to the patch. It is
# assumed that these were intentionally not committed before creating the patch.

# It is also assumed that all necessary files have been committed so that the 
# tag version will only involve a simple incrementental update.

APP_DIR="Projects/tools/LinuxHost/application"

##########################################################
# Check if there are any arguments, if so check if the application is valid
#
if [ $# -eq 0 ]; then
	echo "No application provided, cannot create patch"
	exit
fi

# Check if the applications are valid
for i in "$@"; do 
	if [ ! -d $APP_DIR"/"$i ]; then
		echo $1 "does not exist, cannot create patch"
		exit
	fi
done

##########################################################
#	Create TAG 
#
# First find current tag version of the form vMajor.Minor.Increment
#
TAG=$(git describe --abbrev=0)
#TAG="1.2.3"
#echo "TAG:" $TAG
TAGBASE=$(echo $TAG | sed 's/\([0-9]*\.[0-9]*\.\).*/\1/')
#echo "TAGBASE:" $TAGBASE
INC=$(echo $TAG | sed 's/.*\.//')
#echo "INC:" $INC
NEWINC=$(($INC + 1))
#echo "NEWINC:" $NEWINC
NEWTAG=$TAGBASE$NEWINC
#echo "NEWTAG:" $NEWTAG

# Then create this tag
git tag -a $NEWTAG -m 'Tag for '$1' application patch'
echo $NEWTAG 'Tag for '$1' application patch'

# Now share this tag
git push origin tags/$NEWTAG

##########################################################
# 	Create PATCH for each application
#
# First find differences between working copy and HEAD
# these files should be added to patch. Since we have no
# way of determining what application the modifications
# belong to we must include them in each application's
# patch

DIFFLIST=$(git diff --name-only)

for i in "$@"; do
	echo "Creating "$i"_"$NEWTAG".tar"
# First clean up
	DIR=$(pwd)
	cd $APP_DIR"/"$i
	make clean
	if [ -d "out" ]; then
		rm -rf out
	fi
# Go back to root folder
	cd $DIR
	tar -cf $i"_"$NEWTAG.tar $DIFFLIST $APP_DIR"/"$i
done


##########################################################
# 	Create installer for each PATCH
#
for i in "$@"; do
	echo "Creating "$i"_"$NEWTAG".sh installer"
	echo "#!/bin/sh" > "install_"$i"_"$NEWTAG".sh"
# Installer must be executable
	chmod 755 "install_"$i"_"$NEWTAG".sh"
	echo "# Run this to install "$i "application" >> "install_"$i"_"$NEWTAG".sh"
	echo "# "$i".tar must exist in this folder" >> "install_"$i"_"$NEWTAG".sh"
	echo "# RemoTI-Linux/ must NOT exist in this folder" >> "install_"$i"_"$NEWTAG".sh"
# First clone into RemoTI-Linux
	echo "git clone http://github.com/TI-LPRF-Software/RemoTI-Linux.git tags/"$NEWTAG >> "install_"$i"_"$NEWTAG".sh"
	echo "cd RemoTI-Linux" >> "install_"$i"_"$NEWTAG".sh"
	echo "git checkout tags/"$NEWTAG >> "install_"$i"_"$NEWTAG".sh"
# Then untar the patch into RemoTI-Linux
	echo "tar -xf ../"$i"_"$NEWTAG".tar" >> "install_"$i"_"$NEWTAG".sh"
	echo "cd .." >> "install_"$i"_"$NEWTAG".sh"
	echo "echo Installation Complete!" >> "install_"$i"_"$NEWTAG".sh"
done


