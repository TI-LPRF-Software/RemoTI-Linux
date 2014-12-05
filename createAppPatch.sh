#!/bin/sh

# This script tags the current HEAD and then creates a patch of the application
# given as parameter to this script.

# It finds all modified files from HEAD and adds these to the patch. It is
# assumed that these were intentionally not committed before creating the patch.

# It is also assumed that all necessary files have been committed so that the 
# tag version will only involve a simple incrementental update.

# Copyright (C) {YEAR} Texas Instruments Incorporated - http://www.ti.com/
# 
# 
#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions
#    are met:
# 
#      Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
# 
#      Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the
#      distribution.
# 
#      Neither the name of Texas Instruments Incorporated nor the names of
#      its contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# 
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
APP_DIR="Projects/tools/LinuxHost/application"
NON_BSD_APP_DIR="/home/uva0132614local/Gerrit/RemoTI-Linux_RFMGR/Projects/tools/LinuxHost/application"
#NON_BSD_APP_DIR="~/Gerrit/RemoTI-Linux_RFMGR/Projects/tools/LinuxHost/application"

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
		echo $i "does not exist, cannot create patch"
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
git tag -a $NEWTAG -m 'Tag for application patch'
echo $NEWTAG 'Tag for '$1' '$2' '$3' application patch'

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
	if [ "$i" = "msoComcast" ]; then
		echo "Creating "$i"_nonBSD_"$NEWTAG".tar"
		# Create non-BSD release as well
		cp $i"_"$NEWTAG.tar $i"_nonBSD_"$NEWTAG.tar
		# Replace the makefile to enable Voice and DIU
		cp $APP_DIR"/"$i/makefile $APP_DIR"/"$i/makefile_cp
		tar --delete --file=$i"_nonBSD_"$NEWTAG.tar $APP_DIR"/"$i/makefile
		cp $APP_DIR"/"makefile_nonBSD_msoComcast $APP_DIR"/"$i/makefile
		tar --append --file=$i"_nonBSD_"$NEWTAG.tar $APP_DIR"/"$i/makefile
		mv $APP_DIR"/"$i/makefile_cp $APP_DIR"/"$i/makefile
		# Then add the Voice and DIU files
		cp $NON_BSD_APP_DIR"/"$i/mso_deviceImageUpdate.* $APP_DIR"/"$i/
		cp $NON_BSD_APP_DIR"/"$i/mso_device_image_update_profile.h $APP_DIR"/"$i/
		cp $NON_BSD_APP_DIR"/"$i/mso_voice.* $APP_DIR"/"$i/
		cp $NON_BSD_APP_DIR"/"$i/mso_voice_profile.h $APP_DIR"/"$i/
		cp -r $NON_BSD_APP_DIR"/"$i/lib $APP_DIR"/"$i/

		tar --append --file=$i"_nonBSD_"$NEWTAG.tar $APP_DIR"/"$i/mso_deviceImageUpdate.c
		tar --append --file=$i"_nonBSD_"$NEWTAG.tar $APP_DIR"/"$i/mso_deviceImageUpdate.h
		tar --append --file=$i"_nonBSD_"$NEWTAG.tar $APP_DIR"/"$i/mso_device_image_update_profile.h
		tar --append --file=$i"_nonBSD_"$NEWTAG.tar $APP_DIR"/"$i/mso_voice.c
		tar --append --file=$i"_nonBSD_"$NEWTAG.tar $APP_DIR"/"$i/mso_voice.h
		tar --append --file=$i"_nonBSD_"$NEWTAG.tar $APP_DIR"/"$i/mso_voice_profile.h
		tar --append --file=$i"_nonBSD_"$NEWTAG.tar $APP_DIR"/"$i/lib

		rm $APP_DIR"/"$i/mso_deviceImageUpdate.*
		rm $APP_DIR"/"$i/mso_device_image_update_profile.h
		rm $APP_DIR"/"$i/mso_voice.*
		rm $APP_DIR"/"$i/mso_voice_profile.h
		rm -rf $APP_DIR"/"$i/lib
	fi
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
	echo "git clone http://github.com/TI-LPRF-Software/RemoTI-Linux.git"  >> "install_"$i"_"$NEWTAG".sh"
	echo "cd RemoTI-Linux" >> "install_"$i"_"$NEWTAG".sh"
	echo "git checkout tags/"$NEWTAG >> "install_"$i"_"$NEWTAG".sh"
# Then untar the patch into RemoTI-Linux
	echo "echo Applying patch to checked out RemoTI-Linux tag" >> "install_"$i"_"$NEWTAG".sh"
	echo "tar -xf ../"$i"_"$NEWTAG".tar" >> "install_"$i"_"$NEWTAG".sh"
	echo "cd .." >> "install_"$i"_"$NEWTAG".sh"
	echo "echo Installation Complete!" >> "install_"$i"_"$NEWTAG".sh"
	if [ "$i" = "msoComcast" ]; then
		echo "Creating "$i"_nonBSD_"$NEWTAG".sh installer"
		echo "#!/bin/sh" > "install_"$i"_nonBSD_"$NEWTAG".sh"
# Installer must be executable
		chmod 755 "install_"$i"_nonBSD_"$NEWTAG".sh"
		echo "# Run this to install "$i "application" >> "install_"$i"_nonBSD_"$NEWTAG".sh"
		echo "# "$i".tar must exist in this folder" >> "install_"$i"_nonBSD_"$NEWTAG".sh"
		echo "# RemoTI-Linux/ must NOT exist in this folder" >> "install_"$i"_nonBSD_"$NEWTAG".sh"
# Clone into RemoTI-Linux
		echo "git clone http://github.com/TI-LPRF-Software/RemoTI-Linux.git"  >> "install_"$i"_nonBSD_"$NEWTAG".sh"
		echo "cd RemoTI-Linux" >> "install_"$i"_nonBSD_"$NEWTAG".sh"
		echo "git checkout tags/"$NEWTAG >> "install_"$i"_nonBSD_"$NEWTAG".sh"
# Then untar the patch into RemoTI-Linux
		echo "echo Applying patch to checked out RemoTI-Linux tag" >> "install_"$i"_nonBSD_"$NEWTAG".sh"
		echo "tar -xf ../"$i"_nonBSD_"$NEWTAG".tar" >> "install_"$i"_nonBSD_"$NEWTAG".sh"
		echo "cd .." >> "install_"$i"_nonBSD_"$NEWTAG".sh"
		echo "echo Installation Complete!" >> "install_"$i"_nonBSD_"$NEWTAG".sh"
	fi
done


