#!/bin/bash
# Copyright (c) 2017, Kahles David
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# This script checks if the packages build fine with the dependencies specified in the package.xml. Execute it in the top level directory of your catkin workspace.
# It will delete your build,devel and install directory and build every package from scratch! If a package fails to build, the build log will be
# copied into your home directory and named log.{PACKAGE NAME}.txt. It will try to find all packages by searching for their package.xml and using it's directory
# name as package name.

packages=()
for i in $(find src -name package.xml -printf '%h\n' | grep -v 'asr_nav_core'); do
    packages+=("$(basename $i)");
done

for i in ${packages[@]}; do
    rm -rf build devel install
    output=$(catkin_make --only-pkg-with-deps $i 2>&1)
    if [ $? -ne 0 ]; then
        echo "Package $i failed to build"
        touch ~/log.${i}.txt
        echo "Package $i failed to build: $output" > ~/log.${i}.txt
    else
        echo "Package $i build successfully"
    fi
done

