#! /usr/bin/env sh

set -ev

BUILD_TYPE=${BUILD_TYPE:=Release}

if [ ! -d companiond/build_ci ]; then mkdir companiond/build_ci; fi

cmake -Bcompaniond/build_ci -Hcompaniond -DCMAKE_BUILD_TYPE:string=$BUILD_TYPE
cd companiond/build_ci && make 
