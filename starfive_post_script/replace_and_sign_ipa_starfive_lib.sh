#!/bin/sh
###############################################################################

# Copyright(C) 2023 StarFive Technology Co., Ltd.
# Date: 13/10/2023
# Author: Andy Hu <andy.hu@starfivetech.com>
#         zejian.su <zejian.su@starfivetech.com>

###############################################################################

# @objective: replace and sign with the new full feature ipa lib.
# @desc: the 3A feature in starfive ipa is closed source, and starfive will
#    provide a full feature ipa library. the script will replace with the
#    starfive full feature ipa lib, and re-sign the so after build and install
#    Usage: $0 [path], if not specify path, build/src/ipa/starfive/ is default
# @returns: 0 if no error, non-zero if there's any error.

###############################################################################
curr_dir=$(dirname $0)

src_ipa=$curr_dir/ipa_starfive.so.0.0.0

ipa_so=ipa_starfive.so
if [ $# -gt 0 ]; then
    dst_ipa=$1/$ipa_so
else
    dst_ipa=$curr_dir/../build/src/ipa/starfive/$ipa_so
fi

key_file="$curr_dir/../build/src/ipa-priv-key.pem"
sign_file="${dst_ipa}.sign"
sign_tool=$curr_dir/../src/ipa/ipa-sign.sh

# check valid
if [ ! -e $src_ipa ]; then
    echo "new ipa starfive lib not exist"
    exit 1
fi
if [ ! -e $dst_ipa ]; then
    echo "libcamera had not be built: ipa_starfive.so not exist"
    exit 1
fi
if [ ! -e $key_file ]; then
    echo "libcamera had not be built: ipa-priv-key.pem not exist"
    exit 1
fi
if [ ! -e $sign_tool ]; then
    echo "src/ipa/ipa-sign.sh not exist "
    exit 1
fi

# copy ipa lib to dst path
echo "cp $src_ipa $(dirname $dst_ipa)"
cp $src_ipa $(dirname $dst_ipa)
rm -rf $dst_ipa
ln -sf $(basename $src_ipa) $dst_ipa

# sign
echo "$sign_tool $key_file $dst_ipa $sign_file"
$sign_tool $key_file $dst_ipa $sign_file
if [ $? -ne 0 ]; then
    echo "ipa-sign.sh failed"
    exit 1
fi
