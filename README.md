# Opus Codec on M4

Decode opus binaries to PCM and send it to I2S. The Opus lib is within the project.




########Steps to build Opus 1.3.1 for M4
unset CFLAGS
export CFLAGS="-nostdlib -mcpu=cortex-m4 -O3 -g3 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16"
./configure --host=arm-none-eabi --disable-shared --disable-rtcd --disable-extra-programs --disable-doc
make clean
make install DESTDIR=$PWD/dist
########Output is in dist
