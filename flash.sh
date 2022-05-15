#!/bin/sh
#make
#killall com

#stm32flash -i -dtr,-dtr,-dtr,-dtr,dtr,dtr,dtr,dtr,dtr,dtr,-dtr,-dtr,-dtr,-dtr,-dtr: -w  build/master.bin -g 0x0 /dev/ttyACM0
cargo objcopy --bin soldering_station --release -- -O binary soldering_station.bin

stm32flash -i -dtr,-dtr,-dtr: -w  soldering_station.bin -g 0x0 COM5

st-flash write  soldering_station.bin 0x8000000
