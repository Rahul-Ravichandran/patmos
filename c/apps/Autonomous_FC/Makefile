# MAIN?=Flight_controller_v2
MAIN?=test/imu_test

# This is the trivial Hello World example, distributed over two .c files

LDFLAGS?= \
        -mpatmos-method-cache-size=0x1000 \
        -mpatmos-stack-base=0x080000 -mpatmos-shadow-stack-base=0x078000 \
        -Xgold --defsym -Xgold __heap_end=0x070000

all:
	patmos-clang -I ../.. -O2 $(LDFLAGS) gps/gps.c basic_lib/i2c_master.c $(MAIN).c -o Autonomous_FC.elf -lm
	patmos-clang -I ../.. -O2 $(LDFLAGS) gps/gps.c basic_lib/i2c_master.c $(MAIN).c -o ~/t-crest/patmos/tmp/Autonomous_FC.elf -lm
# 	patmos-clang -o  de10-IMU.elf $(MAIN).c -lm 


	