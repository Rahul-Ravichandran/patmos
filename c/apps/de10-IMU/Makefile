MAIN?=compass_test
#MAIN?=blinking

# This is the trivial Hello World example, distributed over two .c files

LDFLAGS?= \
        -mpatmos-method-cache-size=0x1000 \
        -mpatmos-stack-base=0x080000 -mpatmos-shadow-stack-base=0x078000 \
        -Xgold --defsym -Xgold __heap_end=0x070000

all:
	patmos-clang -I ../.. -O2 $(LDFLAGS) i2c_master.c $(MAIN).c -o de10-IMU.elf -lm
	patmos-clang -I ../.. -O2 $(LDFLAGS) i2c_master.c $(MAIN).c -o ~/t-crest/patmos/tmp/de10-IMU.elf -lm
# 	patmos-clang -o  de10-IMU.elf $(MAIN).c -lm 

