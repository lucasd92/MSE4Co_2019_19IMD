MSE4Co_2019_19IMD

Compilar módulo:
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabi-
make

Compilar test:
arm-linux-gnueabi-gcc test.c -o test

Intalar módulo(desde BBB):
insmod /root/mympu9250/mympu9250.ko

Ejecutar test (desde BBB):
./test

Código de test basado en biblioteca sAPI implementada por Eric Pernia para el proyecto CIAA.

https://github.com/epernia/cese-edu-ciaa-template/blob/master/libs/sapi/sapi_v0.5.2/external_peripherals/src/sapi_imu_mpu9250.c
