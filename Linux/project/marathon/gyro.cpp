#include "gyro.h"
#include <iostream>
#include <stdio.h>
#include <string.h>

using namespace std;
using namespace Robot;


void printGYRO(Robot::CM730 *cm730){
	unsigned char table[128];
	int addr;
	int value;

	if(cm730->ReadTable(200, CM730::P_MODEL_NUMBER_L, CM730::P_RIGHT_MIC_H, &table[CM730::P_MODEL_NUMBER_L], 0) != CM730::SUCCESS)
		{
			printf(" Can not read table!\n");
			return;
		}
	addr = CM730::P_GYRO_Z_L; value = CM730::MakeWord(table[addr], table[addr+1]);
	printf( " GYRO_Z                  (R) [%.3d]:%5d (L:0x%.2X H:0x%.2X)\n", addr, value, table[addr], table[addr+1]);
	addr = CM730::P_GYRO_Y_L; value = CM730::MakeWord(table[addr], table[addr+1]);
	printf( " GYRO_Y                  (R) [%.3d]:%5d (L:0x%.2X H:0x%.2X)\n", addr, value, table[addr], table[addr+1]);
	addr = CM730::P_GYRO_X_L; value = CM730::MakeWord(table[addr], table[addr+1]);
	printf( " GYRO_X                  (R) [%.3d]:%5d (L:0x%.2X H:0x%.2X)\n", addr, value, table[addr], table[addr+1]);
	addr = CM730::P_ACCEL_X_L; value = CM730::MakeWord(table[addr], table[addr+1]);
	printf( " ACCEL_X                 (R) [%.3d]:%5d (L:0x%.2X H:0x%.2X)\n", addr, value, table[addr], table[addr+1]);
	addr = CM730::P_ACCEL_Y_L; value = CM730::MakeWord(table[addr], table[addr+1]);
	printf( " ACCEL_Y                 (R) [%.3d]:%5d (L:0x%.2X H:0x%.2X)\n", addr, value, table[addr], table[addr+1]);
	addr = CM730::P_ACCEL_Z_L; value = CM730::MakeWord(table[addr], table[addr+1]);
	printf( " ACCEL_Z                 (R) [%.3d]:%5d (L:0x%.2X H:0x%.2X)\n", addr, value, table[addr], table[addr+1]);

}