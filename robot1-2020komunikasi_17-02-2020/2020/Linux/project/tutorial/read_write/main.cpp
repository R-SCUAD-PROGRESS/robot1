#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"


using namespace Robot;

int main()
{
	printf( "\n===== Read/Write Tutorial for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyACM0");
	CM730 cm730(&linux_cm730);
	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}
	/////////////////////////////////////////////////////////////////////

	int value;
    cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 0, 0);
    cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 0, 0);
    cm730.WriteWord(JointData::ID_R_ELBOW,          MX28::P_TORQUE_ENABLE, 0, 0);

    cm730.WriteByte(JointData::ID_L_SHOULDER_PITCH, MX28::P_P_GAIN, 8, 0);
    cm730.WriteByte(JointData::ID_L_SHOULDER_ROLL,  MX28::P_P_GAIN, 8, 0);
    cm730.WriteByte(JointData::ID_L_ELBOW,          MX28::P_P_GAIN, 8, 0);

	while(1)
	{
		printf("\r");


		if(cm730.ReadWord(18, MX28::P_PRESENT_LOAD_L, &value, 0) == CM730::SUCCESS)
			printf("%3d\t", value);
		else
			printf("---");
if(cm730.ReadWord(17, MX28::P_PRESENT_LOAD_L, &value, 0) == CM730::SUCCESS)
			printf("%3d\t", value);
		else
			printf("---");
if(cm730.ReadWord(16, MX28::P_PRESENT_LOAD_L, &value, 0) == CM730::SUCCESS)
			printf("%3d\t", value);
		else
			printf("---");
if(cm730.ReadWord(15, MX28::P_PRESENT_LOAD_L, &value, 0) == CM730::SUCCESS)
			printf("%3d\t", value);
		else
			printf("---");
if(cm730.ReadWord(14, MX28::P_PRESENT_LOAD_L, &value, 0) == CM730::SUCCESS)
			printf("%3d\t", value);
		else
			printf("---");
if(cm730.ReadWord(13, MX28::P_PRESENT_LOAD_L, &value, 0) == CM730::SUCCESS)
			printf("%3d\t", value);
		else
			printf("---");
if(cm730.ReadWord(12, MX28::P_PRESENT_LOAD_L, &value, 0) == CM730::SUCCESS)
			printf("%3d\t", value);
		else
			printf("---");

		
		usleep(5000);
	}

	return 0;
}
