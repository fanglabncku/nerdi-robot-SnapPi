#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <wiringPi.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>

#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>

#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include <bcm2835.h>

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyACM0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}


#define SERVER_PORT 8080
#define LENGTH_OF_LISTEN_QUEUE 2
#define BUFFER_SIZE 1024
typedef unsigned int bool;

#define false 0
#define true 1

/*open the ultrasound sensor port*/
int PI_TRIG=4;
int PI_ECHO=5;
float ballAngle = 360;
float boxAngle = 360;
/* For sensor subfunction */
bool StateUltrasound = true;

void setGPIOPin(int pin, int state)
{
        // set pin in/out
	pinMode(pin,OUTPUT);
        // set state
	if(state)
	digitalWrite(pin,HIGH);
	else
	digitalWrite(pin,LOW);
	printf("set GPIO %d to %d",pin,state);
	return -1;
}
int ultrasoundDistance()
{

	int ping      = 0;
	int pong      = 0;
	float distance = 0;

	pinMode(PI_TRIG, OUTPUT);
	pinMode(PI_ECHO, INPUT);

	// Ensure trigger is low.
	digitalWrite(PI_TRIG, LOW);
	digitalWrite(PI_ECHO, LOW);
	delay(50);

	// Trigger the ping.
	digitalWrite(PI_TRIG, HIGH);
	delay(10);
	digitalWrite(PI_TRIG, LOW);

	// Wait for ping response, or timeout.
	while (digitalRead(PI_ECHO) == LOW ) {
	}

	ping = micros();

	// Wait for pong response, or timeout.
	while (digitalRead(PI_ECHO) == HIGH) {
	}

	pong = micros();

	// Convert ping duration to distance.
	distance = (float) (pong - ping) * 0.017150;
	printf("distance: %d\n",distance);
	return (int)distance;
}
int dy_turn(id,degree, mode){

	// Initialize PortHandler Structs
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	int port_num = portHandler(DEVICENAME);

	// Initialize PacketHandler Structs
	packetHandler();

	int dxl_comm_result = COMM_TX_FAIL;             // Communication result

	// Open port
	if (openPort(port_num))
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (setBaudRate(port_num, BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}


	//Set Dynamixal parameter
	uint16_t Goal_Position = 30;
	uint16_t Goal_Position_Value = 0;
	uint16_t Goal_Velocity = 32;
	uint16_t Goal_Velocity_Value = 300;
	uint16_t Now_Goal_Position_Value;

	// Read the present position
	Now_Goal_Position_Value = read2ByteTxRx(port_num, PROTOCOL_VERSION, id, Goal_Position);
	delay(50);
	printf("Present Position Value: %d\n", Now_Goal_Position_Value);
	Goal_Position_Value = Now_Goal_Position_Value + degree * 5;
	switch (mode)
	{
	case 0:
		//printf("do 0");
		while (Goal_Position_Value > 1024)
			Goal_Position_Value = 0 + (Goal_Position_Value - 1024);
		break;
	case 1:
		//printf("do 1");
		while (Goal_Position_Value > 1024)
			Goal_Position_Value = 1024 - (65535 - Goal_Position_Value);
		break;
	case 2:
			Goal_Position_Value = degree;
		break;
	}
	printf("Change Position Value: %d\n", Goal_Position_Value);
	// Do Action
	//write2ByteTxOnly2(port_num, id, Goal_Velocity, Goal_Velocity_Value);
	//delay();
	write2ByteTxOnly2(port_num, id, Goal_Position, Goal_Position_Value);
	delay(50);
	// Close port
	closePort(port_num);

	return 0;
}
int readdynamixel(int state ,uint8_t id) {
	// Initialize PortHandler Structs
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	int port_num = portHandler(DEVICENAME);

	// Initialize PacketHandler Structs
	packetHandler();

	int dxl_comm_result = COMM_TX_FAIL;             // Communication result

													// Open port
	if (openPort(port_num))
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (setBaudRate(port_num, BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	uint16_t Goal_Position = 30;
	uint16_t Value = 0;
	uint16_t Goal_Velocity = 32;
	printf("now ID:%d\n", id);
	if (state == 0) {
		// Read the present position
		Value = read2ByteTxRx(port_num, PROTOCOL_VERSION, id, Goal_Position);
		printf("Present Position Value: %d\n", Value);
	}
	if (state == 1) {
		// Read the present velocity
		Value = read2ByteTxRx(port_num, PROTOCOL_VERSION, id, Goal_Velocity);
		printf("Present Velocity Value: %d\n", Value);
	}

	// Close port
	closePort(port_num);

	return Value;

}
int checkBox()
{
   FILE *fp;
   fp=fopen("/home/pi/box.txt","r");

   char state[10]="";
   float angle;

   if(!feof(fp))
	fscanf(fp,"%s %f",state,&angle);


   if(strlen(state) == 0)
   {
	printf("No data");
	//setdynamixelstate(254,"motorsetLED",0);
        boxAngle = 360;
        fclose(fp);
        return 0;
   }
   if(strlen(state) > 0)
   {
	setdynamixelstate(254,"motorsetLED",1);
	boxAngle = angle;
	fclose(fp);
        return 1;
   }
   return 0;
}
int checkBall()
{
   FILE *fp;
   fp=fopen("/home/pi/ball.txt","r");

   char state[10]="";
   float angle;

   if(!feof(fp))
	fscanf(fp,"%s %f",state,&angle);


   if(strlen(state) == 0)
   {
	printf("No data");
	setdynamixelstate(254,"motorsetLED",0);
        ballAngle = 360;
        fclose(fp);
        return 0;
   }
   if(strlen(state) > 0)
   {
        printf("color:%s angle:%f\n",state,angle);
        if(strcmp(state,"blue") == 0)
            setdynamixelstate(254,"motorsetLED",4);
        else if(strcmp(state,"green") == 0)
            setdynamixelstate(254, "motorsetLED",2);
        else if(strcmp(state,"red") == 0)
	    setdynamixelstate(254, "motorsetLED",5);
        ballAngle = angle;
        fclose(fp);
        return 1;
   }
   return 0;

}
int broadcast(char** str){
  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(DEVICENAME);
  // Initialize PacketHandler Structs
  packetHandler();

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int id;
  int sum=0;
  // Open port
  if (openPort(port_num))
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Try to broadcast ping the Dynamixel
  broadcastPing(port_num, PROTOCOL_VERSION);
  delay(50);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);

  printf("Detected Dynamixel : \n");
  for (id = 0; id < MAX_ID; id++)
  {
	  if (getBroadcastPingResult(port_num, PROTOCOL_VERSION, id)) {
		  printf("ID:%03d\n",id);
		  sprintf(str[sum], "ID:%03d", id);
		  sum += 1;
	  }
  }
  delay(50);
  // Close port
  closePort(port_num);
  return sum;

}
int setdynamixelstate(int id, char* state, int data){
	// Initialize PortHandler Structs
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	int port_num = portHandler(DEVICENAME);
	uint8_t dxl_error = 0;
	// Initialize PacketHandler Structs
	packetHandler();

	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	int sum = 0;
	// Open port
	if (openPort(port_num))
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (setBaudRate(port_num, BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}
	// build the parameter
	uint16_t Torque_Enable = 24;
	uint16_t LED = 25;
	uint16_t LED_Value = data;
	uint8_t  Torque_Enable_Value = data;
	uint16_t Goal_Velocity = 32;
	int16_t Goal_Velocity_Value = data;
	uint8_t Control_Mode = 11;
	uint8_t  Control_Mode_Value  = data;
	// Try to set state
	if (strcmp(state, "motorsetvelocity") == 0 )

	{
		uint8_t Value = 2;
		Value = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, Control_Mode);


		// check the velocity value
		if (Value == 2)
		{
			if (Goal_Velocity_Value > 1023) {
				Goal_Velocity_Value = 1023;
			}
			else if (Goal_Velocity_Value < 0) {
				Goal_Velocity_Value = 0;
			}
		}
		else
		{
			if (Goal_Velocity_Value > 1023) {
				Goal_Velocity_Value = 1023;
			}
			else if (Goal_Velocity_Value < 0) {
				Goal_Velocity_Value = -(Goal_Velocity_Value)+1023;
				if(Goal_Velocity_Value >2047)
					Goal_Velocity_Value = 2047;
			}

		}
		printf("write date to motor\n ");
		write2ByteTxOnly2(port_num, id, Goal_Velocity, Goal_Velocity_Value);

	}
	else if (strcmp(state, "motorallsetvelocity") == 0)
	{
		// check the velocity value
		if (Goal_Velocity_Value > 1023) {
			Goal_Velocity_Value = 1023;
		}
		else if (Goal_Velocity_Value < 0) {
			Goal_Velocity_Value = 0;
		}
		printf("write date to all motor\n ");
		write2ByteTxOnly2(port_num, id, Goal_Velocity, Goal_Velocity_Value);
	}
	else if (strcmp(state, "motorsetmode") == 0)
	{
		printf("ID : %03d\n", id);
		uint8_t Value;
		Value = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, Control_Mode);
		printf("Mode : %03d\n", Value);

		printf("write mode to motor: %03d , loc = %03d\n",Control_Mode_Value,Control_Mode);
		write1ByteTxOnly2(port_num, id, Control_Mode, Control_Mode_Value);
		delay(500);


		Value = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, Control_Mode);
		printf("Mode : %03d\n", Value);
	}
	else if (strcmp(state, "motorsetTorque") == 0)
	{
		printf("write data to motor\n");
		write1ByteTxOnly2(port_num, id, Torque_Enable, Torque_Enable_Value);
	}
	else if (strcmp(state, "motorsetLED") == 0) {
		printf("write data to motor\n");
		write1ByteTxOnly2(port_num, id, LED, LED_Value);
	}
	delay(50);
	// Close port
	closePort(port_num);

}
int sync_write(char* id, char* state, char* data) {
	// for shortage the id and data
	char *temp;
	int  ID[20];
	int  DATA[20];
	int  sum = 0;
	char *comma = "/";
	/* if the state is setTyerSpeed, then
	   the ID should be string.
	*/
	if(strcmp(state, "setTyerSpeed")== 0)
	{
		// 拆解 id 序列
		temp = strtok(id, comma);
		while (temp != NULL) {
			if(strcmp(temp,"left") == 0)
				ID[sum] = 1;
			else if(strcmp(temp,"right") == 0)
				ID[sum] = 2;
			else
				ID[sum] = 0;

			sum += 1;
			temp = strtok(NULL, comma);


		}
	    strcpy(state, "motorsetvelocity");
	}else
	{
		// 拆解 id 序列
		temp = strtok(id, comma);
		while (temp != NULL) {
				ID[sum] = atoi(temp);

			sum += 1;
			temp = strtok(NULL, comma);
		}
	}
	sum = 0;
	// 拆解 data 序列
	temp = strtok(data, comma);

	while (temp != NULL) {
		DATA[sum] = atoi(temp);
		sum += 1;
		temp = strtok(NULL, comma);
	}



	// Initialize PortHandler Structs
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	int port_num = portHandler(DEVICENAME);

	// Initialize PacketHandler Structs
	packetHandler();

	int dxl_comm_result = COMM_TX_FAIL;             // Communication result


	// Open port
	if (openPort(port_num))
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (setBaudRate(port_num, BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}
	// start sync write
	int groupwrite_num =0;
	uint16_t Goal_Velocity = 32;
	uint16_t Goal_Velocity_Value = 300;
	uint16_t Goal_Position = 30;
	uint16_t Goal_Position_Len = 2;
	uint16_t Goal_Velocity_Len = 2;
	uint16_t Now_Goal_Position_Value;
	uint16_t Goal_Position_Value;
	uint16_t Torque_Enable = 24;
	uint8_t Control_Mode = 11;
	int size = sum;
	printf("size = %d\n", sum);
	int i;
	int check = 0;
	uint8_t Value = 2;
	// initial motor velocity (default)
	if (strcmp(state, "motorsetvelocity") == 0){
		groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, Goal_Velocity, Goal_Velocity_Len);


		for (i = 0; i <= size; i++)
		{
			printf("%d",groupwrite_num);


			Value = read1ByteTxRx(port_num, PROTOCOL_VERSION, ID[i], Control_Mode);

			// check the velocity value
			if (Value == 2)
			{
				if (DATA[i] > 1023) {
					Goal_Velocity_Value = 1023;
				}
				else if (DATA[i] < 0) {
					Goal_Velocity_Value = 0;
				}else
					Goal_Velocity_Value = DATA[i];
			}
			else
			{
				if (DATA[i] > 1023) {
					Goal_Velocity_Value = 1023;
				}
				else if (DATA[i] < 0) {
					Goal_Velocity_Value = -(DATA[i])+1023;
					if(DATA[i] >2047)
						Goal_Velocity_Value = 2047;
				}else
					Goal_Velocity_Value = DATA[i];

			}
		printf("Value:%d--write data:ID%d,Velocity%d\n",Value,ID[i],Goal_Velocity_Value);
		if (groupSyncWriteAddParam(groupwrite_num, ID[i], Goal_Velocity_Value,  Goal_Velocity_Len) != True)
				fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", ID[i]);


		}
		check = 1;

	}
	else
		write2ByteTxOnly2(port_num, 254, Goal_Velocity, Goal_Velocity_Value);


	//default the groupwrite_num & write the ID and param;
	if (strcmp(state, "motorclockwise") == 0) {
		printf("do grouptwire initial for motorclockwise\n");
		int groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, Goal_Position, Goal_Position_Len);
		for (i = 0; i < size; i++)
			{
			// Read the present position
			Now_Goal_Position_Value = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID[i], Goal_Position);
			printf("Present Position Value: %d\n", Now_Goal_Position_Value);
			Goal_Position_Value = Now_Goal_Position_Value + DATA[i] * 5;

			if (Goal_Position_Value > 1024)
				Goal_Position_Value = 0 + (Goal_Position_Value - 1024);

			if (groupSyncWriteAddParam(groupwrite_num, ID[i], Goal_Position_Value,  Goal_Position_Len) != True)
				fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", ID[i]);
			}
		check = 1;
		}
	else if (strcmp(state, "motorcounterclockwise") == 0) {
		int groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, Goal_Position, Goal_Position_Len);
		for (i = 0; i < size; i++)
		{
			Now_Goal_Position_Value = read2ByteTxRx(port_num, PROTOCOL_VERSION, ID[i], Goal_Position);
			printf("Present Position Value: %d\n", Now_Goal_Position_Value);
			Goal_Position_Value = Now_Goal_Position_Value + DATA[i] * 5;

			if (Goal_Position_Value > 1024)
				Goal_Position_Value = 1024 - (65535 - Goal_Position_Value);

			if (groupSyncWriteAddParam(groupwrite_num, ID[i], Goal_Position_Value, Goal_Position_Len) != True)
				fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", ID[i]);
		}
		check = 1;
	}
	else if (strcmp(state, "motorsetposition") == 0) {
		int groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, Goal_Position, Goal_Position_Len);
		for (i = 0; i < size; i++)
		{
				Goal_Position_Value = DATA[i];
				printf("%d\n", DATA[i]);
			if (Goal_Position_Value > 1024)
				Goal_Position_Value = 1024;

			if (groupSyncWriteAddParam(groupwrite_num, ID[i], Goal_Position_Value, Goal_Position_Len) != True)
				fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", ID[i]);
		}
		check = 1;
	}
	if(check)
	{
	printf("Start write to the dynamixel");
	groupSyncWriteTxPacket(groupwrite_num);

	if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
		printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
	groupSyncWriteClearParam(groupwrite_num);
	}
	closePort(port_num);
	return 0;
}
int readall(char** str){
	// Initialize PortHandler Structs
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	int port_num = portHandler(DEVICENAME);
	// Initialize PacketHandler Structs
	packetHandler();

	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	int id;
	int sum = 0;
	// Open port
	if (openPort(port_num))
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (setBaudRate(port_num, BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}
	//uint16_t Torque_Enable = 24;
	//uint8_t  Torque_Enable_Value = 1;
	uint16_t Goal_Position = 30;
	uint16_t Value;
	// Try to broadcast ping the Dynamixel
	broadcastPing(port_num, PROTOCOL_VERSION);
	if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
		printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
	//write1ByteTxOnly2(port_num, 254, Torque_Enable, Torque_Enable_Value);
	printf("Detected Dynamixel : \n");
	for (id = 0; id < MAX_ID; id++)
	{
		if (getBroadcastPingResult(port_num, PROTOCOL_VERSION, id)) {
			//printf("ID:%03d\n",id);
			// read angle

			Value = read2ByteTxRx(port_num, PROTOCOL_VERSION, id, Goal_Position);
			//write1ByteTxOnly2(port_num, id, Torque_Enable, 0);
			//sprintf(str[sum], "ID:%03d Pos:%04d", id,Value);
			sprintf(str[sum], "%d,%d", id, Value);
			sum += 1;
		}
	}
	delay(100);
	// Close port
	closePort(port_num);
	return sum;

}
int setID(int new_id) {
	// Initialize PortHandler Structs
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	int port_num = portHandler(DEVICENAME);
	// Initialize PacketHandler Structs
	packetHandler();
	uint8_t dxl_error = 0;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	int sum = 0;
	// Open port
	if (openPort(port_num))
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (setBaudRate(port_num, BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}
	// build the parameter
	int old_id = 0;
	uint8_t ID_addr  = 3;
	uint8_t id;
	uint8_t Value;
	//broadcastPing(port_num, PROTOCOL_VERSION);
	for (id = 1; id < 10; id++)
	{
	Value = read1ByteTxRx(port_num, PROTOCOL_VERSION, id, ID_addr);
	if(Value == id)
		break;
	printf("now id:%d is %d \n",id,Value);
/*
		if (getBroadcastPingResult(port_num, PROTOCOL_VERSION, id)) {

			old_id = id;
			printf("old id:%03d\n", old_id);
			break;
		}
*/
	}
	old_id = Value;
	uint8_t ID_value = new_id;
	uint8_t ID = 254;
	printf("ID : %03d ID_addr : %03d ID_value : %03d", old_id, ID_addr, ID_value);
	// Try to set state
		printf("write ID to motor\n");
		write1ByteTxOnly2(port_num, old_id, ID_addr, ID_value);
		//write1ByteTxRx(port_num, PROTOCOL_VERSION, old_id, ID_addr, ID_value);
		delay(500);

		if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
		{
			printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
		}
		else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
		{
			printRxPacketError(PROTOCOL_VERSION, dxl_error);
		}
		else
		{
			printf("Dynamixel has been successfully connected \n");
		}


		Value = read1ByteTxRx(port_num, PROTOCOL_VERSION, new_id, ID_addr);
		printf("ID : %03d", Value);


	// Close port
	closePort(port_num);

	return -1;
}

char* choose_motion(char *input) {
	char const* id;
	char const* data;
	char const* state;
	char const* comma = ",";
	int dy_id;
	int dy_data;
	int temp;
	static char result[100];
	state = strtok(input, "?");

	if (strcmp("motorsetLED", state) == 0)
	{
		if (id = strtok(NULL, comma)) {
			// 確認是否有輸入id
			dy_id = atoi(id);
			// 確認是否有輸入燈色
			if (data = strtok(NULL, comma)) {
				if (strcmp("red", data) == 0)
					setdynamixelstate(dy_id, state, 1);
				else if (strcmp("green", data) == 0)
					setdynamixelstate(dy_id, state, 2);
				else if (strcmp("yellow", data) == 0)
					setdynamixelstate(dy_id, state, 3);
				else if (strcmp("blue", data) == 0)
					setdynamixelstate(dy_id, state, 4);
				else if (strcmp("blue_green", data) == 0)
					setdynamixelstate(dy_id, state, 6);
				else if (strcmp("pink", data) == 0)
					setdynamixelstate(dy_id, state, 5);
				else if (strcmp("white", data) == 0)
					setdynamixelstate(dy_id, state, 7);
			}
		}

		return -1;
	}
	else if (strcmp("motorcounterclockwise", state) == 0)
	{// send degree & mode(1)=left parameter to func

		//確認是否有ID輸入
		if (id = strtok(NULL, comma)) {
			dy_id = atoi(id);
			//確認是否有角度輸入
			if (data = strtok(NULL, comma)) {
				dy_data = atoi(data);
				dy_turn(dy_id, -dy_data, 1);
			}
		}
		return -1;
	}
	else if (strcmp("motorclockwise", state) == 0)
	{
		// send degree & mode(0)=right parameter to func

		//確認是否有ID輸入
		if (id = strtok(NULL, comma)) {
			dy_id = atoi(id);
			//確認是否有角度輸入
			if (data = strtok(NULL, comma)) {
				dy_data = atoi(data);
				dy_turn(dy_id, dy_data, 0);
			}
		}

		return -1;
	}
	else if (strcmp("motorbroadcast", state) == 0)
	{
		char ** broadcast_temp = malloc(20 * sizeof(char*));
		for (int i = 0; i < 20; i++) {
			broadcast_temp[i] = malloc(20 * sizeof(char*));

		}
		int arr_size = broadcast(broadcast_temp);

		for (int i = 0; i < arr_size; i++) {
			printf("%s\n", broadcast_temp[i]);
			if (i == 0) {
				sprintf(result, "broadcast\n");
				strcpy(result, broadcast_temp[i]);
			}
			else {
				strcat(result, "\n");
				strcat(result, broadcast_temp[i]);
			}
		}

		// free memory
		for (int i = 0; i < 10; i++)
			free(broadcast_temp[i]);

		free(broadcast_temp);

		return result;
	}
	else if (strcmp("motorsetmode", state) == 0)
	{
		// Set motor mode (join or wheel)
		if (id = strtok(NULL, comma)) {

			//確認是否有輸入ID
			dy_id = atoi(id);
			//確認是否有輸入模式
			if (data = strtok(NULL, comma)) {
				printf("do set motor mode : %s\n", data);
				if (strcmp("wheel", data) == 0)
					setdynamixelstate(dy_id, state, 1);
				else if (strcmp("join", data) == 0)
					setdynamixelstate(dy_id, state, 2);
			}
		}
		return -1;
	}
	else if (strcmp("motorreadangle", state) == 0)
	{
		// Read motor present angle

		if (id = strtok(NULL, comma)) {


			dy_id = atoi(id);
			temp = readdynamixel(0, dy_id);
			sprintf(result, "%d", temp);
		}
		else
		{
			return -1;
		}
			return result;
	}
	else if (strcmp("motorreadvelocity", state) == 0)
	{
		// Read motor present velocity

		if (id = strtok(NULL, comma)) {
			dy_id = atoi(id);
			temp = readdynamixel(1, dy_id);
			sprintf(result, "%d", temp);
			return result;
		}
		else
		{
			return -1;
		}

		// result_message = readdynamixel(dy_id);
	}
	else if (strcmp("motorsetvelocity", state) == 0)
	{
		// Set motor velocity
		if (id = strtok(NULL, comma))
		{
			dy_id = atoi(id);
			if (data = strtok(NULL, comma)) {
				dy_data = atoi(data);
				printf("do set motor velocity\n");
				setdynamixelstate(dy_id, state, dy_data);
			}
		}
		return -1;
	}
	else if (strcmp("motorallsetvelocity", state) == 0)
	{
		// Set motor velocity
		dy_id = 254;
		if (data = strtok(NULL, comma))
		{
			dy_data = atoi(data);
			printf("do set all motor velocity\n");
			setdynamixelstate(dy_id, state, dy_data);
		}
		return -1;
	}
	else if (strcmp("sync", state) == 0) {
		state = strtok(NULL, comma);
		id = strtok(NULL, comma);
		data = strtok(NULL, comma);
		sync_write(id, state, data);
		return -1;
	}
	else if (strcmp("motorreadallangle", state) == 0) {
		// Read all motor present angle
		setdynamixelstate(254, "motorsetTorque", 1);
		char ** readall_temp = malloc(20 * sizeof(char*));
		for (int i = 0; i < 20; i++) {
			readall_temp[i] = malloc(20 * sizeof(char*));

		}
		int arr_size = readall(readall_temp);

		for (int i = 0; i < arr_size; i++) {
			printf("%s\n", readall_temp[i]);
			if (i == 0) {
				//sprintf(result, "read all angle\n");
				strcpy(result, readall_temp[i]);
			}
			else {
				strcat(result, "/");
				strcat(result, readall_temp[i]);
			}
		}
		setdynamixelstate(254, "motorsetTorque", 0);
		// free memory
		for (int i = 0; i < 10; i++)
			free(readall_temp[i]);

		free(readall_temp);

		return result;
	}
	else if (strcmp("motorsetposition", state) == 0) {
		// send degree & mode(2)=setposition parameter to func
		if (id = strtok(NULL, comma)) {
			dy_id = atoi(id);
			if (data = strtok(NULL, comma))
			{
				dy_data = atoi(data);
				dy_turn(dy_id, dy_data, 2);
			}
		}
		return -1;
	}
	else if (strcmp("motorsetID", state) == 0) {
		// change motor's ID

		//確認是否有輸入ID
		if (data = strtok(NULL, comma)) {
			dy_data = atoi(data);
			printf("%d\n", dy_data);
			setID(dy_data);
		}
		return -1;
	}
	else if (strcmp("motorsetTorque", state) == 0)
	{
		if (id = strtok(NULL, comma)) {
			//確認是否有輸入ID
			dy_id = atoi(id);
			//確認是否有輸入on/off
			if (data = strtok(NULL, comma))
			{
				if (strcmp("on", data) == 0)
					setdynamixelstate(254, state, 1);
				else if (strcmp("off", data) == 0)
					setdynamixelstate(254, state, 0);
			}
		}
		return -1;

	}
	else if (strcmp("raspiultrasound",state) == 0)
	{

		if(StateUltrasound)
		{
			temp = ultrasoundDistance();
			sprintf(result, "%d", temp);
			return result;
		}else
		{
			sprintf(result,"ultrasound fail");
			return result;
		}
	}
	else if (strcmp("setGPIO",state) == 0){

		if (id = strtok(NULL, comma)){
			//check if there is any pin coming
			dy_id = atoi(id);
			if (data = strtok(NULL, comma))
			{
				if(strcmp("on",data) == 0)
					setGPIOPin(dy_id,1);
				else
					setGPIOPin(dy_id,0);
				return -1;

			}

		}
		return "fail";
	}
	else if (strcmp("setTyerSpeed",state) == 0){

		// Set motor velocity
		if (id = strtok(NULL, comma))
		{
			dy_id = atoi(id);
			if (data = strtok(NULL, comma)) {
				dy_data = atoi(data);
				printf("do set motor velocity\n");
				strcpy(state,"motorsetvelocity");
				setdynamixelstate(dy_id, state, dy_data);
			}
		}
		return -1;
	}
	else if (strcmp("checkBall",state) == 0){

		temp = checkBall();
                if(temp == 1)
		sprintf(result, "true");
                else
                sprintf(result, "false");

		printf("checkBall:%s\n",result);
                return result;

	}
	else if (strcmp("ballAngle",state) == 0){

        FILE *fp = fopen("/home/pi/ball.txt","r");
        char state[10]="";
        float angle = -180;

        if(!feof(fp))
           	fscanf(fp,"%s %f",state,&angle);
        fclose(fp);

        if(angle != -180){
            sprintf(result,"%f",angle);
            return result;
        }else
            return -1;



	}
	else if (strcmp("checkBox",state) == 0){
		temp = checkBox();
                if(temp == 1)
                sprintf(result, "true");
                else
                sprintf(result, "false");

		printf("checkBox:%s\n",result);
        return result;
	}
	else if (strcmp("boxAngle",state) == 0){


        FILE *fp = fopen("/home/pi/box.txt","r");
        char state[10]= "";
        float angle = -180;

        if(!feof(fp))
           	fscanf(fp,"%s %f",state,&angle);
        fclose(fp);

        if(angle != -180){
            sprintf(result,"%f",angle);
            return result;
        }else
            return -1;


	}
	else
		return - 1;
}


int main(int argc, char **argv)
{
    struct sockaddr_in server_addr, client_addr;
    int server_socket,client_socket;
    socklen_t client_len;
    char buf[BUFFER_SIZE] = "";
    char *response_type =
"HTTP/1.1 200 OK\r\n"
"Server: Apache/2.47 (Ubuntu)\r\n"
"Content-type: application/octet-stream\r\n"
"Content-Length: %d\r\n"
"Access-Control-Allow-Origin: *\r\n"
"\r\n";
    char response[BUFFER_SIZE];

printf("%s",buf);
    //bzero(&server_addr,sizeof(server_addr));


    /* create a socket */
    server_socket = socket(AF_INET,SOCK_STREAM,0);
    if( server_socket < 0)
    {
        printf("Create Socket Failed!");
        exit(1);
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    server_addr.sin_port = htons(SERVER_PORT);

    printf("creat socket done\n");

    /* bind socket to a specified address*/
    //setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));  //設置為可重複使用
    if( bind(server_socket,(struct sockaddr*)&server_addr,sizeof(server_addr))==-1)  //把地址綁定
    {
        printf("Server Bind Port : %d Failed!", SERVER_PORT);
        exit(1);
    }
    printf("bind done\n");

     printf ("Raspberry Pi wiringPi HC-SR04 Sonar test program.\n");

     if (wiringPiSetup () == -1) {
	  printf("fail to open wiringPiSetup!\n");
	  StateUltrasound = false;
     }

     if (setuid(getuid()) < 0) {
	  perror("Dropping privileges failed.\n");
	  StateUltrasound = false;
     }

    /* listen a socket */
    if(listen(server_socket, LENGTH_OF_LISTEN_QUEUE)==-1)
    {
        printf("Server Listen Failed!");
        exit(1);
    }
    printf("start to listen\n");

    char *command;
    char *temp_buf;

    /* run server */
    while (1)
    {
        client_len = sizeof(client_addr);
        /* accept socket from client */
        client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_len);
        if( client_socket < 0)
        {
            printf("Server Accept Failed!\n");
            break;
        }
        if(recv(client_socket, buf, 1024, 0) >0)
	{
		//printf("receive data:%s\n",buf);
	     	//split string
		//find special string
		command = strtok(buf,"/");
		if(command != NULL)
		{
			command = strtok(NULL," ");
			printf("%s\n",command);
			//if command is not equal NULL
			//meaning recived command
			//than split the command
			temp_buf = choose_motion(command);

			//printf("temp\n",temp_buf);

			if (temp_buf != -1)
			{
					strcpy(buf,temp_buf); //initial buf;
					//printf("buf:%s",buf);
					/* send data to client*/
					sprintf(response, response_type, strlen(buf));
					strcat(response,buf);
					send(client_socket, response, (int)strlen(response),0);
		 	}else
			{
				strcpy(buf,"succeed write data");
				sprintf(response,response_type, strlen(buf));
				strcat(response,buf);
				send(client_socket, response, (int)strlen(response),0);
			}

		}
	}

	printf("succeed\n");
	fflush(stdout);
  	close(client_socket);
    }

    close(server_socket);
    return 0;
}

