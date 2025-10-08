/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <cmath>
#include "ImuOTOS.h"
#include "lidar_lib.h"

#include <thread>
#include <iomanip>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <fcntl.h>



#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

// Structure pour représenter l'état du robot pour la localisation
// Repère mathématique direct :
//  - X+ = 0 radian
//  - Y+ = +90° (pi/2 rad)
//  - Les angles augmentent dans le sens trigonométrique (anti-horaire)

const char *socket_path = "/tmp/robot.sock";
int server_sock = socket(AF_UNIX, SOCK_STREAM, 0);
struct sockaddr_un addr;
int client_sock = -1;

using namespace sl;




bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


int main() {
    fcntl(server_sock, F_SETFL, O_NONBLOCK);
    
    position poslidar;
    position posImu;
    position posrobot;
    init_socket(server_sock, client_sock, addr ,socket_path);


    ImuOTOS imu("/dev/i2c-1", 0x17);
    if (!imu.openBus()) {
        std::cerr << "Erreur ouverture I2C\n";
        return 1;
    }

    if (!imu.reset()) {
        std::cerr << "Erreur reset\n";
        imu.closeBus();
        return 1;
    }

    if (!imu.calibrate(255)) {
        std::cerr << "Erreur calibrage\n";
        imu.closeBus();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3 * 255));

    if (!imu.enableSignalProcessing(true, true, true, true)) {
        std::cerr << "Erreur config signal\n";
        imu.closeBus();
        return 1;
    }

    int argc = 5; // Nombre d'arguments (nom du programme + 3 arguments)
    const char *argv[] = {
        "./ultra_simple",  // Nom du programme (argv[0])
        "--channel",        // argv[1]
        "--serial",         // argv[2]
        "/dev/ttyUSB0",     // argv[3]
        "256000"            // argv[4]
    };
    position p_init(2200, 1000, 0);
    posrobot= p_init;
    std::vector<float> scan(NUM_ANGLES, -1.0f);
    // crée un tableau de 3 TrackResult
    std::vector<TrackResult> trackedPoints;
    ImuPose p_init_imu = position_to_imu(p_init);
    imu.writePose(p_init_imu);
    inittrackedpoints(trackedPoints, p_init, 3);
    //printf("track points coordinates p0: (%.2f, %.2f)\n", trackedPoints[0].distance, trackedPoints[0].angle);
    //printf("track points coordinates p1: (%.2f, %.2f)\n", trackedPoints[1].distance, trackedPoints[1].angle);
    //printf("track points coordinates p2: (%.2f, %.2f)\n", trackedPoints[2].distance, trackedPoints[2].angle);
    int pillardetected = 0;
	const char * opt_is_channel = NULL; 
	const char * opt_channel = NULL;
    const char * opt_channel_param_first = NULL;
	sl_u32         opt_channel_param_second = 0;
    sl_u32         baudrateArray[2] = {115200, 256000};
    sl_result     op_result;
	int          opt_channel_type = CHANNEL_TYPE_SERIALPORT;

	bool useArgcBaudrate = false;

    IChannel* _channel;
# pragma region intialisation
    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s\n", SL_LIDAR_SDK_VERSION);

	 
	if (argc>1)
	{ 
		opt_is_channel = argv[1];
	}
	else
	{
		print_usage(argc, argv);
		return -1;
	}

	if(strcmp(opt_is_channel, "--channel")==0){
		opt_channel = argv[2];
		if(strcmp(opt_channel, "-s")==0||strcmp(opt_channel, "--serial")==0)
		{
			// read serial port from the command line...
			opt_channel_param_first = argv[3];// or set to a fixed value: e.g. "com3"
            printf("serial port=%s\n", opt_channel_param_first);
			// read baud rate from the command line if specified...
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);	
            printf("baudrate=%d\n", opt_channel_param_second);
			useArgcBaudrate = true;
		}
		else if(strcmp(opt_channel, "-u")==0||strcmp(opt_channel, "--udp")==0)
		{
			// read ip addr from the command line...
			opt_channel_param_first = argv[3];//or set to a fixed value: e.g. "192.168.11.2"
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);//e.g. "8089"
			opt_channel_type = CHANNEL_TYPE_UDP;
		}
		else
		{
			print_usage(argc, argv);
			return -1;
		}
	}
	else
	{
		print_usage(argc, argv);
        return -1;
	}

	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
	{
		if (!opt_channel_param_first) {
#ifdef _WIN32
		// use default com port
		opt_channel_param_first = "\\\\.\\com3";
#elif __APPLE__
		opt_channel_param_first = "/dev/tty.SLAB_USBtoUART";
#else
		opt_channel_param_first = "/dev/ttyUSB0";
#endif
		}
	}

    
    // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    if(opt_channel_type == CHANNEL_TYPE_SERIALPORT){
        if(useArgcBaudrate){
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK((drv)->connect(_channel))) {
                op_result = drv->getDeviceInfo(devinfo);

                if (SL_IS_OK(op_result)) 
                {
	                connectSuccess = true;
                }
                else{
                    delete drv;
					drv = NULL;
                }
            }
        }
        else{
            size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
			for(size_t i = 0; i < baudRateArraySize; ++i)
			{
				_channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK((drv)->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo);

                    if (SL_IS_OK(op_result)) 
                    {
	                    connectSuccess = true;
                        break;
                    }
                    else{
                        delete drv;
					    drv = NULL;
                    }
                }
			}
        }
    }
    else if(opt_channel_type == CHANNEL_TYPE_UDP){
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK((drv)->connect(_channel))) {
            op_result = drv->getDeviceInfo(devinfo);

            if (SL_IS_OK(op_result)) 
            {
	            connectSuccess = true;
            }
            else{
                delete drv;
				drv = NULL;
            }
        }
    }


    if (!connectSuccess) {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)?
			(fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
				, opt_channel_param_first)):(fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n"
				, opt_channel_param_first));
		
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed();
    // start scan...
    drv->startScan(0,1);
#pragma endregion
     std::cout << "Début boucle principale\n";
    // fetech result and print it out...
    std::cout << std::fixed << std::setprecision(5);
    ImuPose pose;

    try_accept_client(client_sock, server_sock);

    while (1) {

        if (client_sock >= 0) {
            float data[3] = {posImu.x, posImu.y, posImu.angle};

            write(client_sock, data, sizeof(data));
        }else {
            try_accept_client(client_sock, server_sock);
        }
       
        if (imu.readPose(pose)) {
            posImu.x = pose.x * 1000.0f; // en mm
            posImu.y = pose.y * 1000.0f; // en mm
            posImu.angle = pose.h ;
        } else {
            std::cerr << "Erreur lecture pose\n";
        }
        posrobot= posImu;

            grabAndUpdateScan(scan, drv);

            trackPoints(scan, trackedPoints, RESOLUTION,posrobot);

            pillardetected = 0;

            for (int i = 0; i < 3; i++) {
                if (!trackedPoints[i].found) {
                    // Recalcul de la position théorique
                    float dx = pillars[i].x - posrobot.x;
                    float dy = pillars[i].y - posrobot.y;
                    trackedPoints[i].distance = sqrt(dx * dx + dy * dy)-RAYON_PILIER;
                    trackedPoints[i].angle = atan2(dy, dx) - posrobot.angle; // tout en radians
                    //garder l'angle entre 0 et 2PI
                    if (trackedPoints[i].angle < 0) trackedPoints[i].angle += 2 * M_PI;
                    if (trackedPoints[i].angle >= 2 * M_PI) trackedPoints[i].angle -= 2 * M_PI;
                } else {
                    pillardetected++;
                }
            std::cout << "\n=> Pilier " << (i+1) << ": angle=" << trackedPoints[i].angle * 180.0f / M_PI << "°, distance=" << trackedPoints[i].distance << " mm, found=" << trackedPoints[i].found << " scan=" << scan[(int)(trackedPoints[i].angle * 180.0f / M_PI / RESOLUTION)] << " mm";
            } 
        std::cout << "\nPilliers détectés a 180°: " << scan[180 / RESOLUTION] << " mm" << std::endl;
        poslidar = computePose(trackedPoints.data());
        //periodiquement corriger la position du robot
        if (pillardetected >= 5) {
            posrobot = poslidar;
            ImuPose p = position_to_imu(posrobot);  
            imu.writePose(p);     
        }
        std::cout << "\n=> Pose calculée: x=" << poslidar.x << " mm, y=" << poslidar.y << " mm, angle=" << poslidar.angle * 180.0f / M_PI << "°\n";
        std::cout << "=> Pose IMU: x=" << posImu.x << " mm, y=" << posImu.y << " mm, angle=" << posImu.angle * 180.0f / M_PI << "°\n";
        std::cout << "=> delta position: dx=" << (poslidar.x - posImu.x) << " mm, dy=" << (poslidar.y - posImu.y) << " mm, dangle=" << (poslidar.angle - posImu.angle) * 180.0f / M_PI << "°\n";
        if (ctrl_c_pressed) {
            break;
        }
    }
    
    drv->stop();
	delay(200);
	if(opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed(0);
    // done!
on_finished:
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}

