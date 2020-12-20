/****************************************************************************
 *   Copyright (c) 2017 Stephen Chaves. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMIED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <iostream>
#include <signal.h>
#include <stdbool.h>
#include <syslog.h>
#include <unistd.h>
#include <stdlib.h> // for getenv

#include "SnapdragonCameraManager.hpp"
#include "SnapdragonCameraTypes.hpp"
#include "ImageStreamer.hpp"

static void PrintUsage()
{
  std::cout << "usage: voxl-image-streamer -c [CAM_TYPE]... ARGS" << std::endl;
  std::cout << "   or: voxl-image-streamer -c [CAM_TYPE] -i [IP]... ARGS" << std::endl;
  std::cout << "-h                print this message." << std::endl;
  std::cout << "-c [CAM_TYPE]     0=HIRES, 1=TRACKING, 2=STEREO" << std::endl;
  std::cout << "-i [IP]           IP address of the VOXL" << std::endl;
  std::cout << "-p [PORT]         port number to stream on, default 5556" << std::endl;
  std::cout << "-e [EXP]          set exposure; EXP between 0 and 1, default 0.36" << std::endl;
  std::cout << "-r [WIDTHxHEIGHT] image widthxheight, default 640x480" << std::endl;
  std::cout << "-g [GAIN]         set gain; GAIN between 0 and 1, default 0.35" << std::endl;
  std::cout << "-n [NUM]          stream every n-th image, default 1" << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Note, this chooses the camera IDs based on teh environment variables" << std::endl;
  std::cout << "set in /etc/modalai/camera_env.sh" << std::endl;
  std::cout << "Set up this file before use with the voxl-configure-cameras tool!" << std::endl;
  std::cout << "" << std::endl;
  std::cout << "The -i ip address argument is optional. If omitted, the TCP stream" << std::endl;
  std::cout << "server will not start but the camera will still initialize." << std::endl;
  std::cout << "Available resolutions are printed when the program starts." << std::endl;
  std::cout << "Note, image_viewer.py currently does not show color." << std::endl;
  std::cout << "" << std::endl;
}

static bool caught_sig_int = false;
static void SigIntHandler(int sig)
{
  caught_sig_int = true;
}


int main(int argc, char* argv[])
{

  int c;
  char* ip_addr = NULL;
  int port_num = 5556;
  bool is_stereo = false;
  int stream_nth = 1;
  int cam_id=-1;

  Snapdragon::CameraParameters param;
  param.camera_config.cam_type = Snapdragon::CameraType::OPTIC_FLOW;
  param.camera_config.cam_format = Snapdragon::CameraFormat::YUV_FORMAT;
  param.camera_config.fps = 30;

  // Getopt command line arguments
  while ((c = getopt(argc, argv, "c:i:p:he:g:n:r:")) != -1)
  {
    switch (c)
    {
      case 'h':
        PrintUsage();
        return 0;
      case 'c':
        switch (atoi(optarg))
        {
          case 0:
           std::cout << "HIRES camera selected." << std::endl;
           cam_id=atoi(getenv("HIRES_CAM_ID"));
           std::cout << "using cam_id=" << cam_id << std::endl;
           if(cam_id<0 || cam_id>3){
              std::cout << "ERROR, invalid cam_id for HIRES cam" << std::endl;
              std::cout << "run voxl-configure-cameras" << std::endl;
              return -1;
           }
           param.camera_config.cam_id=cam_id;
           param.camera_config.cam_type = Snapdragon::CameraType::HIRES;
           param.camera_config.cam_format = Snapdragon::CameraFormat::YUV_FORMAT;
           param.camera_config.num_image_buffers = 3;
           break;
          case 1:
           std::cout << "TRACKING camera selected." << std::endl;
           cam_id=atoi(getenv("TRACKING_CAM_ID"));
           std::cout << "using cam_id=" << cam_id << std::endl;
           if(cam_id<0 || cam_id>3){
              std::cout << "ERROR, invalid cam_id for TRACKING cam" << std::endl;
              std::cout << "run voxl-configure-cameras" << std::endl;
              return -1;
           }
           param.camera_config.cam_id=cam_id;
           param.camera_config.cam_type = Snapdragon::CameraType::OPTIC_FLOW;
           break;
          case 2:
           std::cout << "STEREO camera selected." << std::endl;
           cam_id=atoi(getenv("STEREO_CAM_ID"));
           std::cout << "using cam_id=" << cam_id << std::endl;
           if(cam_id<0 || cam_id>3){
              std::cout << "ERROR, invalid cam_id for STEREO cam" << std::endl;
              std::cout << "run voxl-configure-cameras" << std::endl;
              return -1;
           }
           param.camera_config.cam_id=cam_id;
           param.camera_config.cam_type = Snapdragon::CameraType::STEREO;
           break;
          // case 3:
          //  std::cout << "STEREO camera selected." << std::endl;
          //  param.camera_config.cam_type = Snapdragon::CameraType::STEREO;
          //  is_stereo = true;
          //  break;
          default:
           std::cout << "No camera selected" << std::endl;
           PrintUsage();
           return -1;
           break;
        }
        break;
      case 'i':
        ip_addr = optarg;
        break;
      case 'p':
        port_num = atoi(optarg);
        break;
      case 'e':
        param.camera_config.exposure = atof(optarg);
        break;
      case 'g':
        param.camera_config.gain = atof(optarg);
        break;
      case 'r':
        int w,h,ret;
        printf("startin sscanf\n");
        ret=sscanf(optarg,"%dx%d",&w,&h);
        if(ret!=2){
          printf("invalid resolution argument. try: -r 640x480\n");
          return -1;
        }
        printf("using resolution: width=%d height=%d\n",w,h);
        param.camera_config.pixel_width = w;
        param.camera_config.pixel_height = h;
        break;
      case 'n':
        stream_nth = atoi(optarg);
        break;
      default:
        printf("ERROR: Unsupported option.\n");
        return -1;
    }
  }

  if(cam_id<0){
      std::cout << "please select a camera with the -c argument" << std::endl;
      PrintUsage();
      return -1;
  }

  // CAMERA MANAGER
  Snapdragon::CameraManager camera( &param );
  if (camera.Initialize() != 0){
    printf("ERROR: Error initializing CameraManager. Exiting.\n");
    return -1;
  }
  printf("Starting camera\n");
  if (camera.Start() != 0){
    printf("ERROR: Error starting camera streaming. Exiting.\n");
    return -1;
  }

  // TCP SERVER
  TcpServer server;
  ImageStreamer img_strmr(false, stream_nth);
  if(ip_addr!=NULL){
    server.CreateSocket(ip_addr, port_num);
    server.BindSocket();
    server.ConnectClient();

    // IMAGE STREAMER
    if (img_strmr.Initialize(&camera, &server, &param) != 0){
      printf("ERROR: Error initializing ImageStreamer. Exiting.\n");
    }
    printf("starting image stream\n");
    if (img_strmr.StartProcessing() != 0){
      printf("ERROR: Error starting ImageStreamer processing. Exiting.\n");
      return -1;
    }
  }

  // MAIN LOOP
  signal(SIGINT, SigIntHandler);
  printf("running!\n");
  if(ip_addr!=NULL){
    while (img_strmr.IsRunning() && camera.IsRunning() && !caught_sig_int){
      usleep(1e5);
    }
  }
  else{
    while(camera.IsRunning() && !caught_sig_int){
      usleep(1e5);
    }
  }

  // CLEANUP
  if(ip_addr!=NULL){
    printf("Stopping ImageStreamer processing...\n");
    img_strmr.StopProcessing();
  }

  printf("Stopping CameraManager processing...\n");
  camera.Stop();
  camera.Terminate();

  printf("Done.\n");

  return 0;
}

