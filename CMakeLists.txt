cmake_minimum_required(VERSION 3.22)
project(RoninSDK C)

set(CMAKE_C_STANDARD 99)

add_executable(RoninSDK main.c Handle.c Handle.h custom_crc16.c custom_crc16.h custom_crc32.c custom_crc32.h DJI_RS_SDK.c DJI_RS_SDK.h CmdCombine.c CmdCombine.h )
