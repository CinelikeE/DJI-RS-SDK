cmake_minimum_required(VERSION 3.22)
project(RoninSDK C)

set(CMAKE_C_STANDARD 99)

add_executable(RoninSDK HAL_Version/main.c HAL_Version/Handle.c HAL_Version/Handle.h HAL_Version/custom_crc16.c HAL_Version/custom_crc16.h HAL_Version/custom_crc32.c HAL_Version/custom_crc32.h HAL_Version/DJI_RS_SDK.c HAL_Version/DJI_RS_SDK.h HAL_Version/CmdCombine.c HAL_Version/CmdCombine.h)
