#pragma once

#include <libsg.h>

#include <mLibCore.h>

// Contains constants used by Vizzer
namespace constants {

//! Structs to store constant buffer data for shader computations
struct BasicLightShaderConstantBuffer {
  ml::mat4f worldViewProj;
};

// Kinect One intrinsics from OpenCV calibration
const ml::mat4f KINECT_ONE_INTRINSICS(
  367.50f, 0.0f, 0.0f, 267.09f,
  0.0f, -366.33f, 0.0f, 197.22f,
  0.0f, 0.0f, 1.0f, 0.0f,
  0.0f, 0.0f, 0.0f, 1.0f);
//const ml::mat4f KINECT_ONE_INTRINSICS_INV(
//  1.f / 367.50f,     0.0f,  0.0f, -267.09f / 367.5f,
//  0.0f,    -1.f / 366.33f,  0.0f,  197.22f / 366.33f,
//  0.0f,            0.0f,  1.0f,           0.0f,
//  0.0f,            0.0f,  0.0f,           1.0f);
const ml::mat4f KINECT_ONE_INTRINSICS_INV(
  1.f / 361.56f, 0.0f, 0.0f, -256.f / 361.56f,
  0.0f, -1.f / 367.19f, 0.0f, 212.f / 367.19f,
  0.0f, 0.0f, 1.0f, 0.0f,
  0.0f, 0.0f, 0.0f, 1.0f);
const size_t KINECT_ONE_DEPTH_HEIGHT = 424;
const size_t KINECT_ONE_DEPTH_WIDTH = 512;

}  // namespace constants


