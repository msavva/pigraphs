#pragma once

//
// mLib config options
//

#ifdef _DEBUG
#define MLIB_ERROR_CHECK
#define MLIB_BOUNDS_CHECK
#endif // _DEBUG

//
// mLib includes
//
#define MLIB_GRAPHICSDEVICE_TRANSPARENCY
#include <mLibCore.h>
#include <mLibLodePNG.h>
#include <mLibD3D11.h>
#include <mLibD3D11Font.h>
//#include <mLibOpenMesh.h>
//#include <mLibDepthCamera.h>
//#include <mLibFreeImage.h>


