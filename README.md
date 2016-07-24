This is the code and data repository for the SIGGRAPH 2016 technical paper [PiGraphs: Learning Interaction Snapshots from Observations](http://graphics.stanford.edu/projects/pigraphs/).  If you use this code or data in your research please cite the paper as described in the [project website](http://graphics.stanford.edu/projects/pigraphs/).

See the [Command Overview wiki](../../wiki/Command-Overview) for an overview of the UI and a description of the available commands for synthesizing interaction snapshots.

We provide a set of pre-trained models in the [learned-models.cached.zip](../../releases/download/v1.0/learned-models.cached.zip) archive.  After downloading, extract to the repository root.  To compile and run, you will need to download external dependencies package from [mLibExternal.zip](../../releases/download/v1.0/mLibExternal.zip) and also extract them to the repository root.

# Compiling

Compiles with Visual Studio 2013.  Main solution file is `pigraphs.sln`.  Make sure working directory of project configurations is set to `$(SolutionDir)bin\$(Configuration)`.  You may need to copy missing dlls to the working directory path.

Default parameters are in `$(SolutionDir)bin\parameters_default.txt`.  If `$(SolutionDir)bin\parameters.txt` exists, it acts as an override for any parameter settings contained in `parameters_default.txt`.

### Prerequisites for compiling
 - DirectX11 SDK : http://www.microsoft.com/en-us/download/details.aspx?id=6812
 - mLib and mLibExternal dependencies provided by dependencies archive as described above. After extracting to repository root, add the `<repoRoot>\mLibExternal\libsWindows\dll64` directory to your PATH.
 - Java JNI
   - Install Java 8 JDK
   - Set `JAVA_HOME` to Java 8 JDK
   - Add `%JAVA_HOME%\bin` and `%JAVA_HOME%\jre\bin\server` to path

### Troubleshooting compilation and running
If you have trouble running on Windows 8.1 and you get a `D3D11_CREATE_DEVICE_DEBUG` error, you may need to install the D3D11 SDK Debug Layer with Windows 8.1 SDK: http://msdn.microsoft.com/en-us/windows/desktop/bg162891.aspx .  See http://stackoverflow.com/questions/19121093/directx-11-missing-sdk-component-on-windows-8-1 for more details.

### Regenerating proxy code for interfacing with Weka using Jace
- Compile and package JavaApps as a jar file (`JavaApps/target/SceneGrok-0.0.1.jar`). IntelliJ IDEA with Maven installed is probably the easiest method.
- Run `ant` in the `jace-proxy` subdirectory (requires [ant](http://ant.apache.org/) ) to create the `.cpp` and `.h`  files.  Note: You will need manually add newly generated files to the `jace-proxy` Visual Studio project manually.  Also, you will need to create some code in the `jace-proxy` (see `wekautil` as an example) to indicate what classes/methods you want Jace to autogenerate.  Make sure to include the proper class headers so Jace can properly detect the method signature and autogenerate your code (for instance, if your method uses a java List in the signature, include the hpp for List, just pretend it's already there).

# Scan data documentation
Each scan has a filename with format `sceneId.<ext>` where `<ext>` is one of the extensions below.

## .ply
Stanford PLY meshes of reconstructed environments.  Scale is in meters and the up axis is `+Z`.

## .grid.gz
Gzip compressed volumetric occupancy voxel grid for reconstructed environments.  Stores aggregate occupancy information from scanning process in a dense voxel grid.  Each voxel contains the total counts of being observed as free space, being observed as occupied (in TSDF truncation threshold), and being behind a surface (and it is therefore unknown whether it is occupied or free).

Occupancy grid file format (after gunzipping) starts with ASCII header lines of following format (ignore comments after # on each line):
```
voxelgrid\t1  # file format identifier and version number
dimensions\tdimX dimY dimZ  # vec3ul
worldToGrid\tfloat00 float01 float02 floa03 float10 ... float33  # mat4f
depthMin\tfloat             # minimum depth threshold used during scanning
depthMax\tfloat             # maximum depth threshold used during scanning
voxelSize\tfloat            # voxel dimension in meters
labels\tId1,Id2,Id3,...     # labels of voxels, currently ignored
data:                       # line indicating start of binary data
Binary section
```

Remaining binary section is sequence pairs of uint32 (freeCtr,occCtr) in iteration order x, y, z (outermost to innermost).
The worldToGrid 4x4 matrix transforms from the corresponding .ply file coordinate frame to the voxel grid coordinate frame.  Refer to [`libsg/core/OccupancyGrid.h`](libsg/core/OccupancyGrid.h) and [`libsg/core/OccupancyGrid.cpp`](libsg/core/OccupancyGrid.cpp) for example reader and writer functions.

## .vox
Labeled sparse voxel grid for reconstructed environments where labels are object and object part categories.  Format is very similar to occupancy grid, with following ASCII header:
```
labeledgrid\t1              # file format identifier and version number
dimensions\tdimX dimY dimZ  # vec3ul
worldToGrid\tfloat00 float01 float02 floa03 float10 ... float33  # mat4f
voxelSize\tfloat            # dimensions of voxels in meters
labels\tId1,Id2,Id3,...     # comma separated string labels will be referenced by 0-based integer index in binary data
numVoxels\tsize_t           # total number of voxels in this file
data:                       # line indicating start of binary data
Binary section
```

Binary section is sequence of 4-tuples of int16_t (x, y, z, label) where x, y, and z give the voxel integer coordinates, and label is an index into the comma separated string labels.

## .segs.json
Segmentation and object labels for segments of .ply format meshes of reconstructed scenes.  The file is in JSON format with the following fields:
```
params : record of segmentation parameters used
sceneId : id of scene PLY mesh that this segmentation corresponds to
segGroups[{},...,{}] : array of segmentation group records with fields `id`, `label`, `objectId`, `obb`, `dominantNormal` and `segments`.  The `label` field is the object and part label string, and `segments` is an array of segment ids which are grouped together and assigned the given label.  The `objectId` field is a unique id for each instance of labeled object (to disambiguate between multiple instances e.g., multiple chairs in a scene).
segIndices : array of integer segment ids for each vertex in the corresponding .ply mesh file.  Vertices are numbered in the order they are specified in the .ply file, and all vertices with the same segment id belong to one segment.  The ids are not neccessarily consecutive numbers.
```

Refer to the code in [`libsg/segmentation/SegmentGroup.cpp`](libsg/segmentation/SegmentGroup.cpp) (particularly `SegmentGroupRecord::load`) for an implementation of a reading function.


# Recording data documentation
The file basename has the format `sceneId_cameraId_timestamp.<ext>` where `<ext>` is one of file types below.  The `sceneId` part corresponds to the reconstructed scene in which the recording was taken.

A recording consists of the set of files with the same basename and each following extension:
- `.json` : Recording header in JSON format, containing tracked skeleton data.  See [`libsg/core/Recording.h`](libsg/core/Recording.h) and [`libsg/core/Recording.cpp`](libsg/core/Recording.cpp) for example loading code and documentation.
- `.color.avi` : RGB frame video for recording. Frames in the video file correspond to the timestamps stored in the `.json` header.
- `.depth.avi` : Depth and body segmentation mask video (see details below). Again, frames correspond to the timestamps in the `.json` header.
- `.interactions.txt` : Interaction annotations for recording. Each line specifies a time range in the video and a set of verb:noun pairs that apply for that time range.  The format of each line is `startTimeInSec-endTimeInSec,verb1:noun1|verb2:noun2|...|verbN:nounN`.  In other words, the file is comma-separated with the first column giving a time range in `start-end` format, and the second column giving a string encoding of the verb:noun pairs.  The string encoding uses `|` as a separator between pairs, and `:` as a separator between verb and noun.
- `.color.mp4` and `.depth.mp4` : lossily compressed versions of the `.avi` video files, using the H264 codec.  Useful for viewing of videos on systems where the Lagarith codc is not installed

Video frames can be read from the video files using the [OpenCV VideoCapture](http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html) interface or any other video reading interface that can return frame values as raw byte arrays.

Both color and depth frame videos are encoded using the [Lagarith lossless codec](http://lags.leetcode.net/codec.html) (FOURCC identifier `LAGS`).  Color frames are standard BGR YUY2 frames while the depth frames encode both raw Kinect One depth values as well as tracked body segmentation mask in each pixel.  Both these values are packed into three bytes per pixel as `[R,G,B] = [body,depth0,depth1]`.  The OpenCV code snippet below gives an example of parsing a `.depth.avi` frame:

```
#include <opencv2/core.hpp>
void parseDepthFrame(const cv::Mat& depthAndBodyFrame) {
  // Split RGB channels
  cv::Mat channels[3];
  cv::split(depthAndBodyFrame, channels);
  // First channel is the body index mask (uchar values of 0xff correspond
  // to non-body pixels, anything else is a body id for each tracked person)
  const cv::Mat body = channels[0];
  // Next two channels are merged to get short encoding depth value in mm
  cv::Mat depthChs(512, 424, CV_8UC2);
  cv::merge(&channels[1], 2, depthChs);
  cv::Mat depth(depthChs.rows, depthChs.cols, CV_16UC1);
  memmove(depth.data, depthChs.data, 2 * depthChs.rows * depthChs.cols);
}
```

