call "%VS120COMNTOOLS%\..\..\VC\vcvarsall.bat" x86_amd64
svn update %MLIB_DIR%
svn update %MLIB_EXTERNAL_DIR%
MSBuild pigraphs.sln