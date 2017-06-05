Root Unity Project is the ‘COMP408/‘ folder.

1) Building Instructions

In the Source Folder run these commands:

Create build folder
$ mkdir build

Enter in to build folder
$ cd build

Enable SSE4 for speed
$ cmake .. -DUSE_SSE4_INSTRUCTIONS=ON

Enable AVX for speed
$ cmake .. -DUSE_AVX_INSTRUCTIONS=ON

Build executables
$ cmake --build . --config Release

2)Running
In the Source Folder run these commands:

Open unity app:
$ open ./Demo.app/

Wait and run project
$ ./build/project

