# Real Time Face Filtering

Computer Vision & Pattern Recognition class project @ Koc University  
[Watch Demo](https://goo.gl/CqCtpn)
[Read Project Report](https://goo.gl/pe8qvE)

Root Unity Project is the ‘COMP408/‘ folder.  
### Building Instructions
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

### Running
In the Source Folder run these commands:  

Open unity app:  
$ open ./Demo.app/  

Wait and run project  
$ ./build/project

