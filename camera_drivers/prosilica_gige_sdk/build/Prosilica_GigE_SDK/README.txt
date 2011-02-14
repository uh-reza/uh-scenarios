### Prosilica GigE SDK 1.20 for Linux
###
### 04/02/09
###

Notes:

* This distribution support x86, PPC, armle and x64 (AMD64) Linux. Modify the "ARCH" file located in the Examples directory before building the samples to select the proper architecture: x86, ppc, arm or x64.

* The shared library in ./bin-pc is compiled with GCC 4.2

* Static libraries are provides for GCC 3.3, 3.4, 4.1, 4.2, they can be found in ./lib-pc

* Each of the sample code can be build as follow:

 > make sample ; make install

The executables will be copied into the ./bin-pc folder.

* The provided viewer requiere wxGTK (>= 2.6). The makefile can be modified to use the version you have installed on your system. A compiled version of the viewer is provided (for convenience) for each support architecture. It is staticaly compiled against wxGTK.

* For best streaming performance, the examples and your application should be run as superuser.

* The MTU of the GigE adapter should be changed to accommodate for Jumbo frames using the 'ifconfig' command. The syntax is as follow:

> sudo ifconfig eth0 mtu 8228

where eth0 is the adapter name and 8228 the maximum size of the frames to be used by the camera. If the MTU is set to a lower value, the camera's settings ("PacketSize") should be adjusted.

* As the API use some signals internally, it is required to "protect" some of your system calls (such as sem_wait) from been interupted. This can be done by calling the system call from within a loop and only exiting from it if there was no error or if errno is different from EINTR.

* In order to use multicasting, you may  have to add manualy a route. The syntax is as follow:

> sudo route -n add -net 224.0.0.0 netmask 240.0.0.0 dev eth3

where eth3 is the adapter name (replace by yours). Also be aware that multicasting will only work if the application is been run as root.

* The Java folders contains an JNI interface to PvAPI, plus a set of samples. You will need to use the build.xml file located in each subdirectory to import the project with Eclipse. Each of the following samples: JListAttributes, JListCameras, JSnap, JStream, JThread, JThread3 need to have PvJPI in there build path. For convenience, the JNI dynamic library has been built and placed in the bin-pc folder. Each of the Java samples need in its Run/debug settings the following added to its VM argument: -Djava.library.path=/path/to/the/SDK/bin-pc/x86. The working directory will also have to be /path/to/the/SDK/bin-pc/x86.
