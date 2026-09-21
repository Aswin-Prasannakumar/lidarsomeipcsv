in the .sln file of publisher, change the directory of the csv to the csv holding the data.
install the dependencies that are required to run this.
In the sln of publisher, go to properties, in the C/C++ part, set additional include directories to C:\vsomeip_install\include
Language set it as ISO C++ 17
In the linker, General set, additional library directories to C:\vsomeip_install\lib
in linker, additional dependencies, set vsomeip3.lib;ws2_32.lib;
Also if warning for code 4996 appears, in properties, c/c++, advanced, disable specific warnings set to 4996
