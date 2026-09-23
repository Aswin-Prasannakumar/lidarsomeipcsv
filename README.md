1. In the .sln file of publisher, change the directory of the csv to the csv holding the data. <br>
2. install the dependencies that are required to run.<br>
3. In the sln of publisher, go to properties, in the C/C++ part, set additional include directories to C:\vsomeip_install\include<br>
4. Language set it as ISO C++ 17<br>
5. In the linker, General set, additional library directories to C:\vsomeip_install\lib<br>
6. in linker, additional dependencies, set vsomeip3.lib;ws2_32.lib;<br>
7. Also if warning for code 4996 appears, in properties, c/c++, advanced, disable specific warnings set to 4996<br>
8. copy the csv file lidar_data.csv in any place and update the location in the publisher .sln<br>
9. change the replay timing to false if there is no need to simulate the actual time between frames<br>
10. run the subscriber.exe in sunscriber-x64-lidar_subscriber.exe first. then run the publisher.sln file.<br>
