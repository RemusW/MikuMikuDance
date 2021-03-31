***Linux Instructions***

To build and run:

~~~~
mkdir build
cd build
cmake ..
make
cd ..
./build/bin/skinning
~~~~

***OSX Instructions***

**DEPENDENCIES**
- Download Xcode if you do not already have it (https://developer.apple.com/xcode/). You will need an Apple Developer ID to do this.
- Run the following installs via Homebrew:
    brew install glm
    brew install zlib
    brew install libjpeg-turbo
- Download CMake GUI (https://cmake.org/download/) for MacOS
- Download GLEW source (http://glew.sourceforge.net/). Follow README.md directions to install:
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=/Library/Frameworks ./cmake
    make -j4 
- Download GLFW (https://www.glfw.org/). Follow README.md directions to install (https://www.glfw.org/docs/latest/compile_guide.html#compile_generate):
    mkdir build
    cd build
    cmake ..    
    make -j4

Note: I already had glew/glfw installed via Brew (brew install glew/brew install glfw), and CMake uses those dependencies when configuring, but I still needed these versions of GLEW and GLFW. So the brew steps probably aren't necessary, but if your configure doesn't work, maybe try installing them.

**PROJECT BUILD**

- Create an empty build folder in skinning
- In CMake GUI:
    1) Select skinning as source code
    2) Select empty build folder to build binaries
    3) Click Configure (use Xcode as generator with default native compilers)
    4) If Configure succeeds, hit Generate
- You are done with CMake. Open the .xcodeproj that was generated.
- Create a group called "common" and add glew.c and libglfw3.a to this folder. When it gives you options to add these, select add to targets "skinning". glew.c will be in the folder you downloaded for GLEW. libglfw3.a will be under build/src in the folder you downloaded for GLFW.
- Build the pmdreader and utgraphicsutil libraries by setting those libraries as the active schemes (top bar to the right of the run and stop buttons in Xcode). The static libraries will be generated under build/Debug if you are set to the (default) Debug scheme. Add these two libraries to the "common" folder.
- Under Skinning->Targets->skinning->Build Phases, add glew.c to Compile Sources. 
- Click the + button above "Target Dependencies" and select "New Link Binary with Libraries Phase". Add the following Frameworks/libraries:
    -libiconv.tbd
    -IOKit.framework
    -OpenGL.framework
    -Cocoa.framework
    -libpmdreader.a
    -libutgraphicsutil.a
    -Add Other->libglfw3.a (located in your "common" folder")
    *Note: All of these frameworks should be searchable except libglfw3 which is in your project directory, and Add Other will allow you to navigate to it.
- Run skinning scheme to build the binary. You can add arguments by clicking on "Manage Schemes" as a drop down where you select the scheme.

***Windows Instructions***

1. Download and install Microsoft Visual Studio (with C++ Compiler)
and CMake (https://cmake.org/download/).

2. Clone or Download VCPKG from (https://github.com/Microsoft/vcpkg)

3. Follow the install instruction for VCPKG at
(https://github.com/Microsoft/vcpkg). In particular you need to
perform the following steps:
3a. Open up a command prompt *in administrator mode* (click the
Windows icon, type "cmd", then right-click on "Command Prompt" and
click "Run as administrator").
3b. Navigate to the folder where you extracted VCPKG.
3c. Run the following commands:
.\bootstrap-vcpkg.bat
.\vcpkg integrate install

4. Install the project dependencies:
.\vcpkg install glfw3
.\vcpkg install glew
.\vcpkg install glm
.\vcpkg install zlib
.\vcpkg install libjpeg-turbo

5. Add VCPKG_ROOT to your environment variables. Click the Windows
icon, type "env", and click on "Edit the system environment
variables". Click the "Environment Variables" button. Under "User
variables," click "New..." and add:
Variable Name: VCPKG_ROOT
Value: the path to your VCPKG folder.

6. Open CMake. Set the paths for the starter code:
Where is the source code: /path/to/skinning
Where to build the binaries: /path/to/skinning/build

7. Hit "Configure". Pick the native compilers and Win32 as the compilation platform (*not* x64). You should not get any errors. Keep hitting
"Configure" until the red lines are all white.

8. Hit "Generate".

9. Hit "Open Project". This will launch Visual Studio with your project open.

Additional Tips:
- you will need to click on "skinning" in the Solution Explorer and
right-click->"set as startup project" if you want to debug the code
from within Visual Studio.
- you can change between Debug and Release builds using the drop-down
menu at the top of MSVC.
- to add new source files to the build, put them in the src folder,
and run Configure and Generate again in CMake. Don't add them using
MSVC's built-in GUI.
- to set the command-line arguments during debugging, right click on
"skinning" in the Solution Explorer and click "Properties". Under
"Debugging", change the "Command Arguments". Note that there are
separate command-line argument for Release and Debug mode and you will
need to set both.
- to check that your project is properly configured for grading,
delete the entire "build" folder, and repeat steps 6-9 above. Check
that everything still compiles and runs.
