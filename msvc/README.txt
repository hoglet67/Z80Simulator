Visual Studio 2015 (and newer)
==============================

Only 64-bit configuration is provided (debug/relase).

Install libpng with MS vcpkg tool: https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=vs-2019

 vcpkg install libpng:x64-windows
 vcpkg integrate install

Open msvc/Z80Simulator.sln in the IDE.
You probably want to switch to "Release" configuration for performance reasons.

Project Properties -> Debugging -> Command Arguments, set to "Z80"
Project Properties -> Debugging -> Working Directory, set to ".."

If you just want to build the solution on the command line, use this command
(from the VS2015 Developer Command Prompt):

 msbuild Z80Simulator.sln /property:Configuration=Release
