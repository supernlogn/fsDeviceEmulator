# Filesystem Device Emulator

### About
This is an emulator of a storage device. It can be used to
analyze how an implemented file system works. Also
the installation is kept as simple as possible.

### Dependencies & Platforms
Currently this software has only been tested in Ubuntu Linux. 

- Libraries needed
1. libunwind

- Programs required:
1. xxd
2. ddd

### Configuration
If libunwind is not install to your system you have 2 options:
1. install it from source
2. install it via package manager (only for unix like systems)

install it via package manager:

For debianoids:
type inside a terminal
```bash
apt-get install libunwind8, libunwind8-dev
```


### How To Use
For making all files:
```bash
make clean; make all; 
```
All executables are inside build folder.

For executing the executables directly type in terminal:
```bash
make clean; make run;
```

For debuging a filesystem directly type in terminal:
```bash
make clean; make run;
```

During debug, watch as the json file s25fl_action.json changes due to filesystem actions 
and how the raw device memory also changes in s25fl.dat. Use the command below to see the
hex file of memory any time:
```bash
xxd -b s25fl.dat > s25fl.hex
```


### To be developed
1. Asynchronous operation
2. qemu integration
3. xxd equivalent