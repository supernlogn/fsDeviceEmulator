# Filesystem Device Emulator

### About

### Dependencies & Platforms
Currently this software has only been tested in Ubuntu Linux.

Libraries needed
1. libunwind

### Configuration

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
and how the raw device memory also changes in s25fl.dat.