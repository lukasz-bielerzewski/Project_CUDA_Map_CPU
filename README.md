**Author/Maintenance**:
Dominik Belter

# WalkingRobotSim simulator installation guide
Installation instruction was tested on Ubuntu 20.04 operating system

## Installer

The clone_deps.sh script installs all required software from repositories, clones requires libraries and compiles them:

     - mkdir ~/Sources
     - cd ~/Sources
     - git clone https://github.com/LRMPUT/walkers
     - mkdir ~/Libs

### 1. NVidia PhysX installation:
    1. Run:

    $ cd ~/Libs
    $ git clone https://github.com/NVIDIAGameWorks/PhysX

    2. Modify:

    - Line 40 in ~/Libs/Physx/physx/compiler/public/CMakeLists.txt

        from

        IF(NOT DEFINED CMAKEMODULES_VERSION)

        to

        IF(NOT DEFINED ${CMAKEMODULES_VERSION})
        

    - Line 154 in ~/Libs/PhysX/physx/source/geomutils/src/gjk/GuGJKType.h

        from

        PX_FORCE_INLINE Ps::aos::PsMatTransformV& getRelativeTransform(){ return mAToB; }
        
        to

        const PX_FORCE_INLINE Ps::aos::PsMatTransformV& getRelativeTransform(){ return mAToB; }

	- In ~/Libs/Physx/physx/source/compiler/cmake/linux/CMakeLists.txt  

        remove all "-Werror" and add "-fno-strict-aliasing -Wno-error=nonnull" 

    - To Install libxxf86vm-dev deb package: 
        $ sudo apt-get update
	    $ sudo apt-get install libxxf86vm-dev

    Then:

    $ cd ~/Libs/PhysX/physx
    $ chmod +x generate_projects.sh
    $ ./generate_projects.sh
    Choose number 2 (clang compiler)
    $ cd ~/Libs/PhysX/physx/compiler/linux-release
    $ make -j4
    $ sudo make -j4 install

If During compilation error "PHYSX_ROOT_DIR" occur, just move PhsX from ~/Libs to ~/ .When compilation finish, put it again into ~/Libs

### 2. Go to ~/Sources/walkers/scripts and open clonde_deps.sh:
There is two ways of install walkers and libraries:
- Just use ./clone_dephs.sh in terminal. It is the fastest way, but there is also risk that script won't install some dependencies.
  
  > [HINT] Give execute permission to ./clone_dephs.sh and then run the script:
      $ chmod +x ./clone_dephs.sh

- Install all libraries just by inserting commands from script to Terminal

### 3. Sample demo:
If you want you can run sample demo in path `build/bin/demoVisualizer`

**Contributors**:
- Adam Matecki (PhysX simulator)
- Adam Kurzawa (PhysX simulator)
- Daniel Staśczak (Galgo robot)
- Marcin Orczyk (Galgo robot)
- Marian Wojtkowiak (ANYmal robot)
- Przemyslaw Muszynski (ANYmal robot)
