## Project Setup

First things first, there is nothing special you need to develop for INDI. You can use any
IDE, or none at all. You only need basic standard tools for C++ development.

### Dependencies

This will install the minimum dependencies for driver development. You may need
additional dev libraries depending on your own driver's requirements.

```sh
sudo apt install build-essential devscripts debhelper fakeroot cdbs software-properties-common cmake
sudo add-apt-repository ppa:mutlaqja/ppa
sudo apt install libindi-dev libnova-dev libz-dev libgsl-dev
```

## Compiling

First create a `build` folder in your project's folder, move into it and configure the project with cmake.

```bash
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Debug ../
```

Now we can build our executable.

```bash
make
```

And install it.

```bash
sudo make install
```

## Run

With INDI SERVER installed at your machine, run the command.

```bash
indiserver -v indi_esp32
```


