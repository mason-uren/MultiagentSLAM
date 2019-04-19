# MultiagentSLAM
The Simultaneous Localization and Mapping of an unknown 2-Dimensional plane, using  noisy sonar range sensors, within a multiagent application. Based upon the CSUCI summer surf 2018 NASA Swarmathon code base.

## Tools:
- Boost
- Eigen3
- Intel MKL

### Installing Eigen: 
#### Linux

Download the latest version of Eigen from the wiki [Eigen3 Download](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download). Then, using terminal, navigate to the directory eigen3. 

Type:
- `mkdir build_dir`
- `cd build_dir`
- `cmake source_dir`  
- `make install`

where `source_dir` is your Eigen3 download. (NOTE: you may have to use `sudo` when installing.)

### Installing MKL: 
#### Linux
First we will need to add the Intel-MKL repo to our `source.list`. 
- `wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB`
- `sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-XXXX.PUB` where `XXXX` is the desired year (ie 2019).
- `sudo sh -c 'echo deb https://apt.repos.intel.com/mkl all main > /etc/apt/sources.list.d/intel-mkl.list'`
- `sudo apt-get update`

Next install the desired distribution:
- `sudo apt-get install intel-mkl-64bit-2019.3-062 ` 

You should now be able to see the libraries in `/opt/intel/...`, but we still need to setup a few key environmental variables.
1. INCLUDE 
2. MKLROOT
3. LD_LIBRARY_PATH
4. MANPATH
5. LIBRARY_PATH
6. CPATH
7. NLSPATH
8. PKG_CONFIG_PATH

We can achieve this my running `mklvars.sh <arch>`, located from `<MKL_ROOT>/mkl/bin/mklvars.sh intel64`, where `<arch>` defines the OS architecture (ie `ia32` or `intel64`).

Then source the script:
- `source /opt/intel/mkl/bin/mklvars.sh intel64`


Dowload the latest version of Intel MKL from [MKL Download](https://software.intel.com/en-us/mkl/choose-download/linux).

Installing:
- `cd` into your local `MKL` download folder.
- `./install.sh` (follow the prompts).

(NOTE: further information at [MKL Download Info](http://sep.stanford.edu/sep/claudio/Research/Prst_ExpRefl/ShtPSPI/intel/mkl/10.0.3.020/doc/Install.txt)).


