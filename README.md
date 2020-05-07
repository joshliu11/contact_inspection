# contact_inspection

## 1 Pre-requisites
  Ubuntu 64-bit 16.04 or 18.04
  ViSP Packages: sudo apt-get install libvisp-dev
  
## 2 Build 
    git clone https://github.com/joshliu11/contact_inspection.git
    cd ibvs
    cmake -DCMAKE_BUILD_TYPE=Release
    make tutorial-ibvs-4pts
## 3 Run 
    ./tutorial-ibvs-4pts 