# Visual Servoing

Includes a proof-of-concept image-based visual servoing technique for use in a contact inspection application.

## 1 Pre-requisites
1. Ubuntu 64-bit 16.04 or 18.04
2. ViSP Packages
    sudo apt-get install libvisp-dev

## 2 Build 
    git clone https://github.com/joshliu11/contact_inspection.git
    cd visual_servo
    cmake -DCMAKE_BUILD_TYPE=Release
    make visual_servo

## 3 Run 
    ./visual_servo

## 4 Todo
1. Integrate with tracker to obtain dataframes that contain the rectangle bounding box points 