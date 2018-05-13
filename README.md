# libviso-with-pointcloud
This is a project modified on the basis of libviso2 to show the semi-dense pointcloud using Pangolin.
If you want to konow  more about libviso2, please cite (http://www.cvlibs.net/software/libviso/)


The whole project is tested in **Ubuntu ** Platorforms

## Prerequisites needed for compiling and running the raw libviso2 :

- libpng (available at: http://www.libpng.org/pub/png/libpng.html)
- libpng++ (available at: http://www.nongnu.org/pngpp/)

libpng and png++ are needed for reading the png input images. On a ubuntu
box you can get them via apt:
```
- sudo apt-get install libpng12-dev
- sudo apt-get install libpng++-dev
```

## Additional Prerequisites for this project
Besides,to build this project, you need the followings:

#### Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and interface. 
Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

#### OpenCV
We use [OpenCV] to simply read and display image.

#### Eigen3
Download and install instructions can be found at: http://eigen.tuxfamily.org. 

#### C++11 or C++0x Compiler
We use the some functionalities of C++11.


## Build
Finaly ``` cd ```to the project folder ,and 

```
cmake .
make 
```

## Run
This project is mainly for KITTI ODemotry dataset.You need to download a sequence. The dataset is available through http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets  
To Run the project, you need to modify the camera instrinsic parameters in line 119~122 in the `demo.cpp`  :   

*param.calib.f  = fx or fy   
param.calib.cu = cx   
param.calib.cv = cy  
param.base     =  baseline in meters*


And

```
./viso2  path_to_KITTI_dataset
```
where "the path_to_KITTI_dataset" is dataset sequence root directory which contains "image_0" , "image_1" and so on



## Result
**Pangolin GUI:** .  
 
![](https://github.com/SongJiaxinHIT/libviso-with-pointcloud/raw/master/image/result.png)  
