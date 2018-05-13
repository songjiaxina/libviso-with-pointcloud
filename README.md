# libviso-with-pointcloud
This is a project modified on the basis libviso2 to show the semi-dense pointcloud
If you want to konow  more about libviso2, please cite (http://www.cvlibs.net/software/libviso/)

Prerequisites needed for compiling and running the raw libviso2 :

- libpng (available at: http://www.libpng.org/pub/png/libpng.html)
- libpng++ (available at: http://www.nongnu.org/pngpp/)

libpng and png++ are needed for reading the png input images. On a ubuntu
box you can get them via apt:
```
- sudo apt-get install libpng12-dev
- sudo apt-get install libpng++-dev
```

Besides,to build this project, you need the followings:

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. 
Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV] to simply to read and displat image.

## Eigen3
Download and install instructions can be found at: http://eigen.tuxfamily.org. 


Build the project:
```
cmake .
make 
```


Finally Run the project:

```
./viso2  path_to_KITTI_dataset
```

# 4. Result
**Pangolin GUI:** .  
 
![](https://github.com/SongJiaxinHIT/libviso-with-pointcloud/raw/master/image/result.png)  
