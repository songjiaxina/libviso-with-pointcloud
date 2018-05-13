/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdint.h>

#include <png++/png.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <timer.h>

#include <viso_stereo.h>
#include <eigen3/Eigen/Eigen>
#include <pangolin/pangolin.h>

using namespace std;
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,vector<string> &vstrImageColor,
                vector<double> &vTimestamps);

inline pangolin::OpenGlMatrix Matrix2OpenglMatrix(Matrix mat)
{

  pangolin::OpenGlMatrix OpenglMat;


  OpenglMat.m[0]=mat.val[0][0];
  OpenglMat.m[1]=mat.val[1][0];
  OpenglMat.m[2]=mat.val[2][0];
  OpenglMat.m[3]=mat.val[3][0];
  OpenglMat.m[4]=mat.val[0][1];
  OpenglMat.m[5]=mat.val[1][1];
  OpenglMat.m[6]=mat.val[2][1];
  OpenglMat.m[7]=mat.val[3][1];
  OpenglMat.m[8]=mat.val[0][2];
  OpenglMat.m[9]=mat.val[1][2];
  OpenglMat.m[10]=mat.val[2][2];
  OpenglMat.m[11]=mat.val[3][2];
  OpenglMat.m[12]=mat.val[0][3];
  OpenglMat.m[13]=mat.val[1][3];
  OpenglMat.m[14]=mat.val[2][3];
  OpenglMat.m[15]=mat.val[3][3];

  return OpenglMat;

}

inline Eigen::Vector3f ChangeCoordinate(Eigen::Vector3f& pt,Matrix& Twc)
{

    Eigen::Matrix4f T;
    T<<Twc.val[0][0],Twc.val[0][1],Twc.val[0][2],Twc.val[0][3],
       Twc.val[1][0],Twc.val[1][1],Twc.val[1][2],Twc.val[1][3],
       Twc.val[2][0],Twc.val[2][1],Twc.val[2][2],Twc.val[2][3],
       Twc.val[3][0],Twc.val[3][1],Twc.val[3][2],Twc.val[3][3];

    Eigen::Vector4f temp(pt(0,0),pt(1,0),pt(2,0),1.0);

    Eigen::Vector4f temp2=T*temp;
    Eigen::Vector3f result;
    result<<temp2(0,0),temp2(1,0),temp2(2,0);
    return result;

}


int main (int argc, char** argv)
{

  //const string prefix="/home/dl/dataset/KITTI/00"
  if(argc!=2)
  {
      std::cout<<"input the right file path"<<std::endl;
      return -1;
  }
  const string prefix=argv[1];

  // set most important visual odometry parameters
  // for a full parameter list, look at: viso_stereo.h
  VisualOdometryStereo::parameters param;

  /*
  ////kitti03
  param.calib.f  = 721.5377; // focal length in pixels
  param.calib.cu = 609.5593; // principal point (u-coordinate) in pixels
  param.calib.cv = 172.854; // principal point (v-coordinate) in pixels
  param.base     = 0.5371506; // baseline in meters
   */

    ////kitti 00
  param.calib.f  = 718.85; // focal length in pixels
  param.calib.cu = 607.19; // principal point (u-coordinate) in pixels
  param.calib.cv = 185.21; // principal point (v-coordinate) in pixels
  param.base     = 0.5371506; // baseline in meters
  
  // init visual odometry
  VisualOdometryStereo viso(param);
  
  // current pose (this matrix transforms a point from the current
  // frame's camera coordinates to the first frame's camera coordinates)
  Matrix pose = Matrix::eye(4);

  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<string> vstrImageColor;
  vector<double> vTimestamps;

  LoadImages(prefix, vstrImageLeft, vstrImageRight,vstrImageColor, vTimestamps);

  const int nImages = vstrImageLeft.size();

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;


  int w=640;
  int h=480;
  int UI_WIDTH=175;

  pangolin::CreateWindowAndBind("libviso2 with pointclouds",1024,768);
  ////created the pangolin gui
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(w,h,400,400,w/2,h/2,0.1,1000),
            ////here to change the view direction

            ////in this view we follow the camera
            pangolin::ModelViewLookAt(-0,-3,-30, 0,0,0, pangolin::AxisNegY)
            ////俯视
            ////pangolin::ModelViewLookAt(0,-100,-0.1, 0,0,0,0.0,-1.0, 0.0)
    );

  pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -w/float(h))
            .SetHandler(new pangolin::Handler3D(s_cam));




  pangolin::View& left_image = pangolin::Display("imgKFDepth")
            .SetAspect(w/(float)h);


  pangolin::GlTexture imageTexture(w,h,GL_RGB,false,0,
                                     GL_RGB,GL_UNSIGNED_BYTE);


  pangolin::CreateDisplay()
            .SetBounds(0.0, 0.3, pangolin::Attach::Pix(UI_WIDTH), 1.0)
            .SetLayout(pangolin::LayoutEqual)
            .AddDisplay(left_image);



 pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(UI_WIDTH));

 pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
 pangolin::Var<bool> menuShowImage("menu.Show Image",true,true);
 pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
 pangolin::Var<bool> menuShowmpangolinPoints("menu.Show Points",true,true);





  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  vector<Matrix>mvPoses;
  vector<Eigen::Vector3f>totalMapPoints;


  // loop through all frames i=0:372
  for (int32_t i=0; i<nImages; i++)
  {

    // input file names
    char base_name[256]; sprintf(base_name,"%06d.png",i);
    string left_img_file_name  = vstrImageLeft[i];
    string right_img_file_name = vstrImageRight[i];
    string color_img_file_name = vstrImageColor[i];
    //cv::Mat image=cv::imread(left_img_file_name,0);
    //cv::imshow("zuotu",image);
    //cv::waitKey(5);


    // catch image read/write errors here
    try {

      // load left and right input image
      png::image< png::gray_pixel > left_img(left_img_file_name);
      png::image< png::gray_pixel > right_img(right_img_file_name);

      // image dimensions
      int32_t width  = left_img.get_width();
      int32_t height = left_img.get_height();

      // convert input images to uint8_t buffer
      uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
      uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
      int32_t k=0;
      for (int32_t v=0; v<height; v++)
      {
        for (int32_t u=0; u<width; u++)
        {
          left_img_data[k]  = left_img.get_pixel(u,v);
          right_img_data[k] = right_img.get_pixel(u,v);
          k++;
        }
      }

      // status
      //cout << "Processing: Frame: " << i;

      // compute visual odometry
      int32_t dims[] = {width,height,width};
      if (viso.process(left_img_data,right_img_data,dims))
      {

        // on success, update current pose
        pose = pose * Matrix::inv(viso.getMotion());

        // output some statistics
        double num_matches = viso.getNumberOfMatches();
        double num_inliers = viso.getNumberOfInliers();
        //cout << ", Matches: " << num_matches;
        // cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
        //cout << pose << endl << endl;
        mvPoses.push_back(pose);

      }
      else
        {
        cout << " ... failed!" << endl;
      }


        //while( !pangolin::ShouldQuit() )
        //{

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            //清除颜色缓存
            d_cam.Activate(s_cam);


            for(int k=0;k<mvPoses.size();k++)
            {
                if (menuFollowCamera)
                {
                    s_cam.Follow(Matrix2OpenglMatrix(mvPoses[k]));
                }


                const float &w = 1.0;
                const float h = w * 0.55;
                const float z = w * 0.2;


                glPushMatrix();

                ////注意此时为列优先

                glMultMatrixd(Matrix2OpenglMatrix(mvPoses[k]).m);

                //设置绘制图形时线的宽度
                glLineWidth(0.2);
                //设置当前颜色为绿色(相机图标显示为绿色)
                glColor3f(1.f, 0.f, 0.f);
                //用线将下面的顶点两两相连
                glBegin(GL_LINES);
                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();

            }


        if(menuShowGraph&&mvPoses.size()>2)
        {
            glLineWidth(5.0);
            glColor3f(1.0f,0.4f,0.7f);
            glBegin(GL_LINES);
            for(int j=1;j<mvPoses.size();j++)
            {
                glVertex3f(float(Matrix2OpenglMatrix(mvPoses[j]).m[12]),
                           float(Matrix2OpenglMatrix(mvPoses[j]).m[13]),
                           float(Matrix2OpenglMatrix(mvPoses[j]).m[14]));
                glVertex3f(float(Matrix2OpenglMatrix(mvPoses[j-1]).m[12]),
                           float(Matrix2OpenglMatrix(mvPoses[j-1]).m[13]),
                           float(Matrix2OpenglMatrix(mvPoses[j-1]).m[14]));


            }
            glEnd();
        }

        /////之前把点的显示放在图片东风恶显示后面,点就画不出来了,很奇怪

        vector<Eigen::Vector3f>temppoints;

        viso.GetPointsMap(temppoints,false);



        for(int ips=0;ips<temppoints.size();ips++)
        {
            Eigen::Vector3f pt_world;
            pt_world=ChangeCoordinate(temppoints[ips],pose);
            totalMapPoints.push_back(pt_world);

        }



        if(menuShowmpangolinPoints)
        {

            glPointSize(0.5);
            glBegin(GL_POINTS);
            glColor3f(0.0f,0.545,1.0f);
            //cout<<"the size is"<<totalMapPoints.size()<<endl;


            for(int n=0;n<totalMapPoints.size();n++)
            {
                //cout<<"do l "<<endl;
                glVertex3f(totalMapPoints[n](0,0),totalMapPoints[n](1,0), totalMapPoints[n](2,0));
            }

            glEnd();


        }

        if(menuShowImage)
        {

            unsigned char* pointer=new unsigned char[3*w*h];
            cv::Mat color=cv::imread(color_img_file_name,CV_LOAD_IMAGE_UNCHANGED) ;
            color.convertTo(color,CV_8UC3);
            assert(color.type() == CV_8UC3);
            //cv::Mat output;
            cv::resize(color,color,cv::Size(w,h));
            memcpy(pointer, color.data, color.rows*color.cols*3);


            imageTexture.Upload(pointer,GL_BGR,GL_UNSIGNED_BYTE);

            left_image.Activate();
            ////三部曲,先activiate
            glColor4f(1.0f,1.0f,1.0f,1.0f);
            imageTexture.RenderToViewportFlipY();

        }
        
            pangolin::FinishFrame();
        //}



      // release uint8_t buffers
      free(left_img_data);
      free(right_img_data);


    // catch image read errors here
    }
    catch (...)
    {
      cerr << "ERROR: Couldn't read input files!" << endl;
      return 1;
    }


  }


  mvPoses.clear();
  totalMapPoints.clear();
  // output
  cout << "Demo complete! Exiting ..." << endl;

  while(1);
  // exit
  return 0;
}


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight,vector<string> &vstrImageColor,
                vector<double> &vTimestamps)
{
  ifstream fTimes;
  string strPathTimeFile = strPathToSequence + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());
  while(!fTimes.eof())
  {
    string s;
    getline(fTimes,s);
    if(!s.empty())
    {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  string strPrefixLeft = strPathToSequence + "/image_0/";
  string strPrefixRight = strPathToSequence + "/image_1/";
  string strPrefixColor= strPathToSequence + "/image_2/";

  const int nTimes = vTimestamps.size();
  vstrImageLeft.resize(nTimes);
  vstrImageRight.resize(nTimes);
  vstrImageColor.resize(nTimes);

  for(int i=0; i<nTimes; i++)
  {
    stringstream ss;
    ss << setfill('0') << setw(6) << i;
    vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
    vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    vstrImageColor[i]= strPrefixColor+ ss.str() + ".png";
  }
}
