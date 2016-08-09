/************************************************************************************/
/* Use OpenCV 2.2 SURF algorithm(set g_algo = DENSE, functions in cvFuncs.cpp) or	*/
/*	StereoSGBM algorithm(set g_algo = FEATURE_PT, funcs in cvFuncs2.cpp) to recover	*/
/*	stereo 2D images to 3D coordinates, then use OpenGL to reconstruct 3D model.	*/
/* The 2 stereo images should have parallel principal axis.							*/
/* I cannot guarantee the reconstruct effect, especially when the images are shot	*/
/*	with inexact parallel principal axis.											*/
/* The 3D reconstruct algorithm still needs improvements, for there are so many		*/
/*	glitches.																		*/
/* Yan Ke, THUEE, xjed09@gmail.com, 201106											*/
/************************************************************************************/
#include "stdlib.h"  
#include <direct.h>  
#include <string.h>  


#include "header.h"


namespace reconstruction
{



	enum Algorithm {FEATURE_PT, DENSE};

	//#define PARAM	// algorithm parameters that can be modified
	//PARAM
	Algorithm g_algo = DENSE; // 2 algorithms to select corresponding points and reconstruct 3D scene

};


using namespace reconstruction;

int main(int argc, char* argv[])
{
	/************************************************************************/
	/* load and resize images                                               */
	/************************************************************************/
	string folder = "D:\\Download\\code_all\\code_all\\stereoimage-7\\",
		groupname = "Cloth", // Cloth,Midd,Baby,Bowling,Bed
		//filenameL = "\\view1.png", filenameR = "\\view5.png"; // L: the camera on the left
		filenameL = "\\view1s.jpg", filenameR = "\\view5s.jpg"; // L: the camera on the left

	char* buffer;  

	// Get the current working directory:   
	if( (buffer = _getcwd( NULL, 0 )) == NULL )  
		perror( "_getcwd error" );  
	else  
	{  
		printf( "%s \nLength: %d\n", buffer, strnlen(buffer,1024) );  
		free(buffer);  
	} 

	cout<<folder + groupname + filenameL<<endl;

	string str = "view1s.jpg";
	
	FILE* fp;

	fp = fopen(str.c_str(), "rb");//这块调试，似乎没东西。。。
	if (!fp)
	{
		return NULL;
	}

	Mat imgL = imread("view1s.jpg"); 
	Mat	imgR = imread("view5s.jpg");
	imshow("l",imgL);
	waitKey(0);

	if (!(imgL.data) || !(imgR.data))
	{

		cerr<<"can't load image!"<<endl;
		exit(1);
	}

	float stdWidth = 600, resizeScale = 1;
	if (imgL.cols > stdWidth * 1.2)
	{
		resizeScale = stdWidth / imgL.cols;
		Mat imgL1,imgR1;
		resize(imgL, imgL1, Size(), resizeScale, resizeScale);
		resize(imgR, imgR1, Size(), resizeScale, resizeScale);
		imgL = imgL1.clone();
		imgR = imgR1.clone();
	}

	/************************************************************************/
	/* decide which points in the left image should be chosen               */
	/* and calculate their corresponding points in the right image          */
	/************************************************************************/
	cout<<"calculating feature points..."<<endl;
	vector<Point2f> ptsL, ptsR;
	vector<int> ptNum;
	if (g_algo == FEATURE_PT)
	{
		//if ( ! LoadPtsPairs(ptsL, ptsR, groupname+".pairs"))	{
		GetPair(imgL, imgR, ptsL, ptsR);
		//SavePtsPairs(ptsL, ptsR, groupname+".pairs");	}
	}
	else if (g_algo == DENSE)
		GetPairBM(imgL, imgR, ptsL, ptsR);

	/************************************************************************/
	/* calculate 3D coordinates                                             */
	/************************************************************************/
	vector<Point3f> pts3D;
	float focalLenInPixel = 3740 * resizeScale,
		baselineInMM = 160;
	Point3f center3D;
	Vec3f size3D;
	float scale = .2; // scale the z coordinate so that it won't be too large spreaded
	//float imgHinMM = 400, // approximate real height of the scene in picture, useless
	//float MMperPixel = imgHinMM / imgL.rows;
	//float focalLenInMM = focalLenInPixel * MMperPixel;
	focalLenInPixel *= scale;

	cout<<"calculating 3D coordinates..."<<endl;
	StereoTo3D(ptsL, ptsR, pts3D, 
		focalLenInPixel, baselineInMM, 
		imgL, center3D, size3D);

	/************************************************************************/
	/* Delaunay triangulation                                               */
	/************************************************************************/
	cout<<"doing triangulation..."<<endl;
	size_t pairNum = ptsL.size();
	vector<Vec3i> tri;
	TriSubDiv(ptsL, imgL, tri);

	/************************************************************************/
	/* Draw 3D scene using OpenGL                                           */
	/************************************************************************/
	glutInit(&argc, argv); // must be called first in a glut program
	InitGl(); // must be called first in a glut program

	cout<<"creating 3D texture..."<<endl;
	GLuint tex = Create3DTexture(imgL, tri, ptsL, pts3D, center3D, size3D);
	Show(tex, center3D, size3D);


	return 0;
}


// for FEATURE_PT algorithm, save the corresponding points' coordinates
void SavePtsPairs( vector<Point2f> &ptsL, vector<Point2f> &ptsR, string &filename ) 
{
	ofstream os(filename.c_str());
	vector<Point2f>::iterator iterL = ptsL.begin(),
		iterR = ptsR.begin();
	os<<ptsL.size()<<endl;

	for ( ; iterL != ptsL.end(); iterL++, iterR++)
	{
		os<<iterL->x<<'\t'<<iterL->y<<"\t\t"
			<<iterR->x<<'\t'<<iterR->y<<endl;
	}
	os.close();
}

bool LoadPtsPairs( vector<Point2f> &ptsL, vector<Point2f> &ptsR, string &filename ) 
{
	ifstream is(filename.c_str());
	if (!is)
	{
		cerr<<filename<<" unable to read"<<endl;
		return false;
	}

	Point2f buf;
	int cnt;
	is>>cnt;
	for (int i = 0; i < cnt; i++)
	{
		is>>buf.x>>buf.y;
		ptsL.push_back(buf);
		is>>buf.x>>buf.y;
		ptsR.push_back(buf);
	}
	is.close();
	return true;

}
