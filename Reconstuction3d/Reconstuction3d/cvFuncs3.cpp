#include "header.h"


//This colors the segmentations
void floodFillPostprocess( Mat& img, const Scalar& colorDiff=Scalar::all(1) )
{
	CV_Assert( !img.empty() );
	RNG rng = theRNG();
	Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
	for( int y = 0; y < img.rows; y++ )
	{
		for( int x = 0; x < img.cols; x++ )
		{
			if( mask.at<uchar>(y+1, x+1) == 0 )
			{
				Scalar newVal( rng(256), rng(256), rng(256) );
				floodFill( img, mask, Point(x,y), newVal, 0, colorDiff, colorDiff );
			}
		}
	}
}

void GetPairSegBM( Mat &imgL, Mat &imgR, vector<Point2f> &ptsL, vector<Point2f> &ptsR, vector<int> &ptNum ) 
{
	Mat_<float> disp;
	//imshow("left image", imgL);

	PARAM int numOfDisp = 80; // number of disparity, must be divisible by 16
	CalcDisparity(imgL, imgR, disp, numOfDisp);
	//waitKey();

	Mat dispTemp, disp24;
	normalize(disp, dispTemp, 0, 1, NORM_MINMAX);
	dispTemp.convertTo(dispTemp, CV_8U, 255);
	cvtColor(dispTemp, disp24, CV_GRAY2RGB);

	Mat seg;
	PARAM double spatialRad = 10, colorRad = 20;
	PARAM int maxPyrLevel = 0;
	pyrMeanShiftFiltering( disp24, seg, spatialRad, colorRad, maxPyrLevel );

	//cout<<seg(Range(30,40),Range(180,190))<<endl;
	Mat_<char> seg8;
	cvtColor(seg, seg8, CV_RGB2GRAY);
	floodFillPostprocess(seg);
	imshow("segment", seg);
	waitKey();

}

void TriSubDivSeg( vector<Point2f> &ptsL, vector<int> &ptNum, Mat &imgL, vector<Vec3i> &tri ) 
{
	throw std::exception("The method or operation is not implemented.");
}
