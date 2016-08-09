#include "header.h"

// sift is 50 times slower but get 7 times more matched points
// FAST detect more points than SURF
// STAR/MSER generate very few keypoints,
#define DETECTOR_TYPE	"FAST" // FAST,SIFT,SURF,STAR,MSER,GFTT,HARRIS...see the create function
#define DESCRIPTOR_TYPE	"SIFT" // SURF,SIFT,BRIEF,...BRIEF seemed to has bug
#define MATCHER_TYPE	"FlannBased" // BruteForce,FlannBased,BruteForce-L1,...

#define MAXM_FILTER_TH	.8	// threshold used in GetPair
#define HOMO_FILTER_TH	60	// threshold used in GetPair
#define NEAR_FILTER_TH	40	// diff points should have distance more than NEAR_FILTER_TH


// choose the corresponding points in the stereo images for 3d reconstruction
void GetPair( Mat &imgL, Mat &imgR, vector<Point2f> &ptsL, vector<Point2f> &ptsR ) 
{
	Mat descriptorsL, descriptorsR;
	double tt = (double)getTickCount();

	Ptr<FeatureDetector> detector = FeatureDetector::create( DETECTOR_TYPE ); // factory mode
	vector<KeyPoint> keypointsL, keypointsR; 
	detector->detect( imgL, keypointsL );
	detector->detect( imgR, keypointsR );

	Ptr<DescriptorExtractor> de = DescriptorExtractor::create(DESCRIPTOR_TYPE);
	//SurfDescriptorExtractor de(4,2,true);
	de->compute( imgL, keypointsL, descriptorsL );
	de->compute( imgR, keypointsR, descriptorsR );

	tt = ((double)getTickCount() - tt)/getTickFrequency(); // 620*555 pic, about 2s for SURF, 120s for SIFT

	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create( MATCHER_TYPE );
	vector<vector<DMatch>> matches;
	matcher->knnMatch( descriptorsL, descriptorsR, matches, 2 ); // L:query, R:train

	vector<DMatch> passedMatches; // save for drawing
	DMatch m1, m2;
	vector<Point2f> ptsRtemp, ptsLtemp;
	for( size_t i = 0; i < matches.size(); i++ )
	{
		m1 = matches[i][0];
		m2 = matches[i][1];
		if (m1.distance < MAXM_FILTER_TH * m2.distance)
		{
			ptsRtemp.push_back(keypointsR[m1.trainIdx].pt);
			ptsLtemp.push_back(keypointsL[i].pt);
			passedMatches.push_back(m1);
		}
	}

	Mat HLR;
	HLR = findHomography( Mat(ptsLtemp), Mat(ptsRtemp), CV_RANSAC, 3 );
	cout<<"Homography:"<<endl<<HLR<<endl;
	Mat ptsLt; 
	perspectiveTransform(Mat(ptsLtemp), ptsLt, HLR);

	vector<char> matchesMask( passedMatches.size(), 0 );
	int cnt = 0;
	for( size_t i1 = 0; i1 < ptsLtemp.size(); i1++ )
	{
		Point2f prjPtR = ptsLt.at<Point2f>((int)i1,0); // prjx = ptsLt.at<float>((int)i1,0), prjy = ptsLt.at<float>((int)i1,1);
		 // inlier
		if( abs(ptsRtemp[i1].x - prjPtR.x) < HOMO_FILTER_TH &&
			abs(ptsRtemp[i1].y - prjPtR.y) < 2) // restriction on y is more strict
		{
			vector<Point2f>::iterator iter = ptsL.begin();
			for (;iter!=ptsL.end();iter++)
			{
				Point2f diff = *iter - ptsLtemp[i1];
				float dist = abs(diff.x)+abs(diff.y);
				if (dist < NEAR_FILTER_TH) break;
			}
			if (iter != ptsL.end()) continue;

			ptsL.push_back(ptsLtemp[i1]);
			ptsR.push_back(ptsRtemp[i1]);
			cnt++;
			if (cnt%1 == 0) matchesMask[i1] = 1; // don't want to draw to many matches
		}
	}

	Mat outImg;
	drawMatches(imgL, keypointsL, imgR, keypointsR, passedMatches, outImg, 
		Scalar::all(-1), Scalar::all(-1), matchesMask, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	char title[50];
	sprintf_s(title, 50, "%.3f s, %d matches, %d passed", tt, matches.size(), cnt);
	imshow(title, outImg);
	waitKey();
}


// used for doing delaunay trianglation with opencv function
bool isGoodTri( Vec3i &v, vector<Vec3i> & tri ) 
{
	int a = v[0], b = v[1], c = v[2];
	v[0] = min(a,min(b,c));
	v[2] = max(a,max(b,c));
	v[1] = a+b+c-v[0]-v[2];
	if (v[0] == -1) return false;

	vector<Vec3i>::iterator iter = tri.begin();
	for(;iter!=tri.end();iter++)
	{
		Vec3i &check = *iter;
		if (check[0]==v[0] &&
			check[1]==v[1] &&
			check[2]==v[2])
		{
			break;
		}
	}
	if (iter == tri.end())
	{
		tri.push_back(v);
		return true;
	}
	return false;
}

void TriSubDiv( vector<Point2f> &pts, Mat &img, vector<Vec3i> &tri ) 
{
	CvSubdiv2D* subdiv;//The subdivision itself // 细分 
	CvMemStorage* storage = cvCreateMemStorage(0); ;//Storage for the Delaunay subdivsion //用来存储三角剖分 
	Rect rc = Rect(0,0, img.cols, img.rows); //Our outer bounding box //我们的外接边界盒子 

	subdiv = cvCreateSubdiv2D( CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv),
		sizeof(CvSubdiv2DPoint),
		sizeof(CvQuadEdge2D),
		storage );//为数据申请空间  

	cvInitSubdivDelaunay2D( subdiv, rc );//rect sets the bounds 

	//如果我们的点集不是32位的，在这里我们将其转为CvPoint2D32f，如下两种方法。
	for (size_t i = 0; i < pts.size(); i++)
	{
		CvSubdiv2DPoint *pt = cvSubdivDelaunay2DInsert( subdiv, pts[i] );
		pt->id = i;
	}

	CvSeqReader reader;
	int total = subdiv->edges->total;
	int elem_size = subdiv->edges->elem_size;

	cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );
	Point buf[3];
	const Point *pBuf = buf;
	Vec3i verticesIdx;
	Mat imgShow = img.clone();

	srand( (unsigned)time( NULL ) );   
	for( int i = 0; i < total; i++ ) 
	{   
		CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);   

		if( CV_IS_SET_ELEM( edge )) 
		{
			CvSubdiv2DEdge t = (CvSubdiv2DEdge)edge; 
			int iPointNum = 3;
			Scalar color = CV_RGB(rand()&255,rand()&255,rand()&255);

			//bool isNeg = false;
			int j;
			for(j = 0; j < iPointNum; j++ )
			{
				CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t );
				if( !pt ) break;
				buf[j] = pt->pt;
				//if (pt->id == -1) isNeg = true;
				verticesIdx[j] = pt->id;
				t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );
			}
			if (j != iPointNum) continue;
			if (isGoodTri(verticesIdx, tri))
			{
				//tri.push_back(verticesIdx);
				polylines( imgShow, &pBuf, &iPointNum, 
					1, true, color,
					1, CV_AA, 0);
				//printf("(%d, %d)-(%d, %d)-(%d, %d)\n", buf[0].x, buf[0].y, buf[1].x, buf[1].y, buf[2].x, buf[2].y);
				//printf("%d\t%d\t%d\n", verticesIdx[0], verticesIdx[1], verticesIdx[2]);
				//imshow("Delaunay", imgShow);
				//waitKey();
			}

			t = (CvSubdiv2DEdge)edge+2;

			for(j = 0; j < iPointNum; j++ )
			{
				CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t );
				if( !pt ) break;
				buf[j] = pt->pt;
				verticesIdx[j] = pt->id;
				t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );
			}   
			if (j != iPointNum) continue;
			if (isGoodTri(verticesIdx, tri))
			{
				//tri.push_back(verticesIdx);
				polylines( imgShow, &pBuf, &iPointNum, 
					1, true, color,
					1, CV_AA, 0);
				//printf("(%d, %d)-(%d, %d)-(%d, %d)\n", buf[0].x, buf[0].y, buf[1].x, buf[1].y, buf[2].x, buf[2].y);
				//printf("%d\t%d\t%d\n", verticesIdx[0], verticesIdx[1], verticesIdx[2]);
				//imshow("Delaunay", imgShow);
				//waitKey();
			}
		}

		CV_NEXT_SEQ_ELEM( elem_size, reader );

	}

	//RemoveDuplicate(tri);
	char title[100];
	sprintf_s(title, 100, "Delaunay: %d Triangles", tri.size());
	imshow(title, imgShow);
	waitKey();
}


// calculate 3d coordinates.
// for rectified stereos: pointLeft.y == pointRight.y
// the origin for both image is the top-left corner of the left image.
// the x-axis points to the right and the y-axis points downward on the image.
// the origin for the 3d real world is the optical center of the left camera
// object -> optical center -> image, the z value decreases.

void StereoTo3D( vector<Point2f> ptsL, vector<Point2f> ptsR, vector<Point3f> &pts3D,
				float focalLenInPixel, float baselineInMM, Mat img,
				Point3f &center3D, Vec3f &size3D) // output variable, the center coordinate and the size of the object described by pts3D
{
	vector<Point2f>::iterator iterL = ptsL.begin(),
		iterR = ptsR.begin();

	float xl, xr, ylr;
	float imgH = float(img.rows), imgW = float(img.cols);
	Point3f pt3D;
	float minX = 1e9, maxX = -1e9;
	float minY = 1e9, maxY = -1e9;
	float minZ = 1e9, maxZ = -1e9;

	Mat imgShow = img.clone();
	char str[100];
	int ptCnt = ptsL.size(), showPtNum = 30, cnt = 0;
	int showIntv = max(ptCnt/showPtNum, 1);
	for ( ; iterL != ptsL.end(); iterL++, iterR++)
	{
		xl = iterL->x;
		xr = iterR->x; // need not add baseline
		ylr = (iterL->y + iterR->y)/2;

		//if (yl-yr>5 || yr-yl>5) // may be wrong correspondence, discard. But vector can't be changed during iteration
		//{}

		pt3D.z = -focalLenInPixel * baselineInMM / (xl-xr); // xl should be larger than xr, if xl is shot by the left camera
		pt3D.y = -(-ylr + imgH/2) * pt3D.z / focalLenInPixel;
		pt3D.x = (imgW/2 - xl) * pt3D.z / focalLenInPixel;

		minX = min(minX, pt3D.x); maxX = max(maxX, pt3D.x);
		minY = min(minY, pt3D.y); maxY = max(maxY, pt3D.y);
		minZ = min(minZ, pt3D.z); maxZ = max(maxZ, pt3D.z);
		pts3D.push_back(pt3D);

		if ((cnt++)%showIntv == 0)
		{
			Scalar color = CV_RGB(rand()&64,rand()&64,rand()&64);
			sprintf_s(str, 100, "%.0f,%.0f,%.0f", pt3D.x, pt3D.y, pt3D.z);
			putText(imgShow, str, Point(xl-13,ylr-3), FONT_HERSHEY_SIMPLEX, .3, color);
			circle(imgShow, *iterL, 2, color, 3);
		}
		
	}

	imshow("back project", imgShow);
	waitKey();

	center3D.x = (minX+maxX)/2;
	center3D.y = (minY+maxY)/2;
	center3D.z = (minZ+maxZ)/2;
	size3D[0] = maxX-minX;
	size3D[1] = maxY-minY;
	size3D[2] = maxZ-minZ;
}
