#include "ros/ros.h"
#include "uh_study/FacePoseSrv.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

class fd_main
{
private:
	int level;
	int minsize;
	double scale;
	double x;
	double y;
	int faceHowMany;
	double xLocation[10];
	double yLocation[10];
	int ic;

	CvHaarClassifierCascade* cascade1;
	CvHaarClassifierCascade* cascade2;
	CvHaarClassifierCascade* cascade3;

	CvMemStorage* storage1;
	CvMemStorage* storage2;
	CvMemStorage* storage3;

	IplImage *img;
	IplImage *gray;
	IplImage *scaled_gray;
	IplImage *scaled_gray_Flip;
public:
	fd_main();
	~fd_main();

	int run(IplImage *frame);
};

fd_main::fd_main()
{
	level = 2;
	minsize = 30;
	scale = 2;
	x = 0.25;
	y = 0.05;

	storage1 = cvCreateMemStorage(0);
	storage2 = cvCreateMemStorage(0);
	storage3 = cvCreateMemStorage(0);

	const char* cascade_name1 = "C:\\Programs\\OpenCV2.1\\data\\haarcascades\\haarcascade_frontalface_alt.xml";
	const char* cascade_name2 = "C:\\Programs\\OpenCV2.1\\data\\haarcascades\\haarcascade_profileface.xml";
	const char* cascade_name3 = "C:\\Programs\\OpenCV2.1\\data\\haarcascades\\haarcascade_upperbody.xml";

	cascade1 = (CvHaarClassifierCascade*)cvLoad( cascade_name1, 0, 0, 0 );
	cascade2 = (CvHaarClassifierCascade*)cvLoad( cascade_name2, 0, 0, 0 );
	cascade3 = (CvHaarClassifierCascade*)cvLoad( cascade_name3, 0, 0, 0 );

	if( level>0 && !cascade1 ) printf("\n ERROR: Could not load the classifier 1 ");
	if( level>2 && !cascade2 ) printf("\n ERROR: Could not load the classifier 2 ");
	if( level>1 && !cascade3 ) printf("\n ERROR: Could not load the classifier 3 ");

	cvNamedWindow( "FaceDetection", CV_WINDOW_AUTOSIZE );
	ic = 0;
	img = NULL;
}
fd_main::~fd_main()
{
	cvDestroyWindow( "FaceDetection" );
	cvReleaseImage( &img );
	cvReleaseImage( &gray );
	cvReleaseImage( &scaled_gray );
	cvReleaseImage( &scaled_gray_Flip );
}
int fd_main::run(IplImage *frame)
{

	int f1=0;
	int f2=0;
	int f3=0;
	CvSeq *faces1;
	CvSeq *faces2;
	CvSeq *faces3;
	faceHowMany=0;

	printf("\n FD  %d ", ic++);

	if( !cascade1 ) {printf(" no cascade ");return 0;}
	if( frame == NULL ) {printf(" no image ");return 0;}

	if ( img == NULL )
	{
		img = cvCreateImage( cvSize(frame->width,frame->height), IPL_DEPTH_8U, frame->nChannels );
		gray = cvCreateImage( cvSize(img->width,img->height), 8, 1 );
		scaled_gray = cvCreateImage( cvSize( cvRound (img->width/scale), cvRound (img->height/scale)), 8, 1 );
		scaled_gray_Flip = cvCreateImage( cvSize( cvRound (img->width/scale), cvRound (img->height/scale)), 8, 1 );
	}

	if( frame->origin == IPL_ORIGIN_TL ) cvCopy( frame, img, 0 );
	else  cvFlip( frame, img, 0 );

	cvCvtColor( img, gray, CV_BGR2GRAY );
	cvResize( gray, scaled_gray, CV_INTER_LINEAR );
	cvEqualizeHist( scaled_gray, scaled_gray );

	cvClearMemStorage( storage1 );
	faces1 = cvHaarDetectObjects( scaled_gray, cascade1, storage1, 1.1, 2, 0, cvSize(minsize, minsize) );
	f1 = (faces1 ? faces1->total : 0);
	if ( level > 1 && cascade3 ) //&& f1 < 1)
	{
		cvClearMemStorage( storage3 );
		faces3 = cvHaarDetectObjects( scaled_gray, cascade3, storage3, 1.1, 2, 0, cvSize(minsize, minsize) );
		f3 = (faces3 ? faces3->total : 0);
	}
	if ( level > 2 && cascade2 ) //&& f1 < 1 && f3 < 1)
	{
		cvClearMemStorage( storage2 );
		faces2 = cvHaarDetectObjects( scaled_gray, cascade2, storage2, 1.1, 2, 0, cvSize(minsize, minsize) );
		f2 = (faces2 ? faces2->total : 0);
		if ( f2 == 0 )
		{
			cvFlip( scaled_gray, scaled_gray_Flip, 1);
			faces2 = cvHaarDetectObjects( scaled_gray_Flip, cascade2, storage2, 1.1, 2, 0, cvSize(minsize, minsize) );
			f2 = (faces2 ? faces2->total : 0);
		}
	}

	for(int i = 0; i < f1; i++ )
	{
		CvPoint center;
		CvRect* r = (CvRect*)cvGetSeqElem( faces1, i );
		center.x = cvRound((r->x + r->width*0.5)*scale);
		center.y = cvRound((r->y + r->height*0.5)*scale);
		int radius = cvRound((r->width + r->height)*0.25*scale);
		cvCircle( img, center, radius, cvScalar(0,0,255), 3, 8, 0 );

		xLocation[faceHowMany] = (double)(2*center.x - img->width) / (img->width);
		yLocation[faceHowMany] = (double)(2*center.y - img->height) / (img->height);
		faceHowMany++;
	}

	for(int i = 0; i < f3; i++ )
	{
		CvPoint center;
		CvRect* r = (CvRect*)cvGetSeqElem( faces3, i );
		center.x = cvRound((r->x + r->width*0.5)*scale);
		center.y = cvRound((r->y + r->height*0.5)*scale);
		int radius = cvRound((r->width + r->height)*0.25*scale);
		cvCircle( img, center, radius, cvScalar(255,0,0), 3, 8, 0 );

		xLocation[faceHowMany] = (double)(2*center.x - img->width) / (img->width);
		yLocation[faceHowMany] = (double)(2*center.y - img->height) / (img->height);
		faceHowMany++;
	}

	for(int i = 0; i < f2; i++ )
	{
		CvPoint center;
		CvRect* r = (CvRect*)cvGetSeqElem( faces2, i );
		center.x = cvRound((r->x + r->width*0.5)*scale);
		center.y = cvRound((r->y + r->height*0.5)*scale);
		int radius = cvRound((r->width + r->height)*0.25*scale);
		cvCircle( img, center, radius, cvScalar(0,255,0), 3, 8, 0 );

		xLocation[faceHowMany] = (double)(2*center.x - img->width) / (img->width);
		yLocation[faceHowMany] = (double)(2*center.y - img->height) / (img->height);
		faceHowMany++;
	}

	cvRectangle( img, cvPoint(cvRound(x * img->width), 0), cvPoint(cvRound((1-x) * img->width), img->height), CV_RGB(0,255,0), 1, 8, 0 );
	cvRectangle( img, cvPoint(0, cvRound(y * img->height)), cvPoint(img->width, cvRound((1-y) * img->height)), CV_RGB(0,255,0), 1, 8, 0 );
	cvShowImage( "FaceDetection", img );
}





bool add(uh_study::FacePoseSrv::Request  &req,
         uh_study::FacePoseSrv::Response &res )
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "face_pose_srv_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("face_pose_srv", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}

