
#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxCvBlobTracker.h"
#include "ofxCvTrackedBlob.h"
#include "ofxCvTrackingConstants.h"
#include "ofxTuioServer.h"
#include "ofxXmlSettings.h"



typedef struct {
    
	float 	x;
	float 	y;
	bool 	bBeingDragged;
	bool 	bOver;
	float 	radius;
	
}	draggableVertex;


class testApp : public ofSimpleApp, public ofxCvBlobListener 
{

	public:

		void setup();
		void update();
		void draw();
        void exit();
		
        void keyPressed(int key);
        void mouseMoved(int x, int y );
        void mouseDragged(int x, int y, int button);
        void mousePressed(int x, int y, int button);
        void mouseReleased(int x, int y, int button);        
        
        void blobOn( float x, float y, float z, int id, int order );
        void blobMoved( float x, float y, float z, int id, int order );
        void blobOff( float x, float y, float z, int id, int order );

		ofxKinect kinect;
        ofxCvBlobTracker  blobTracker;
    
        draggableVertex activeAreaVertices[2];
        ofRectangle activeArea;

        ofxTuioServer server;
        TuioCursor* cursor;

		ofxXmlSettings XML;
    
};

#endif
