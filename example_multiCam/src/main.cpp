#include "ofMain.h"
#include "ofApp.h"

#include <exception>

//========================================================================
int main( ){
	ofSetupOpenGL(1280,720,OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
    try {
	    ofRunApp(new ofApp());
    }
    catch (std::exception & e) {
		std::cerr<<endl<<"Error: "<<e.what()<<std::endl<<std::endl;
    	// ofSystemAlertDialog(e.what());										// Gtk-ERROR **: GTK+ 2.x symbols detected. Using GTK+ 2.x and GTK+ 3 in the same process is not supported
    	ofLogFatalError("RSBLOB")<<e.what();
    }
}
