#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxLibRealSense2.hpp"

class ofApp : public ofBaseApp{

public:
    void setup() throw(std::runtime_error);
    void update();
    void draw();
    void exit();
    void keyPressed(int key);

    std::shared_ptr<ofxLibRealSense2> _realsense2 = std::make_shared<ofxLibRealSense2>(0);
    ofxPanel         _gui;
    ofEasyCam        _cam;
};
