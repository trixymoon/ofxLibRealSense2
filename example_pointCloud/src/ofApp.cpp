#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
throw(std::runtime_error){

    ofSetVerticalSync(false);
    _realsense2->setupDepth(480, 270, 15);
    _realsense2->enablePointcloud(true);
    _realsense2->startPipeline(true);

    _gui.setup("appSettings.xml");
    _gui.add(_realsense2->params);
}

//--------------------------------------------------------------
void ofApp::update(){
    _realsense2->update();
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(120);

    ofSetColor(255);
    if(_realsense2->depthEnabled()) {
        _realsense2->getDepthTex()->draw(0, 0);
    }


    _cam.begin();
    float s = 200;
    ofScale(s,-s,-s);
    ofDrawAxis(1);

    ofPushMatrix();
    ofTranslate(0, 1, 0);
    ofRotateZDeg(90);
    ofSetColor(0, 200);
    ofDrawGridPlane(1, 5, true);
    ofPopMatrix();

    if (_realsense2->pointcloudEnabled()) {
        _realsense2->getVboMesh()->drawVertices();
    }

    _cam.end();
    ofDrawBitmapString(ofToString(ofGetFrameRate()), 10, 10);
    _gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
}

void ofApp::exit()
{
}
