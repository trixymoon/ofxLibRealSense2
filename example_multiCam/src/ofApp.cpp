#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
throw(std::runtime_error){

    ofSetVerticalSync(false);

    _gui.setup("appSettings.xml");

    int deviceCnt = ofxLibRealSense2::getDeviceCount();
    for(int i=0; i<deviceCnt; ++i) {
        std::shared_ptr<ofxLibRealSense2> rs = std::make_shared<ofxLibRealSense2>();
        rs->setupDevice(i);
        rs->setupColor(640, 360, 30);
        rs->startPipeline(true);
        _gui.add(rs->params);

        _rsList.push_back(rs);

    }
}

//--------------------------------------------------------------
void ofApp::update(){
    for (int i=0; i<_rsList.size(); ++i) {
        _rsList[i]->update();
    }
}

//--------------------------------------------------------------
void ofApp::draw(){

    ofBackground(0);
    for (int i=0; i<_rsList.size(); ++i) {
        if(_rsList[i]->colorEnabled())
            _rsList[i]->getColorTex()->draw(640*i, 0);
    }

    ofDrawBitmapString(ofToString(ofGetFrameRate()), 10, 10);

    _gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

void ofApp::exit()
{
    for(int i=0; i<_rsList.size(); ++i) {
        _rsList[i]->exit();
    }
    _rsList.clear();
}
