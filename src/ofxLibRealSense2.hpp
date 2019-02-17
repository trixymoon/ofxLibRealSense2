//
//  ofxLibRealSense2.hpp
//  example
//
//  Created by shiyamon on 2018/05/25.
//

#pragma once

#include "librealsense2/rs.hpp"
#include "ofParameter.h"
#include "ofThread.h"
#include "ofTexture.h"
#include "ofxGui.h"

class ofxLibRealSense2 : public ofThread
{
public:
    static int getDeviceCount();
    
public:
    void setupDevice(int deviceID);
    void setupColor(int width, int height, int fps=60);
    void setupIR(int width, int height, int fps=60);
    void setupDepth(int width, int height, int fps=60);
    void startPipeline(bool useThread);
    void enablePointcloud(bool enabled);
    void update();
    void exit();
    
    ofTexture*  getColorTex()    { return &_colTex; }
    ofTexture*  getIrTex()       { return &_irTex; }
    ofTexture*  getDepthTex()    { return &_depthTex; }
    ofTexture*  getRawDepthTex() { return &_rawDepthTex; }
    ofVboMesh*  getVboMesh()     { return &_mesh; }
    
    int getColorWidth()          { return _colorWidth; }
    int getColorHeight()         { return _colorHeight; }
    int getIrWidth()             { return _irWidth; }
    int getIrHeight()            { return _irHeight; }
    int getDepthWidth()          { return _depthWidth; }
    int getDepthHeight()         { return _depthHeight; }
    bool isFrameNew()            { return _hasNewFrame; }
    bool colorEnabled()          { return _colorEnabled; }
    bool irEnabled()             { return _irEnabled; }
    bool depthEnabled()          { return _depthEnabled; }
    bool pointcloudEnabled()     { return (_pointcloudEnabled && _depthEnabled); }
    
    ofxGuiGroup *getGui();
    
    ofxLibRealSense2() : _setupFinished(false), _colorEnabled(false), _irEnabled(false), _depthEnabled(false), _pointcloudEnabled(false), _pipelineStarted(false), _useThread(false) {}
    
private:
    rs2::device     _device;
    int             _curDeviceID;
    
    rs2::config     _config;
    rs2::pipeline   _pipeline;
    rs2::colorizer  _colorizer;
    rs2::points     _points;
    rs2::pointcloud _pointcloud;
    bool            _useThread;
    bool            _setupFinished;
    bool            _pipelineStarted;
    bool            _colorEnabled, _irEnabled, _depthEnabled, _pointcloudEnabled;
    int             _colorWidth, _irWidth, _depthWidth;
    int             _colorHeight, _irHeight, _depthHeight;
    
    uint8_t         *_colBuff, *_irBuff, *_depthBuff;
    uint16_t        *_rawDepthBuff;
    ofVboMesh       _mesh;
    ofTexture       _colTex, _irTex, _depthTex, _rawDepthTex;
    bool            _hasNewColor, _hasNewIr, _hasNewDepth, _hasNewFrame;
    
    ofxGuiGroup     _D400Params;
    ofParameter<bool> _autoExposure;
    ofParameter<bool> _enableEmitter;
    ofParameter<int> _irExposure;
    ofParameter<float> _depthMin{ "Min Depth", true };
    ofParameter<float> _depthMax{ "Max Depth", true };

    void threadedFunction();
    void updateFrameData();
    void setupGUI(std::string serialNumber);
    void onD400BoolParamChanged(bool &value);
    void onD400IntParamChanged(int &value);
    void onD400ColorizerParamChanged(float &value);
};
