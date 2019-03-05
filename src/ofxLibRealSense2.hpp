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
#include "ofVboMesh.h"
#ifdef USE_OFXGUI
#include "ofxGui.h"
#endif

class ofxLibRealSense2 : public ofThread
{
public:
    static int getDeviceCount();

    void setupDevice(int deviceID) throw(std::runtime_error);
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

    void hwReset(rs2_error ** error);

    /** Class constructor */
    ofxLibRealSense2();

    /* gui params */
    ofParameterGroup   params;
    ofParameter<bool>  autoExposure;
    ofParameter<bool>  enableEmitter;
    ofParameter<int>   irExposure;
    ofParameter<float> depthMin;
    ofParameter<float> depthMax;

private:
    rs2::device     _device;
    int             _curDeviceID;

    rs2::config     _config;
    rs2::pipeline   _pipeline;
    rs2::colorizer  _colorizer;
    rs2::points     _points;
    rs2::pointcloud _pointcloud;
    bool            _setupFinished;
    bool            _colorEnabled;
    bool            _irEnabled;
    bool            _depthEnabled;
    bool            _pointcloudEnabled;
    bool            _pipelineStarted;
    bool            _useThread;
    int             _colorWidth, _irWidth, _depthWidth;
    int             _colorHeight, _irHeight, _depthHeight;

    uint8_t         *_colBuff;
    uint8_t         *_irBuff;
    uint8_t         *_depthBuff;
    uint16_t        *_rawDepthBuff;
    ofVboMesh       _mesh;
    ofTexture       _colTex, _irTex, _depthTex, _rawDepthTex;
    bool            _hasNewColor;
    bool            _hasNewIr;
    bool            _hasNewDepth;
    bool            _hasNewFrame;

    ofEventListeners _paramListeners;

    void setupParams(const std::string & serialNumber);

    void threadedFunction();
    void updateFrameData();
};
