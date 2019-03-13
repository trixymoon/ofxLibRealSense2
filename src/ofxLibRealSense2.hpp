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

class ofxLibRealSense2 : public ofThread {
private:
    rs2::device      _device;
    rs2::config      _config;
    rs2::pipeline    _pipeline;
    rs2::colorizer   _colorizer;
    rs2::points      _points;
    rs2::pointcloud  _pointcloud;
    bool             _setupFinished;
    bool             _colorEnabled;
    bool             _irEnabled;
    bool             _depthEnabled;
    bool             _pointcloudEnabled;
    bool             _pipelineStarted;
    bool             _useThread;
    int              _colorWidth, _irWidth, _depthWidth;
    int              _colorHeight, _irHeight, _depthHeight;

    uint8_t         *_colBuff;
    uint8_t         *_irBuff;
    uint8_t         *_depthBuff;
    uint16_t        *_rawDepthBuff;
    ofVboMesh        _mesh;
    ofTexture        _colTex;
	ofTexture		 _irTex;
	ofTexture        _depthTex;
	ofTexture		 _rawDepthTex;

	std::shared_ptr<rs2::depth_frame> _depthFrame;

    bool             _hasNewColor;
    bool             _hasNewIr;
    bool             _hasNewDepth;
    bool             _hasNewFrame;

    ofEventListeners _paramListeners;

    void setupParams(const std::string & serialNumber);
    void threadedFunction();
    void updateFrameData();
    void hwReset()               { _device.hardware_reset(); }

public:
    static int getDeviceCount();
    static void deviceInfo();
    static void resetDevice(const std::string serial) throw(std::runtime_error);

    void setupColor(int width, int height, int fps=30);
    void setupIR(int width, int height, int fps=30);
    void setupDepth(int width, int height, int fps=30);
    void startPipeline(bool useThread);
    void enablePointcloud(bool enabled);
    void update();

    ofTexture* getColorTex()     { return &_colTex; }
    ofTexture* getIrTex()        { return &_irTex; }
    ofTexture* getDepthTex()     { return &_depthTex; }
    ofTexture* getRawDepthTex()  { return &_rawDepthTex; }
    ofVboMesh* getVboMesh()      { return &_mesh; }

    int getColorWidth()          { return _colorWidth; }
    int getColorHeight()         { return _colorHeight; }
    int getIrWidth()             { return _irWidth; }
    int getIrHeight()            { return _irHeight; }
    int getDepthWidth()          { return _depthWidth; }
    int getDepthHeight()         { return _depthHeight; }

    float get_distance(int x, int y) const;
    const uint16_t * getDepthRawData() const;

    bool isFrameNew()            { return _hasNewFrame; }
    bool colorEnabled()          { return _colorEnabled; }
    bool irEnabled()             { return _irEnabled; }
    bool depthEnabled()          { return _depthEnabled; }
    bool pointcloudEnabled()     { return (_pointcloudEnabled && _depthEnabled); }

    /** Class constructor */
    ofxLibRealSense2(int deviceID) throw(std::runtime_error);
    ofxLibRealSense2(std::string serial) throw(std::runtime_error);

	/** class destructor */
	~ofxLibRealSense2();

    /* gui params */
    ofParameterGroup   params;
    ofParameter<bool>  autoExposure;
    ofParameter<bool>  enableEmitter;
    ofParameter<int>   irExposure;
    ofParameter<float> depthMin;
    ofParameter<float> depthMax;
};
