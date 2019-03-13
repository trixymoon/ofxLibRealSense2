//
//  ofxLibRealSense2.cpp
//  example
//
//  Created by shiyamon on 2018/05/25.
//

#include "ofxLibRealSense2.hpp"
#include "ofSystemUtils.h"
#include "ofLog.h"

using namespace::std;

ofxLibRealSense2::ofxLibRealSense2(int deviceID)
throw(std::runtime_error) :
    _setupFinished(false),
    _colorEnabled(false),
    _irEnabled(false),
    _depthEnabled(false),
    _pointcloudEnabled(false),
    _pipelineStarted(false),
    _useThread(false),
    _depthFrame(nullptr),
    _hasNewColor(false),
    _hasNewIr(false),
    _hasNewDepth(false),
    _hasNewFrame(false)
{
    rs2::context ctx;
    rs2::device_list devList = ctx.query_devices();

    if (devList.size() <= 0) {
        throw std::runtime_error{"RealSense device not found!"};
    }
    if (deviceID >= devList.size()) {
        throw std::runtime_error{"Requested device id is invalid!"};
    }

    _device = devList[deviceID];
    std::string serial = _device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    ofLogVerbose()<< "Try to open device: "
            << _device.get_info(RS2_CAMERA_INFO_NAME)
            << " | Serial number: " << serial << std::endl << std::endl;

    _config.enable_device(serial);
    _setupFinished = true;
    setupParams(serial);
}

ofxLibRealSense2::ofxLibRealSense2(std::string __serial)
throw(std::runtime_error) :
    _setupFinished(false),
    _colorEnabled(false),
    _irEnabled(false),
    _depthEnabled(false),
    _pointcloudEnabled(false),
    _pipelineStarted(false),
    _useThread(false),
    _depthFrame(nullptr),
    _hasNewColor(false),
    _hasNewIr(false),
    _hasNewDepth(false),
    _hasNewFrame(false)
{
    rs2::context ctx;
    rs2::device_list devList = ctx.query_devices();
    std::string serial = "";
    bool devFound = false;

    if (devList.size() <= 0) {
        throw std::runtime_error{"RealSense device not found!"};
    }

    for (auto&& dev : devList) {
        serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (__serial.compare(serial) == 0) {
            devFound = true;
            _device  = dev;
            break;
        }
    }
    if (!devFound) {
        throw std::runtime_error{"RealSense device with serial "
            + __serial +" not found!"};
    }

    ofLogVerbose()<< "Try to open device: "
            << _device.get_info(RS2_CAMERA_INFO_NAME)
            << " | Serial number: " << serial << std::endl << std::endl;

    _config.enable_device(serial);
    _setupFinished = true;
    setupParams(serial);
}

ofxLibRealSense2::~ofxLibRealSense2() {
    waitForThread();
    stopThread();
    if (_pipelineStarted) _pipeline.stop();
}

/* --------------------------- STATIC METHODS ------------------------------ */

void ofxLibRealSense2::deviceInfo() {
    rs2::context ctx;
    rs2::device_list devList = ctx.query_devices();
    std::string name   = "";
    std::string serial = "";
    std::string prodId = "";
    std::string fw     = "";

    for (auto&& dev : devList) {
        name   = dev.get_info(RS2_CAMERA_INFO_NAME);
        serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        prodId = dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
        fw     = "";

        if (name == "Intel RealSense D415" || name == "Intel RealSense D435") {
            fw = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
        }
        std::cout<<"\nDevice name: "<< name <<"\nSerial number: "
                << serial <<"\nFirmware: "<< fw <<"\nProdID "<< prodId
                << std::endl << std::endl;
    }
}

void ofxLibRealSense2::resetDevice(const std::string __serial)
throw(std::runtime_error) {
    rs2::context ctx;
    rs2::device_list devList = ctx.query_devices();
    bool found = false;

    for (auto&& dev : devList) {
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (__serial.compare(serial) == 0) {
            found = true;
            dev.hardware_reset();
            break;
        }
    }

    if (!found) {
        throw std::runtime_error{"RealSense device with serial "
            + __serial +" not found!"};
    }
    else {
        std::cout<<"\nReset realsense with serial: "+ __serial +" done."
                <<std::endl<<std::endl;
    }
}

// static method
int ofxLibRealSense2::getDeviceCount() {
    rs2::context ctx;
    return ctx.query_devices().size();
}

/* --------------------------- STATIC METHODS END -------------------------- */


void ofxLibRealSense2::setupColor(int width, int height, int fps) {
    _colorWidth = width;
    _colorHeight = height;
    _colTex.allocate(_colorWidth, _colorHeight, GL_RGB);
    _config.enable_stream(RS2_STREAM_COLOR, -1, _colorWidth, _colorHeight, RS2_FORMAT_RGB8, fps);
    _colorEnabled = true;
}

void ofxLibRealSense2::setupIR(int width, int height, int fps) {
    _irWidth = width;
    _irHeight = height;
    _irTex.allocate(_irWidth, _irHeight, GL_LUMINANCE);
    _config.enable_stream(RS2_STREAM_INFRARED, 1, _irWidth, _irHeight, RS2_FORMAT_Y8, fps);
    _irEnabled = true;
}

void ofxLibRealSense2::setupDepth(int width, int height, int fps) {
    _depthWidth = width;
    _depthHeight = height;
    _depthTex.allocate(_depthWidth, _depthHeight, GL_RGB);
    _rawDepthTex.allocate(_depthWidth, _depthHeight, GL_LUMINANCE16);
    _config.enable_stream(RS2_STREAM_DEPTH, -1, _depthWidth, _depthHeight, RS2_FORMAT_Z16, fps);
    _colorizer.set_option(RS2_OPTION_COLOR_SCHEME, 2);
    _depthEnabled = true;
}

void ofxLibRealSense2::startPipeline(bool useThread) {
    if (!_setupFinished)
    	return;
    
    _pipeline.start(_config);
    _pipelineStarted=true;
    
    _useThread = useThread;
    if (_useThread)
        startThread();
}

void ofxLibRealSense2::enablePointcloud(bool enabled) {
    _pointcloudEnabled = enabled;
    if (!_depthEnabled) {
    	ofLogWarning() << "processing a pointcloud"
    			<<" requires to enable depth data!";
    }
}

void ofxLibRealSense2::threadedFunction() {
    while(isThreadRunning()) {
        if (lock()) {
            updateFrameData();
            unlock();
        }
    }
}

float ofxLibRealSense2::get_distance(int x, int y) const {
    return _depthFrame->get_distance(x, y);
}

const uint16_t * ofxLibRealSense2::getDepthRawData() const{
    return _rawDepthBuff;
}

void ofxLibRealSense2::updateFrameData() {
    rs2::frameset frameset;

    if (_pipeline.poll_for_frames(&frameset)) {
        if (_colorEnabled) {
            rs2::video_frame colFrame = frameset.get_color_frame();
            _colBuff = reinterpret_cast<uint8_t*>(const_cast<void*>(colFrame.get_data()));
            _colorWidth = colFrame.get_width();
            _colorHeight = colFrame.get_height();
            _hasNewColor = true;
        }
        if (_irEnabled) {
            rs2::video_frame irFrame = frameset.get_infrared_frame();
            _irBuff = reinterpret_cast<uint8_t*>(const_cast<void*>(irFrame.get_data()));
            _irWidth = irFrame.get_width();
            _irHeight = irFrame.get_height();
            _hasNewIr = true;
        }
        if (_depthEnabled) {
            // rs2::depth_frame depthFrame = frameset.get_depth_frame();
            _depthFrame = make_shared <rs2::depth_frame>(frameset.get_depth_frame());
            _rawDepthBuff = reinterpret_cast<uint16_t*>(const_cast<void*>(_depthFrame->get_data()));

            rs2::video_frame normFrame = _colorizer.process(*_depthFrame);		// normalized Depth Frame
            _depthBuff = reinterpret_cast<uint8_t*>(const_cast<void*>(normFrame.get_data()));
            _depthWidth = _depthFrame->get_width();
            _depthHeight = _depthFrame->get_height();
            _hasNewDepth = true;
            
            if (_pointcloudEnabled) {
                _points = _pointcloud.process(*_depthFrame);
            }
        }
    }
}


void ofxLibRealSense2::update() {
    if (!_pipelineStarted)
    	return;
    
    rs2::frameset frameset;
    if( !_useThread ) {
        updateFrameData();
    }
    
    _hasNewFrame = _hasNewColor | _hasNewIr | _hasNewDepth;

    if(_depthBuff && _hasNewDepth) {
        if (_pointcloudEnabled) {
            _mesh.clear();
            int n = _points.size();
            if(n!=0){
                const rs2::vertex * vs = _points.get_vertices();
                for(int i=0; i<n; i++){
                    if(vs[i].z >= depthMin && vs[i].z <= depthMax){
                        
                        const rs2::vertex v = vs[i];
                        glm::vec3 v3(v.x,v.y,v.z);
                        ofFloatColor color(ofMap(v.z, depthMin, depthMax, 1, 0.25));
                        _mesh.addVertex(v3);
                        _mesh.addColor(color);
                    }
                }
            }
        }
        _rawDepthTex.loadData(_rawDepthBuff, _depthWidth, _depthHeight, GL_LUMINANCE);
        _depthTex.loadData(_depthBuff, _depthWidth, _depthHeight, GL_RGB);
        _hasNewDepth = false;
    }

    if (_irBuff && _hasNewIr) {
        _irTex.loadData(_irBuff, _irWidth, _irHeight, GL_LUMINANCE);
        _hasNewIr = false;
    }

    if (_colBuff && _hasNewColor) {
        _colTex.loadData(_colBuff, _colorWidth, _colorHeight, GL_RGB);
        _hasNewColor = false;
    }
}

void ofxLibRealSense2::setupParams(const std::string & serialNumber) {
    rs2::sensor sensor = _device.query_sensors()[0];
    rs2::option_range orExp = sensor.get_option_range(rs2_option::RS2_OPTION_EXPOSURE);
    rs2::option_range orGain = sensor.get_option_range(rs2_option::RS2_OPTION_GAIN);
    rs2::option_range orMinDist = _colorizer.get_option_range(rs2_option::RS2_OPTION_MIN_DISTANCE);
    rs2::option_range orMaxDist = _colorizer.get_option_range(rs2_option::RS2_OPTION_MAX_DISTANCE);

    autoExposure.set("Auto-exposure", true);
    enableEmitter.set("Emitter", true);
    irExposure.set("IR Exposure", orExp.def, orExp.min, 26000);
    depthMin.set("Min Depth", orMinDist.def, orMinDist.min, orMinDist.max);
    depthMax.set("Max Depth", orMaxDist.def, orMaxDist.min, orMaxDist.max);

    _paramListeners.push(autoExposure.newListener([this](bool &)
    {
        if (!_pipelineStarted) return;

        rs2::sensor sensor = _pipeline.get_active_profile().get_device().first<rs2::depth_sensor>();
        if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
        {
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, autoExposure ? 1.0f : 0.0f);
        }
    }));

    _paramListeners.push(enableEmitter.newListener([this](bool &)
    {
        if (!_pipelineStarted) return;

        rs2::sensor sensor = _pipeline.get_active_profile().get_device().first<rs2::depth_sensor>();
        if (sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, enableEmitter ? 1.0f : 0.0f);
        }
    }));

    _paramListeners.push(irExposure.newListener([this](int &)
    {
        if (!_pipelineStarted) return;

        rs2::sensor sensor = _pipeline.get_active_profile().get_device().first<rs2::depth_sensor>();
        if (sensor.supports(rs2_option::RS2_OPTION_EXPOSURE))
        {
            sensor.set_option(rs2_option::RS2_OPTION_EXPOSURE, (float)irExposure);
        }
    }));

    _paramListeners.push(depthMin.newListener([this](float &)
    {
        if (!_pipelineStarted) return;

        _colorizer.set_option(rs2_option::RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 0);

        if (_colorizer.supports(rs2_option::RS2_OPTION_MIN_DISTANCE))
        {
            _colorizer.set_option(rs2_option::RS2_OPTION_MIN_DISTANCE, depthMin);
        }
        if (_colorizer.supports(rs2_option::RS2_OPTION_MAX_DISTANCE))
            _colorizer.set_option(rs2_option::RS2_OPTION_MAX_DISTANCE, depthMax);
    }));

    _paramListeners.push(depthMax.newListener([this](float &)
    {
        if (!_pipelineStarted) return;

        _colorizer.set_option(rs2_option::RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 0);

        if (_colorizer.supports(rs2_option::RS2_OPTION_MAX_DISTANCE))
        {
            _colorizer.set_option(rs2_option::RS2_OPTION_MAX_DISTANCE, depthMax);
        }
    }));

    params.setName("RS2 " + serialNumber);
    params.add(autoExposure, enableEmitter, irExposure, depthMin, depthMax);
}

