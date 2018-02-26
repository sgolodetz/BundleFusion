#include "stdafx.h"

#include "ITMSequenceReader.h"
#include "GlobalAppState.h"
#include "PoseHelper.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
namespace bf = boost::filesystem;

#include <opencv2/opencv.hpp>

void ITMSequenceReader::createFirstConnected()
{
  m_sequenceDir = GlobalAppState::get().s_itmSequenceDir;
  if(!bf::is_directory(m_sequenceDir))
  {
    throw std::runtime_error("Error: " + m_sequenceDir.string() + " is not a directory");
  }

  m_currentFrameNo = GlobalAppState::get().s_itmInitialFrameNo;
  m_depthImageMask = GlobalAppState::get().s_itmDepthImageMask;
  m_poseFileMask = GlobalAppState::get().s_itmPoseFileMask;
  m_rgbImageMask = GlobalAppState::get().s_itmRgbImageMask;

  // TODO: Load in the calibration parameters rather than hard-coding them.
#if 1
  const vec2i rgbImgSize(480, 270);
  const vec2i depthImgSize(224, 172);
#else
  const vec2i rgbImgSize(640, 480);
  const vec2i depthImgSize(640, 480);
#endif
  const int depthRingBufferSize = 1;
  RGBDSensor::init(depthImgSize.x, depthImgSize.y, rgbImgSize.x, rgbImgSize.y, depthRingBufferSize);

#if 1
  const float fxRgb = 370.045f, fyRgb = 369.5f;
  const float cxRgb = 241.2495f, cyRgb = 134.66325f;
  const float fxDepth = 215.965f, fyDepth = 215.965f;
  const float cxDepth = 111.453f, cyDepth = 85.0044f;
#else
  const float fxRgb = 532.569f, fyRgb = 531.541f;
  const float cxRgb = 320, cyRgb = 240;
  const float fxDepth = 532.569f, fyDepth = 531.541f;
  const float cxDepth = 320, cyDepth = 240;
#endif
	initializeDepthIntrinsics(fxDepth, fyDepth, cxDepth, cyDepth);
	initializeColorIntrinsics(fxRgb, fyRgb, cxRgb, cyRgb);

	initializeDepthExtrinsics(mat4f::identity());
	initializeColorExtrinsics(mat4f::identity());

  m_initialTransformInv = mat4f::identity();
  m_initialTransformInv = getRigidTransform();
  m_initialTransformInv.invert();
}

mat4f ITMSequenceReader::getRigidTransform() const
{
  bf::path poseFilename = m_sequenceDir / (boost::format(m_poseFileMask) % m_currentFrameNo).str();

  if(!GlobalAppState::get().s_playData || !bf::exists(poseFilename))
  {
    return mat4f::identity();
  }

  std::ifstream fs(poseFilename.c_str());

  mat4f m;

  for(unsigned char y = 0; y < 4; ++y)
  {
    for(unsigned char x = 0; x < 4; ++x)
    {
      fs >> m.at(y,x);
    }
  }

  return m_initialTransformInv * m;
}

bool ITMSequenceReader::processDepth()
{
  bf::path depthFilename = m_sequenceDir / (boost::format(m_depthImageMask) % m_currentFrameNo).str();
  bf::path rgbFilename = m_sequenceDir / (boost::format(m_rgbImageMask) % m_currentFrameNo).str();
  if(!bf::exists(depthFilename) || !bf::exists(rgbFilename))
  {
    GlobalAppState::get().s_playData = false;
    stopReceivingFrames();
    m_currentFrameNo = 0;
  }

  if(GlobalAppState::get().s_playData)
  {
    cv::Mat depthImage = cv::imread(depthFilename.string(), CV_LOAD_IMAGE_ANYDEPTH);
    for(int y = 0; y < m_depthHeight; ++y)
    {
      const unsigned short *srcRow = depthImage.ptr<unsigned short>(y);
      float *row = getDepthFloat() + y * m_depthWidth;
      for(int x = 0; x < m_depthWidth; ++x)
      {
        row[x] = static_cast<float>(srcRow[x]) / 1000.0f;
      }
    }

    cv::Mat colourImage = cv::imread(rgbFilename.string());
    for(int y = 0; y < m_colorHeight; ++y)
    {
      const cv::Vec3b *srcRow = colourImage.ptr<cv::Vec3b>(y);
      ml::vec4uc *row = m_colorRGBX + y * m_colorWidth;
      for(int x = 0; x < m_colorWidth; ++x)
      {
        row[x].r = srcRow[x][0];
        row[x].g = srcRow[x][1];
        row[x].b = srcRow[x][2];
      }
    }

    ++m_currentFrameNo;
    return true;
  }
  else
  {
    return false;
  }
}
