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

ITMSequenceReader::ITMSequenceReader()
{
#if 0
	m_NumFrames = 0;
	m_CurrFrame = 0;
	m_bHasColorData = false;
	//parameters are read from the calibration file
#endif
}

ITMSequenceReader::~ITMSequenceReader()
{
#if 0
	releaseData();
#endif
}


void ITMSequenceReader::createFirstConnected()
{
#if 1
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
#else
	releaseData();

	std::string filename = GlobalAppState::get().s_binaryDumpSensorFile;

	std::cout << "Start loading binary dump" << std::endl;
	//BinaryDataStreamZLibFile inputStream(filename, false);
	BinaryDataStreamFile inputStream(filename, false);
	inputStream >> m_data;
	std::cout << "Loading finished" << std::endl;
	std::cout << m_data << std::endl;

	//default
	//m_data.m_CalibrationDepth.m_Intrinsic = mat4f(
	//	525.0f, 0.0f, 319.5f, 0.0f,
	//	0.0f, 525.0f, 239.5f, 0.0f,
	//	0.0f, 0.0f, 1.0f, 0.0f,
	//	0.0f, 0.0f, 0.0f, 1.0f);
	//m_data.m_CalibrationDepth.m_IntrinsicInverse = m_data.m_CalibrationDepth.m_Intrinsic.getInverse();
	//m_data.m_CalibrationColor.m_Intrinsic = mat4f(
	//	525.0f, 0.0f, 319.5f, 0.0f,
	//	0.0f, 525.0f, 239.5f, 0.0f,
	//	0.0f, 0.0f, 1.0f, 0.0f,
	//	0.0f, 0.0f, 0.0f, 1.0f);
	//m_data.m_CalibrationColor.m_IntrinsicInverse = m_data.m_CalibrationColor.m_Intrinsic.getInverse();
	//fr1
	//m_data.m_CalibrationDepth.m_Intrinsic = mat4f(
	//	591.1f, 0.0f, 331.0f, 0.0f,
	//	0.0f, 590.1f, 234.0f, 0.0f,
	//	0.0f, 0.0f, 1.0f, 0.0f,
	//	0.0f, 0.0f, 0.0f, 1.0f);
	//m_data.m_CalibrationDepth.m_IntrinsicInverse = m_data.m_CalibrationDepth.m_Intrinsic.getInverse();
	//m_data.m_CalibrationColor.m_Intrinsic = mat4f(
	//	517.3f, 0.0f, 318.6f, 0.0f,
	//	0.0f, 516.5f, 255.3f, 0.0f,
	//	0.0f, 0.0f, 1.0f, 0.0f,
	//	0.0f, 0.0f, 0.0f, 1.0f);
	//m_data.m_CalibrationColor.m_IntrinsicInverse = m_data.m_CalibrationColor.m_Intrinsic.getInverse();
	//fr2
	//m_data.m_CalibrationDepth.m_Intrinsic = mat4f(		// fr2 
	//	580.8f, 0.0f, 308.8f, 0.0f,
	//	0.0f, 581.8f, 253.0f, 0.0f,
	//	0.0f, 0.0f, 1.0f, 0.0f,
	//	0.0f, 0.0f, 0.0f, 1.0f);
	//m_data.m_CalibrationDepth.m_IntrinsicInverse = m_data.m_CalibrationDepth.m_Intrinsic.getInverse();
	//m_data.m_CalibrationColor.m_Intrinsic = mat4f(		// fr2 
	//	520.9f, 0.0f, 325.1f, 0.0f,
	//	0.0f, 521.0f, 249.7f, 0.0f,
	//	0.0f, 0.0f, 1.0f, 0.0f,
	//	0.0f, 0.0f, 0.0f, 1.0f);
	//m_data.m_CalibrationColor.m_IntrinsicInverse = m_data.m_CalibrationColor.m_Intrinsic.getInverse();
	//fr3
	//m_data.m_CalibrationDepth.m_Intrinsic = mat4f(		// fr3 
	//	567.6f, 0.0f, 324.7f, 0.0f,
	//	0.0f, 570.2f, 250.1f, 0.0f,
	//	0.0f, 0.0f, 1.0f, 0.0f,
	//	0.0f, 0.0f, 0.0f, 1.0f);
	//m_data.m_CalibrationDepth.m_IntrinsicInverse = m_data.m_CalibrationDepth.m_Intrinsic.getInverse();
	//m_data.m_CalibrationColor.m_Intrinsic = mat4f(		// fr3 
	//	535.4f, 0.0f, 320.1f, 0.0f,
	//	0.0f, 539.2f, 247.6f, 0.0f,
	//	0.0f, 0.0f, 1.0f, 0.0f,
	//	0.0f, 0.0f, 0.0f, 1.0f);
	//m_data.m_CalibrationColor.m_IntrinsicInverse = m_data.m_CalibrationColor.m_Intrinsic.getInverse();

	RGBDSensor::init(m_data.m_DepthImageWidth, m_data.m_DepthImageHeight, std::max(m_data.m_ColorImageWidth,1u), std::max(m_data.m_ColorImageHeight,1u), 1);
	initializeDepthIntrinsics(m_data.m_CalibrationDepth.m_Intrinsic(0,0), m_data.m_CalibrationDepth.m_Intrinsic(1,1), m_data.m_CalibrationDepth.m_Intrinsic(0,2), m_data.m_CalibrationDepth.m_Intrinsic(1,2));
	initializeColorIntrinsics(m_data.m_CalibrationColor.m_Intrinsic(0,0), m_data.m_CalibrationColor.m_Intrinsic(1,1), m_data.m_CalibrationColor.m_Intrinsic(0,2), m_data.m_CalibrationColor.m_Intrinsic(1,2));

	initializeDepthExtrinsics(m_data.m_CalibrationDepth.m_Extrinsic);
	initializeColorExtrinsics(m_data.m_CalibrationColor.m_Extrinsic);


	m_NumFrames = m_data.m_DepthNumFrames;
	assert(m_data.m_ColorNumFrames == m_data.m_DepthNumFrames || m_data.m_ColorNumFrames == 0);		

	if (m_data.m_ColorImages.size() > 0) {
		m_bHasColorData = true;
	} else {
		m_bHasColorData = false;
	}
#endif
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
#if 1
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
#else
	if(m_CurrFrame >= m_NumFrames)
	{
		GlobalAppState::get().s_playData = false;
		//std::cout << "binary dump sequence complete - press space to run again" << std::endl;
		stopReceivingFrames();
		std::cout << "binary dump sequence complete - stopped receiving frames" << std::endl;
		m_CurrFrame = 0;
	}

	if(GlobalAppState::get().s_playData) {

		float* depth = getDepthFloat();
		memcpy(depth, m_data.m_DepthImages[m_CurrFrame], sizeof(float)*getDepthWidth()*getDepthHeight());

		incrementRingbufIdx();

		if (m_bHasColorData) {
			memcpy(m_colorRGBX, m_data.m_ColorImages[m_CurrFrame], sizeof(vec4uc)*getColorWidth()*getColorHeight());
		}

		m_CurrFrame++;
		return true;
	} else {
		return false;
	}
#endif
}

#if 0
void ITMSequenceReader::releaseData()
{
	m_CurrFrame = 0;
	m_bHasColorData = false;
	m_data.deleteData();
}

void ITMSequenceReader::evaluateTrajectory(const std::vector<mat4f>& trajectory) const
{
	std::vector<mat4f> referenceTrajectory = m_data.m_trajectory;
	const size_t numTransforms = std::min(trajectory.size(), referenceTrajectory.size());
	// make sure reference trajectory starts at identity
	mat4f offset = referenceTrajectory.front().getInverse();
	for (unsigned int i = 0; i < referenceTrajectory.size(); i++) referenceTrajectory[i] = offset * referenceTrajectory[i];

	const auto transErr = PoseHelper::evaluateAteRmse(trajectory, referenceTrajectory);
	std::cout << "*********************************" << std::endl;
	std::cout << "ate rmse = " << transErr.first << ", " << transErr.second << std::endl;
	std::cout << "*********************************" << std::endl;
	//{
	//	std::vector<mat4f> optTrajectory = trajectory;
	//	optTrajectory.resize(numTransforms);
	//	referenceTrajectory.resize(numTransforms);
	//	PoseHelper::saveToPoseFile("debug/opt.txt", optTrajectory);
	//	PoseHelper::saveToPoseFile("debug/gt.txt", referenceTrajectory);
	//}
}
#endif
