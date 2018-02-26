#pragma once


/************************************************************************/
/* Reads an InfiniTAM sequence from disk                                */
/************************************************************************/

#include "GlobalAppState.h"
#include "RGBDSensor.h"
#include "stdafx.h"

#include <boost/filesystem.hpp>

class ITMSequenceReader : public RGBDSensor
{
public:
	//! initializes the sensor
	void createFirstConnected();

	//! reads the next depth frame
	bool processDepth();

	bool processColor()	{
		//everything done in process depth since order is relevant (color must be read first)
		return true;
	}

	std::string getSensorName() const {
    return "ITMSequenceReader";
	}

  mat4f getRigidTransform() const;

	void stopReceivingFrames() { m_bIsReceivingFrames = false; }

private:
  int m_currentFrameNo;
  std::string m_depthImageMask;
  mat4f m_initialTransformInv;
  std::string m_poseFileMask;
  std::string m_rgbImageMask;
  boost::filesystem::path m_sequenceDir;
};
