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

	//! Constructor
	ITMSequenceReader();

	//! Destructor; releases allocated ressources
	~ITMSequenceReader();

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

#if 0
	mat4f getRigidTransform() const {
		if (m_CurrFrame-1 >= m_data.m_trajectory.size()) throw MLIB_EXCEPTION("invalid trajectory index " + std::to_string(m_CurrFrame-1));
		return m_data.m_trajectory[m_CurrFrame-1];
	}
#else
  mat4f getRigidTransform() const;
#endif

#if 0
	unsigned int getNumTotalFrames() const {
		return m_NumFrames;
	}
#endif

	void stopReceivingFrames() { m_bIsReceivingFrames = false; }

#if 0
	void evaluateTrajectory(const std::vector<mat4f>& trajectory) const;
#endif

private:
  int m_currentFrameNo;
  std::string m_depthImageMask;
  mat4f m_initialTransformInv;
  std::string m_poseFileMask;
  std::string m_rgbImageMask;
  boost::filesystem::path m_sequenceDir;
#if 0
	//! deletes all allocated data
	void releaseData();

	CalibratedSensorData m_data;

	unsigned int	m_NumFrames;
	unsigned int	m_CurrFrame;
	bool			m_bHasColorData;
#endif
};
