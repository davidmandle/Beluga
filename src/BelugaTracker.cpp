#include "BelugaTracker.h"

#include "CalibrationDataFile.h"

#include "TrackerExtras.h"
#include "BelugaDynamics.h"

#include "BelugaConstants.h"

/* uncomment to see extra output from the tracker in the console */
//#define DEBUG_VERBOSE

/* used for readability of grabbing the last row of a CvMat */
#define LAST_ROW(a_cv_mat) (a_cv_mat)->rows - 1

/* default color to draw blobs */
const MT_Color DEFAULT_BLOB_COLOR = MT_Red;

/* number of past positions to use when estimating orientation */
const unsigned int N_hist = 5;

const double DEFAULT_GATE_DIST2 = 100.0*100.0;

/* defined for readability of assert(false), which DOES assert
 * i.e., assert(BELUGA_DO_ASSERT) WILL assert and abort */
const bool BELUGA_DO_ASSERT = false;

/* VFLIP needs to be defined as "m_iFrameHeight -" on Pod because of 
 * the way that the screen and camera coordinates index from the top
 * of the image rather than the bottom. */
#define VFLIP m_iFrameHeight -
/* On other systems (suspect OS X and *nix), you may need to
 * do this instead */
//#define VFLIP

/* TODO:  This could be in MT */
template <class T>
int indexInVector(const std::vector<T>& v, const T& value)
{
    for(unsigned int i = 0; i < v.size(); i++)
    {
        if(v[i] == value)
        {
            return i;
        }
    }
    return -1;
}

/**********************************************************************
 * Segmenter Class
 *********************************************************************/

void BelugaSegmenter::usePrevious(MT_DSGYA_Blob* obj, unsigned int i)
{
    MT_DSGYA_Segmenter::usePrevious(obj, i);
    m_pTracker->notifyNoMeasurement(i);
	// TODO:  Handling rho contribution when a measurement goes missing
}

bool BelugaSegmenter::areAdjacent(MT_DSGYA_Blob* obj, const YABlob& blob)
{
    if(obj->m_dRhoContrib > 0)
    {
        // TODO see usePrevious
    }

    double dx = obj->m_dXCenter - blob.COMx;
    double dy = obj->m_dYCenter - blob.COMy;
    double rho = m_dOverlapFactor*(obj->m_dRhoContrib + blob.major_axis);
    return dx*dx + dy*dy < rho*rho;
}


/**********************************************************************
 * Tracker Class
 *********************************************************************/

/* Constructor - this gets called when we create a new instance of
 * this class.  It should initialize all of the member variables and
 * do whatever memory allocation is necessary. */
BelugaTracker::BelugaTracker(IplImage* ProtoFrame,
							unsigned int n_obj
							)
    : MT_TrackerBase(ProtoFrame),
      m_dWaterDepth(DEFAULT_WATER_DEPTH),
      m_iBlobValThresh(DEFAULT_BG_THRESH),
	  m_iBlobAreaThreshLow(DEFAULT_MIN_BLOB_AREA),
	  m_iBlobAreaThreshHigh(DEFAULT_MAX_BLOB_AREA),
      m_dOverlapFactor(1.0),
	  m_iVThresh(DEFAULT_V_THRESH),
	  m_iHThresh_Low(DEFAULT_H_THRESH_LOW),
	  m_iHThresh_High(DEFAULT_H_THRESH_HIGH),
	  m_iSThresh_Low(DEFAULT_S_THRESH_LOW),
	  m_iSThresh_High(DEFAULT_S_THRESH_HIGH),
      m_bNoHistory(true),
	  m_iSearchAreaPadding(DEFAULT_SEARCH_AREA_PADDING),
      m_iStartFrame(-1),
      m_iStopFrame(-1),
      m_vpUKF(n_obj, NULL),
      m_dSigmaPosition(DEFAULT_SIGMA_POSITION),
      m_dSigmaZ(DEFAULT_SIGMA_Z),
      m_dSigmaZDot(DEFAULT_SIGMA_ZDOT),
      m_dSigmaSpeed(DEFAULT_SIGMA_SPEED),      
      m_dSigmaHeading(DEFAULT_SIGMA_HEADING),
      m_dSigmaOmega(DEFAULT_SIGMA_OMEGA),
      m_dSigmaPositionMeas(DEFAULT_SIGMA_POSITION_MEAS),
      m_dSigmaHeadingMeas(DEFAULT_SIGMA_HEADING_MEAS),
      m_dSigmaZMeas(DEFAULT_SIGMA_Z_MEAS),
      m_dPrevSigmaPosition(0),
      m_dPrevSigmaZ(0),
      m_dPrevSigmaZDot(0),
      m_dPrevSigmaSpeed(0),      
      m_dPrevSigmaHeading(0),
      m_dPrevSigmaOmega(0),      
      m_dPrevSigmaPositionMeas(0),
      m_dPrevSigmaZMeas(0),
      m_dPrevSigmaHeadingMeas(0),
      m_bShowBlobs(true),
      m_bShowTracking(true),
      m_dDt(0.1),
      m_iFrameCounter(0),
      m_iNObj(n_obj),
	  m_dCurrentTime(0),
      m_iFrameHeight(0),
	  m_iFrameWidth(0),
      m_iCurrentCamera(0),
	  m_pQ(NULL),
	  m_pR(NULL),
	  m_px0(NULL),
	  m_pz(NULL),
      m_pu(NULL),
      m_pTempFrame1(NULL),
      m_pUndistortMapX(NULL),
      m_pUndistortMapY(NULL),
      m_YABlobber(),
      m_vdBlobs_X(0),
      m_vdBlobs_Y(0),
      m_vdBlobs_Area(0),
      m_vdBlobs_Orientation(0),
      m_vdBlobs_MajorAxis(0),
      m_vdBlobs_MinorAxis(0),
      m_vdBlobs_Speed(0),
      m_vdTracked_X(0),
      m_vdTracked_Y(0),
      m_vdTracked_Z(0),
      m_vdTracked_ZDot(0),
      m_vdTracked_Speed(0),      
      m_vdTracked_Heading(0),
      m_vdTracked_Omega(0),
      m_vvdMeas_X(0),
      m_vvdMeas_Y(0),
      m_vvdMeas_Hdg(0),
      m_vviMeas_A(0),
      m_vviMeas_Cam(0),
      m_vdDepthMeasurement(0),
      m_vdSpeedCommand(0),
      m_vdVerticalCommand(0),
      m_vdTurnCommand(0),
      m_vbLastMeasValid(0),
      m_vdHistories_X(0),
      m_vdHistories_Y(0),
      m_dInitAdjacencyThresh(1.0) /* meters */
{
    doInit(ProtoFrame);
}

/* Destructor - basically the opposite of the constructor, gets called
 * whenever an object is deleted or goes out of scope.  It should
 * de-allocate any memory that was allocated */
BelugaTracker::~BelugaTracker()
{
    /* cvReleaseMat deallocates matrices given to it */
    cvReleaseMat(&m_px0);
    cvReleaseMat(&m_pz);
    cvReleaseMat(&m_pu);    
    cvReleaseMat(&m_pQ);
    cvReleaseMat(&m_pR);

	if(m_pHSVFrames[0])
	{
		for(int i = 0; i < 4; i++)
		{
			cvReleaseImage(&m_pUndistortedFrames[i]);
			cvReleaseImage(&m_pHSVFrames[i]);
			cvReleaseImage(&m_pHFrames[i]);
			cvReleaseImage(&m_pSFrames[i]);
			cvReleaseImage(&m_pVFrames[i]);
			cvReleaseImage(&m_pHThreshFrames[i]);
			cvReleaseImage(&m_pSThreshFrames[i]);
			cvReleaseImage(&m_pVThreshFrames[i]);
			cvReleaseImage(&m_pThreshFrames[i]);
		}
		cvReleaseImage(&m_pTempFrame1);
	}

	if(m_pMasks[0])
	{
		cvReleaseImage(&m_pMasks[0]);
		cvReleaseImage(&m_pMasks[1]);
		cvReleaseImage(&m_pMasks[2]);
		cvReleaseImage(&m_pMasks[3]);
	}

	if(m_pCameraMatrices[0])
	{
		for(int i = 0; i < 4; i++)
		{
			cvReleaseMat(&m_pCameraMatrices[i]);
			cvReleaseMat(&m_pDistortionCoeffs[i]);
			cvReleaseMat(&m_pRotationVectors[i]);
			cvReleaseMat(&m_pRotationMatrices[i]);
			cvReleaseMat(&m_pTranslationVectors[i]);
		}
	}

	if(m_pUndistortMapX)
	{
		cvReleaseImage(&m_pUndistortMapX);
		cvReleaseImage(&m_pUndistortMapY);
	}

    for(int i = 0; i < 3; i++)
    {
        delete m_pAuxFrameGroups[i];
    }

    /* MT_UKFFree frees up memory used by the UKF */
    for(int i = 0; i < m_iNObj; i++)
    {
        MT_UKFFree(&m_vpUKF[i]);
    }
}

/* this is the main initialization function and gets called by the
 * constructor.  It does memory allocation, etc. */
void BelugaTracker::doInit(IplImage* ProtoFrame)
{

	for(int i = 0; i < 4; i++)
	{
		m_SearchArea[i].resize(0);
		m_SearchIndexes[i].resize(0);
	}

    /* Not using the built-in tracked objects functions - setting
     * this pointer to NULL will ensure that the appropriate code is
     * disabled  */
    m_pTrackedObjects = NULL;

    /* It's always a good idea to initialize pointers to NULL so that
       other pieces of code can use e.g. if(p) to check for allocation 
	   NOTE that normally you should do this in the constructor's 
	    initializer list, but we have to do these here because they're
		arrays */
	for(int i = 0; i < 4; i++)
	{
		m_pGSFrames[i] = NULL;
		m_pUndistortedFrames[i] = NULL;
		m_pThreshFrames[i] = NULL;

		m_pHSVFrames[i] = NULL;
		m_pHFrames[i] = NULL;
		m_pSFrames[i] = NULL;
		m_pVFrames[i] = NULL;
		m_pHThreshFrames[i] = NULL;
		m_pSThreshFrames[i] = NULL;
		m_pVThreshFrames[i] = NULL;

		m_pMasks[i] = NULL;

		m_pCameraMatrices[i] = NULL;
		m_pDistortionCoeffs[i] = NULL;
		m_pRotationVectors[i] = NULL;
		m_pRotationMatrices[i] = NULL;
		m_pTranslationVectors[i] = NULL;

		m_vBlobs[i].resize(0);
        m_vPredictedBlobs[i].resize(m_iNObj);
	}

    /* grab the frame height */
    m_iFrameHeight = ProtoFrame->height;
	m_iFrameWidth = ProtoFrame->width;

    /* Call the base class's doInit method.
     * This initializes variables to safe values and
     * calls doTrain to set the background frame,
     * frame sizes, and calls createFrames(). */
    MT_TrackerBase::doInit(ProtoFrame);

    /* resize all of our vectors. note that the std::vector object
       deallocates memory on its own, so we won't have to do that later */
    m_vdBlobs_X.resize(m_iNObj);
    m_vdBlobs_Y.resize(m_iNObj);          
    m_vdBlobs_Area.resize(m_iNObj);
    m_vdBlobs_Orientation.resize(m_iNObj);
    m_vdBlobs_MajorAxis.resize(m_iNObj);  
    m_vdBlobs_MinorAxis.resize(m_iNObj);  
    m_vdBlobs_Speed.resize(m_iNObj);
    m_viMatchAssignments.resize(m_iNObj);
    m_vdTracked_X.resize(m_iNObj);
    m_vdTracked_Y.resize(m_iNObj);
    m_vdTracked_Z.resize(m_iNObj);
    m_vdTracked_ZDot.resize(m_iNObj);
    m_vdTracked_Speed.resize(m_iNObj);    
    m_vdTracked_Heading.resize(m_iNObj);
    m_vdTracked_Omega.resize(m_iNObj);
    for(int i = 0; i < m_iNObj; i++)
    {
        /* assume we start on the surface */
        m_vdTracked_X[i] = 0;
        m_vdTracked_Y[i] = 0;
        m_vdTracked_Z[i] = m_dWaterDepth;
        m_vdTracked_ZDot[i] = 0;
        m_vdTracked_Speed[i] = 0;
        m_vdTracked_Heading[i] = 0;
        m_vdTracked_Omega[i] = 0;
    }
	m_vvdMeas_X.resize(m_iNObj);
	m_vvdMeas_Y.resize(m_iNObj);
	m_vviMeas_A.resize(m_iNObj);
	m_vviMeas_Cam.resize(m_iNObj);
	m_vvdMeas_Hdg.resize(m_iNObj);
	for(int i = 0; i < 4; i++)
	{
		m_vdaTracked_XC[i].resize(m_iNObj);
		m_vdaTracked_YC[i].resize(m_iNObj);
	}
	for(int i = 0; i < m_iNObj; i++)
	{
		m_vvdMeas_X[i].resize(0);
		m_vvdMeas_Y[i].resize(0);
		m_vviMeas_A[i].resize(0);
		m_vviMeas_Cam[i].resize(0);
		m_vvdMeas_Hdg[i].resize(0);
	}

	m_vdDepthMeasurement.resize(m_iNObj, 0.0);
    
	m_vdSpeedCommand.resize(m_iNObj, 0.0);
	m_vdVerticalCommand.resize(m_iNObj, 0.0);
	m_vdTurnCommand.resize(m_iNObj, 0.0);

    /* these don't need to be resized to 4 because they're
     * straight arrays, not std::vectors */
    for(unsigned int i = 0; i < 4; i++)
    {
		/* but their elements do need to be resized */
        m_vbValidMeasCamObj[i].resize(m_iNObj);
        m_viAssignments[i].resize(0);
        m_vInitBlobs[i].resize(0);
    }
    
    m_vdHistories_X.resize(m_iNObj);
    m_vdHistories_Y.resize(m_iNObj);
	m_vbLastMeasValid.resize(m_iNObj);
    /* there is an X and Y history for each object.  They need to be
       initially zero in length so that we can fill them up with real
       data as it becomes available. */
    for(int i = 0; i < m_iNObj; i++)
    {
        m_vdHistories_X[i].resize(0);
        m_vdHistories_Y[i].resize(0); 
		m_vbLastMeasValid[i] = false;
    }


    /* sets up the frames that are available in the "view" menu */
    m_pTrackerFrameGroup = new MT_TrackerFrameGroup();
	m_pTrackerFrameGroup->pushFrame(&m_pUndistortedFrames[0], "Undistorted");
    m_pTrackerFrameGroup->pushFrame(&m_pThreshFrames[0],    "Threshold Frame");
	m_pTrackerFrameGroup->pushFrame(&m_pHSVFrames[0], "HSV");
	m_pTrackerFrameGroup->pushFrame(&m_pHFrames[0], "H");
	m_pTrackerFrameGroup->pushFrame(&m_pSFrames[0], "S");
	m_pTrackerFrameGroup->pushFrame(&m_pVFrames[0], "V");
	m_pTrackerFrameGroup->pushFrame(&m_pHThreshFrames[0], "H Thresh");
	m_pTrackerFrameGroup->pushFrame(&m_pSThreshFrames[0], "S Thresh");
	m_pTrackerFrameGroup->pushFrame(&m_pVThreshFrames[0], "V Thresh");
	m_pTrackerFrameGroup->pushFrame(&m_pGSFrames[0], "GS");

    for(unsigned int i = 0; i < 3; i++)
    {
        m_pAuxFrameGroups[i] = new MT_TrackerFrameGroup();
		m_pAuxFrameGroups[i]->pushFrame(&m_pUndistortedFrames[i+1], "Undistorted");
        m_pAuxFrameGroups[i]->pushFrame(&m_pThreshFrames[i+1], "Threshold Frame");
        m_pAuxFrameGroups[i]->pushFrame(&m_pHSVFrames[i+1], "HSV");
        m_pAuxFrameGroups[i]->pushFrame(&m_pHFrames[i+1], "H");
        m_pAuxFrameGroups[i]->pushFrame(&m_pSFrames[i+1], "S");
        m_pAuxFrameGroups[i]->pushFrame(&m_pVFrames[i+1], "V");
        m_pAuxFrameGroups[i]->pushFrame(&m_pHThreshFrames[i+1], "H Thresh");
        m_pAuxFrameGroups[i]->pushFrame(&m_pSThreshFrames[i+1], "S Thresh");
        m_pAuxFrameGroups[i]->pushFrame(&m_pVThreshFrames[i+1], "V Thresh");
        m_pAuxFrameGroups[i]->pushFrame(&m_pGSFrames[i+1], "GS");        
    }

    /* Data group and Data report setup.
     *
     * This is how the tracker class interacts with the GUI.  When the
     * tracker is initialized, the GUI pulls the contents of
     * m_vDataGroups and automatically creates dialog boxes enabling
     * the user to view/edit these values.
     *
     * An MT_DataGroup is a list of parameters and can be read/write -
     * e.g. minimum blob size, or drawing color.
     * An MT_DataReport is a list of vectors of numbers and is
     * read-only.  e.g. a report of the found blobs positions.
     */
    
    /* set up the parameter groups for parameter modification, etc. */
    /* first group is for blob tracking parameters */
    MT_DataGroup* dg_blob = new MT_DataGroup("Blob Tracking Parameters");
	dg_blob->AddUInt("HSV V Threshold", &m_iVThresh, MT_DATA_READWRITE, 0, 255);
	dg_blob->AddUInt("HSV H Threshold Low", &m_iHThresh_Low, MT_DATA_READWRITE, 0, 255);
	dg_blob->AddUInt("HSV H Threshold High", &m_iHThresh_High, MT_DATA_READWRITE, 0, 255);
	dg_blob->AddUInt("HSV S Threshold Low", &m_iSThresh_Low, MT_DATA_READWRITE, 0, 255);
	dg_blob->AddUInt("HSV S Threshold High", &m_iSThresh_High, MT_DATA_READWRITE, 0, 255);
    dg_blob->AddDouble("Water Depth", &m_dWaterDepth, MT_DATA_READWRITE, 0);
    dg_blob->AddDouble("Blobber Overlap", &m_dOverlapFactor, MT_DATA_READWRITE, 0);
    dg_blob->AddDouble("Init Adjacency Thresh", &m_dInitAdjacencyThresh, MT_DATA_READWRITE, 0);
    dg_blob->AddUInt("Difference Threshold", /* parameter name */
                     &m_iBlobValThresh,      /* pointer to variable */
                     MT_DATA_READWRITE,      /* read-only or not */
                     0,                      /* minimum value */
                     255);                   /* maximum value */
    dg_blob->AddUInt("Min Blob Area",
                    &m_iBlobAreaThreshLow,
                    MT_DATA_READWRITE,
                    0);
    dg_blob->AddUInt("Max Blob Area",
                    &m_iBlobAreaThreshHigh,
                    MT_DATA_READWRITE,
                    0);
	dg_blob->AddUInt("Search Area Padding",
		             &m_iSearchAreaPadding,
					 MT_DATA_READWRITE,
					 0);
    dg_blob->AddDouble("Position Disturbance Sigma",
                       &m_dSigmaPosition,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Vertical Disturbance Sigma",
                       &m_dSigmaZ,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Vertical Speed Disturbance Sigma",
                       &m_dSigmaZDot,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Speed Disturbance Sigma",
                       &m_dSigmaSpeed,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Heading Disturbance Sigma",
                       &m_dSigmaHeading,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Omega Disturbance Sigma",
                       &m_dSigmaOmega,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Position Measurement Sigma",
                       &m_dSigmaPositionMeas,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Z Measurement Sigma",
                       &m_dSigmaZMeas,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Heading Measurement Sigma",
                       &m_dSigmaHeadingMeas,
                       MT_DATA_READWRITE,
                       0);

    MT_DataGroup* dg_draw = new MT_DataGroup("Drawing Options");
    dg_draw->AddBool("Show blob arrows", &m_bShowBlobs);
    dg_draw->AddBool("Show tracking arrows", &m_bShowTracking);
    
    /* now stuff the parameter groups into m_vDataGroups, which
     * MT_TrackerBase will automagically report to the GUI */
    m_vDataGroups.resize(0);
    m_vDataGroups.push_back(dg_blob);
    m_vDataGroups.push_back(dg_draw);
    
    MT_DataReport* dr_tracked = new MT_DataReport("Tracked data");
    dr_tracked->AddDouble("X", &m_vdTracked_X);
    dr_tracked->AddDouble("Y", &m_vdTracked_Y);
    dr_tracked->AddDouble("Z", &m_vdTracked_Z);
    dr_tracked->AddDouble("Zdot", &m_vdTracked_ZDot);
	dr_tracked->AddDouble("Hdg", &m_vdTracked_Heading);
    dr_tracked->AddDouble("Omega", &m_vdTracked_Omega);
    dr_tracked->AddDouble("Spd", &m_vdTracked_Speed);
	dr_tracked->AddDouble("Zmeas", &m_vdDepthMeasurement);

	MT_DataReport* dr_blobs = new MT_DataReport("Blob Info");
	dr_blobs->AddDouble("X", &m_vdBlobs_X);
	dr_blobs->AddDouble("Y", &m_vdBlobs_Y);
	dr_blobs->AddDouble("Area", &m_vdBlobs_Area);

    m_vDataReports.resize(0);
    m_vDataReports.push_back(dr_blobs);
	m_vDataReports.push_back(dr_tracked);

}

/* this would normally be where we capture e.g. a background
   image, but we're not using background subtraction here
   we DO, however, need to grab the correct image sizes */
void BelugaTracker::doTrain(IplImage* frame)
{
	m_iFrameWidth = frame->width;
	m_iFrameHeight = frame->height;
	CvSize fsize = cvSize(m_iFrameWidth, m_iFrameHeight);

	MT_TrackerBase::doTrain(frame);
}

/* This gets called by MT_TrackerBase::doInit.  I use it here more to
 * allocate other bits of memory - a lot of this could actually go
 * into doInit, but there's no real harm */
void BelugaTracker::createFrames()
{
    /* this makes sure that the BG_frame is created */
    MT_TrackerBase::createFrames();

	if(m_pHSVFrames[0])
	{
		for(int i = 0; i < 4; i++)
		{
			cvReleaseImage(&m_pUndistortedFrames[i]);
			cvReleaseImage(&m_pHSVFrames[i]);
			cvReleaseImage(&m_pHFrames[i]);
			cvReleaseImage(&m_pSFrames[i]);
			cvReleaseImage(&m_pVFrames[i]);
			cvReleaseImage(&m_pHThreshFrames[i]);
			cvReleaseImage(&m_pSThreshFrames[i]);
			cvReleaseImage(&m_pVThreshFrames[i]);
			cvReleaseImage(&m_pThreshFrames[i]);
		}
		cvReleaseImage(&m_pTempFrame1);
	}

	if(m_pUndistortMapX)
	{
		cvReleaseImage(&m_pUndistortMapX);
		cvReleaseImage(&m_pUndistortMapY);
	}

	for(int i = 0; i < 4; i++)
	{
		m_pUndistortedFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
			IPL_DEPTH_8U, 3);
		m_pThreshFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                           IPL_DEPTH_8U, 1);
		m_pHSVFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                        IPL_DEPTH_8U, 3);
		m_pHFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                      IPL_DEPTH_8U, 1);
		m_pSFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                      IPL_DEPTH_8U, 1);
		m_pVFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                      IPL_DEPTH_8U, 1);
		m_pHThreshFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                      IPL_DEPTH_8U, 1);
		m_pSThreshFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                      IPL_DEPTH_8U, 1);
		m_pVThreshFrames[i] = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight),
                                      IPL_DEPTH_8U, 1);
	}

	m_pTempFrame1 = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_8U, 1);

	m_pUndistortMapX = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_32F, 1);
	m_pUndistortMapY = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_32F, 1);

    /* Initialize the Hungarian Matcher */
    m_HungarianMatcher.doInit(m_iNObj);

    /* Initialize some of the matrices used for tracking */
    m_pQ = cvCreateMat(BELUGA_NUM_STATES, BELUGA_NUM_STATES, CV_64FC1);
    m_pR = cvCreateMat(BELUGA_NUM_MEAS, BELUGA_NUM_MEAS, CV_64FC1);
    cvSetIdentity(m_pQ);
    cvSetIdentity(m_pR);
    m_px0 = cvCreateMat(BELUGA_NUM_STATES, 1, CV_64FC1);
    m_pz = cvCreateMat(BELUGA_NUM_MEAS, 1, CV_64FC1);
    m_pu = cvCreateMat(BELUGA_NUM_INPUTS, 1, CV_64FC1);
    
    /* Create the UKF objects */
    m_vpUKF.resize(m_iNObj);
    for(int i = 0; i < m_iNObj; i++)
    {
        m_vpUKF[i] = MT_UKFInit(BELUGA_NUM_STATES,
                                BELUGA_NUM_MEAS,
                                BELUGA_UKF_ALPHA); 
    }

} /* endof createFrames */

/* this gets called during the destructor */
void BelugaTracker::releaseFrames()
{
	/* we're doing this elsewhere - it should eventually be moved here */
}

/* Initialize a data file for output.  Gets called from the GUI or
 * command line */
void BelugaTracker::initDataFile()
{
    /* An XDF can store parameters in XML format and manages data
       output to other files that are keyed in the master XML file.

       These can then be read by e.g. MATLAB */
    char v[10];
    
    sprintf(v, "%d", m_iNObj);
    m_XDF.writeParameterToXML("Number of Objects Tracked", v);    

    m_XDF.addDataStream("Blob X", "blob_x.dat");
    m_XDF.addDataStream("Blob Y", "blob_y.dat");
    m_XDF.addDataStream("Blob Area", "blob_area.dat");
    m_XDF.addDataStream("Blob Orientation", "blob_orientation.dat");

    m_XDF.addDataStream("Time", "time_stamp.dat");

    m_XDF.addDataStream("Tracked X", "tracked_x.dat");
    m_XDF.addDataStream("Tracked Y", "tracked_y.dat");  
    m_XDF.addDataStream("Tracked Z", "tracked_z.dat");
    m_XDF.addDataStream("Tracked ZDot", "tracked_zdot.dat");
    m_XDF.addDataStream("Tracked Speed", "tracked_speed.dat");    
    m_XDF.addDataStream("Tracked Heading", "tracked_heading.dat");
    m_XDF.addDataStream("Tracked Omega", "tracked_omega.dat");

	m_XDF.addDataStream("Tracked Camera X", "tracked_camera_x.dat");
	m_XDF.addDataStream("Tracked Camera Y", "tracked_camera_y.dat");

	m_XDF.addDataStream("Depth Measurement", "depth_meas.dat");
	m_XDF.addDataStream("Speed Command", "cmd_speed.dat");
    m_XDF.addDataStream("Vertical Command", "cmd_vert.dat");
    m_XDF.addDataStream("Turn Command", "cmd_turn.dat");
    
    MT_TrackerBase::initDataFile();
}

/* a helper function to write data to a file */
void BelugaTracker::writeData()
{
    /* the XDF object handles writing data to the right files - all we
       have to do is pass the data as vectors */
	std::vector<double> tmp1, tmp2, tmp3, tmp4;
	tmp1.resize(0);
	tmp1.push_back(m_dCurrentTime);
    m_XDF.writeData("Time"             , tmp1);

    tmp1.resize(4*m_iNObj);
	tmp2.resize(4*m_iNObj);
	tmp3.resize(4*m_iNObj);
	tmp4.resize(4*m_iNObj);

    for(int i = 0; i < m_iNObj; i++)
    {
        for(unsigned int c = 0; c < 4; c++)
        {
            int k = indexInVector(m_vviMeas_Cam[i], c);
            if(k < 0)
            {
                tmp1[4*i + c] = -1;
                tmp2[4*i + c] = -1;
                tmp3[4*i + c] = -1;
                tmp4[4*i + c] = -1;                
            }
            else
            {
                tmp1[4*i + c] = m_vvdMeas_X[i][k];
                tmp2[4*i + c] = m_vvdMeas_Y[i][k];
                tmp3[4*i + c] = m_vvdMeas_Hdg[i][k];
                tmp4[4*i + c] = m_vviMeas_A[i][k];                
            }
        }
    }
    
	m_XDF.writeData("Blob X"		   , tmp1);
	m_XDF.writeData("Blob Y"		   , tmp2);
	m_XDF.writeData("Blob Orientation" , tmp3);     
	m_XDF.writeData("Blob Area"		   , tmp4); 

    m_XDF.writeData("Tracked X"        , m_vdTracked_X); 
    m_XDF.writeData("Tracked Y"        , m_vdTracked_Y); 
    m_XDF.writeData("Tracked Z"        , m_vdTracked_Z);
    m_XDF.writeData("Tracked ZDot"     , m_vdTracked_ZDot);
    m_XDF.writeData("Tracked Speed"    , m_vdTracked_Speed);     
    m_XDF.writeData("Tracked Heading"  , m_vdTracked_Heading); 
    m_XDF.writeData("Tracked Omega"    , m_vdTracked_Omega); 

	tmp1.resize(0);
	tmp2.resize(0);
	for(unsigned int i = 0; i < m_vdaTracked_XC[0].size(); i++)
	{
		tmp1.push_back(m_vdaTracked_XC[0][i]);
		tmp1.push_back(m_vdaTracked_XC[1][i]);
		tmp1.push_back(m_vdaTracked_XC[2][i]);
		tmp1.push_back(m_vdaTracked_XC[3][i]);

		tmp2.push_back(m_vdaTracked_YC[0][i]);
		tmp2.push_back(m_vdaTracked_YC[1][i]);
		tmp2.push_back(m_vdaTracked_YC[2][i]);
		tmp2.push_back(m_vdaTracked_YC[3][i]);
	}
	m_XDF.writeData("Tracked Camera X", tmp1);
	m_XDF.writeData("Tracked Camera Y", tmp2);
    
	m_XDF.writeData("Depth Measurement", m_vdDepthMeasurement);
	m_XDF.writeData("Speed Command", m_vdSpeedCommand);
	m_XDF.writeData("Vertical Command", m_vdVerticalCommand);
	m_XDF.writeData("Turn Command", m_vdTurnCommand);

}

void BelugaTracker::setMasks(const char* maskfile1,
		const char* maskfile2,
		const char* maskfile3,
		const char* maskfile4)
{
	if(!maskfile1 || !maskfile2 || !maskfile3 || !maskfile4)
	{
		return;
	}

	const char* files[4] = {maskfile1, maskfile2, maskfile3, maskfile4};

	for(int i = 0; i < 4; i++)
	{
		if(m_pMasks[i])
		{
			cvReleaseImage(&m_pMasks[i]);
		}

		m_pMasks[i] = cvLoadImage(files[i], CV_LOAD_IMAGE_GRAYSCALE);
		if(m_pMasks[i] == 0)
		{
			fprintf(stderr, "BelugaTracker Error:  Unable to load mask file %s\n", files[i]);
		}
		else
		{
			if(m_pMasks[i]->width != m_pHSVFrames[i]->width || m_pMasks[i]->height != m_pHSVFrames[i]->height)
			{
				fprintf(stderr, "BelugaTracker Error:  Mask file %s has the wrong size\n", files[i]);
				m_pMasks[i] = NULL;
			}
		}
	}
}

void BelugaTracker::setCalibrations(const char* calibfile1,
		const char* calibfile2,
		const char* calibfile3,
		const char* calibfile4)
{
	if(!calibfile1 || !calibfile2 || !calibfile3 || !calibfile4)
	{
		return;
	}

	const char* files[4] = {calibfile1, calibfile2, calibfile3, calibfile4};

	for(int i = 0; i < 4; i++)
	{
		Beluga_CalibrationDataFile f(files[i]);
		if(!f.didLoadOK())
		{
			fprintf(stderr, "BelugaTracker Error:  Unable to load calibration %s", files[i]);
		}
		else
		{
			CalibrationData d;
			f.getCalibration(&d);

			if(!m_pCameraMatrices[i])
			{
				m_pCameraMatrices[i] = cvCreateMat(3, 3, CV_32FC1);
				m_pDistortionCoeffs[i] = cvCreateMat(5, 1, CV_32FC1);
				m_pRotationVectors[i] = cvCreateMat(3, 1, CV_32FC1);
				m_pRotationMatrices[i] = cvCreateMat(3, 3, CV_32FC1);
				m_pTranslationVectors[i] = cvCreateMat(3, 1, CV_32FC1);
			}

            m_CoordinateTransforms[i].setCalibrationAndWaterDepth(d, m_dWaterDepth);

			if(!calibrationDataToOpenCVCalibration(d,
				m_pCameraMatrices[i],
				m_pDistortionCoeffs[i],
				m_pRotationVectors[i],
				m_pTranslationVectors[i],
				m_pRotationMatrices[i]))
			{
				fprintf(stderr, "BelugaTracker Error:  "
                        "Unable to copy calibration from %s", files[i]);
				cvReleaseMat(&m_pCameraMatrices[i]);
				cvReleaseMat(&m_pDistortionCoeffs[i]);
				cvReleaseMat(&m_pRotationVectors[i]);
				cvReleaseMat(&m_pRotationMatrices[i]);
				cvReleaseMat(&m_pTranslationVectors[i]);
				m_pCameraMatrices[i] = NULL;
				m_pDistortionCoeffs[i] = NULL;
				m_pRotationVectors[i] = NULL;
				m_pTranslationVectors[i] = NULL;
				m_pRotationMatrices[i] = NULL;
			}

		}
	}

	if(m_pCameraMatrices[0])
	{
		cvInitUndistortMap(m_pCameraMatrices[0], 
			m_pDistortionCoeffs[0],
			m_pUndistortMapX,
			m_pUndistortMapY);
	}
}

/* EVENTUALLY:  This could really be an
 *  MT_HSVSplit(IplImage* frame, IplImage* H, IplImage* S, IplImage* V) */
void BelugaTracker::HSVSplit(IplImage* frame, int i)
{
	cvCvtColor(frame, m_pHSVFrames[i], CV_RGB2HSV);
	cvSplit(m_pHSVFrames[i], 
		m_pHFrames[i], 
		m_pSFrames[i], 
		m_pVFrames[i],
		NULL);
}

void BelugaTracker::doTimeKeeping()
{
    static double t_prev = MT_getTimeSec();
    double t_now = MT_getTimeSec();
    m_dDt = t_now - t_prev;
    t_prev = t_now;
	m_dCurrentTime = t_now;
}

void BelugaTracker::updateUKFParameters()
{
    /* This checks every time step to see if the UKF parameters have
       changed and modifies the UKF structures accordingly.  This will
       also get called the first time through b/c the "Prev" values get
       set to zero initially.   There may be a more efficient way to do
       this, but because the values need to be embedded into the CvMat
       objects I'm not sure how else to do it. */ 
    if(
        m_dSigmaPosition != m_dPrevSigmaPosition ||
        m_dSigmaZ != m_dPrevSigmaZ ||
        m_dSigmaZDot != m_dPrevSigmaZDot ||
        m_dSigmaSpeed != m_dPrevSigmaSpeed ||        
        m_dSigmaHeading != m_dPrevSigmaHeading ||
        m_dSigmaOmega != m_dPrevSigmaOmega ||
        m_dSigmaPositionMeas != m_dPrevSigmaPositionMeas ||
        m_dSigmaZMeas != m_dPrevSigmaZMeas ||
        m_dSigmaHeadingMeas != m_dPrevSigmaHeadingMeas
        )
    {
        /* these are the diagonal entries of the "Q" matrix, which
           represents the variances of the process noise.  They're
           modeled here as being independent and uncorrellated. */
        cvSetReal2D(m_pQ, BELUGA_STATE_X, BELUGA_STATE_X, m_dSigmaPosition*m_dSigmaPosition);
        cvSetReal2D(m_pQ, BELUGA_STATE_Y, BELUGA_STATE_Y, m_dSigmaPosition*m_dSigmaPosition);
        cvSetReal2D(m_pQ, BELUGA_STATE_Z, BELUGA_STATE_Z, m_dSigmaZ*m_dSigmaZ);
        cvSetReal2D(m_pQ, BELUGA_STATE_ZDOT, BELUGA_STATE_ZDOT, m_dSigmaZDot*m_dSigmaZDot);
        cvSetReal2D(m_pQ, BELUGA_STATE_SPEED, BELUGA_STATE_SPEED, m_dSigmaSpeed*m_dSigmaSpeed);
        cvSetReal2D(m_pQ, BELUGA_STATE_THETA, BELUGA_STATE_THETA, m_dSigmaHeading*m_dSigmaHeading);
        cvSetReal2D(m_pQ, BELUGA_STATE_OMEGA, BELUGA_STATE_OMEGA, m_dSigmaOmega*m_dSigmaOmega);

        /* these are the diagonal entries of the "R matrix, also
           assumed to be uncorrellated. */
        cvSetReal2D(m_pR, BELUGA_MEAS_X, BELUGA_MEAS_X, m_dSigmaPositionMeas*m_dSigmaPositionMeas);
        cvSetReal2D(m_pR, BELUGA_MEAS_Y, BELUGA_MEAS_Y, m_dSigmaPositionMeas*m_dSigmaPositionMeas);
        cvSetReal2D(m_pR, BELUGA_MEAS_THETA, BELUGA_MEAS_THETA,
                    m_dSigmaHeadingMeas*m_dSigmaHeadingMeas);
        cvSetReal2D(m_pR, BELUGA_MEAS_Z, BELUGA_MEAS_Z, m_dSigmaZMeas*m_dSigmaZMeas);

		/* makes sure we only have to copy the numbers once - it will
		 * automatically get copied again later if necessary */
		adjustRMatrixAndZForMeasurementSize(m_pR, m_pz, 1);
		for(int i = 0; i < m_iNObj; i++)
		{
			m_vpUKF[i]->m = m_pR->rows;
		}

		m_dPrevSigmaPosition = m_dSigmaPosition;
        m_dPrevSigmaZ = m_dSigmaZ;
        m_dPrevSigmaZDot = m_dSigmaZDot;
		m_dPrevSigmaSpeed = m_dSigmaSpeed;        
		m_dPrevSigmaHeading = m_dSigmaHeading;
        m_dPrevSigmaOmega = m_dSigmaOmega;
		m_dPrevSigmaPositionMeas = m_dSigmaPositionMeas;
        m_dPrevSigmaZMeas = m_dSigmaZMeas;
		m_dPrevSigmaHeadingMeas = m_dSigmaHeadingMeas;

        /* this step actually copies the Q and R matrices to the UKF
           and makes sure that it's internals are properly initialized -
           it's set up to handle the fact that the sizes of these
           matrices could have changed. */
        for(int i = 0; i < m_iNObj; i++)
        {
            MT_UKFCopyQR(m_vpUKF[i], m_pQ, m_pR);
        }
    }
    
}

void BelugaTracker::doImageProcessingInCamera(unsigned int cam_number,
                                              const IplImage* frame)
{
    /* correct for barrel distortion */
    cvRemap(frame,
            m_pUndistortedFrames[cam_number],
            m_pUndistortMapX,
            m_pUndistortMapY);

    /* split into HSV frames */
    HSVSplit(m_pUndistortedFrames[cam_number], cam_number);

    /* just for display - the "grayscale" image is the V channel */
    m_pGSFrames[cam_number] = m_pVFrames[cam_number];

    /* Tv = V < Vthresh */
    cvThreshold(m_pVFrames[cam_number],
                m_pVThreshFrames[cam_number],
                m_iVThresh, 255, CV_THRESH_BINARY_INV);

    /* Th = (H > H_hi) OR (H < H_lo)
     *          i.e., reject between H_lo and H_hi */
    cvThreshold(m_pHFrames[cam_number],
                m_pHThreshFrames[cam_number],
                m_iHThresh_High, 255, CV_THRESH_BINARY);
    cvThreshold(m_pHFrames[cam_number],
                m_pTempFrame1,
                m_iHThresh_Low, 255, CV_THRESH_BINARY_INV);
    cvOr(m_pHThreshFrames[cam_number], m_pTempFrame1, m_pHThreshFrames[cam_number]);

    /* Ts = (S > S_lo) AND (S < S_hi)
     *         i.e., S_lo < S < S_hi */
    cvThreshold(m_pSFrames[cam_number],
                m_pSThreshFrames[cam_number],
                m_iSThresh_Low, 255, CV_THRESH_BINARY);
    cvThreshold(m_pSFrames[cam_number],
                m_pTempFrame1,
                m_iSThresh_High, 255, CV_THRESH_BINARY_INV);
    cvAnd(m_pSThreshFrames[cam_number], m_pTempFrame1, m_pSThreshFrames[cam_number]);

    /* T = Tv AND Th AND Tv */
    cvAnd(m_pVThreshFrames[cam_number], m_pHThreshFrames[cam_number], m_pTempFrame1);
    cvAnd(m_pTempFrame1, m_pSThreshFrames[cam_number], m_pThreshFrames[cam_number]);

    /* apply a median filter to get rid of "salt and pepper" noise */
    cvSmooth(m_pThreshFrames[cam_number], m_pThreshFrames[cam_number], CV_MEDIAN, 3);

    /* apply the mask image */
    if(m_pMasks[cam_number])
    {
        cvAnd(m_pThreshFrames[cam_number],
              m_pMasks[cam_number],
              m_pThreshFrames[cam_number]);
    }
}

void BelugaTracker::notifyNoMeasurement(unsigned int obj_num)
{
    m_vbValidMeasCamObj[m_iCurrentCamera][obj_num] = false;
}

void BelugaTracker::doBlobFindingInCamera(unsigned int cam_number)
{

    m_iCurrentCamera = cam_number;
    
    BelugaSegmenter segmenter(this);
	/* uncomment to see debugging info for the segmenter module */
    //segmenter.setDebugFile(stdout);
    segmenter.m_iMinBlobPerimeter = 1;
    segmenter.m_iMinBlobArea = m_iBlobAreaThreshLow;
    segmenter.m_iMaxBlobArea = m_iBlobAreaThreshHigh;
    segmenter.m_dOverlapFactor = m_dOverlapFactor;

    if(m_bNoHistory)
    {
        /* on the first frame, we'll just find what we can in this
           image and  copy them directly into the blobs */
        std::vector<YABlob> yblobs = m_YABlobber.FindBlobs(m_pThreshFrames[cam_number],
                                                         1, /* min perimeter */
                                                         m_iBlobAreaThreshLow,
                                                         -1, /* max perimeter (no limit) */
                                                         m_iBlobAreaThreshHigh);
        m_vBlobs[cam_number].resize(yblobs.size());
        for(unsigned int blob = 0; blob < yblobs.size(); blob++)
        {
            m_vBlobs[cam_number][blob] = MT_DSGYA_Blob(yblobs[blob]);
        }
    }
    else
    {
        generatePredictedBlobs(cam_number);
        
        /* by default, all measurements are valid */
        m_vbValidMeasCamObj[cam_number].assign(m_iNObj, true);

        m_vBlobs[cam_number] = segmenter.doSegmentation(m_pThreshFrames[cam_number],
                                                        m_vPredictedBlobs[cam_number]);

		m_viAssignments[cam_number] =
            segmenter.getAssignmentVector(&m_iAssignmentRows[cam_number],
                                          &m_iAssignmentCols[cam_number]);
        m_vInitBlobs[cam_number] = segmenter.getInitialBlobs();
    }
}

void BelugaTracker::generatePredictedBlobs(unsigned int cam_number)
{
    MT_DSGYA_Blob* pb;
    for(int obj = 0; obj < m_iNObj; obj++)
    {
        pb = &m_vPredictedBlobs[cam_number][obj];
        pb->m_dXCenter = m_vdaTracked_XC[cam_number][obj];
        pb->m_dYCenter = m_vdaTracked_YC[cam_number][obj];
        pb->m_dOrientation = m_vdTracked_Heading[obj];
    }
}

void BelugaTracker::calculateTrackedPositionsInCameras()
{
	double u, v;
	for(unsigned int c = 0; c < 4; c++)
	{
		for(int obj = 0; obj < m_iNObj; obj++)
		{
			calculatePositionOfObjectInCamera(obj, c, &u, &v);
			/* for display */
			m_vdaTracked_XC[c][obj] = u;
			m_vdaTracked_YC[c][obj] = v;
		}
	}
}

void BelugaTracker::calculatePositionOfObjectInCamera(unsigned int obj_num, unsigned int cam_num, double* u, double* v)
{
	double x, y, z;
	x = y = z = 0;
	x = m_vdTracked_X[obj_num];
	y = m_vdTracked_Y[obj_num];
	z = m_vdTracked_Z[obj_num];
        
	m_CoordinateTransforms[cam_num].worldToImage(x, y, z, u, v, false);
	*v = VFLIP *v;
}

void BelugaTracker::getObjectMeasurements(unsigned int obj_number)
{
    m_vvdMeas_X[obj_number].resize(0);
    m_vvdMeas_Y[obj_number].resize(0);
    m_vvdMeas_Hdg[obj_number].resize(0);
    m_vviMeas_A[obj_number].resize(0);
    m_vviMeas_Cam[obj_number].resize(0);    

    for(unsigned int c = 0; c < 4; c++)
    {
        if(m_vbValidMeasCamObj[c][obj_number])
        {
            /* valid measurement */
            MT_DSGYA_Blob* p_blob = &m_vBlobs[c][obj_number];
            m_vvdMeas_X[obj_number].push_back(p_blob->m_dXCenter);
            m_vvdMeas_Y[obj_number].push_back(p_blob->m_dYCenter);
            m_vvdMeas_Hdg[obj_number].push_back(p_blob->m_dOrientation);
            m_vviMeas_A[obj_number].push_back(p_blob->m_dArea);
            m_vviMeas_Cam[obj_number].push_back(c);
        }
        else
        {
            /* no measurement */
            /* MAYBE: there's no measurement in this camera - do we
               need to do anything with that? I don't think so... */
        }
    }

}

void BelugaTracker::updateObjectState(unsigned int obj_number)
{
    unsigned int nmeas = m_vvdMeas_X[obj_number].size();

    if(nmeas)
    {
        adjustRMatrixAndZForMeasurementSize(m_pR, m_pz, nmeas);
        MT_UKFCopyQR(m_vpUKF[obj_number], m_pQ, m_pR);
        m_vpUKF[obj_number]->m = m_pR->rows;
    }

    bool valid_meas = (!m_bNoHistory) && nmeas > 0;

    /* if any state is NaN, reset the UKF
     * This shouldn't happen anymore, but it's a decent safety
     * check.  It could probably be omitted if we want to
     * optimize for speed... */
    if(!m_bNoHistory &&
       (!CvMatIsOk(m_vpUKF[obj_number]->x) ||
        !CvMatIsOk(m_vpUKF[obj_number]->P)))
    {
        MT_UKFFree(&(m_vpUKF[obj_number]));
        m_vpUKF[obj_number] = MT_UKFInit(BELUGA_NUM_STATES,
                                         m_pR->rows,
                                         BELUGA_UKF_ALPHA);
        MT_UKFCopyQR(m_vpUKF[obj_number], m_pQ, m_pR);
        valid_meas = false;
    }

    m_vbLastMeasValid[obj_number] = valid_meas;

    /* if we're going to accept this measurement */
    if(valid_meas)
    {
        applyUKFToObject(obj_number);
    }
    else
    {
        // on the first frame, take the average of the measurements
        // MAYBE:  Do we ever actually get here?
        if(m_bNoHistory)
        {
            if(nmeas > 0)
            {
                useAverageMeasurementForObject(obj_number);
            }
            else
            {
                /* should never get here */
            }
        }
        else
        {
            usePredictedStateForObject(obj_number);
        }
    }
    
}

void BelugaTracker::applyUKFToObject(unsigned int obj_number)
{

    unsigned int nmeas = m_vvdMeas_X[obj_number].size();

#ifdef DEBUG_VERBOSE
	printf("Applying UKF to object %d with %d measurements\n", obj_number, nmeas);
#endif // DEBUG_VERBOSE

    /* build the control input vector to be used in the UKF */
    cvSetReal2D(m_pu, BELUGA_INPUT_VERTICAL_SPEED, 0, m_vdVerticalCommand[obj_number]);
    cvSetReal2D(m_pu, BELUGA_INPUT_FORWARD_SPEED,  0, m_vdSpeedCommand[obj_number]);
    cvSetReal2D(m_pu, BELUGA_INPUT_STEERING,       0, m_vdTurnCommand[obj_number]);

    /* UKF prediction step, note we use function pointers to
       the beluga_dynamics and beluga_measurement functions defined
       above.  */
    BelugaDynamicsParameters::m_dDt = m_dDt;
    BelugaDynamicsParameters::m_dWaterDepth = m_dWaterDepth;
	BelugaDynamicsParameters::m_dK_t = m_vdK_t[obj_number];
	BelugaDynamicsParameters::m_dK_d1 = m_vdK_d1[obj_number];
	BelugaDynamicsParameters::m_dm_0 = m_vdm_0[obj_number];
	BelugaDynamicsParameters::m_dm_1 = m_vdm_1[obj_number];
	BelugaDynamicsParameters::m_dr_1 = m_vdr_1[obj_number];
	BelugaDynamicsParameters::m_dK_omega = m_vdK_omega[obj_number];
	BelugaDynamicsParameters::m_deta_up = m_vdeta_up[obj_number];
	BelugaDynamicsParameters::m_deta_down = m_vdeta_down[obj_number];
	BelugaDynamicsParameters::m_dv_off = m_vdv_off[obj_number];
	BelugaDynamicsParameters::m_dk_d = m_vdk_d[obj_number];
	BelugaDynamicsParameters::m_dz_off = m_vdz_off[obj_number];
	BelugaDynamicsParameters::m_dk_teth = m_vdk_teth[obj_number];
	BelugaDynamicsParameters::m_dk_vp = m_vdk_vp[obj_number];
	BelugaDynamicsParameters::m_dJ = m_vdJ[obj_number];

    MT_UKFPredict(m_vpUKF[obj_number],
                  &beluga_dynamics,
                  &beluga_measurement,
                  m_pu);

    /* build the measurement vector */
    double x, y, z, th;
    double th_prev = cvGetReal2D(m_vpUKF[obj_number]->x, BELUGA_STATE_THETA, 0);
    unsigned int cam_no;
    /* we're using the tracked Z to estimate position.  we could use
     *  the depth measurement, but that would be more prone to errors */
    double d_last = m_dWaterDepth - m_vdTracked_Z[obj_number];
    for(unsigned int j = 0; j < nmeas; j++)
    {
        cam_no = m_vviMeas_Cam[obj_number][j];
        m_CoordinateTransforms[cam_no].imageAndDepthToWorld(m_vvdMeas_X[obj_number][j],
                                                            VFLIP m_vvdMeas_Y[obj_number][j],
                                                            d_last, &x, &y, &z, false);
        th = rectifyAngleMeasurement(m_vvdMeas_Hdg[obj_number][j],
                                     m_vdHistories_X[obj_number],
                                     m_vdHistories_Y[obj_number],
                                     N_hist,
                                     th_prev);
        m_vvdMeas_Hdg[obj_number][j] = MT_RAD2DEG*th;
        cvSetReal2D(m_pz, j*(BELUGA_NUM_MEAS - 1) + BELUGA_MEAS_X, 0, x);
        cvSetReal2D(m_pz, j*(BELUGA_NUM_MEAS - 1) + BELUGA_MEAS_Y, 0, y);
        cvSetReal2D(m_pz, j*(BELUGA_NUM_MEAS - 1) + BELUGA_MEAS_THETA, 0, th);
    }
    double z_new = m_dWaterDepth - m_vdDepthMeasurement[obj_number];
    cvSetReal2D(m_pz, LAST_ROW(m_pz), 0, z_new);

    MT_UKFSetMeasurement(m_vpUKF[obj_number], m_pz);
    MT_UKFCorrect(m_vpUKF[obj_number]);

    constrain_state(m_vpUKF[obj_number]->x,
                    m_vpUKF[obj_number]->x1,
                    BELUGA_TANK_RADIUS,
                    m_dWaterDepth);
}

void BelugaTracker::useAverageMeasurementForObject(unsigned int obj_number)
{
    unsigned int nmeas = m_vvdMeas_X[obj_number].size();
    
    double xavg = 0;
    double yavg = 0;
    double zavg = 0;
    double qx = 0;
    double qy = 0;
    double x, y, z;
    unsigned int cam_no;
    double d_last = m_dWaterDepth - m_vdTracked_Z[obj_number];    
    for(unsigned int j = 0; j < nmeas; j++)
    {
        cam_no = m_vviMeas_Cam[obj_number][j];
        m_CoordinateTransforms[cam_no].imageAndDepthToWorld(m_vvdMeas_X[obj_number][j],
                                                            VFLIP m_vvdMeas_Y[obj_number][j],
                                                            d_last, &x, &y, &z, false);
        xavg += x;
        yavg += y;
        zavg += z;
        qx += cos(MT_DEG2RAD*m_vvdMeas_Hdg[obj_number][j]);
        qy += sin(MT_DEG2RAD*m_vvdMeas_Hdg[obj_number][j]);
    }
    xavg /= (double) nmeas;
    yavg /= (double) nmeas;
    zavg /= (double) nmeas;
    double th = atan2(qy, qx);

    // MAYBE:  We're using the last known depth here.  We could
    // concievably use the depth measurements, since we're calling
    // this function mainly when we don't have valid tracking data but
    // we may have valid depth data.
    
    cvSetReal2D(m_vpUKF[obj_number]->x, BELUGA_STATE_X, 0, xavg);
    cvSetReal2D(m_vpUKF[obj_number]->x, BELUGA_STATE_Y, 0, yavg);
    cvSetReal2D(m_vpUKF[obj_number]->x, BELUGA_STATE_Z, 0, zavg);
    cvSetReal2D(m_vpUKF[obj_number]->x, BELUGA_STATE_ZDOT, 0, 0);
    cvSetReal2D(m_vpUKF[obj_number]->x, BELUGA_STATE_SPEED, 0, 0);    
    cvSetReal2D(m_vpUKF[obj_number]->x, BELUGA_STATE_THETA, 0, th);
    cvSetReal2D(m_vpUKF[obj_number]->x, BELUGA_STATE_OMEGA, 0, 0);
    m_vbLastMeasValid[obj_number] = true;
}

void BelugaTracker::usePredictedStateForObject(unsigned int obj_number)
{
    // use the prediction
    cvCopy(m_vpUKF[obj_number]->x1, m_vpUKF[obj_number]->x);
}

unsigned int BelugaTracker::testInitAdjacency(double x1, double y1, double x2, double y2)
{
    double d2 = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
    
    if(d2 < m_dInitAdjacencyThresh*m_dInitAdjacencyThresh)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

bool BelugaTracker::tryInitStateVectors()
{
    bool result = false;
    MT_DSGYA_Blob* pb;

    /* world positions of every blob found */
    std::vector<double> X(0, 0);
    std::vector<double> Y(0, 0);
    std::vector<double> Xc(0, 0);
    std::vector<double> Yc(0, 0);
    std::vector<double> TH(0, 0);
    std::vector<double> A(0, 0);
    std::vector<double> C(0, 0);
    double x1, y1, z; //, th;
    double x2, y2;

    for(unsigned int c = 0; c < 4; c++)
    {
        for(unsigned int b = 0; b < m_vBlobs[c].size(); b++)
        {
            pb = &m_vBlobs[c][b];
            
            // convert image position to world, assuming depth = 0
            m_CoordinateTransforms[c].imageAndDepthToWorld(pb->m_dXCenter,
                                                           VFLIP pb->m_dYCenter,
                                                           0, &x1, &y1, &z, false);
            X.push_back(x1);
            Y.push_back(y1);

            Xc.push_back(pb->m_dXCenter);
            Yc.push_back(pb->m_dYCenter);
            TH.push_back(pb->m_dOrientation);
            A.push_back(pb->m_dArea);
            C.push_back(c);
        }
    }

    // BiCC the positions to figure out which ones are likely touching
    unsigned int rows = X.size();
    unsigned int cols = X.size();
    MT_BiCC bicc(rows, cols);
    std::vector<unsigned int> adj(rows*cols, 0);

    // build the BiCC adjacency
    for(unsigned int i = 0; i < X.size(); i++)
    {
        for(unsigned int j = 0; j < X.size(); j++)
        {
            if(i == j)
            {
                /* blob adjacent to itself => single blobs come out
                 * as CCs */
                adj[i*rows + j] = 1;
            }
            else
            {
                x1 = X[i];
                y1 = Y[i];
                x2 = X[j];
                y2 = Y[j];
                adj[i*rows + j] = testInitAdjacency(x1, y1, x2, y2);
            }
        }
    }

    int n_cc = bicc.findComponents(adj);

    /* if the # connected components matches the # of objects we're
       looking for, then we can use them! */
    if(n_cc = m_iNObj)
    {
        /* find the blobs that were involved in each CC, stuff them
           into the measurement vectors, then we'll use the averages to
           set the initial state */
        /* Note component labels 1-indexed, thus all the i-1's */
        for(int i = 1; i <= min(n_cc, (int) rows); i++)
        {
            m_vvdMeas_X[i-1].resize(0);
            m_vvdMeas_Y[i-1].resize(0);
            m_vvdMeas_Hdg[i-1].resize(0);
            m_vviMeas_A[i-1].resize(0);
            m_vviMeas_Cam[i-1].resize(0);    
            for(unsigned int k = 0; k < rows; k++)
            {
                if((int) bicc.getLabelElement(k) == i)
                {
                    m_vvdMeas_X[i-1].push_back(Xc[k]);
                    m_vvdMeas_Y[i-1].push_back(Yc[k]);
                    m_vvdMeas_Hdg[i-1].push_back(TH[k]);
                    m_vviMeas_A[i-1].push_back(A[k]);
                    m_vviMeas_Cam[i-1].push_back(C[k]);                                        
                }
            }
            useAverageMeasurementForObject(i-1);
        }

        result = true;
    }
    else
    {
        result = false;
    }
    
    return result;
}

/* Single-camera version of doTracking - should not get called */
void BelugaTracker::doTracking(IplImage* frame)
{
	std::cerr << "ERROR:  This function should never get called!  BelugaTracker::doTracking(IplImage* frame)\n";
    assert(BELUGA_DO_ASSERT);  /* assert regardless, because we've
                                * gotten here via some error */
}

/* Main tracking function - gets called by MT_TrackerFrameBase every
 * time step when the application is not paused. */
void BelugaTracker::doTracking(IplImage* frames[4])
{
    /* calculate time stamp */
    doTimeKeeping();

    /* update the frame counter */
    m_iFrameCounter++;

    /* update UKF parameters if they have changed */
    updateUKFParameters();

	/* preprocessing steps for each camera */
	for(int i = 0; i < 4; i++)
	{
        doImageProcessingInCamera(i, frames[i]);
        doBlobFindingInCamera(i);

        /* update the water depth from the GUI */
        m_CoordinateTransforms[i].setWaterDepth(m_dWaterDepth);
	}

	/* reset measurement vectors to zero */
	for(int i = 0; i < m_iNObj; i++)
	{
		m_vvdMeas_X[i].resize(0);
		m_vvdMeas_Y[i].resize(0);
		m_vviMeas_A[i].resize(0);
		m_vviMeas_Cam[i].resize(0);
		m_vvdMeas_Hdg[i].resize(0);
	}

    if(m_bNoHistory)
    {
        /* try to set the state vectors using what we have */
        m_bNoHistory = !tryInitStateVectors();

        //x = m_px0; // TODO fix?
    }
    else
    {
        /* filtering */    
        for(int i = 0; i < m_iNObj; i++)
        {
            /* have a history and can use the UKF */
            getObjectMeasurements(i);
        
            updateObjectState(i);

            rollHistories(&m_vdHistories_X[i], 
                          &m_vdHistories_Y[i], 
                          cvGetReal2D(m_vpUKF[i]->x, 0, 0),
                          cvGetReal2D(m_vpUKF[i]->x, 1, 0),
                          N_hist);

        }
    }

	if(!m_bNoHistory)
	{
		for(int i = 0; i < m_iNObj; i++)
		{
            /* grab the state estimate and store it in variables that will
               make it convenient to save it to a file. */
            CvMat* x = m_vpUKF[i]->x;

            // MAYBE: move outside if
            m_vdTracked_X[i] = cvGetReal2D(x, BELUGA_STATE_X, 0);
            m_vdTracked_Y[i] = cvGetReal2D(x, BELUGA_STATE_Y, 0);
            m_vdTracked_Z[i] = cvGetReal2D(x, BELUGA_STATE_Z, 0);
            m_vdTracked_ZDot[i] = cvGetReal2D(x, BELUGA_STATE_ZDOT, 0);
            m_vdTracked_Speed[i] = cvGetReal2D(x, BELUGA_STATE_SPEED, 0);            
            m_vdTracked_Heading[i] = cvGetReal2D(x, BELUGA_STATE_THETA, 0);
            m_vdTracked_Omega[i] = cvGetReal2D(x, BELUGA_STATE_OMEGA, 0);

#ifdef DEBUG_VERBOSE
			dumpStates(i);
#endif // DEBUG_VERBOSE

		}

		calculateTrackedPositionsInCameras();
	}

	writeData();

}

void BelugaTracker::dumpStates(unsigned int i)
{
	CvMat* xp = m_vpUKF[i]->x1;
	printf("\t\t x\ty\tz\tzdot\tspd\ttheta\tomega\n");
	printf("T:\t\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n", 
		m_vdTracked_X[i], m_vdTracked_Y[i], m_vdTracked_Z[i], m_vdTracked_ZDot[i],
		m_vdTracked_Speed[i], m_vdTracked_Heading[i], m_vdTracked_Omega[i]);
	printf("P:\t\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n", 
		cvGetReal2D(xp, BELUGA_STATE_X, 0), 
		cvGetReal2D(xp, BELUGA_STATE_Y, 0), 
		cvGetReal2D(xp, BELUGA_STATE_Z, 0), 
		cvGetReal2D(xp, BELUGA_STATE_ZDOT, 0), 
		cvGetReal2D(xp, BELUGA_STATE_SPEED, 0), 
		cvGetReal2D(xp, BELUGA_STATE_THETA, 0), 
		cvGetReal2D(xp, BELUGA_STATE_OMEGA, 0));
}

std::vector<double> BelugaTracker::getBelugaState(unsigned int i)
{
	std::vector<double> r;
	r.resize(0);

	if((int) i >= m_iNObj || i >= m_vdTracked_X.size())
	{
		fprintf(stderr, "BelugaTracker Error:  State request out of range; returning empty state.\n");
		return r;
	}

	r.resize(BELUGA_NUM_STATES);
	r[BELUGA_STATE_X] = m_vdTracked_X[i];
	r[BELUGA_STATE_Y] = m_vdTracked_Y[i];
	r[BELUGA_STATE_Z] = m_vdTracked_Z[i];
    r[BELUGA_STATE_ZDOT] = m_vdTracked_ZDot[i];
	r[BELUGA_STATE_SPEED] = m_vdTracked_Speed[i];    
	r[BELUGA_STATE_THETA] = m_vdTracked_Heading[i];
	r[BELUGA_STATE_OMEGA] = m_vdTracked_Omega[i];

	return r;
}

void BelugaTracker::setRobotData(const std::vector<double>& depth_meas,
		const std::vector<double>& speed,
		const std::vector<double>& vert,
		const std::vector<double>& turn,
		const std::vector<double>& vdK_t,
		const std::vector<double>& vdK_d1,
		const std::vector<double>& vdm_0,
		const std::vector<double>& vdm_1,
		const std::vector<double>& vdr_1,
		const std::vector<double>& vdK_omega,
		const std::vector<double>& vdeta_up,
		const std::vector<double>& vdeta_down,
		const std::vector<double>& vdv_off,
		const std::vector<double>& vdk_d,
		const std::vector<double>& vdz_off,
		const std::vector<double>& vdk_teth,
		const std::vector<double>& vdk_vp,
		const std::vector<double>& vdJ)
{
	m_vdDepthMeasurement = depth_meas;
	m_vdSpeedCommand = speed;
	m_vdVerticalCommand = vert;
	m_vdTurnCommand = turn;
	m_vdK_t = vdK_t;
	m_vdK_d1 = vdK_d1;
	m_vdm_0 = vdm_0;
	m_vdm_1 = vdm_1;
	m_vdr_1 = vdr_1;
	m_vdK_omega = vdK_omega;
	m_vdeta_up = vdeta_up;
	m_vdeta_down = vdeta_down;
	m_vdv_off = vdv_off;
	m_vdk_d = vdk_d;
	m_vdz_off = vdz_off;
	m_vdk_teth = vdk_teth;
	m_vdk_vp = vdk_vp;
	m_vdJ = vdJ;
}

void BelugaTracker::getWorldXYZFromImageXYAndDepthInCamera(double* x,
		double* y,
		double* z,
		double u,
		double v,
		double d,
		bool undistort,
		unsigned int camera)
{
	if(camera >= 4 || !x || !y || !z)
	{
		return;
	}

	m_CoordinateTransforms[camera].imageAndDepthToWorld(u, VFLIP v, d, x, y, z, undistort);

}

void BelugaTracker::getCameraXYFromWorldXYandDepth(int* camera, double* u, double* v, double x, double y, double depth, bool distort)
{
	if(!u || !v)
	{
		return;
	}

	if(x >= 0 && y >= 0)
	{
		*camera = 0;
	}
	if(x < 0 && y >= 0)
	{
		*camera = 1;
	}
	if(x <0 && y < 0)
	{
		*camera = 2;
	}
	if(x >= 0 && y < 0)
	{
		*camera = 3;
	}

	m_CoordinateTransforms[*camera].worldToImage(x, y, m_dWaterDepth-depth, u, v, distort);
	*v = VFLIP *v;
}

void BelugaTracker::getCameraXYFromWorldXYandDepthFixedCamera(int camera,
                                                              double* u,
                                                              double* v,
                                                              double x,
                                                              double y,
                                                              double depth,
                                                              bool distort)
{
	if(!u || !v)
	{
		return;
	}

	m_CoordinateTransforms[camera].worldToImage(x, y, m_dWaterDepth-depth, u, v, distort);
	*v = VFLIP *v;
}

/* Drawing function - gets called by the GUI
 * All of the drawing is done with OpenGL */
void BelugaTracker::doGLDrawing(int flags)
{
	unsigned int Q = flags;

	switch(flags)
	{
	case 1:
		// Q2 Drawing
		break;
	case 2:
		// Q3 Drawing
		break;
	case 3:
		// Q4 Drawing
		break;
	default:
		Q = 0;
		// Q1 Drawing
		break;
	}

    /* MT_R3 is a 3-vector class, used here for convenience */
    MT_R3 blobcenter;

    /* m_bShowBlobs is a boolean in the "Drawing Options" data group.
       If the user selects "Drawing Options" from the "Tracking" menu,
       a dialog will pop up with a check box labeled "Show blob arrows",
       and its state will be linked (via a pointer) to the value of this
       variable */
    if(m_bShowBlobs)
    {
        for (unsigned int i = 0; i < m_vBlobs[Q].size(); i++)
        {

            /* note that we have to subtract y from the frame_height
               to account for the window coordinates originating from the
               bottom left but the image coordinates originating from
               the top left */
            blobcenter.setx(m_vBlobs[Q].at(i).m_dXCenter);
            blobcenter.sety(VFLIP m_vBlobs[Q].at(i).m_dYCenter);
            blobcenter.setz( 0 );

            /* draws an arrow using OpenGL */
            MT_DrawArrow(blobcenter,  // center of the base of the arrow
                         20.0,        // arrow length (pixels)
                         MT_DEG2RAD*m_vBlobs[Q].at(i).m_dOrientation, // arrow angle
                         MT_White,//MT_Primaries[(i+1) % MT_NPrimaries], // color
                         1.0 // fixes the arrow width
                );    
        }

		for(unsigned int i = 0; i < m_vInitBlobs[Q].size(); i++)
		{
            /* note that we have to subtract y from the frame_height
               to account for the window coordinates originating from the
               bottom left but the image coordinates originating from
               the top left */
            blobcenter.setx(m_vInitBlobs[Q].at(i).COMx);
            blobcenter.sety(VFLIP m_vInitBlobs[Q].at(i).COMy);
            blobcenter.setz( 0 );

            /* draws an arrow using OpenGL */
            MT_DrawArrow(blobcenter,  // center of the base of the arrow
                         20.0,        // arrow length (pixels)
                         MT_DEG2RAD*m_vInitBlobs[Q].at(i).orientation, // arrow angle
                         MT_Blue,//MT_Primaries[(i+1) % MT_NPrimaries], // color
                         1.0 // fixes the arrow width
                );   
		}
    }

    /* essentially the same as above, but with the tracked positions
       instead of the blob positions */
    if(m_bShowTracking)
    {
        for (unsigned int i = 0; i < m_vdTracked_X.size(); i++)
        {

            blobcenter.setx(m_vdaTracked_XC[Q][i]);
            blobcenter.sety(VFLIP m_vdaTracked_YC[Q][i]);
            blobcenter.setz( 0 );

            MT_DrawArrow(blobcenter,
                         25.0, 
                         m_vdTracked_Heading[i],
                         MT_Red,//MT_Primaries[(i+1) % MT_NPrimaries],
                         1.0 
                );    
        }
    }

	for(int i = 0; i < m_iNObj; i++)
	{
		for(unsigned int j = 0; j < m_vvdMeas_X[i].size(); j++)
		{
			if(m_vviMeas_Cam[i][j] == Q)
			{
				blobcenter.setx(m_vvdMeas_X[i][j]);
				blobcenter.sety(VFLIP m_vvdMeas_Y[i][j]);
				blobcenter.setz(0);

				MT_DrawArrow(blobcenter,
					10.0,
					MT_DEG2RAD*m_vvdMeas_Hdg[i][j],
					MT_Green,//MT_Primaries[(i+1) % MT_NPrimaries],
					0.8f);
			}
		}
	}

	for(unsigned int i = 0; i < m_SearchArea[Q].size(); i++)
	{
		MT_DrawRectangle(m_SearchArea[Q][i].x,
     		// if VFLIP is m_iFrameHeight -, then VFLIP m_iFrameHeight = 0
			VFLIP m_iFrameHeight ? m_SearchArea[Q][i].y : 
			   VFLIP m_SearchArea[Q][i].y - m_SearchArea[Q][i].height,
			m_SearchArea[Q][i].width, 
			m_SearchArea[Q][i].height);
	}

	//

}

