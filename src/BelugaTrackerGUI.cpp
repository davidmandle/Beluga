#include "BelugaTrackerGUI.h"

#include "BelugaVideoSetupDialog.h"

#include "BelugaConstants.h"

const std::string RobotNames[] = {"Beluga 1",
                                  "Beluga 2",
                                  "Beluga 3",
                                  "Beluga 4",
                                  "Other 1",
                                  "Other 2",
                                  "Other 3"};


const int NO_ROBOT = -1;

const unsigned int FRAME_PERIOD_MSEC = 50;

enum
{
    ID_MENU_POP_ROBOT = MT_RFB_ID_HIGHEST + 10,
    ID_MENU_FILE_CAM_SETUP
};

/**********************************************************************
 * GUI Frame Class
 *********************************************************************/

BelugaTrackerFrame::BelugaTrackerFrame(wxFrame* parent,
                               wxWindowID id,
                               const wxString& title, 
                               const wxPoint& pos, 
                               const wxSize& size,     
                               long style)
  : MT_RobotFrameBase(parent, id, title, pos, size, style),
    m_iNToTrack(1),
	m_dGotoDist(50.0),
	m_dGotoMaxSpeed(15.0),
	m_dGotoTurningGain(25.0),
	m_bControlActive(false),
	m_iGrabbedTrackedObj(NO_ROBOT),
	m_bGotoActive(false),
	m_iGotoCam(0),
	m_dGotoXC(0),
	m_dGotoYC(0),
	m_dGotoXW(0),
	m_dGotoYW(0),
	m_bCamerasReady(false),
	m_bConnectSocket(false),
    m_Controller(4 /* # robots */),
    m_IPCClient("127.0.0.1", 1234),
    m_pBelugaControlFrame(NULL)
{
	for(unsigned int i = 0; i < 4; i++)
	{
        m_pSlaves[i] = NULL;
        m_apWaypointController[i] = NULL;
        m_apLowLevelController[i] = NULL;
	}
	m_pBelugaTracker = NULL;
}

BelugaTrackerFrame::~BelugaTrackerFrame()
{
    for(unsigned int i = 0; i < 4; i++)
    {
        if(m_apWaypointController[i])
        {
            delete m_apWaypointController[i];
            m_apWaypointController[i] = NULL;
        }
        if(m_apLowLevelController[i])
        {
            delete m_apLowLevelController[i];
            m_apLowLevelController[i] = NULL;
        }
    }
}

void BelugaTrackerFrame::doUserQuit()
{
	m_bCamerasReady = false;
	doPause();
	m_pTimer->Stop();
    this->stopTimedEvents();

	for(unsigned int i = 1; i < 4; i++)
	{
		if(m_pSlaves[i])
		{
			m_pSlaves[i]->setImage(NULL);
			m_pSlaves[i]->prepareToClose();

            m_pSlaves[i]->Destroy();
            m_pSlaves[i] = NULL;
		}
	}

	MT_RobotFrameBase::doUserQuit();
}

void BelugaTrackerFrame::initUserData()
{

    for(unsigned int i = 0; i < 4; i++)
    {
        m_uiaIndexMap[i] = i;
    }
    
    MT_RobotFrameBase::initUserData();
    
    m_CmdLineParser.AddOption(wxT("n"),
                              wxT("num_to_track"),
                              wxT("Number of objects to track. Default is 3."),
                              wxCMD_LINE_VAL_NUMBER);

	m_pPreferences->AddDouble("Goto Cutoff Distance",
                              &m_dGotoDist,
                              MT_DATA_READWRITE,
                              0);
	m_pPreferences->AddDouble("Goto Max Speed",
                              &m_dGotoMaxSpeed,
                              MT_DATA_READWRITE,
                              0);
	m_pPreferences->AddDouble("Goto Turning Gain",
                              &m_dGotoTurningGain,
                              MT_DATA_READWRITE,
                              0);

    std::vector<std::string> botnames;
    for(unsigned int i = 0; i < 7; i++)
    {
        botnames.push_back(RobotNames[i]);
    }
    m_Robots.SetRobotNames(botnames);

    m_pSetupInfo = new MT_DataGroup("Camera Setup Info");
    m_pSetupInfo->AddString("Quadrant I Calibration",
                            &m_sQuad1CalibrationPath);
    m_pSetupInfo->AddString("Quadrant I Camera",
                            &m_sQuad1Camera);
    m_pSetupInfo->AddString("Quadrant II Calibration",
                            &m_sQuad2CalibrationPath);
    m_pSetupInfo->AddString("Quadrant II Camera",
                            &m_sQuad2Camera);
    m_pSetupInfo->AddString("Quadrant III Calibration",
                            &m_sQuad3CalibrationPath);
    m_pSetupInfo->AddString("Quadrant III Camera",
                            &m_sQuad3Camera);
    m_pSetupInfo->AddString("Quadrant IV Calibration",
                            &m_sQuad4CalibrationPath);
    m_pSetupInfo->AddString("Quadrant IV Camera",
                            &m_sQuad4Camera);
    m_pSetupInfo->AddString("Quadrant I Mask",
                            &m_sQuad1MaskPath);
    m_pSetupInfo->AddString("Quadrant II Mask",
                            &m_sQuad2MaskPath);
    m_pSetupInfo->AddString("Quadrant III Mask",
                            &m_sQuad3MaskPath);
    m_pSetupInfo->AddString("Quadrant IV Mask",
                            &m_sQuad4MaskPath);

    initController();
                            
    setTimer(FRAME_PERIOD_MSEC);

}

void BelugaTrackerFrame::writeUserXML()
{
    MT_RobotFrameBase::writeUserXML();

    WriteDataGroupToXML(&m_XMLSettingsFile, m_pSetupInfo);
}

void BelugaTrackerFrame::readUserXML()
{
    MT_RobotFrameBase::readUserXML();

    ReadDataGroupFromXML(m_XMLSettingsFile, m_pSetupInfo);
}

void BelugaTrackerFrame::makeFileMenu(wxMenu* file_menu)
{
    file_menu->Append(ID_MENU_FILE_CAM_SETUP,
                      wxT("Configure Video Sources..."));
    wxFrame::Connect(ID_MENU_FILE_CAM_SETUP,
                     wxEVT_COMMAND_MENU_SELECTED,
                     wxCommandEventHandler(BelugaTrackerFrame::onMenuFileCamSetup));

    file_menu->Append(MT_TFB_ID_MENU_FILE_SELECT_DATAFILE, wxT("Select Data Output &File..."));
    wxFrame::Connect(MT_TFB_ID_MENU_FILE_SELECT_DATAFILE,
                     wxEVT_COMMAND_MENU_SELECTED,
                     wxCommandEventHandler(MT_TrackerFrameBase::onMenuFileSelectDataFile));

    file_menu->AppendSeparator();

    MT_FrameBase::makeFileMenu(file_menu);
}

void BelugaTrackerFrame::handleCommandLineArguments(int argc, wxChar** argv)
{

    long temp;
    if(m_CmdLineParser.Found(wxT("n"), &temp))
    {
        m_iNToTrack = temp;
    }

    MT_TrackerFrameBase::handleCommandLineArguments(argc, argv);
}

MT_RobotBase* BelugaTrackerFrame::getNewRobot(const char* config, const char* name)
{
    Beluga* thebot = new Beluga(config, name);
    ReadDataGroupFromXML(m_XMLSettingsFile, thebot->GetParameters());

	// Fixing robot 0 as tracked object 0 for now
	m_Robots.TrackingIndex[0] = 0;

    return thebot;
}

void BelugaTrackerFrame::initController()
{
    m_ControlMode = UNKNOWN;
    
    for(unsigned int i = 0; i < 4; i++)
    {
        m_apWaypointController[i] = new BelugaWaypointControlLaw();
        m_apLowLevelController[i] = new BelugaLowLevelControlLaw();
        m_Controller.appendControlLawToBot(i, m_apWaypointController[i], mt_CONTROLLER_NO_GC);
        m_Controller.appendControlLawToBot(i, m_apLowLevelController[i], mt_CONTROLLER_NO_GC);

        m_adWaypointX[i] = BELUGA_WAYPOINT_NONE;
        m_adWaypointY[i] = BELUGA_WAYPOINT_NONE;
        m_adWaypointZ[i] = BELUGA_WAYPOINT_NONE;

        m_adSpeedCommand[i] = BELUGA_CONTROL_NONE;
        m_adOmegaCommand[i] = BELUGA_CONTROL_NONE;
        m_adZDotCommand[i] = BELUGA_CONTROL_NONE;	   
    }
}

void BelugaTrackerFrame::doUserStep()
{
    //printf("DUS %ld\n", wxThread::GetCurrentId());
	MT_RobotFrameBase::doUserStep();
    //printf("DUS-done\n");
}

void BelugaTrackerFrame::acquireFrames()
{
	if(m_bCamerasReady)
	{
		for(unsigned int i = 0; i < 4; i++)
		{
			m_pCameraFrames[i] = m_pCapture->getFrame(MT_FC_NEXT_FRAME, m_uiaIndexMap[i]);
			if(i > 0 && m_pSlaves[i])
			{
				m_pSlaves[i]->setCurrentFrame(m_pCameraFrames[i]);
			}
		}
        m_pCurrentFrame = m_pCameraFrames[0];
	}
}

void BelugaTrackerFrame::sendRobotDataToTracker()
{
    std::vector<double> depth, speed, vert, turn, u, z;

    /* we want values for each object we're tracking, even if
     * we don't have actual values - a value of zero should be OK */
    depth.resize(m_iNToTrack, 0.0);
    speed.resize(m_iNToTrack, 0.0);
    vert.resize(m_iNToTrack, 0.0);
    turn.resize(m_iNToTrack, 0.0);

    int ti;
    for(unsigned int i = 0; i < MT_MAX_NROBOTS; i++)
    {
        ti = m_Robots.TrackingIndex[i];
        if(ti != MT_NOT_TRACKED)
        {
            z = m_Robots[i]->GetMeasurements();
            depth[ti] = z[BELUGA_ROBOT_MEASUREMENT_DEPTH];

            u = m_Robots[i]->GetControl();
            speed[ti] = u[BELUGA_CONTROL_FWD_SPEED];
            vert[ti] = u[BELUGA_CONTROL_VERT_SPEED];
            turn[ti] = u[BELUGA_CONTROL_STEERING];
        }
    }
    m_pBelugaTracker->setRobotData(depth, speed, vert, turn);    
}

void BelugaTrackerFrame::runTracker()
{
    //printf("Tracker\n");
	if(m_pCameraFrames[0])
	{
        sendRobotDataToTracker();
		m_pBelugaTracker->doTracking(m_pCameraFrames);
	}

}

bool BelugaTrackerFrame::tryIPCConnect()
{
    // TODO:  replace with rhubarb code

    bool connected = false;
    std::string motd("");

    if(!m_IPCClient.doConnect(&motd))
    {
        MT_ShowErrorDialog(this, wxT("Could not connect to IPC server."));
        connected = false;
    }
    else
    {
        printf("Success connecting to IPC server, server says:\n\t\"%s\"\n",
               motd.c_str());
        connected = true;
    }
    
    return connected;
}

void BelugaTrackerFrame::manageIPCConnection()
{
    /* determine if we need to establish a connection */
    if(m_bConnectSocket && !m_IPCClient.isConnected())
    {
        m_bConnectSocket = tryIPCConnect();
    }

    /* or we need to close the connection */
    if(!m_bConnectSocket && m_IPCClient.isConnected())
    {
        m_IPCClient.doDisconnect();
    }
}

void BelugaTrackerFrame::doIPCExchange()
{
    manageIPCConnection();

	if(m_IPCClient.isConnected())
	{

        std::vector<double> X(4);
        std::vector<double> Y(4);
        std::vector<double> Z(4);
        std::vector<unsigned int> robots(m_iNToTrack);

        int i1 = m_iNToTrack;
        if(!m_pTracker)
        {
            i1 = 0;
        }
        
		for(int i = 0; i < i1; i++)
		{
			X[i] = m_pBelugaTracker->getBelugaX(i);
			Y[i] = m_pBelugaTracker->getBelugaY(i);
			Z[i] = m_pBelugaTracker->getBelugaZ(i);
            robots[i] = i;
		}

        for(unsigned int i = i1; i < 4; i++)
        {
            X[i] = BELUGA_NOT_TRACKED_X;
            X[i] = BELUGA_NOT_TRACKED_Y;
            X[i] = BELUGA_NOT_TRACKED_Z;
        }

        BELUGA_CONTROL_MODE mode;
        
        bool r = m_IPCClient.setAllPositions(&X, &Y, &Z);
        r &= m_IPCClient.getControls(robots, &mode, &X, &Y, &Z);
        if(!r)
        {
            MT_ShowErrorDialog(this, wxT("Error during exchange with IPC server."));
            return;
        }

        double* control_a = NULL;
        double* control_b = NULL;
        double* control_c = NULL;
        
        switch(mode)
        {
        case WAYPOINT:
            control_a = m_adWaypointX;
            control_b = m_adWaypointX;
            control_c = m_adWaypointX;
            break;
        case KINEMATICS:
            control_a = m_adSpeedCommand;
            control_b = m_adOmegaCommand;
            control_c = m_adZDotCommand;
            break;
        }

        m_ControlMode = mode;

        if(control_a)
        {
            for(int i = 0; i < m_iNToTrack; i++)
            {
                control_a[i] = X[i];
                control_a[i] = Y[i];
                control_a[i] = Z[i];                
            }
        }

        // TODO: calculate waypoints in camera coordinates
        
	}
    
}

void BelugaTrackerFrame::doUserControl()
{

    doIPCExchange();
    
	std::vector<double> u(BELUGA_CONTROL_SIZE, 0.0);

    if(!m_pTracker)
	{
		return;
	}

	if(!m_Robots.IsPhysical(0) || (m_Robots.TrackingIndex[0] == MT_NOT_TRACKED))
	{
		return;
	}

	if(!m_bControlActive)
	{
		return;
	}

	if(!m_bGotoActive || !m_bControlActive)
	{
		m_Robots[0]->SetControl(u);
		m_Robots[0]->Control();
		return;
	}

    std::vector<double> u_waypoint(BELUGA_WAYPOINT_SIZE, 0.0);
    u_waypoint[BELUGA_WAYPOINT_X] = m_dGotoXW;
    u_waypoint[BELUGA_WAYPOINT_Y] = m_dGotoYW;
    u_waypoint[BELUGA_WAYPOINT_Z] = 0;

    mt_dVectorCollection_t X_all(4);
    mt_dVectorCollection_t u_in_all(4);
    for(int i = 0; i < m_iNToTrack; i++)
    {
        if(m_Robots.IsPhysical(i) && m_Robots.TrackingIndex[i] != MT_NOT_TRACKED)
        {
            X_all[i] = m_pBelugaTracker->getBelugaState(m_Robots.TrackingIndex[i]);
            u_in_all[i] = mt_CONTROLLER_EMPTY_VECTOR;
        }
        else
        {
            X_all[i] = mt_CONTROLLER_EMPTY_VECTOR;
            u_in_all[i] = mt_CONTROLLER_EMPTY_VECTOR;
        }
    }

    m_apWaypointController[0]->doActivate();

    u_in_all[0] = u_waypoint;

    mt_dVectorCollection_t u_all;
    u_all = m_Controller.doControl(X_all, u_in_all);

	m_Robots[0]->SetControl(u_all[0]);
	m_Robots[0]->Control();

}

void BelugaTrackerFrame::initTracker()
{

	if(m_pTracker)
	{
		return;
	}

    if(!m_pSlaves[1])
    {
        MT_ShowErrorDialog(this, wxT("Initialize the video sources first."));
        return;
    }
    
    /* TODO: ask for number of objects to track */
    m_pBelugaTracker = new BelugaTracker(m_pCurrentFrame, m_iNToTrack);
    m_pTracker = (MT_TrackerBase *) m_pBelugaTracker;

	m_pBelugaTracker->setMasks(m_sQuad1MaskPath.c_str(),
			m_sQuad2MaskPath.c_str(),
			m_sQuad3MaskPath.c_str(),
			m_sQuad4MaskPath.c_str());
	m_pBelugaTracker->setCalibrations(m_sQuad1CalibrationPath.c_str(),
			m_sQuad2CalibrationPath.c_str(),
			m_sQuad3CalibrationPath.c_str(),
			m_sQuad4CalibrationPath.c_str());

    for(unsigned int i = 1; i < 4; i++)
    {
        m_pSlaves[i]->setTracker(m_pTracker);
        m_pSlaves[i]->setTrackerFrameGroup(m_pBelugaTracker->getAuxFrameGroup(i-1));
    }

    
    /* note - do NOT call MT_TrackerBase::initTracker, which is
     * a placeholder function that sets m_pTracker to NULL! */

	setTimer(FRAME_PERIOD_MSEC);

}

void BelugaTrackerFrame::doRobotTimedEvents()
{
    printf("Robot - %ld\n", wxThread::GetCurrentId());
}

void BelugaTrackerFrame::updateRobotStatesFromTracker()
{

    if(!m_pTracker)
	{
		return;
	}

    int ti;
	std::vector<double> curr_state;
    for(unsigned int i = 0; i < MT_MAX_NROBOTS; i++)
    {
        ti = m_Robots.TrackingIndex[i];
        if(ti != MT_NOT_TRACKED)
        {
			curr_state = m_pBelugaTracker->getBelugaState(ti);
            m_Robots[i]->Update(curr_state);
        }
    }

}

bool BelugaTrackerFrame::toggleControlActive()
{
    m_bControlActive = !m_bControlActive;
    m_bGotoActive = m_bControlActive;
    m_pBelugaControlFrame->setControlActive(m_bControlActive);
    
    return m_bControlActive;
}

bool BelugaTrackerFrame::toggleIPCActive()
{
    m_bConnectSocket = !m_bConnectSocket;

    manageIPCConnection();

    m_pBelugaControlFrame->setIPCActive(m_bConnectSocket);

    return m_bConnectSocket;
    
}

bool BelugaTrackerFrame::doSlaveKeyboardCallback(wxKeyEvent& event, int slave_index)
{
	bool result = MT_DO_BASE_KEY;

	char k = event.GetKeyCode();

	switch(k)
	{
	case 'g':
		toggleControlActive();
		break;
	case 'q':
		doQuit();
		break;
	}

	return result;
}

bool BelugaTrackerFrame::doKeyboardCallback(wxKeyEvent& event)
{
	bool result = MT_DO_BASE_KEY;

	char k = event.GetKeyCode();

	switch(k)
	{
	case 'g':
        toggleControlActive();
		break;
	}

	bool tresult = MT_RobotFrameBase::doKeyboardCallback(event);
	return result && tresult;
}

bool BelugaTrackerFrame::doSlaveMouseCallback(wxMouseEvent& event, double viewport_x, double viewport_y, int slave_index)
{
	bool result = MT_DO_BASE_MOUSE;

		if(event.LeftUp())
		{
			if(m_bControlActive)
			{
				m_bGotoActive = true;
				m_iGotoCam = slave_index;
				m_dGotoXC = viewport_x;
				m_dGotoYC = m_ClientSize.GetHeight() - viewport_y;
				if(m_pBelugaTracker)
				{
					double z = 0;
					m_pBelugaTracker->getWorldXYZFromImageXYAndDepthInCamera(
						&m_dGotoXW,
						&m_dGotoYW,
						&z,
						m_dGotoXC,
						m_dGotoYC,
						0,
						false,
						slave_index);
				}
			}
		}

	return result;
}

bool BelugaTrackerFrame::doMouseCallback(wxMouseEvent& event, double viewport_x, double viewport_y)
{
	const double click_thresh = 10.0*10.0;
	bool result = MT_DO_BASE_MOUSE;

	if(m_pTracker)
	{
		if(event.RightUp())
		{
			int imin = -1;
			double dmin;
			double d2, dx, dy;
			for(int i = 0; i < m_iNToTrack; i++)
			{
				dx = viewport_x - m_pBelugaTracker->getBelugaX(i);
				dy = viewport_y - m_pBelugaTracker->getBelugaY(i);
				d2 = dx*dx + dy*dy;
				if(d2 < click_thresh)
				{
					if(imin < 0 || d2 < dmin)
					{
						dmin = d2;
						imin = i;
					}
				}
			}
			m_iGrabbedTrackedObj = imin;
			if(imin != NO_ROBOT)
			{
				wxString label;
				wxMenu pmenu;

				unsigned int np = 0;

				for(unsigned int i = 0; i < MT_MAX_NROBOTS; i++)
				{
					if(!m_Robots.IsPhysical(i))
					{
						continue;
					}
					np++;
					label.Printf("Identify Robot %s", m_Robots.RobotName[i].c_str());
					pmenu.Append(ID_MENU_POP_ROBOT + i, label);
					Connect(ID_MENU_POP_ROBOT + i, 
						wxEVT_COMMAND_MENU_SELECTED,
						wxCommandEventHandler(BelugaTrackerFrame::onMenuAssign));
				}
				if(np > 0)
				{
					PopupMenu(&pmenu);
					result = MT_SKIP_BASE_MOUSE;
				}
			}
		} /* event.RightUp */

		if(event.LeftUp())
		{
			if(m_bControlActive)
			{
				m_bGotoActive = true;
				m_iGotoCam = 0;
				m_dGotoXC = viewport_x;
				m_dGotoYC =  m_ClientSize.GetHeight() - viewport_y;
				if(m_pBelugaTracker)
				{
					double z = 0;
					m_pBelugaTracker->getWorldXYZFromImageXYAndDepthInCamera(
						&m_dGotoXW,
						&m_dGotoYW,
						&z,
						m_dGotoXC,
						m_dGotoYC,
						0,
						false,
						0);
				}
			}
		}
	}

    bool tresult = MT_RobotFrameBase::doMouseCallback(event, viewport_x, viewport_y);
    return tresult && result;
}

void BelugaTrackerFrame::onMenuAssign(wxCommandEvent& event)
{
	if(m_iGrabbedTrackedObj == NO_ROBOT)
	{
		return;
	}
	int robot_selected = event.GetId() - ID_MENU_POP_ROBOT;
	if(robot_selected >= MT_MAX_NROBOTS)
	{
		return;
	}

	m_Robots.TrackingIndex[robot_selected] = m_iGrabbedTrackedObj;
	
	m_iGrabbedTrackedObj = NO_ROBOT;
}

void BelugaTrackerFrame::doUserGLDrawing()
{
	MT_RobotFrameBase::doUserGLDrawing();

	if(m_bGotoActive && m_iGotoCam == 0)
	{
		MT_DrawCircle(m_dGotoXC, m_ClientSize.GetHeight() - m_dGotoYC, MT_Green, 15.0*m_dGotoDist);
	}
}

void BelugaTrackerFrame::doSlaveGLDrawing(int slave_index)
{
	if(m_bGotoActive && m_iGotoCam == slave_index)
	{
		MT_DrawCircle(m_dGotoXC, m_ClientSize.GetHeight() - m_dGotoYC, MT_Green, 15.0*m_dGotoDist);
	}
}


void BelugaTrackerFrame::onMenuFileCamSetup(wxCommandEvent& event)
{

    doPause();

    std::vector<std::string> camList = m_pCapture->listOfAvailableCameras(4);

	if(camList.size() < 4)
	{
		MT_ShowErrorDialog(this, 
			wxT("Unable to get list of cameras.  "
			"Make sure the right drivers are installed "
			"and that the caemras are available in SmartView."));
		return;
	}

    std::vector<std::string*> calibList;
    calibList.resize(4);
    calibList[0] = &m_sQuad1CalibrationPath;
    calibList[1] = &m_sQuad2CalibrationPath;
    calibList[2] = &m_sQuad3CalibrationPath;
    calibList[3] = &m_sQuad4CalibrationPath;

    std::vector<std::string*> maskList;
    maskList.resize(4);
    maskList[0] = &m_sQuad1MaskPath;
	maskList[1] = &m_sQuad2MaskPath;
	maskList[2] = &m_sQuad3MaskPath;
	maskList[3] = &m_sQuad4MaskPath;

    Beluga_VideoSetupDialog* dlg = new Beluga_VideoSetupDialog(m_pCapture,
                                                               camList,
                                                               calibList,
                                                               maskList,
                                                               m_uiaIndexMap,
                                                               this);
	registerDialogForXML(dlg);
    dlg->Show();
    dlg->UpdateView();

    int r = dlg->ShowModal();

    m_sQuad1Camera = camList[m_uiaIndexMap[0]];
    m_sQuad2Camera = camList[m_uiaIndexMap[1]];
    m_sQuad3Camera = camList[m_uiaIndexMap[2]];
    m_sQuad4Camera = camList[m_uiaIndexMap[3]];

	m_bCamerasReady = true;

    bool firstTime = false;
    if(!m_pSlaves[0])
    {
        firstTime = true;
    }

	for(unsigned int i = 1; i < 4; i++)
	{
        MT_CameraSlaveFrame* frame = m_pSlaves[i];
        if(!m_pSlaves[i])
        {
            frame = new MT_CameraSlaveFrame(NULL);
            m_pSlaves[i] = frame;
            frame->doMasterInitialization();
            frame->Show();
            frame->Raise();
        }

		m_pCameraFrames[i] = m_pCapture->getFrame(MT_FC_NEXT_FRAME, m_uiaIndexMap[i]);
		m_pSlaves[i]->setCurrentFrame(m_pCameraFrames[i]);
		m_pSlaves[i]->setImage(m_pCameraFrames[i]);
        m_pSlaves[i]->setIndex(i);
		m_pSlaves[i]->setMTParent(this);

        /* don't want the user to be able to close these */
		frame->EnableCloseButton(false);
	}

    if(firstTime)
    {
        m_pSlaves[1]->SetTitle(wxT("View in Quadrant II"));
        m_pSlaves[2]->SetTitle(wxT("View in Quadrant III"));
        m_pSlaves[3]->SetTitle(wxT("View in Quadrant IV"));
        registerDialogForXML(m_pSlaves[1]);
        registerDialogForXML(m_pSlaves[2]);
        registerDialogForXML(m_pSlaves[3]);

        m_pSlaves[0] = NULL;
    }
    
	m_pCameraFrames[0] = m_pCapture->getFrame(MT_FC_NEXT_FRAME, m_uiaIndexMap[0]);

    m_pCurrentFrame = m_pCameraFrames[0];
	setImage(m_pCurrentFrame);
	int framewidth_pixels = m_pCapture->getFrameWidth(m_uiaIndexMap[0]);
	int frameheight_pixels = m_pCapture->getFrameHeight(m_uiaIndexMap[0]);
	setSizeByClient(framewidth_pixels, frameheight_pixels);
    setViewport(MT_BlankRectangle, true);
	lockCurrentViewportAsOriginal();

    if(firstTime)
    {
        setTimer(10);
        m_pTrackerControlFrame->enableButtons();
    
        onNewCapture();
    }

	if(m_pBelugaTracker)
	{
		m_pBelugaTracker->setMasks(m_sQuad1MaskPath.c_str(),
			m_sQuad2MaskPath.c_str(),
			m_sQuad3MaskPath.c_str(),
			m_sQuad4MaskPath.c_str());
		m_pBelugaTracker->setCalibrations(m_sQuad1CalibrationPath.c_str(),
			m_sQuad2CalibrationPath.c_str(),
			m_sQuad3CalibrationPath.c_str(),
			m_sQuad4CalibrationPath.c_str());
	}

}

MT_ControlFrameBase* BelugaTrackerFrame::createControlDialog()
{
    m_pBelugaControlFrame = new BelugaControlFrame(this);
	m_pRobotControlFrame = (MT_RobotControlFrameBase *) m_pBelugaControlFrame;
	m_pTrackerControlFrame = (MT_TrackerControlFrameBase *) m_pRobotControlFrame; 
	return (MT_ControlFrameBase*) m_pRobotControlFrame;
}

/**********************************************************************
 * Control Frame Class
 *********************************************************************/

enum
{
    ID_CONTROL_ACTIVE_BUTTON = MT_RCF_ID_HIGHEST + 10,
    ID_IPC_ACTIVE_BUTTON
};

BelugaControlFrame::BelugaControlFrame(BelugaTrackerFrame* parent,
                                       const int Buttons,
                                       const wxPoint& pos, 
                                       const wxSize& size)
    : MT_RobotControlFrameBase(parent, Buttons, pos, size),
      m_pBelugaTrackerFrame(parent),
      m_pControlActiveButton(NULL),
      m_pIPCActiveButton(NULL)
{
}

unsigned int BelugaControlFrame::createButtons(wxBoxSizer* pSizer, wxPanel* pPanel)
{
    unsigned int nbuttons = MT_RobotControlFrameBase::createButtons(pSizer, pPanel);

    m_pControlActiveButton = new wxToggleButton(pPanel,
                                                ID_CONTROL_ACTIVE_BUTTON,
                                                wxT("Activate Control"));
    m_pIPCActiveButton = new wxToggleButton(pPanel,
                                            ID_IPC_ACTIVE_BUTTON,
                                            wxT("Activate IPC"));

    pSizer->Add(m_pControlActiveButton, 0, wxALL | wxCENTER, 10);
    pSizer->Add(m_pIPCActiveButton, 0, wxALL | wxCENTER, 10);

    m_pControlActiveButton->Disable();
    m_pControlActiveButton->SetValue(false);
    m_pIPCActiveButton->Enable();
    m_pIPCActiveButton->SetValue(false);

    Connect(ID_CONTROL_ACTIVE_BUTTON,
            wxEVT_COMMAND_TOGGLEBUTTON_CLICKED,
            wxCommandEventHandler(BelugaControlFrame::onControlActiveButtonClicked));
    Connect(ID_IPC_ACTIVE_BUTTON,
            wxEVT_COMMAND_TOGGLEBUTTON_CLICKED,
            wxCommandEventHandler(BelugaControlFrame::onIPCActiveButtonClicked));

    return nbuttons + 2;
    
}

void BelugaControlFrame::onControlActiveButtonClicked(wxCommandEvent& WXUNUSED(event))
{
    m_pBelugaTrackerFrame->toggleControlActive();
}

void BelugaControlFrame::setControlActive(bool value)
{
    m_pControlActiveButton->SetValue(value);
    if(value)
    {
        m_pControlActiveButton->SetLabel(wxT("Disable Control"));
    }
    else
    {
        m_pControlActiveButton->SetLabel(wxT("Enable Control"));
    }
}

void BelugaControlFrame::onIPCActiveButtonClicked(wxCommandEvent& WXUNUSED(event))
{
    m_pBelugaTrackerFrame->toggleIPCActive();
}

void BelugaControlFrame::setIPCActive(bool value)
{
    m_pIPCActiveButton->SetValue(value);
    if(value)
    {
        m_pIPCActiveButton->SetLabel(wxT("Disable IPC"));
    }
    else
    {
        m_pIPCActiveButton->SetLabel(wxT("Enable IPC"));        
    }
}

void BelugaControlFrame::enableButtons()
{
    if(m_pControlActiveButton)
    {
        m_pControlActiveButton->Enable();
        m_pIPCActiveButton->Enable();
    }
    MT_RobotControlFrameBase::enableButtons();
}

/**********************************************************************
 * GUI App Class
 *********************************************************************/

IMPLEMENT_APP(BelugaTrackerApp)
