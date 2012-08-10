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
    m_iNToTrack(1 /* SET THIS EQUAL TO THE NUMBER OF VEHICLES */),
    m_dGotoMaxSpeed(0.35),	
    m_dGotoDistThreshold(0.30),
    m_dGotoTurningGain(40.0),
    m_dBoundaryGain(1e-6),
    /*	m_sForceFileNameX("../../src/Boundary_ForcesX.txt"),
	m_sForceFileNameY("../../src/Boundary_ForcesY.txt"), */
    m_bControlActive(false),
    m_iGrabbedTrackedObj(NO_ROBOT),
    m_bGotoActive(false),
    m_bCamerasReady(false),
    m_bConnectSocket(false),
    m_Controller(MT_MAX_NROBOTS /* # robots */),    
    m_pBelugaControlFrame(NULL)
{
  for(unsigned int i = 0; i < 4; i++)
    {
      m_pSlaves[i] = NULL;      
      m_apBoundaryController[i] = NULL;
      m_apLowLevelController[i] = NULL;

      for(unsigned int j = 0; j < 4; j++)
        {
	  m_adGotoXC[i][j] = BELUGA_WAYPOINT_NONE;
	  m_adGotoYC[i][j] = BELUGA_WAYPOINT_NONE;            
        }
    }
  m_pBelugaTracker = NULL;
  m_pPublisher = NULL;
  m_pSubscriber = NULL;
}

BelugaTrackerFrame::~BelugaTrackerFrame()
{
  for(unsigned int i = 0; i < 4; i++)
    {      
      if(m_apBoundaryController[i])
        {
	  delete m_apBoundaryController[i];
	  m_apBoundaryController[i] = NULL;
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
  delete m_pPublisher;
  delete m_pSubscriber;
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
			    wxT("Number of objects to track. Default is 1."),
			    wxCMD_LINE_VAL_NUMBER);

  m_CmdLineParser.AddOption(wxT("pp"),
				wxT("publish_port"),
				wxT("Port on which to publish robot data."),
			    wxCMD_LINE_VAL_NUMBER);
    m_CmdLineParser.AddOption(wxT("sm"),
				wxT("subscribe_machine"),
				wxT("Machine or IP address from which to subscribe."),
			    wxCMD_LINE_VAL_STRING);
	m_CmdLineParser.AddOption(wxT("sp"),
				wxT("subscribe_port"),
				wxT("Port from which to subscribe."),
			    wxCMD_LINE_VAL_STRING);

  



  m_pPreferences->AddDouble("Boundary Gain",
			    &m_dBoundaryGain);
  m_pPreferences->AddString("X Force File Name",
			    &m_sForceFileNameX);
  m_pPreferences->AddString("Y Force File Name",
			    &m_sForceFileNameY);

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

  /* THIS SHOULD NOT HAPPEN HERE - THIS IS A HACK */
  std::vector<std::string> botnames;
  for(unsigned int i = 0; i < 7; i++)
    {
      botnames.push_back(m_Robots.RobotName[i]);
    }
  m_MouseControlledRobot = MT_Choice(botnames, 0);  
  m_pPreferences->AddChoice("Mouse Controlled Robot",
			    &m_MouseControlledRobot);
  initController();
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
  long publishing_port = 1234;
  wxString subscribing_machine = wxT("localhost");
  wxString subscribing_port = wxT("1235");
  m_CmdLineParser.Found(wxT("pp"), &publishing_port);
  m_CmdLineParser.Found(wxT("sm"), &subscribing_machine);
  m_CmdLineParser.Found(wxT("sp"), &subscribing_port);

  m_pPublisher = new MT_AgentStatesPublisher(publishing_port);
  m_pSubscriber = new MT_AgentStatesSubscriber(subscribing_machine.mb_str(), subscribing_port.mb_str());

  MT_TrackerFrameBase::handleCommandLineArguments(argc, argv);
}

MT_RobotBase* BelugaTrackerFrame::getNewRobot(const char* config, const char* name)
{
  Beluga* thebot = new Beluga(config, name);
  ReadDataGroupFromXML(m_XMLSettingsFile, thebot->GetParameters());

  // Fixing robot 0 as tracked object 0 for now
  // TODO: Fix
  //m_Robots.TrackingIndex[0] = 0;

  return thebot;
}

void BelugaTrackerFrame::initController()
{
  m_ControlMode = UNKNOWN;
    
  for(unsigned int i = 0; i < 4; i++)
    {
      printf("Trying to load force file |%s|\n", m_sForceFileNameX.c_str());
      m_apBoundaryController[i] = new BelugaBoundaryControlLaw(m_sForceFileNameX.c_str(),
															   m_sForceFileNameY.c_str());
      m_apLowLevelController[i] = new BelugaLowLevelControlLaw();
      m_Controller.appendControlLawToBot(i, m_apBoundaryController[i], mt_CONTROLLER_NO_GC);
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
  std::vector<double> depth, speed, vert, turn, u, z,
						vdK_t,
	vdK_d1,
	vdm_0,
	vdm_1,
	vdr_1,
	vdK_omega,
	vdeta_up,
	vdeta_down,
	vdv_off,
	vdk_d,
	vdz_off,
	vdk_teth,
	vdk_vp,
	vdJ;

  /* we want values for each object we're tracking, even if
   * we don't have actual values - a value of zero should be OK */
  depth.resize(m_iNToTrack, 0.0);
  speed.resize(m_iNToTrack, 0.0);
  vert.resize(m_iNToTrack, 0.0);
  turn.resize(m_iNToTrack, 0.0);
  vdK_t.resize(m_iNToTrack, BELUGA_DEFAULT_K_T);
  vdK_d1.resize(m_iNToTrack, BELUGA_DEFAULT_K_D1);
  vdm_0.resize(m_iNToTrack, BELUGA_DEFAULT_M_0);
  vdm_1.resize(m_iNToTrack, BELUGA_DEFAULT_M_1);
  vdr_1.resize(m_iNToTrack, BELUGA_DEFAULT_R_1);
  vdK_omega.resize(m_iNToTrack, BELUGA_DEFAULT_K_OMEGA);
  vdeta_up.resize(m_iNToTrack, BELUGA_DEFAULT_ETA_UP);
  vdeta_down.resize(m_iNToTrack, BELUGA_DEFAULT_ETA_DOWN);
  vdv_off.resize(m_iNToTrack, BELUGA_DEFAULT_V_OFF);
  vdk_d.resize(m_iNToTrack, BELUGA_DEFAULT_K_D);
  vdz_off.resize(m_iNToTrack, BELUGA_DEFAULT_Z_OFF);
  vdk_teth.resize(m_iNToTrack, BELUGA_DEFAULT_K_TETH);
  vdk_vp.resize(m_iNToTrack, BELUGA_DEFAULT_K_VP);
  vdJ.resize(m_iNToTrack, BELUGA_DEFAULT_J);

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
	MT_DataGroup* robot_params = m_Robots[i]->GetParameters();
	  vdK_t[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("K t"));
	vdK_d1[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("K d1"));
	vdm_0[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("m 0"));
	vdm_1[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("m 1"));
	vdr_1[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("r 1"));
	vdK_omega[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("K omega"));
	vdeta_up[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("eta up"));
	vdeta_down[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("eta down"));
	vdv_off[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("v off"));
	vdk_d[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("k d"));
	vdz_off[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("z off"));
	vdk_teth[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("k teth"));
	vdk_vp[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("k vp"));
	vdJ[ti] = robot_params->GetNumericValue(robot_params->GetIndexByName("J"));
        }
    }
  m_pBelugaTracker->setRobotData(depth, speed, vert, turn,vdK_t,
	vdK_d1,
	vdm_0,
	vdm_1,
	vdr_1,
	vdK_omega,
	vdeta_up,
	vdeta_down,
	vdv_off,
	vdk_d,
	vdz_off,
	vdk_teth,
	vdk_vp,
	vdJ);    
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

void BelugaTrackerFrame::doUserControl()
{    
  std::vector<double> u(BELUGA_CONTROL_SIZE, 0.0);

  if(!m_pTracker)
    {
      return;
    }

  if(!m_bGotoActive && !m_bControlActive)
    {
      return;
    }
  mt_dVectorCollection_t X_all(MT_MAX_NROBOTS);
  mt_dVectorCollection_t u_in_all(MT_MAX_NROBOTS);
  
  MT_AgentStates agent_states;
  for (int i = 0; i < MT_MAX_NROBOTS; i++) {          
    if(m_Robots.IsPhysical(i) && m_Robots.TrackingIndex[i] != MT_NOT_TRACKED) {
	  MT_DataGroup* robot_params = m_Robots[i]->GetParameters();
	  double K_t = robot_params->GetNumericValue(robot_params->GetIndexByName("K t"));
	  double K_d1 = robot_params->GetNumericValue(robot_params->GetIndexByName("K d1"));
	  double m_0 = robot_params->GetNumericValue(robot_params->GetIndexByName("m 0"));
	  double m_1 = robot_params->GetNumericValue(robot_params->GetIndexByName("m 1"));
	  double r_1 = robot_params->GetNumericValue(robot_params->GetIndexByName("r 1"));
	  double K_omega = robot_params->GetNumericValue(robot_params->GetIndexByName("K omega"));
	  double eta_up = robot_params->GetNumericValue(robot_params->GetIndexByName("eta up"));
	  double eta_down = robot_params->GetNumericValue(robot_params->GetIndexByName("eta down"));
	  double v_off = robot_params->GetNumericValue(robot_params->GetIndexByName("v off"));
	  double k_d = robot_params->GetNumericValue(robot_params->GetIndexByName("k d"));
	  double z_off = robot_params->GetNumericValue(robot_params->GetIndexByName("z off"));
	  double k_teth = robot_params->GetNumericValue(robot_params->GetIndexByName("k teth"));
	  double k_vp = robot_params->GetNumericValue(robot_params->GetIndexByName("k vp"));
	  double J = robot_params->GetNumericValue(robot_params->GetIndexByName("J"));

	  std::vector<double> BoundaryControlLawParams;
	  BoundaryControlLawParams.push_back(m_0 + m_1);
	  m_apBoundaryController[i]->setParameters(BoundaryControlLawParams);

	  std::vector<double> LowLevelControlLawParams;
	  LowLevelControlLawParams.push_back(K_t);
	  LowLevelControlLawParams.push_back(K_d1);
	  LowLevelControlLawParams.push_back(m_0);
	  LowLevelControlLawParams.push_back(m_1);
	  LowLevelControlLawParams.push_back(r_1);
	  LowLevelControlLawParams.push_back(K_omega);
	  LowLevelControlLawParams.push_back(eta_up);
	  LowLevelControlLawParams.push_back(eta_down);
	  LowLevelControlLawParams.push_back(v_off);
	  LowLevelControlLawParams.push_back(k_d);
	  LowLevelControlLawParams.push_back(z_off);
	  LowLevelControlLawParams.push_back(k_teth);
	  LowLevelControlLawParams.push_back(k_vp);
	  LowLevelControlLawParams.push_back(J);
	  m_apLowLevelController[i]->setParameters(LowLevelControlLawParams);

      MT_AgentState* new_agent_state = agent_states.add_agent_state();
      std::vector<double> state = m_pBelugaTracker->getBelugaState(m_Robots.TrackingIndex[i]);
      MT_TrackedAgentState* new_tracked_agent_state = new_agent_state->mutable_tracked_agent_state();	      
      new_tracked_agent_state->set_x(state[BELUGA_STATE_X]);
      new_tracked_agent_state->set_y(state[BELUGA_STATE_Y]);
      new_tracked_agent_state->set_z(state[BELUGA_STATE_Z]);
      new_tracked_agent_state->set_z_dot(state[BELUGA_STATE_ZDOT]);
      new_tracked_agent_state->set_speed(state[BELUGA_STATE_SPEED]);
      new_tracked_agent_state->set_heading(state[BELUGA_STATE_THETA]);
      new_tracked_agent_state->set_omega(state[BELUGA_STATE_OMEGA]);
      X_all[i] = state;
      if (m_bGotoActive) {
	// control via mouse click -> waypoint            
	int robot_index = m_MouseControlledRobot.GetIntValue();
	if (m_adWaypointX[i] != BELUGA_WAYPOINT_NONE && m_adWaypointY[i] != BELUGA_WAYPOINT_NONE && i == robot_index) {
	  MT_GenericAgentControl* new_generic_agent_control = new_agent_state->mutable_generic_agent_control();
	  MT_Waypoint* new_waypoint = new_generic_agent_control->mutable_waypoint();
	  new_waypoint->set_x(m_adWaypointX[i]);
	  new_waypoint->set_y(m_adWaypointY[i]);
	  new_waypoint->set_z(new_agent_state->tracked_agent_state().z());
	}
	else {
	  new_agent_state->clear_tracked_agent_state();
	}
      }
    }
  }
  
  bool successful_publish = m_pPublisher->Publish(agent_states);

  if (!m_pSubscriber->connected() || !m_pPublisher->connected()) {
    // If publisher or subscriber is disconnected, stop all robots
    for(unsigned int i = 0; i < MT_MAX_NROBOTS; i++)
      {
	if(m_Robots.IsPhysical(i))
	  {
	    m_Robots[i]->SetControl(u);
	    m_Robots[i]->Control();
	  }
      }
    return;
  }

  MT_AgentStates received_agent_states;
  if (m_pSubscriber->NumberOfQueuedMessages() > 0) {
	 received_agent_states = m_pSubscriber->PopMostRecentReceivedMessage();
	  m_pSubscriber->EmptyQueue();
  }
  else {
	  return;
  }

  int received_state_count = 0;
  for (int i = 0; i < MT_MAX_NROBOTS; i++) {
    if (m_Robots.IsPhysical(i) && m_Robots.TrackingIndex[i] != MT_NOT_TRACKED) {
      u[BELUGA_CONTROL_FWD_SPEED] = received_agent_states.agent_state(received_state_count).generic_agent_control().kinematics().speed();
      u[BELUGA_CONTROL_STEERING] = received_agent_states.agent_state(received_state_count).generic_agent_control().kinematics().omega();
      u[BELUGA_CONTROL_VERT_SPEED] = received_agent_states.agent_state(received_state_count).generic_agent_control().kinematics().z_dot();
	  if (m_bControlActive) {
      if(received_agent_states.agent_state(received_state_count).generic_agent_control().has_waypoint())
	  {
	  MT_Waypoint waypoint = received_agent_states.agent_state(received_state_count).generic_agent_control().waypoint();
	  double w = m_pBelugaTracker->getWaterDepth();
	  m_adWaypointX[i] = waypoint.x();
	  m_adWaypointY[i] = waypoint.y();
	  m_adWaypointZ[i] = waypoint.z();
	  for(unsigned int cam_num = 0; cam_num < BELUGA_NUM_CAMS; cam_num++)
	    {
	      double d = w - m_adWaypointZ[i];                    
	      m_pBelugaTracker->getCameraXYFromWorldXYandDepthFixedCamera(
									  cam_num,
									  &m_adGotoXC[i][cam_num],
									  &m_adGotoYC[i][cam_num],
									  m_adWaypointX[i],
									  m_adWaypointY[i],
									  d, /* depth */
									  false); /* no distortion */
	    }            
        }
      else { 
	  m_adWaypointX[i] = BELUGA_WAYPOINT_NONE;
	  m_adWaypointY[i] = BELUGA_WAYPOINT_NONE;
	  m_adWaypointZ[i] = BELUGA_WAYPOINT_NONE;
	  for(unsigned int cam_num = 0; cam_num < BELUGA_NUM_CAMS; cam_num++) {
	    m_adGotoXC[i][cam_num] = BELUGA_WAYPOINT_NONE;
	    m_adGotoYC[i][cam_num] = BELUGA_WAYPOINT_NONE;
	  }
      }            
	  }



      received_state_count++;
      u_in_all[i] = u;
    }
  }

  mt_dVectorCollection_t u_all;
  u_all = m_Controller.doControl(X_all, u_in_all);

  for(unsigned int i = 0; i < MT_MAX_NROBOTS; i++)
    {
      if(m_Robots.IsPhysical(i))
        {						
	  m_Robots[i]->SetControl(u_all[i]);
	  m_Robots[i]->Control();
        }
    }
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

void BelugaTrackerFrame::stopAllRobots()
{
  // TODO: there could be a MT_AllRobotContainer::stopAllRobots()
  for(unsigned int i = 0; i < MT_MAX_NROBOTS; i++)
    {
      if(m_Robots.IsPhysical(i))
        {
	  m_Robots[i]->SafeStop();
        }
    }
}

bool BelugaTrackerFrame::doSlaveKeyboardCallback(wxKeyEvent& event, int slave_index)
{
  bool result = MT_DO_BASE_KEY;

  char k = event.GetKeyCode();

  switch(k)
    {
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

  /*
    switch(k)
    {
    case 'g':
    toggleControlActive();
    break;
    }
  */
  bool tresult = MT_RobotFrameBase::doKeyboardCallback(event);
  return result && tresult;
}

bool BelugaTrackerFrame::doCommonMouseCallback(wxMouseEvent& event,
                                               double viewport_x,
                                               double viewport_y,
                                               int slave_index)
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
	      dx = viewport_x - m_pBelugaTracker->getBelugaCameraX(i, slave_index);
	      dy = (m_ClientSize.GetHeight() - viewport_y) - m_pBelugaTracker->getBelugaCameraY(i, slave_index);
                
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
		  label.Printf(wxT("Identify Robot %s"), m_Robots.RobotName[i].c_str());
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
	  setWaypointFromMouseClick(viewport_x,
				    viewport_y,
				    slave_index);
	}
    }
  return result;
    
}

void BelugaTrackerFrame::setWaypointFromMouseClick(double viewport_x,
                                                   double viewport_y,
                                                   int slave_index)
{
	if(m_bControlActive) {
		return;
	}

  int robot_index = m_MouseControlledRobot.GetIntValue();
  m_adGotoXC[robot_index][slave_index] = viewport_x;
  m_adGotoYC[robot_index][slave_index] =  m_ClientSize.GetHeight() - viewport_y;
  for (int i = 0; i < MT_MAX_NROBOTS; i++) {
    if (i != robot_index) {
		m_adWaypointX[i] = BELUGA_WAYPOINT_NONE;
      m_adWaypointY[i] = BELUGA_WAYPOINT_NONE;
		for (int cam_num = 0; cam_num < BELUGA_NUM_CAMS; cam_num++) {
      m_adGotoXC[i][cam_num] = BELUGA_WAYPOINT_NONE;
      m_adGotoYC[i][cam_num] =  BELUGA_WAYPOINT_NONE;      
		}
    }
  }

  double z = 0;
  m_pBelugaTracker->getWorldXYZFromImageXYAndDepthInCamera(
							   &m_adWaypointX[robot_index],
							   &m_adWaypointY[robot_index],
							   &z,
							   m_adGotoXC[robot_index][slave_index],
							   m_adGotoYC[robot_index][slave_index],
							   0,
							   false,
							   slave_index);
  for(unsigned int i = 0; i < BELUGA_NUM_CAMS; i++)
    {  
      m_pBelugaTracker->getCameraXYFromWorldXYandDepthFixedCamera(i,
								  &m_adGotoXC[robot_index][i],
								  &m_adGotoYC[robot_index][i],
								  m_adWaypointX[robot_index],
								  m_adWaypointY[robot_index],
								  0, /* depth */
								  false); /* no distortion */
    }
}

bool BelugaTrackerFrame::doSlaveMouseCallback(wxMouseEvent& event, double viewport_x, double viewport_y, int slave_index)
{
  return doCommonMouseCallback(event, viewport_x, viewport_y, slave_index);
}

bool BelugaTrackerFrame::doMouseCallback(wxMouseEvent& event, double viewport_x, double viewport_y)
{
  /* don't short-circuit this */
  bool result = doCommonMouseCallback(event, viewport_x, viewport_y, 0);

  bool tresult = result && MT_RobotFrameBase::doMouseCallback(event, viewport_x, viewport_y);
        
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

  doCommonGLDrawing(0);
}

void BelugaTrackerFrame::doSlaveGLDrawing(int slave_index)
{
  doCommonGLDrawing(slave_index);
}

void BelugaTrackerFrame::doCommonGLDrawing(int slave_index)
{
  double h = (double) m_ClientSize.GetHeight();
  glLineWidth(3.0);

  for(unsigned int i = 0; i < BELUGA_NUM_BOTS; i++)
    {
      if((m_adGotoXC[i][slave_index] >= 0 && m_adGotoYC[i][slave_index] >= 0 &&
	  m_adGotoXC[i][slave_index] <= m_ClientSize.GetWidth() &&
	  m_adGotoYC[i][slave_index] <= h))
	{		
	  MT_DrawCircle(m_adGotoXC[i][slave_index],
			h - m_adGotoYC[i][slave_index],
			MT_Green, 15.0*m_dGotoDistThreshold);
	}
    }
  glLineWidth(1.0);

  if(m_pTracker)
    {
      double x = 0;
      double y = 0;
      double fx = 0;
      double fy = 0;

      int ti;
      std::vector<double> curr_state;
      for(unsigned int i = 0; i < MT_MAX_NROBOTS; i++)
	{
	  ti = m_Robots.TrackingIndex[i];
	  if(ti != MT_NOT_TRACKED)
	    {
	      x = m_pBelugaTracker->getBelugaCameraX(ti, slave_index);
	      y = m_pBelugaTracker->getBelugaCameraY(ti, slave_index);

	      m_apBoundaryController[i]->getLastForce(&fx, &fy);

	      MT_R3 center(x, m_ClientSize.GetHeight() - y, 0);
	      double length = sqrt(fx*fx + fy*fy);
	      double orientation = atan2(fy, fx);

	      //printf("fx = %f, fy = %f, length = %f, orientation = %f\n", fx, fy, length, orientation);

	      if(length > 0)
		{
		  MT_DrawArrow(center, length, orientation, MT_White, 1);
		}

	    }
	}

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

void BelugaTrackerFrame::activateMouseControl() {
  m_bControlActive = false;
  m_bGotoActive = true;
}

void BelugaTrackerFrame::activateExternalControl() {
  m_bControlActive = true;
  m_bGotoActive = false;
}

void BelugaTrackerFrame::deactivateControl() {
  stopAllRobots();
  m_bControlActive = false;
  m_bGotoActive = false;
}

/**********************************************************************
 * Control Frame Class
 *********************************************************************/

enum
  {
    ID_CONTROL_ACTIVE_BUTTON = MT_RCF_ID_HIGHEST + 10,
    ID_MOUSE_CONTROL_ACTIVE_BUTTON,
    ID_EXTERNAL_CONTROL_ACTIVE_BUTTON
  };

BelugaControlFrame::BelugaControlFrame(BelugaTrackerFrame* parent,
                                       const int Buttons,
                                       const wxPoint& pos, 
                                       const wxSize& size)
  : MT_RobotControlFrameBase(parent, Buttons, pos, size),
    m_pBelugaTrackerFrame(parent),
    m_pControlActiveButton(NULL),
    m_pMouseControlActiveButton(NULL),
    m_pExternalControlActiveButton(NULL)
{
}

unsigned int BelugaControlFrame::createButtons(wxBoxSizer* pSizer, wxPanel* pPanel)
{

  unsigned int nbuttons = MT_RobotControlFrameBase::createButtons(pSizer, pPanel);

  m_pControlActiveButton = new wxButton(pPanel,
					ID_CONTROL_ACTIVE_BUTTON,
					wxT("Deactivate Control"));

  m_pMouseControlActiveButton = new wxButton(pPanel,ID_MOUSE_CONTROL_ACTIVE_BUTTON,wxT("Activate Mouse Control"));

  m_pExternalControlActiveButton = new wxButton(pPanel,
						ID_EXTERNAL_CONTROL_ACTIVE_BUTTON,
						wxT("Activate External Control"));

  pSizer->Add(m_pControlActiveButton, 0, wxALL | wxCENTER, 10);
  pSizer->Add(m_pMouseControlActiveButton, 0, wxALL | wxCENTER, 10);
  pSizer->Add(m_pExternalControlActiveButton, 0, wxALL | wxCENTER, 10);

  m_pControlActiveButton->Disable();
  m_pMouseControlActiveButton->Disable();
  m_pExternalControlActiveButton->Disable();

  Connect(ID_CONTROL_ACTIVE_BUTTON,
	  wxEVT_COMMAND_BUTTON_CLICKED,
	  wxCommandEventHandler(BelugaControlFrame::onControlActiveButtonClicked));
  Connect(ID_MOUSE_CONTROL_ACTIVE_BUTTON,
	  wxEVT_COMMAND_BUTTON_CLICKED,
	  wxCommandEventHandler(BelugaControlFrame::onMouseControlActiveButtonClicked));
  Connect(ID_EXTERNAL_CONTROL_ACTIVE_BUTTON,
	  wxEVT_COMMAND_BUTTON_CLICKED,
	  wxCommandEventHandler(BelugaControlFrame::onExternalControlActiveButtonClicked));

  return nbuttons + 3;
    
}

void BelugaControlFrame::onControlActiveButtonClicked(wxCommandEvent& WXUNUSED(event))
{  
  m_pBelugaTrackerFrame->deactivateControl();
  m_pControlActiveButton->Disable();
  m_pMouseControlActiveButton->Enable();
  m_pExternalControlActiveButton->Enable();
}

void BelugaControlFrame::onMouseControlActiveButtonClicked(wxCommandEvent& WXUNUSED(event))
{
  m_pBelugaTrackerFrame->activateMouseControl();
  m_pControlActiveButton->Enable();
  m_pMouseControlActiveButton->Disable();
  m_pExternalControlActiveButton->Enable();
}

void BelugaControlFrame::onExternalControlActiveButtonClicked(wxCommandEvent& WXUNUSED(event))
{
  m_pBelugaTrackerFrame->activateExternalControl();
  m_pControlActiveButton->Enable();
  m_pMouseControlActiveButton->Enable();
  m_pExternalControlActiveButton->Disable();
}



void BelugaControlFrame::enableButtons()
{
  if(m_pMouseControlActiveButton)
    {
      m_pMouseControlActiveButton->Enable();
    }
  if(m_pExternalControlActiveButton)
    {
      m_pExternalControlActiveButton->Enable();
    }
  MT_RobotControlFrameBase::enableButtons();
}

/**********************************************************************
 * GUI App Class
 *********************************************************************/

IMPLEMENT_APP(BelugaTrackerApp)
