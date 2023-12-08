/* Include the controller definition */
#include "footbot_flocking.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

void CFootBotFlocking::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
      
      GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
      m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
      GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
      GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
   
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotFlocking::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CFootBotFlocking::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   //Real fNormDistExp = ::pow(LoopFunctions->TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

CFootBotFlocking::CFootBotFlocking() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   //m_pcLight(NULL),
   m_pcLEDs(NULL),
   m_pcCamera(NULL),
   m_compassSensor(NULL),
   LoopFunctions(NULL),
   Robot_state(MOVING)    {}

/****************************************/
/****************************************/

void CFootBotFlocking::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the XML tag of the
    * device whose handle we want to have. For a list of allowed values, type at the
    * command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
    *       the lists provided the configuration file at the
    *       <controllers><footbot_diffusion><actuators> and
    *       <controllers><footbot_diffusion><sensors> sections. If you forgot to
    *       list a device in the XML and then you request it here, an error occurs.
    */
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator          >("differential_steering");
   //m_pcLight  = GetSensor  <CCI_FootBotLightSensor                    >("footbot_light");
   m_pcLEDs   = GetActuator<CCI_LEDsActuator                          >("leds");
   m_pcCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   m_compassSensor   = GetSensor <CCI_PositioningSensor>("positioning");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   
      
   /*
    * Parse the config file
    */
   try {
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Flocking-related */
      m_sFlockingParams.Init(GetNode(t_node, "flocking"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }
    
    controllerID = GetId();
    Target = CVector2(9, 3);
   /*
    * Other init stuff
    */
   Reset();
}


void CFootBotFlocking::SetLoopFunctions(CFlockingLoopFunctions* lf) {
	LoopFunctions = lf;
	
}

int CFootBotFlocking::GetStatus(){
	return Robot_state;
	}

/****************************************/
/****************************************/

void CFootBotFlocking::ControlStep() {
	
	m_sFlockingParams.TargetDistance = LoopFunctions->GetTargetDistance();  
	 //LOG <<"LoopFunctions.TargetDistance =" << m_sFlockingParams.TargetDistance<<endl; 
	
	
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
  
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size(); 
    /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
    CRadians cAngle = cAccumulator.Angle();
   
   if(m_sWheelTurningParams.m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_sWheelTurningParams.m_fDelta ) {
      /* Go straight to the target location*/
      //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      SetWheelSpeedsFromVector(VectorToTarget() + FlockingVector());
      Robot_state = MOVING;
   }
   else {
      /* Turn, depending on the sign of the angle */
      Robot_state = COLLIDING;
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_sWheelTurningParams.m_fWheelVelocity, 0.0f);
         //LOG<<"right turn ..."<<endl;
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_sWheelTurningParams.m_fWheelVelocity);
         //LOG<<"left turn ..."<<endl;
      }
  } 

}

/****************************************/
/****************************************/

void CFootBotFlocking::Reset() {
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::RED);
}

/****************************************/
/****************************************/
CVector2 CFootBotFlocking::VectorToTarget(){
	CVector2 c, currPos;
	//const CCI_PositioningSensor::SReadings& sReadings = m_compassSensor->GetReadings();
	argos::CVector3 position3D = m_compassSensor->GetReading().Position;
	const argos::CQuaternion orientation = m_compassSensor->GetReading().Orientation;
	//LOG<< "orien=" << orientation<< endl;
	/* convert the quaternion to euler angles */
	argos::CRadians z_angle, y_angle, x_angle;
	orientation.ToEulerAngles(z_angle, y_angle, x_angle);
	
	
	//LOG<<"angle="<< z_angle<< endl;
	float x = position3D.GetX();
	float y = position3D.GetY();
	currPos = CVector2(x, y);
	//LOG<<"robot ID="<< controllerID <<std::endl;
   //LOG<<"currPos = "<< currPos<<std::endl;
	//LOG<<"Target = "<< Target<<std::endl;
	//LOG<<"length = "<< (currPos - Target).Length()<<endl;
	
	if ((Target - currPos).Length() > 0.05){
		//argos::CRadians heading(orientation.GetW()); 
		//LOG<<"heading = "<< z_angle<<endl;
		//LOG<<"(currPos - Target).Angle()="<<(currPos - Target).Angle()<< endl;
		//LOG<<"(Target - currPos).Angle()="<<(Target - currPos).Angle()<< endl;
		
		//LOG<< "(currPos - Target).Angle()-heading="<< (currPos - Target).Angle()-z_angle << endl;
		//c = CVector2((Target - currPos).Length(), (Target - currPos).Angle()-z_angle);
		c = CVector2(1.0, (Target - currPos).Angle()-z_angle);
		
		c.Normalize();
		c *= 0.25f * m_sWheelTurningParams.MaxSpeed;
		//LOG << "c="<< c<<std::endl;
		}
		else c = CVector2(0, 0);
	//LOG << "VectorToTarget()="<< c<< std::endl;
    return c;
	
}

/****************************************/
/****************************************/

CVector2 CFootBotFlocking::FlockingVector() {
	
   /* Get the camera readings */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   
   /* Go through the camera readings to calculate the flocking interaction vector */
   if(! sReadings.BlobList.empty()) {
      CVector2 cAccum;
      Real fLJ;
      size_t unBlobsSeen = 0;
      
      connectedRobots.clear();
      
      argos::CVector3 position3D = m_compassSensor->GetReading().Position;
	  float x = position3D.GetX();
	  float y = position3D.GetY();
	  CVector2 pos2D = CVector2(x, y);
	  float posX, posY;
	    
      for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
         /*
          *  /*
          * The camera perceives the light as a yellow blob
          * The robots have their red beacon on
          * So, consider only red blobs
          * In addition: consider only the closest neighbors, to avoid
          * attraction to the farthest ones. Taking 180% of the target
          * distance is a good rule of thumb.
          */
        // if(sReadings.BlobList[i]->Color == CColor::RED &&
        //    sReadings.BlobList[i]->Distance < m_sFlockingParams.TargetDistance * 1.80f) {
		// detect neighbor robots
        //LOG  <<"ID = "<< controllerID << ", pos2D= (" << x << ", " << y<< ") "  << ", sReadings.BlobList[" << i << "]->Distance = " << sReadings.BlobList[i]->Distance << ", Angle = " << sReadings.BlobList[i]->Angle <<endl;
		//posX = x + 0.01 * sReadings.BlobList[i]->Distance * cos(sReadings.BlobList[i]->Angle.GetValue());
		//posY = y + 0.01 * sReadings.BlobList[i]->Distance * sin(sReadings.BlobList[i]->Angle.GetValue());
		//LOG <<"robot loc: ("<< posX << ", " << posY << ")" << endl; 
		
		robotPos = LoopFunctions->GetRobotPosition();
		//Real dist=100;
		string neighborID;
		for(map<string, CVector2>::iterator it= robotPos.begin(); it!= robotPos.end(); ++it) {
			//LOG<< "it->second="<< it->second<< ", pos2D="<< pos2D << endl;
			//LOG<<"(it->second - pos2D).Length()="<<(it->second - pos2D).Length() << ", dist="<< sReadings.BlobList[i]->Distance <<endl;
			if( abs((it->second - pos2D).Length() - 0.01 * sReadings.BlobList[i]->Distance) < 0.1 ){
				//LOG<< "get the neigbor "<< it->first << endl;
				neighborID = it->first;
				connectedRobots[neighborID] = sReadings.BlobList[i]->Distance;
				break;
				}
		}
		robotPos.clear();	
		
       //for(map<string, double>::iterator it= connectedRobots.begin(); it!= connectedRobots.end(); ++it) {
	   //LOG<< "connected "<< it->first << ", dist=" << it->second << endl;
	   //}
	   	
		if(sReadings.BlobList[i]->Color == CColor::RED &&
            sReadings.BlobList[i]->Distance < LoopFunctions->GetTargetDistance() * 1.80f) {
				
            /*
             * Take the blob distance and angle
             * With the distance, calculate the Lennard-Jones interaction force
             * Form a 2D vector with the interaction force and the angle
             * Sum such vector to the accumulator
             */
            /* Calculate LJ */
            fLJ = m_sFlockingParams.GeneralizedLennardJones(sReadings.BlobList[i]->Distance);
            /* Sum to accumulator */
            cAccum += CVector2(fLJ, sReadings.BlobList[i]->Angle);
            /* Increment the blobs seen counter */
            ++unBlobsSeen;
		}
	  }
      if(unBlobsSeen > 0) {
         /* Divide the accumulator by the number of blobs seen */
         cAccum /= unBlobsSeen;
         /* Clamp the length of the vector to the max speed */
         if(cAccum.Length() > m_sWheelTurningParams.MaxSpeed) {
            cAccum.Normalize();
            cAccum *= m_sWheelTurningParams.MaxSpeed;
         }
         return cAccum;
      }else
         return CVector2();
    }
    else { //blob sensor empty
         return CVector2();
    }
}


/****************************************/
/****************************************/

void CFootBotFlocking::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotFlocking, "footbot_flocking_controller")
