// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <aquatic_gazebo_sensor_plugins/HydrophoneROSPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
HydrophoneROSPlugin::HydrophoneROSPlugin() : ROSBaseModelPlugin()
{ }

/////////////////////////////////////////////////
HydrophoneROSPlugin::~HydrophoneROSPlugin()
{ }

/////////////////////////////////////////////////
void HydrophoneROSPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  // SDF PARAMETERS to C++ VARIABLES
  GetSDFParam<std::string>(_sdf, "beacon_parent", this->beacon_parent, "bluerov2");
  GetSDFParam<double>(_sdf, "sensitivity", this->sensitivity, -190.0);
  GetSDFParam<double>(_sdf, "pre_ampGain", this->pre_ampGain, 20.0);
  GetSDFParam<double>(_sdf, "cut_off_frequency", this->cut_off_frequency, 10000);
  GetSDFParam<double>(_sdf, "noise_stddev", this->stdDev, 0.01);
  GetSDFParam<std::string>(_sdf, "sound_speed_model", this->sound_speed_model, "Leroy");
  GetSDFParam<std::string>(_sdf, "spreading_model", this->spreading_model, "Combined");
  GetSDFParam<std::string>(_sdf, "operational_mode", this->operational_mode, "acoustic_pressure");

  /* Wave Periods */
  this->wave_period = 0;


  // In the Future we can create a Class that finds every Beacons Sensors in the World
  // With that will be possible create others types of Acoustic Localization Systems
  // And principal for a Multi-Robot Localization with this system
  // e.g Long Base Line (LBL) -> Beacons on the seabed
  // OR we can subscribe all beacon topics but we just need the position of an acoustic source
  // And a trigger so get that with GetLink can be a better choice ?
  // BUT! For two cases we must make a multi-source model too
  // for the acoustic propagation

  // this->beacon_model = this->world->GetModel(this->beacon_parent);
  // this->beacon_link = this->beacon_model->GetLink(this->beacon_parent + "/beacon_link");

  // From another way, possible we are a introduct a possible position shift
  // because, when computer reads here the position is not exactly equal when signal
  // as triggered. So we will assume that when comes a topic message from the Beacon
  // we are ready to compute, and publish Hydrophone message when his time is equal
  // to the computed arrival time

  /*  ROS INPUT TOPIC - SUBSCRIBER - Note: Try put a Gazebo Input Topic */
  this->rosSensorInputSub = this->rosNode->subscribe<aquatic_gazebo_sensor_plugins_msgs::Beacon>(
    "/" + this->beacon_parent + "/beacon", 1,
      boost::bind(&HydrophoneROSPlugin::InputCallBack, this, _1));

  /* ROS HYDROPHONE OUTPUT TOPIC - PUBLISHER */
  this->rosSensorOutputPub =
    this->rosNode->advertise<aquatic_gazebo_sensor_plugins_msgs::Hydrophone>(
      this->sensorOutputTopic, 1);


  /* IMPULSES SOUND WAVE OUTPUT TOPIC - PUBLISHER */
  if (!this->operational_mode.compare("PressureLevel")) {

    this->SensorLevelOutputPub =
      this->rosNode->advertise<std_msgs::Float64>(
        this->sensorOutputTopic + "/acoustic_pressure", 1);

  }else if(!this->operational_mode.compare("ReceiverLevel")) {

    this->SensorLevelOutputPub =
      this->rosNode->advertise<std_msgs::Float64>(
        this->sensorOutputTopic + "/receiver_level", 1);

  }else if(!this->operational_mode.compare("SignalLevel")) {

    this->SensorLevelOutputPub =
      this->rosNode->advertise<std_msgs::Float64>(
        this->sensorOutputTopic + "/signal_level", 1);
  }

  /* Gazebo output Msg - ALTERNATIVE */
  if (this->gazeboMsgEnabled)
  {
    this->gazeboSensorOutputPub =
      this->gazeboNode->Advertise<sensor_msgs::msgs::Hydrophone>(
          this->robotNamespace + "/" + this->sensorOutputTopic, 1);
  }

  ROS_WARN("Loaded Hydrophone plugin with a parent... %s",_model->GetName().c_str());
}
/////////////////////////////////////////////////
bool HydrophoneROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  /* Publish sensor state */
  this->PublishState();

  if (!this->EnableMeasurement(_info))
    return false;

  /* Read the current simulation time */
  common::Time time_now;

#if GAZEBO_MAJOR_VERSION >= 8
  time_now = this->world->SimTime();
#else
  time_now = this->world->GetSimTime();
#endif

  /* ROS Hydrophone message */
  std_msgs::Float64 rosMsg;

  this->NL = Noise();

  /* Impulses Simulation of Sound Pressure Wave [Simulation Time] */

  double t_impulse = this->initial_time.sec*1000.0 + this->initial_time.nsec/1000000.0
                     + this->TOF;

  double time_now_ms = time_now.sec*1000.0 + time_now.nsec/1000000.0;


  if ((time_now_ms <= t_impulse) || this->pulse == true) { // !(time_now_ms <= t_impulse)

    /* JUST NOISE */

    rosMsg.data = ContinuosWave(0.0, 26000, time_now_ms);

  }else{

    rosMsg.data = ContinuosWave(this->RL, 26000, time_now_ms);

    /* Impulse @ Path Time + Initial Time  */
    this->wave_period++;

    if(this->wave_period > 10) {
      this->pulse = true;
      this->wave_period = 0;
    }
  }

  /* Publish ROS Hydrophone message */
  this->SensorLevelOutputPub.publish(rosMsg);

  /* Read the current simulation time */
#if GAZEBO_MAJOR_VERSION >= 8
  this->lastMeasurementTime = this->world->SimTime();
#else
  this->lastMeasurementTime = this->world->GetSimTime();
#endif
  return true;
}
/////////////////////////////////////////////////

// Callback from ROS Input TOPIC

void HydrophoneROSPlugin::InputCallBack(const aquatic_gazebo_sensor_plugins_msgs::Beacon::ConstPtr &_msg){

  /* Get the Hydrophone Pose */
  ignition::math::Vector3d hydrophone_pos;

#if GAZEBO_MAJOR_VERSION >= 8
  hydrophone_pos = this->link->WorldPose().Pos();
  //beacon_pos = this->beacon_link->WorldPose().Pos();
#else
  hydrophone_pos = this->link->GetWorldPose().Ign().Pos();
  //beacon_pos = this->beacon_link->GetWorldPose().Ign().Pos();
#endif

  this->pulse = false;

  /* Get the Beacon Data */

  /* TIME */
  this->initial_time.sec = _msg->header.stamp.sec;
  this->initial_time.nsec = _msg->header.stamp.nsec;
  //_msg->initial_time;

  /* POSE */
  // TEST if it's equal to the method get Link bla bla,
  // but occours a segmentation fault if the beacon not running at the momment
  ignition::math::Vector3d beacon_pos;
  beacon_pos.X() = _msg->initial_pose.x; //50.0;
  beacon_pos.Y() = _msg->initial_pose.y; //0.0;
  beacon_pos.Z() = std::abs(_msg->initial_pose.z); //50.0;

  /* BEACON PARAMETERS */
  double f = _msg->operating_frequency/1000.0;
  double SL = _msg->SPL;
  double BEAM = _msg->beam_pattern;

  /* Get Environmet Variables - FOR TESTING UNDER CONSTRUCTION */
  double Z[5] = { 0.0, 10.0, 20.0, 30.0, 50.0};

  double T[5] = { 16.5122, 16.3834, 16.1425, 15.7513, 14.8349 };
  double S[5] = { 35.7917, 35.8013, 35.8060, 35.8060, 35.8481 };
  double pH[5] = { 8.0000, 7.8532, 7.8079, 7.7575, 7.7067 };

  /* Depth */
  double H = 0.1;
  double alphaR = 0.0;
  double R_ = 0.0;

  /* Variables Initialization/Reset */
  this->TOF = 0.0;

  /* Compute the Horizontal range (D) [m] */
  double a, b;

  if (hydrophone_pos.X() < beacon_pos.X()) {
    a = beacon_pos.X() - hydrophone_pos.X();
  }else{
    a = hydrophone_pos.X() - beacon_pos.X();
  }

  if (hydrophone_pos.Y() < beacon_pos.Y()) {
    b = beacon_pos.Y() - hydrophone_pos.Y();
  }else{
    b = hydrophone_pos.Y() - beacon_pos.Y();
  }

  double D = std::sqrt(std::pow(a, 2) + std::pow(b, 2));
  

  /* Compute the truth range (R) [m] */
  double R = std::sqrt(std::pow(D, 2) + std::pow(beacon_pos.Z() - std::abs(hydrophone_pos.Z()), 2));

  /* Incident Angle from Source [rad] */
  double theta = asin( (beacon_pos.Z() - std::abs(hydrophone_pos.Z())) / R );

  /* Compute Ray Dynamic Layers */
  for(int n = 0; Z[n] < beacon_pos.Z(); n++){

    // INJECT SOME GAUSIAN ERROR to variate like the Real Worls */

    // Compute Sound Speed for a given T, S, pH and Depth (Z) for n layers < Z (Depth)
    // Probaly this calculation is better to do on Beacon and send as parameter
    // to all hydrophone... But, we will test it on each hydrophone that implies
    // the same calculation four times and if is not heavy...
    double c_n = SoundSpeed(this->sound_speed_model, T[n], S[n], Z[n]);

    // Compute Absorption Loss Coeficient for a give T, S, and Depth (Z)
    double alpha_n = Absorption(f, c_n, T[n], S[n], pH[n], Z[n]);

    // Compute Ray Range per Layer [m]
    double Rn = RayRangeByLayer(std::abs(beacon_pos.Z()), std::abs(hydrophone_pos.Z()), Z[n], Z[n+1], theta);

    // Depth Dependence
    alphaR += alpha_n * Rn/1000.0;

    //R_ += Rn; // Test - And it's equal to R Check

    // Compute Path Time per Layer [ms]
    this->TOF += PathTime(Rn, c_n);

  }
  this->TOF += this->GetGaussianNoise(this->stdDev);

  // Compute Transmission Loss (TL) or Propagation Loss per Layer [dB]
  double TL = PropagationLoss(this->spreading_model, R, H, alphaR);

  /* Compute Receiver Level [dB] */
  this->RL = ReceiverLevel(SL, TL);

  /* Compute Directivity Index */
  double ohmega = PI; // this->beam_pattern;
  double DI = 10*log10(ohmega/4*PI);;

  /* Compute Signal-to-Noise-Ratio */
  double SNR = SignalToNoiseRatio(SL, TL, this->NL, DI);

  //gzdbg << "DEBUG: " << c_n << std::endl;


  /* Read the current simulation time */
  common::Time time_now;

#if GAZEBO_MAJOR_VERSION >= 8
  time_now = this->world->SimTime();
#else
  time_now = this->world->GetSimTime();
#endif

  /* ROS Hydrophone message */
  aquatic_gazebo_sensor_plugins_msgs::Hydrophone rosMsg;

  /* Get the Header */
  rosMsg.header.stamp.sec  = time_now.sec;
  rosMsg.header.stamp.nsec = time_now.nsec;
  rosMsg.header.frame_id = this->link->GetName();

  /* Building ROS Beacon message */

  /* TIME [ms] */
  rosMsg.TOF = this->TOF;

  /* SIGNAL AMPLITUDE */
  rosMsg.SNR = SNR;

  /* DISCRETE NOISE */
  rosMsg.noise[0] = this->NL_turbulence[0];
  rosMsg.noise[1] = this->NL_turbulence[1];

  rosMsg.noise[2] = this->NL_shipping[0];
  rosMsg.noise[3] = this->NL_shipping[1];

  rosMsg.noise[4] = this->NL_wave[0];
  rosMsg.noise[5] = this->NL_wave[1];

  rosMsg.noise[6] = this->NL_rain[0];
  rosMsg.noise[7] = this->NL_rain[1];

  rosMsg.noise[8] = this->NL_thermal[0];
  rosMsg.noise[9] = this->NL_thermal[1];

  /* GROUND TRUTH */
  rosMsg.range_gt = R;

  /* Publish ROS Hydrophone message */
  this->rosSensorOutputPub.publish(rosMsg);

  if (this->gazeboMsgEnabled) {

    /* Gazebo Hydropone message */
    sensor_msgs::msgs::Hydrophone gazeboMsg;

    gazeboMsg.set_tof(this->TOF);
    gazeboMsg.set_snr(SNR);
    // gazeboMsg.set_noise(noise[4]);

    /* Publish Gazebo Hydrophone message, if enabled */
    this->gazeboSensorOutputPub->Publish(gazeboMsg);
  }
}
/////////////////////////////////////////////////

// Compute Sound Speed for a give T, S, and Depth (Z)

double HydrophoneROSPlugin::SoundSpeed(std::string sound_speed_model, double T, double S, double Z)
{
  double c = 0.0;
  double lat = 42.0; // TESTING - we must call the Gazebo environment variable

  if(!sound_speed_model.compare("Leroy")){

      /* Leroy et al. 2008 Sound Velocity Model - Table 2.1 [Etter, Paul]
         Pp to 12km depth
         0-42 Salinity Range (ppt) */

       c = 1402.5 + 5*T -5.44*std::pow(10,-2)*std::pow(T
         ,2) + 2.1*std::pow(10,-4)*std::pow(T,3) + 1.33*S
          -1.23*std::pow(10,-2)*S*T + 8.7*std::pow(10,-5)*S*std::pow(T,2) + 1.56*std::pow(10,-2)*Z
          +2.55*std::pow(10,-7)*std::pow(Z,2) - 7.3*std::pow(10,-12)*std::pow(Z,3) + 1.2*std::pow(10,-6)*Z*(lat-45)
          -9.5*std::pow(10,-13)*T*std::pow(Z,3) + 3*std::pow(10,-7)*std::pow(T,2)*Z + 1.43*std::pow(10,-5)*S*Z;

  }else if(!sound_speed_model.compare("Medwin")){

      /* Medwin, 1975 for simplicity (not so computational heavy)
         but limited to 1000m, 0-45 Salinity Range (ppt) */
      c = 1449.2 + 4.6*T - 0.055*std::pow(T, 2) + 0.00029*std::pow(T, 3) + (1.34 - 0.01*T)*(S - 35) + 0.016*Z;

  }else{
      /* The sound speed aproximation given in F&G Model */
      c = 1412 + 3.21*T + 1.19*S + 0.016*Z;

  }
  return c;
}
/////////////////////////////////////////////////

// Compute Absorption Loss Coeficient for a give T, S, and Depth (Z)

double HydrophoneROSPlugin::Absorption(int f, double c, double T, double S, double pH, double Z)
{
  /* Model of Fracois-Garrison: the absorption coefficent of seawater in dB / km
     f = frequency in Hz
     T = temperature in C
     S = salinity in ppt
     pH = pH
     Z = depth in Km */

     /* The contribution of botic acid: pH */
     double A1 = (8.86/c)*std::pow(10,0.78*pH-5);
     double P1 = 1;
     double f1 = 2.8*std::sqrt(S/35)*std::pow(10,4-1245/(T+273));

     double boric_acid = A1*P1*(f1*std::pow(f,2))/(std::pow(f1,2) + std::pow(f,2));

     /* The contribution of magnesium sulfate: salt */
     double A2 = 21.44*(S/c)*(1+0.025*T);
     double P2 = 1-1.37*std::pow(10,-4)*Z + 6.2*std::pow(10,-9)*std::pow(Z,2);
     double f2 = 8.17*std::pow(10,8-1990/(T+273))/(1+0.0018*(S-35));

     double magnesium_sulfate = A2*P2*(f2*std::pow(f,2))/(std::pow(f2,2) + std::pow(f,2));

     /* The contribution of pure water viscosity */
     double P3 = 1 - 3.83*std::pow(10, -5)*Z + 4.9*std::pow(10,-10)*std::pow(Z,2);

     double A3;
     if(T <= 20){
        A3 = 4.937*std::pow(10, -4) - 2.59*std::pow(10, -5)*T + 9.11*std::pow(10, -7)
             *std::pow(T, 2) - 1.5*std::pow(10, -8)*std::pow(T, 3);

     }else if(T > 20){
        A3 = 3.964*std::pow(10, -4) - 1.146*std::pow(10, -5)*T + 1.45*std::pow(10, -7)
             *std::pow(T, 2) - 6.5*std::pow(10, -10)*std::pow(T,3);
     }

     double pure_water = A3*P3*std::pow(f,2);

     double alpha = boric_acid + magnesium_sulfate + pure_water;

     return alpha;
}
/////////////////////////////////////////////////

// Compute Ray Range per Layer [m]

double HydrophoneROSPlugin::RayRangeByLayer(double zs, double zr, double z1, double z2, double theta)
{
  double Rn = 0.0;

  /* Receiver and Source on the same layer */
  if((z1 < zr && zr < z2) && (z1 < zs && zs < z2)){

      Rn = (zs - zr)/sin(theta);

  /* Receiver Case */
  }else if(z1 < zr && zr < z2){

      Rn = (z2 - zr)/sin(theta);

  /* Source Case */
  }else if(z1 < zs && zs < z2){

      Rn = (zs - z1)/sin(theta);

  /* Inter-medium Cases */
  }else{

      Rn = (z2 - z1)/sin(theta);

  }

  return Rn;
}
/////////////////////////////////////////////////

// Compute Transmission/Propagation or absortion Loss [dB]

double HydrophoneROSPlugin::PropagationLoss(std::string spreading_model, double R, double H, double alpha_R)
{
  /* Geometrical Spreading Loss corrected from attenuation
     Where alpha [dB/km] will multipy by R (range), so we have
     Total Absorption = [dB/km] * km */
     double TL = 0.0;

     /* [m] to [km] */
     R = R/1000.0;

     if(!spreading_model.compare("Spherical")){

         /* Spherical Case */
         TL = 20*std::log10(R/0.001) + alpha_R;

     }else if(!spreading_model.compare("Cylindrical")){

         /* Cylindrical Case */
         TL = 10*std::log10(R/0.001) + alpha_R;

     }else if(!spreading_model.compare("Combined")){

        /* Combined Case:
           Propagate spherically near the source. At some range R, the
           spherical wave hits the sea floor, from here on, sound propagates
           cylindrically. When a range R greater than the water depth H.
           [PocketBook 3rd p.24] && [Lurton, Xavier p.36]                     */

         if(R > H){

            TL = 10*std::log10(H/0.001) + 10*std::log10(R/0.001) + alpha_R;
            // TL = 20*log10(R0/0.001) + 10*log10((R/R0)) + alpha*R;

         }else{

             /* Spherical Case */
             TL = 20*std::log10(R/0.001) + alpha_R;

         }
     }
     return TL;
}
/////////////////////////////////////////////////

// Compute Path Time [ms]

double HydrophoneROSPlugin::PathTime(double R, double c)
{
  /* x1000 to [ms] */
  return (R * 1000) / c;
}
/////////////////////////////////////////////////

// Compute Receiver Level [dB]

double HydrophoneROSPlugin::ReceiverLevel(double SL, double TL)
{
  /* Source Parameters:
        SL = Radiated signal @ source [dB]

     Medium Parameters:
        TL = Transmission or propagation loss @ medium [dB]

     Receiver Parameters:
        RL = Signal Level @ receiver [dB]                 */

  return SL - TL;
}
/////////////////////////////////////////////////

// Compute Noise [dB]

double HydrophoneROSPlugin::Noise()
{
    /* [Richard P. Hodges] double f_Turbulence, double NL_shipping, double NL_wave, double f_Thermal, double NL_rain
        7 Ambient Noise (at Hydrophone)
        7.1 Ambient Noise Models: Wenz ("Wenz Curves"), Crouch, Urick, Sadowski
        7.2 Seismic Noise: Earthquakes and Explosions 1Hz to 100Hz - Not considered
        7.9 Depth Effects on Noise
            Up to 10 kHz, ambient noise is relatively independent of depth because the noise is originating locally.
            Above 10 kHz, the ambient noise level will drop rapidly with depth as the noise is absorbed in the water,
            until the frequency region is reached where thermal noise dominates.  */


    /* Noise Turbulence, Shipping Noise, Wave Noise, Thermal Noise, Rain Noise */
    float noise[5][5] = { { 10.0,  500, 1000, 0.0, 0.0}, // Frequency [Hz]
                          { 0.0, 37.9, 44.8, 0.0, 0.0}, // Noise [dB]
                          {   1,    1,    1, 0.0, 0.0}
                        };

    /* Compute Total Noise NL with Gaussian Error [dB] - TESTING
      Noise Turbulence, Shipping Noise, Wave Noise, Thermal Noise, Rain Noise */
    double NL = 0.0;

    /* 7.3 Ocean Turbulence  [dB] re: 1 uPa, 1Hz, f in Hz
             The dominant source of noise in the very low frequency
             region from 1 to 10Hz                                   */

    // [Wenz]
    //this->NL_turbulence[1][i] = noise[0][0]; // + this->GetGaussianNoise(0.05);
    //this->NL_turbulence[0][i] = 107 - 30*std::log(this->NL_turbulence[1][i]);

    /* Our River Turbulance */
    this->NL_turbulence[1] = 40.0 + this->GetGaussianNoise(0.001);
    this->NL_turbulence[0] = 51.88 + this->GetGaussianNoise(0.1);

    NL += this->NL_turbulence[0];


    /* 7.4 Shipping Noise 10Hz to 10kHz [Sadowski et al] Table 7.1 - SIPPING LEVELS 1-7 */

    this->NL_shipping[0] = noise[1][1] + this->GetGaussianNoise(0.25);
    this->NL_shipping[1] = noise[0][1] + this->GetGaussianNoise(10.0);

    NL += this->NL_shipping[0];

    /* 7.5 Wave Noise mid-hundreds Hz to 50kHz [Sadowski et al] Table 7.2 - SEASTATES LEVELS 0-6 */
    this->NL_wave[0] = noise[1][2] + this->GetGaussianNoise(0.35);
    this->NL_wave[1] = noise[0][2] + this->GetGaussianNoise(100.0);

    NL += this->NL_wave[0];

    /* 7.6 Thermal Noise greater than 50kHz [Mellan] */
    if (noise[0][3] > 50000) {
      this->NL_thermal[1] = noise[0][3]; // + this->GetGaussianNoise(10.0);
      this->NL_thermal[0] = 20*std::log(this->NL_thermal[1]) - 75;    // [dB] re: 1 uPa, 1Hz, f in Hz

      NL += this->NL_thermal[0];
    }else{
      this->NL_thermal[0] = 0.0;
      this->NL_thermal[1] = 0.0;
    }

    /* 7.7 Rain Noise max at 1kHz but goes to 100kHz [Sadowski et al] Table 7.3 - L, M, H, EH */
    this->NL_rain[0] = noise[1][4]; // + this->GetGaussianNoise(10.0);
    this->NL_rain[1] = noise[0][4]; // + this->GetGaussianNoise(1000.0);

    NL += this->NL_rain[0];

  return NL;
}
/////////////////////////////////////////////////

// Compute Signal-to-Noise-Ratio [dB]

double HydrophoneROSPlugin::SignalToNoiseRatio(double SL, double TL, double NL, double DI)
{
  /* 2.1 Signal-to-Noise Ratio [dB]

         S/N = Signal/PropLoss x ArrayGain/TotalNoise

     Source Parameters:
         SL = Radiated signal @ source [dB]
         DIs = Directivity index @ source [dB]

     Medium Parameters:
         TL = Transmission or propagation loss @ medium [dB]
         NL = Total noise @ medium [dB]

     Receiver Parameters:
         RL = Signal Level @ receiver [dB]
         DIr = Directivity index @ receiver [dB]
         DT = Detection Threshold (SNR) @ receiver [dB] */

  return SL - TL - NL + DI;
}
/////////////////////////////////////////////////

// Compute Pressure Level [uPa]

double HydrophoneROSPlugin::PressureLevel(double RL)
{
  /*
     Receiver Level [dB] to [Pa]
                                 */

    return std::pow(10, RL/20)*std::pow(10, -6); //*1.0*std::pow(10, -6)*std::pow(10, 6);
}
/////////////////////////////////////////////////

// Compute Signal Level [V]

double HydrophoneROSPlugin::SignalLevel(double SPL, double Gain, double Sensitivity)
{
  /*
     Convert Pressure to a Voltage Signal
                                          */
    return std::pow(10, (SPL + Gain + Sensitivity)/20);
}
/////////////////////////////////////////////////

// Compute Continuos Wave (CW)

double HydrophoneROSPlugin::LevelConversion(double A)
{

  if (!this->operational_mode.compare("ReceiverLevel")) {

    return A;

  }else if(!this->operational_mode.compare("PressureLevel")) {

    /* Compute Pressure Level [uPa] */
    return PressureLevel(A);

  }else if(!this->operational_mode.compare("SignalLevel")) {

    /* Compute Signal Level [V] */
    return SignalLevel(A, this->pre_ampGain, this->sensitivity);
  }
}
/////////////////////////////////////////////////

// Compute Continuos Wave (CW)

double HydrophoneROSPlugin::ContinuosWave(double A, double w, double t)
{

  double cw = 0.0;

  /* [m] to [km] */
  //Rn = Rn/1000;

  /* [ms] to [s] */
  t = t/1000;

  /* Wave Signal */
  cw = LevelConversion(A) * sin(w*(t - this->TOF));

  /* Wave Noise @ Mutiple Ranges */
  cw += LevelConversion(this->NL) * sin(this->NL_turbulence[1]*(t - 0));
  cw += LevelConversion(this->NL) * sin(this->NL_shipping[1]*(t - 0));
  cw += LevelConversion(this->NL) * sin(this->NL_wave[1]*(t - 0));
  cw += LevelConversion(this->NL) * sin(this->NL_rain[1]*(t - 0));

  if (this->NL_thermal[1] > 50000) {
    cw += LevelConversion(this->NL) * sin(this->NL_thermal[1]*(t - 0));
  }


  return cw;
}
/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(HydrophoneROSPlugin)
}
