#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <json/json.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <sstream>
#include <random>

namespace companion {
using gz::math::Vector3d;using gz::math::Pose3d;
double clip(double v,double lo,double hi){return std::clamp(v,lo,hi);}
double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
constexpr double RHO=1.225,VOLUME=4.0/3.0*M_PI*.5*.225*.165;
constexpr double TOTAL_MASS=RHO*VOLUME,DEG=M_PI/180.0;
struct Rect{double x,y,hx,hy;};
const std::vector<Rect> obstacles={{0,1.65,.85,.45},{2.65,-1.55,.45,.7}};
bool crosses(double x,double y,double dx,double dy,const Rect&r,double pad,double end){
 double lo=0,hi=end;
 for(int i=0;i<2;i++){
  double p=i?y:x,d=i?dy:dx,c=i?r.y:r.x,h=(i?r.hy:r.hx)+pad;
  if(std::abs(d)<1e-9){if(p<c-h||p>c+h)return false;}
  else{double a=(c-h-p)/d,b=(c+h-p)/d;if(a>b)std::swap(a,b);lo=std::max(lo,a);hi=std::min(hi,b);if(lo>hi)return false;}
 }
 return hi>=0&&lo<=end;
}
class Companion:public gz::sim::System,public gz::sim::ISystemConfigure,
 public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate{
 gz::sim::Model model;gz::sim::Link body,leftLink,rightLink,ballastLink;
 gz::sim::Joint leftJoint,rightJoint,slider;
 gz::transport::Node node;gz::transport::Node::Publisher publisher;
 std::mutex mutex;std::deque<Json::Value> commands;
 std::string mode="float",status="floating",reason;
 Vector3d position{-2,0,1.6},velocity,omega,goal{-2,0,1.6},wind,push,pushTorque;
 gz::math::Quaterniond rotation;
 double time=0,lastPublish=-1,phaseL=0,phaseR=0,frequencyL=0,frequencyR=0,amplitudeL=0,amplitudeR=0;
 double qL=0,qR=0,dqL=0,dqR=0,ballast=0,ballastVelocity=0,ballastGoal=0,noseUp=0;
 double requestedFrequencyL=1.0,requestedFrequencyR=1.0,requestedAmplitudeL=40,requestedAmplitudeR=40;
 double forceL=0,forceR=0,thrustCoefficient=1.2,trimForce=0,controlYaw=0,pushUntil=0;
 double throttle=.65,turn=0,manualUntil=0,desiredSpeed=.18,desiredHeight=1.6;
 double wingServoTorqueL=0,wingServoTorqueR=0,ballastForce=0;
 bool lift=true,enabled=true,safety=true,resetPending=false;int ack=0;
 Pose3d resetPose{-2,0,1.6,0,0,0};
 double value(const Json::Value&j,const char*name,double def,double lo,double hi){
  if(!j[name].isNumeric())return def;double v=j[name].asDouble();return std::isfinite(v)?clip(v,lo,hi):def;
 }
 static double jointValue(gz::sim::Joint&j,const gz::sim::EntityComponentManager&e,bool vel=false){
  auto a=vel?j.Velocity(e):j.Position(e);return a&&!a->empty()?(*a)[0]:0;
 }
 void reset(const Json::Value&j){
  mode="float";status="floating";reason="";lift=enabled=safety=true;phaseL=phaseR=0;
  frequencyL=frequencyR=amplitudeL=amplitudeR=0;forceL=forceR=0;ballastGoal=noseUp=0;
  requestedFrequencyL=requestedFrequencyR=1;requestedAmplitudeL=requestedAmplitudeR=40;
  wind=push=pushTorque=Vector3d::Zero;pushUntil=manualUntil=0;trimForce=0;thrustCoefficient=1.2;
  throttle=.65;turn=0;desiredSpeed=.18;
  resetPose=Pose3d(value(j,"x",-2,-4,4),value(j,"y",0,-3,3),value(j,"z",1.6,.3,3),
   value(j,"roll",0,-.5,.5),value(j,"pitch",0,-.5,.5),value(j,"yaw",0,-M_PI,M_PI));
  goal=resetPose.Pos();desiredHeight=goal.Z();controlYaw=resetPose.Rot().Yaw();resetPending=true;
 }
 void command(const Json::Value&j){
  if(j["id"].isInt())ack=j["id"].asInt();auto a=j.get("action","").asString();
  if(a=="reset")reset(j);
  else if(a=="configure"){
   thrustCoefficient=value(j,"thrust_coefficient",thrustCoefficient,.0,3);
   trimForce=value(j,"trim_grams",trimForce*1000/9.81,-5,5)*9.81/1000;
   wind.Set(value(j,"wind_x",wind.X(),-.5,.5),value(j,"wind_y",wind.Y(),-.5,.5),0);
   desiredSpeed=value(j,"speed",desiredSpeed,.03,.3);
   if(j.isMember("buoyancy"))lift=j["buoyancy"].asBool();
   if(j.isMember("enabled"))enabled=j["enabled"].asBool();
   if(j.isMember("safety"))safety=j["safety"].asBool();
  }else if(a=="mode"){
   auto m=j.get("mode","float").asString();if(m=="float"||m=="swim"||m=="turn_left"||m=="turn_right"||m=="climb"||m=="descend"||m=="manual"||m=="off"){
    mode=m;controlYaw=rotation.Yaw();goal=position;desiredHeight=position.Z();noseUp=m=="climb"?20:m=="descend"?-20:0;
   }
  }else if(a=="wings"){
   mode="wings";requestedFrequencyL=value(j,"left_hz",requestedFrequencyL,0,1.5);requestedFrequencyR=value(j,"right_hz",requestedFrequencyR,0,1.5);
   requestedAmplitudeL=value(j,"left_amplitude",requestedAmplitudeL,0,50);requestedAmplitudeR=value(j,"right_amplitude",requestedAmplitudeR,0,50);
   noseUp=value(j,"nose_up_deg",noseUp,-25,25);
  }else if(a=="ballast"){
   mode="ballast";ballastGoal=value(j,"position",0,-.11,.11);
  }else if(a=="manual"){
   mode="manual";throttle=value(j,"throttle",throttle,0,1);turn=value(j,"turn",0,-1,1);noseUp=value(j,"nose_up_deg",noseUp,-25,25);manualUntil=time+.4;
  }else if(a=="goto"){
   goal=Vector3d(value(j,"x",position.X(),-4,4),value(j,"y",position.Y(),-3,3),value(j,"z",position.Z(),.4,2.9));mode="goto";
  }else if(a=="push"){
   push.Set(value(j,"fx",0,-.2,.2),value(j,"fy",0,-.2,.2),value(j,"fz",0,-.2,.2));
   pushTorque.Set(value(j,"tx",0,-.015,.015),value(j,"ty",0,-.015,.015),value(j,"tz",0,-.015,.015));pushUntil=time+value(j,"duration",.3,.02,1);
  }
 }
 public:void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&,
 gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override{
  model=gz::sim::Model(e);body=gz::sim::Link(model.LinkByName(ecm,"body"));leftLink=gz::sim::Link(model.LinkByName(ecm,"left_wing"));rightLink=gz::sim::Link(model.LinkByName(ecm,"right_wing"));ballastLink=gz::sim::Link(model.LinkByName(ecm,"ballast"));
  leftJoint=gz::sim::Joint(model.JointByName(ecm,"left_hinge"));rightJoint=gz::sim::Joint(model.JointByName(ecm,"right_hinge"));slider=gz::sim::Joint(model.JointByName(ecm,"ballast_slide"));
  for(auto*l:{&body,&leftLink,&rightLink,&ballastLink})l->EnableVelocityChecks(ecm);
  for(auto*j:{&leftJoint,&rightJoint,&slider}){j->EnablePositionCheck(ecm);j->EnableVelocityCheck(ecm);}
  publisher=node.Advertise<gz::msgs::StringMsg>("/companion/state");node.Subscribe("/companion/command",&Companion::receive,this);
 }
 void receive(const gz::msgs::StringMsg&m){
  Json::Value j;Json::CharReaderBuilder b;std::string error;std::istringstream s(m.data());
  if(Json::parseFromStream(b,s,&j,&error)&&j.isObject()){std::lock_guard<std::mutex>guard(mutex);if(commands.size()<64)commands.push_back(j);}
 }
 void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm)override{
  if(info.paused)return;time=std::chrono::duration<double>(info.simTime).count();double dt=std::chrono::duration<double>(info.dt).count();if(dt<=0||dt>.02)return;
  auto pose=body.WorldPose(ecm).value_or(Pose3d());position=pose.Pos();rotation=pose.Rot();velocity=body.WorldLinearVelocity(ecm).value_or(Vector3d::Zero);omega=body.WorldAngularVelocity(ecm).value_or(Vector3d::Zero);
  qL=jointValue(leftJoint,ecm);qR=jointValue(rightJoint,ecm);dqL=jointValue(leftJoint,ecm,true);dqR=jointValue(rightJoint,ecm,true);
  ballast=jointValue(slider,ecm);ballastVelocity=jointValue(slider,ecm,true);
  {std::lock_guard<std::mutex>guard(mutex);for(auto&j:commands)command(j);commands.clear();}
  if(resetPending){
   model.SetWorldPoseCmd(ecm,resetPose);
   for(auto*l:{&body,&leftLink,&rightLink,&ballastLink}){l->SetLinearVelocity(ecm,Vector3d::Zero);l->SetAngularVelocity(ecm,Vector3d::Zero);}
   for(auto*j:{&leftJoint,&rightJoint}){j->ResetPosition(ecm,{-.6981317});j->ResetVelocity(ecm,{0});}
   slider.ResetPosition(ecm,{0});slider.ResetVelocity(ecm,{0});resetPending=false;return;
  }
  status=mode;reason="";double collective=0,steering=0;double pitch=rotation.Pitch(),yaw=rotation.Yaw();
  if(mode=="swim"||mode=="climb"||mode=="descend")collective=.7;
  if(mode=="turn_left"||mode=="turn_right"){collective=.65;steering=mode=="turn_left"?.9:-.9;}
  if(mode=="manual"){
   collective=time<manualUntil?throttle:0;steering=time<manualUntil?turn:0;
   if(time>=manualUntil){status="gliding";reason="manual_input_timeout";noseUp=0;}
  }
  bool blocked=false;
  if(mode=="goto"){
   Vector3d delta=goal-position;double distance=std::hypot(delta.X(),delta.Y());
   // Slow down before the goal; wings cannot reverse thrust or translate vertically in place.
   double stopping=std::hypot(velocity.X(),velocity.Y())*2.4+.12;
   double wanted=std::atan2(delta.Y(),delta.X());double yawError=wrap(wanted-yaw);
   collective=distance<stopping?0:clip(distance*.45,.25,.9);
   if(distance>.2){controlYaw=wanted;steering=clip(1.8*yawError-1.6*omega.Z(),-1,1);if(std::abs(yawError)>.7)collective=std::max(collective,.55);}
   noseUp=clip((goal.Z()-position.Z())*50-velocity.Z()*35,-22,22);
   if(distance<.25&&std::abs(goal.Z()-position.Z())<.2){collective=0;status="gliding_at_waypoint";noseUp=0;}
   if(safety){for(auto&r:obstacles)if(crosses(position.X(),position.Y(),delta.X(),delta.Y(),r,.55,1))blocked=true;}
  }
  // Forward-only ideal proximity guard; no person/marker and no route planning.
  Vector3d front=rotation.RotateVector(Vector3d(1,0,0));double range=10;
  for(double d=.05;d<8;d+=.05){auto p=position+front*d;bool hit=std::abs(p.X())>4.75||std::abs(p.Y())>3.75;
   for(auto&r:obstacles)if(std::abs(p.X()-r.x)<r.hx&&std::abs(p.Y()-r.y)<r.hy)hit=true;if(hit){range=d;break;}}
  if(safety&&range<.75+std::hypot(velocity.X(),velocity.Y())*2.5)blocked=true;
  if(blocked){collective=0;status="gliding_obstacle_stop";reason="wings_stopped_for_clearance";}
  double targetFL=0,targetFR=0,targetAL=0,targetAR=0;
  if(collective>.005){
   double driveL=clip(collective-steering*.5,0,1),driveR=clip(collective+steering*.5,0,1);
   targetFL=driveL>.01?.3+1.2*std::sqrt(driveL):0;targetFR=driveR>.01?.3+1.2*std::sqrt(driveR):0;
   targetAL=50*std::sqrt(driveL);targetAR=50*std::sqrt(driveR);
  }
  if(mode=="wings"&&!blocked){targetFL=requestedFrequencyL;targetFR=requestedFrequencyR;targetAL=requestedAmplitudeL;targetAR=requestedAmplitudeR;}
  if(mode=="float"||mode=="off"||mode=="ballast"||!enabled)targetFL=targetFR=targetAL=targetAR=0;
  // No sub-minimum active frequencies: the measured prototype used 0.3–1.5 Hz.
  if(targetFL>0)targetFL=clip(targetFL,.3,1.5);if(targetFR>0)targetFR=clip(targetFR,.3,1.5);
  double alpha=1-std::exp(-dt/.2);frequencyL+=alpha*(targetFL-frequencyL);frequencyR+=alpha*(targetFR-frequencyR);
  amplitudeL+=alpha*(targetAL*DEG-amplitudeL);amplitudeR+=alpha*(targetAR*DEG-amplitudeR);
  phaseL=std::fmod(phaseL+2*M_PI*frequencyL*dt,2*M_PI);phaseR=std::fmod(phaseR+2*M_PI*frequencyR*dt,2*M_PI);
  double desiredQL=-40*DEG+amplitudeL*std::sin(phaseL),desiredQR=-40*DEG+amplitudeR*std::sin(phaseR);
  double desiredDL=amplitudeL*2*M_PI*frequencyL*std::cos(phaseL),desiredDR=amplitudeR*2*M_PI*frequencyR*std::cos(phaseR);
  wingServoTorqueL=clip(.025*(desiredQL-qL)+.0009*(desiredDL-dqL),-.02,.02);
  wingServoTorqueR=clip(.025*(desiredQR-qR)+.0009*(desiredDR-dqR),-.02,.02);
  // Internal mass is a real prismatic body. Its gravitational torque pitches the envelope.
  if(mode!="ballast"){
   double desiredPitch=-noseUp*DEG;
   ballastGoal=clip(.26*(desiredPitch-pitch)-.12*omega.Y(),-.11,.11);
   if(mode=="float"||mode=="off")ballastGoal=0;
  }
  ballastForce=clip(1.2*(ballastGoal-ballast)-.11*ballastVelocity,-.08,.08);
  bool actuated=enabled&&mode!="off";
  leftJoint.SetForce(ecm,{actuated?wingServoTorqueL:0});rightJoint.SetForce(ecm,{actuated?wingServoTorqueR:0});slider.SetForce(ecm,{actuated?ballastForce:0});
  // Phenomenological, cycle-rectified membrane model. Actual measured joint speeds
  // (not the command) generate forward thrust; there is no separate heave actuator.
  // F = .5 rho S C_T (r_eff * qdot)^2. C_T requires physical calibration.
  auto wingForce=[&](double speed,double angle){return .5*RHO*.026*thrustCoefficient*std::pow(.11*speed,2)*(.75+.25*std::cos(2*angle));};
  forceL=wingForce(dqL,qL);forceR=wingForce(dqR,qR);
  // Apply membrane forces at the physical wing links. Native joints transmit them.
  leftLink.AddWorldForce(ecm,rotation.RotateVector(Vector3d(forceL,0,0)));
  rightLink.AddWorldForce(ecm,rotation.RotateVector(Vector3d(forceR,0,0)));
  if(lift)body.AddWorldForce(ecm,Vector3d(0,0,RHO*VOLUME*9.81),Vector3d(0,0,.018));
  auto rel=velocity-wind;
  Vector3d drag(-.025*rel.X()-.035*std::abs(rel.X())*rel.X(),-.05*rel.Y()-.09*std::abs(rel.Y())*rel.Y(),-.035*rel.Z()-.06*std::abs(rel.Z())*rel.Z());
  // Trim is a passive constant load error, not a commanded vertical motor.
  drag.Z(drag.Z()-trimForce);
  body.AddWorldWrench(ecm,drag+(time<pushUntil?push:Vector3d::Zero),-omega*.008+(time<pushUntil?pushTorque:Vector3d::Zero));
 }
 void PostUpdate(const gz::sim::UpdateInfo&info,const gz::sim::EntityComponentManager&ecm)override{
  if(info.paused||time-lastPublish<.04)return;lastPublish=time;auto pose=body.WorldPose(ecm).value_or(Pose3d());
  auto vec=[](const Vector3d&v){Json::Value a(Json::arrayValue);a.append(v.X());a.append(v.Y());a.append(v.Z());return a;};
  Json::Value s;s["time"]=time;s["ack"]=ack;s["model_revision"]="helium_flapping_v2";s["mode"]=mode;s["status"]=status;s["reason"]=reason;
  s["position"]=vec(pose.Pos());s["velocity"]=vec(body.WorldLinearVelocity(ecm).value_or(Vector3d::Zero));s["angles"]=vec(pose.Rot().Euler());s["angular_velocity"]=vec(omega);s["goal"]=vec(goal);
  s["wing_angles"]=vec(Vector3d(jointValue(leftJoint,ecm),jointValue(rightJoint,ecm),0));s["wing_velocities"]=vec(Vector3d(jointValue(leftJoint,ecm,true),jointValue(rightJoint,ecm,true),0));
  s["wing_hz"]=vec(Vector3d(frequencyL,frequencyR,0));s["wing_amplitudes_deg"]=vec(Vector3d(amplitudeL/DEG,amplitudeR/DEG,0));s["wing_forces"]=vec(Vector3d(forceL,forceR,0));
  s["wing_servo_torques"]=vec(Vector3d(wingServoTorqueL,wingServoTorqueR,0));s["ballast_position"]=jointValue(slider,ecm);s["ballast_goal"]=ballastGoal;s["nose_up_command_deg"]=noseUp;
  s["mass"]=TOTAL_MASS;s["dry_mass"]=(RHO-.164)*VOLUME;s["volume_litres"]=VOLUME*1000;s["buoyancy"]=lift?RHO*VOLUME*9.81:0;s["weight_force"]=TOTAL_MASS*9.81;
  s["wind"]=vec(wind);s["trim_grams"]=trimForce*1000/9.81;s["thrust_coefficient"]=thrustCoefficient;s["enabled"]=enabled;s["lift_enabled"]=lift;s["safety"]=safety;
  s["propulsion_model"]="unvalidated_rectified_membrane";s["dimensions_m"]=vec(Vector3d(1,.45,.33));
  gz::msgs::StringMsg msg;Json::StreamWriterBuilder b;b["indentation"]="";msg.set_data(Json::writeString(b,s));publisher.Publish(msg);
 }
};
}
GZ_ADD_PLUGIN(companion::Companion,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(companion::Companion,"companion::Companion")
