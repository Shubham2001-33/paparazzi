#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "modules/ins/ins_int.h"
#include "state.h"
#include "modules/imu/imu.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"
#include "modules/loggers/logger_file.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/pprzlink_device.h"
#include "pprzlink/intermcu_msg.h"
#include "modules/ff/ff.h"



struct FloatVect3 pos_leader_bf = {0.0f, 0.0f, 0.0f};
struct FloatVect3 P_cmd = {0.0f,0.0f,0.0f};
struct FloatVect3 P_cmd_aug = {0.0f,0.0f,0.0f};
struct FloatVect3 P_cmd_rot = {0.0f,0.0f,0.0f};
struct FloatVect3 P_s = {0.0f,0.0f,0.0f};
struct FloatVect3 P_l = {0.0f,0.0f,0.0f};
struct FloatVect3 XZ_sp = {0.0f,0.0f,0.0f};
struct FloatVect3 Y_sp = {0.0f,0.0f,0.0f};
struct FloatVect3 XZ_sp_rot = {0.0f,0.0f,0.0f};
struct FloatVect3 Y_sp_rot = {0.0f,0.0f,0.0f};
struct FloatVect3 XYZ_sp_rot = {0.0f,0.0f,0.0f};
struct FloatVect3 V_wind = {0.0f,0.0f,0.0f};
struct FloatVect3 V_al = {0.0f,0.0f,0.0f}; //airspeed of leader (NED)
struct FloatVect3 V_l = {0.0f,0.0f,0.0f}; //inertial speed of leader (NED)
float heading = 0;
float heading_rate = 0;
float heading_rate_adjusted = 0;
float r=0;
float R=0;
float theta=0;
float angle_crab =0;
float angle_heading = 0;
//struct FloatRMat P_l_nedtobodyrot[9] = {1.0f,0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,1.0f};
//struct FloatRMat rmat[9] = {1.0f,0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,1.0f};
bool begin = false;

struct FloatRMat DCM_comp(float angle);




#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

static void send_leader_info(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_FORMATION_SP(trans, dev, AC_ID, &P_s.x,&P_s.y,&P_s.z, &heading);
}
#endif

void ff_init(void)
{ 
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FORMATION_SP, send_leader_info);
  #endif
}

void ff_start(void)
{
  ff_ff_run_status = MODULES_RUN; //why?
  printf("%s","SSSS");

  P_l.x = stateGetPositionNed_f()->x;
  //printf("Plx leader side:%f",P_l.x);
  P_l.y = stateGetPositionNed_f()->y;
  //printf("Ply leader side:%f",P_l.y);
  P_l.z = stateGetPositionNed_f()->z;
  //printf("Plz leader side:%f",P_l.z);
  
  
 
  V_l.x = stateGetSpeedNed_f()->x;
  //printf("Vlx:%f",V_l.x);
  V_l.y = stateGetSpeedNed_f()->y;
  //printf("Vly:%f",V_l.y);
  V_l.z = stateGetSpeedNed_f()->z;
  //printf("Vlz:%f",V_l.z);

  V_wind.x = stateGetWindspeed_f()->x;
  //printf("Vwx:%f",V_l.x);
  V_wind.y = stateGetWindspeed_f()->y;
  //printf("Vwy:%f",V_l.y);
  V_wind.z = stateGetWindspeed_f()->z;
  //printf("Vwz:%f",V_l.z);
  followstart = true;
  //printf("followstart:%d",followstart);
  begin = true;
  //printf("begin:%d",begin);
}


void ff_stop(void)
{
  begin = false;
}



void ff_run()
{
  if(begin==false)
  {
    return;
  }
  //printf("I am in run: %s");
  heading = stateGetNedToBodyEulers_f()->psi;
  //printf("heading:%f",heading);
  P_l.x = stateGetPositionNed_f()->x;
 // printf("Plx:%f",P_l.x);
  P_l.y = stateGetPositionNed_f()->y;
  //printf("Ply:%f",P_l.y);
  P_l.z = stateGetPositionNed_f()->z; //get position of leader drone in NED frame
 // printf("Plz:%f",P_l.z);
  


  //struct FloatRMat *P_l_nedtobodyrot = stateGetNedToBodyRMat_f();  //get rotation matrix to convert position of leader
                                                  // in NED frame to Body frame of leader

  RMAT_VECT3_MUL(pos_leader_bf,*stateGetNedToBodyRMat_f(),P_l); //leader position in it's body frame
  //printf("posleaderbfx:%f",pos_leader_bf.x);
  //printf("posleaderbfy:%f",pos_leader_bf.y);
  //printf("posleaderbfz:%f",pos_leader_bf.z);

   
  V_l.x = stateGetSpeedNed_f()->x;
  V_l.y = stateGetSpeedNed_f()->y;
  V_l.z = stateGetSpeedNed_f()->z;
  
  V_wind.x = stateGetWindspeed_f()->x;
  V_wind.y = stateGetWindspeed_f()->y;
  V_wind.z = stateGetWindspeed_f()->z;
  
  VECT3_DIFF(V_al,V_l,V_wind);
  //printf("Valx:%f",V_al.x);
  //printf("Valy:%f",V_al.y);
  //printf("Valz:%f",V_al.z);

  P_cmd.x = pos_leader_bf.x -3; //x position of P_cmd in body frame of leader
  //printf("P_cmdx:%f",P_cmd.x);
  P_cmd.y = pos_leader_bf.y;     //y position of P_cmd in body frame of leader
  //printf("P_cmdy:%f",P_cmd.y);
  P_cmd.z = pos_leader_bf.z -2;    //z position of P_cmd in body frame of leader
  //printf("P_cmdz:%f",P_cmd.z);
  //printf("dotpr:%f",VECT3_DOT_PRODUCT(V_al,V_l));
  //printf("normVAL:%f", sqrt(VECT3_NORM2(V_al)));
  //printf("dotpr:%f",sqrt(VECT3_NORM2(V_l)));
  //printf("crabvcomp:%f", VECT3_DOT_PRODUCT(V_al,V_l)/ (sqrt(VECT3_NORM2(V_al))*sqrt(VECT3_NORM2(V_l))));
  angle_crab = acosf(VECT3_DOT_PRODUCT(V_al,V_l)/ (sqrt(VECT3_NORM2(V_al))*sqrt(VECT3_NORM2(V_l)))); //crab angle = dot prod (Val,Vl) / norm(Vl).norm(Val) ; P_cmd is rotated by this angle
  //printf("crab angle:%f",angle_crab);
  RMAT_VECT3_TRANSP_MUL(P_cmd_aug,DCM_comp(angle_crab),P_cmd); //Pcmd augmentation
  //printf("P_cmd_augx:%f",P_cmd_aug.x);
  //printf("P_cmd_augy:%f",P_cmd_aug.y);
  //printf("P_cmd_augz:%f",P_cmd_aug.z);

  heading_rate = stateGetBodyRates_f()->r; //doubt? //rate of change of leader's heading, gyro measured   
  heading_rate_adjusted = heading_rate* VECT3_DOT_PRODUCT(V_l,V_al)/VECT3_DOT_PRODUCT(V_l,V_l); //heading rate change of leader adjusted for wind
   
  //computing arc characteristics

  r=VECT3_NORM2(V_l)/heading_rate_adjusted; //radius of arc
  theta = P_cmd_aug.x/r; //angle of arc
  R=2*r*sin(theta/2);  //chord length of arc


  //computing final augmented setpoint P_s to be followed by the follower UAV 

  XZ_sp.x = R;
  XZ_sp.z = P_cmd_aug.z;
  RMAT_VECT3_MUL(XZ_sp_rot,DCM_comp(theta/2),XZ_sp);

  Y_sp.y = P_cmd_aug.y; 
  RMAT_VECT3_MUL(Y_sp_rot,DCM_comp(theta),Y_sp);

  VECT3_ADD(XZ_sp_rot,Y_sp_rot);

  angle_heading = stateGetHorizontalSpeedDir_f();  //direction of V_l

  RMAT_VECT3_MUL(XYZ_sp_rot,DCM_comp(angle_heading),XZ_sp_rot); //angle_heading not calculated yet, check line again?
  VECT3_SUM(P_s, XYZ_sp_rot,P_l); //augmented setpoint in NAV frame
  printf("Ps_x:%f",P_s.x);
  printf("Ps_y:%f",P_s.y);
  printf("Ps_z:%f",P_s.z);

}


struct FloatRMat DCM_comp(float angle) {
    struct FloatRMat DCM;
    // Populate the rotation matrix for a given angle
    DCM.m[0] = cos(angle);  // First row
    DCM.m[1] = -sin(angle);
    DCM.m[2] = 0.0;

    DCM.m[3] = sin(angle);  // Second row
    DCM.m[4] = cos(angle);
    DCM.m[5] = 0.0;

    DCM.m[6] = 0.0;         // Third row
    DCM.m[7] = 0.0;
    DCM.m[8] = 1.0;

    return DCM;
}

