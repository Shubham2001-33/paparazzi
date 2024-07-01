#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "modules/ff_f/ff_f.h"
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "modules/ins/ins_int.h"
#include "state.h"
#include "modules/imu/imu.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "filters/low_pass_filter.h"
#include "modules/core/abi.h"
#include "modules/loggers/logger_file.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/pprzlink_device.h"
#include "pprzlink/intermcu_msg.h"


#define FF_PERIODIC_FREQUENCY 500.0

struct ffmessage ffinfo = {
  .device = (&((DOWNLINK_DEVICE).device)),
  .enabled = true,
  .msg_available = false,
};

uint8_t ff_msg_buf[256] __attribute__((aligned)); 

struct FloatVect3 P_s = {0.0,0.0,0.0};
//struct FloatVect3 test_point = {-19,14.3,-70};
struct FloatVect3 P_f = {0.0,0.0,0.0};
struct FloatVect3 v_f = {0.0,0.0,0.0};
struct FloatVect3 rel_pos = {0.0,0.0,0.0};
struct FloatVect3 rel_vel = {0.0,0.0,0.0};
struct FloatVect3 rel_vel_err = {0.0,0.0,0.0};
struct FloatVect3 accel_des = {0.0,0.0,0.0};
//struct formation_flight ff_log;
float position_gain = 0.0;
float speed_gain = 0.0;
float heading_cmd =0.0;
int counter = 0;
struct StabilizationSetpoint quat_follower_des;
struct Int32Quat q_des;
bool in_flight;
bool begin = false;
bool message= false;
uint8_t flag = 1;
int gain = 0;


void ff_parse_FORMATION_SP(uint8_t *buf);
static void saturate(struct FloatVect3 *vector, int ind); 
//void ff_info(struct formation_flight *ff_log, struct FloatVect3 *leader_pos, struct FloatVect3 *follower_pos, struct FloatVect3 *relative_pos_fl, struct FloatVect3 *follower_vel, struct FloatVect3 *relative_vel_fl, struct FloatVect3 *rel_vel_err_fl, struct FloatVect3 *accel_desired);
//struct formation_flight *ff_info_logger(void);

void ff_f_init()
{
  //guidance_indi_init();
  //guidance_indi_enter();
  pprz_transport_init(&ffinfo.transport); 

}

void ff_f_start()
{
  ff_f_ff_f_run_status = MODULES_RUN;
  position_gain = 3;
  speed_gain = 2;
  followstartfollower = true;
  begin = true;
  printf("%s","ffstart");
}

void ff_f_stop(void)
{
  begin = false;
}





void ff_f_run(void)
{
  if (begin == false)
  {
    return;
  }

  //printf("%s","Im in run");
  P_f.x= stateGetPositionNed_f()->x;
  //printf("Follower x position:%f",P_f.x);
  P_f.y = stateGetPositionNed_f()->y;
  //printf("Follower y position:%f",P_f.y);
  P_f.z = stateGetPositionNed_f()->z;
  //printf("Follower z position:%f",P_f.z);


  /*rel_pos.x = test_point.x - P_f.x;
  //printf("relposx%f",rel_pos.x);
  rel_pos.y = test_point.y - P_f.y;
  //printf("relposy%f",rel_pos.y);
  rel_pos.z = test_point.z - P_f.z;
  printf("relposz%f",rel_pos.z);*/
  
  rel_pos.x = P_s.x - P_f.x;
  printf("relposx%f",rel_pos.x);
  rel_pos.y = P_s.y - P_f.y;
  printf("relposy%f",rel_pos.y);
  rel_pos.z = P_s.z - P_f.z;
  printf("relposz%f",rel_pos.z);
  
  VECT3_SMUL(rel_vel,rel_pos,position_gain);

  Bound(rel_vel.x, -10.0, 10.0);
  Bound(rel_vel.y, -10.0, 10.0);
  Bound(rel_vel.z, -10.0, 10.0);
  
  v_f.x= stateGetSpeedNed_f()->x;
  v_f.y= stateGetSpeedNed_f()->y;
  v_f.z= stateGetSpeedNed_f()->z;
  rel_vel_err.x = rel_vel.x - v_f.x;
  //printf("relvelerrx%f",rel_vel.x);
  rel_vel_err.y = rel_vel.y - v_f.y;
  //printf("relvelerry%f",rel_vel.y);
  rel_vel_err.z = rel_vel.z - v_f.z;
  //printf("relvelerrz%f",rel_vel.z);
  //printf("vfz%f",v_f.z);

  VECT3_SMUL(accel_des,rel_vel_err,speed_gain);
  Bound(accel_des.x, -6.0, 6.0)
  Bound(accel_des.y, -6.0, 6.0);
  Bound(accel_des.z, -6.0, 6.0);
  //printf("desaccelx:%f",accel_des.x);
  //printf("desaccely:%f",accel_des.y);
  //printf("desaccelz:%f",accel_des.z);
  //printf("desaccelx%f",accel_des.x);
  //printf("desaccely%f",accel_des.y);
  //printf("desaccelz%f",accel_des.z);
  //saturate(&accel_des, 1);
  //ff_info(&ff_log, &P_s, &P_f, &rel_pos, &v_f, &rel_vel, &rel_vel_err, &accel_des);
  AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, flag, &accel_des,heading_cmd);
  //AbiSendMsgACCEL_SP(ACCEL_SP_FCR_ID, flag, &accel_des);
}

void ff_f_event(void)
{
  /* Parse incoming bytes */
  if (ffinfo.enabled) {
    pprz_check_and_parse(ffinfo.device, &ffinfo.transport, ff_msg_buf, &ffinfo.msg_available);
    //PRINT("%d", ffinfo.msg_available);
    if (counter ==50){
      //PRINT("%d", ffinfo.msg_available);
      counter = 0;
    }
    if (ffinfo.msg_available) {
      message = true;
      uint8_t class_id = pprzlink_get_msg_class_id(ff_msg_buf);
      
      ffinfo.time_since_last_frame = 0;
      dl_parse_msg(ffinfo.device, &ffinfo.transport.trans_tx, ff_msg_buf);
    }
    ffinfo.msg_available = false;
    counter++;
  }
}

static void saturate(struct FloatVect3 *vector, int ind) {
  if (ind == 1){
    vector->x = fmax(-2.0, fmin(2.0, vector->x));
    vector->y = fmax(-2.0, fmin(2.0, vector->y));
    vector->z = fmax(-2.0, fmin(2.0, vector->z));
  }
  else if(ind == 2){
    vector->x = fmax(-3.0, fmin(3.0, vector->x));
    vector->y = fmax(-3.0, fmin(3.0, vector->y));
    vector->z = fmax(-3.0, fmin(3.0, vector->z));
  }
  else {
    vector->x = fmax(-1.0, fmin(1.0, vector->x));
    vector->y = fmax(-1.0, fmin(1.0, vector->y));
    vector->z = fmax(-1.0, fmin(1.0, vector->z));    
  }

}


/*void ff_info(struct formation_flight *ff_log, struct FloatVect3 *leader_pos, struct FloatVect3 *follower_pos, struct FloatVect3 *relative_pos_fl, struct FloatVect3 *follower_vel, struct FloatVect3 *relative_vel_fl, struct FloatVect3 *rel_vel_err_fl, struct FloatVect3 *accel_desired){
  log->P_s = *leader_pos; 
  log->P_f = *follower_pos;
  log->rel_pos = *relative_pos_fl;
  log->v_f = *follower_vel;
  log->rel_vel = *relative_vel_fl;
  log->rel_vel_err = *rel_vel_err_fl;
  log->accel_des = *accel_desired;

}

struct formation_flight *ff_info_logger(void){
  return &ff_log;
}*/



void ff_parse_FORMATION_SP(uint8_t *buf)
{
  P_s.x = DL_FORMATION_SP_to_follower_Ps_x(buf);
  P_s.y = DL_FORMATION_SP_to_follower_Ps_y(buf);
  P_s.z = DL_FORMATION_SP_to_follower_Ps_z(buf);
  heading_cmd = DL_FORMATION_SP_to_follower_headingcmd(buf);
  printf("Setpoint to be followed Psx:%f", P_s.x);
  printf("Setpoint to be followed Psy:%f", P_s.y);
  printf("Setpoint to be followed Psz:%f", P_s.z);
  printf("Heading to be followed:%f", heading_cmd);
}