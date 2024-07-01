#ifndef FF_H
#define FF_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/pprzlink_device.h"

struct ffmessage {
  struct link_device *device;       ///< Device used for communication
  struct pprz_transport transport;  ///< Transport over communication line (PPRZ)
  uint8_t time_since_last_frame;    ///< Time since last frame
  bool enabled;                     ///< If the InterMCU communication is enabled
  bool msg_available;               ///< If we have an InterMCU message
};
/*struct formation_flight {
  struct FloatVect3 P_s;
  struct FloatVect3 P_f; 
  struct FloatVect3 rel_pos; 
  struct FloatVect3 v_f; 
  struct FloatVect3 rel_vel;
  struct FloatVect3 rel_vel_err; 
  struct FloatVect3 accel_des;
};*/



extern void ff_f_start(void);
extern void ff_f_stop(void);
extern void ff_f_init(void);
extern void ff_f_run(void);
extern void ff_f_event(void);
extern void ff_parse_FORMATION_SP(uint8_t *buf);
//extern struct formation_flight *ff_info_logger(void);
#endif


