#include <stdint.h>


void send_cmd (uint8_t cmd);
void stop_traction(void);
void stop_brake(void);
void move_traction_up(void);
void move_traction_down(void);
void move_brake_up(void);
void move_brake_down(void);

void release_brake(void);
void activate_brake(void);
void set_traction_to_hold(void);
void release_traction(void);
void move_brake_to_pos(void);
void move_traction_to_pos(void);
