
#ifndef DYNAMIXEL_LIST_H_
#define DYNAMIXEL_LIST_H_


#include "dynamixel_pro.h"
#include <iostream>
#include <unistd.h>


using namespace DXL_PRO;

bool check_vaild_dxl_from_id(int id);
dxl_pro_data& dxl_from_id(int id);

extern dxl_pro_data dxlLists[8][10];
extern dxl_gains dxlGains[8][10];

void make_dxl_count();
bool dxl_initailize();
void make_inverse_access_data();
bool dynamixel_motor_init();
void motion_init_proc(bool *isDone);
void motor_test();

#endif
