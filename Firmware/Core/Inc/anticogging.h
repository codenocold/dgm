#ifndef __ANTICOGGING_H__
#define __ANTICOGGING_H__

#include "main.h"
#include <stdbool.h>
#include "controller.h"

extern bool AnticoggingValid;

void ANTICOGGING_start(void);
void ANTICOGGING_abort(void);
bool ANTICOGGING_is_running(void);
void ANTICOGGING_loop(ControllerStruct *controller);

#endif
