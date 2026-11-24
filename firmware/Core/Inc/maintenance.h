#ifndef BAIZE_MAINTENANCE_H
#define BAIZE_MAINTENANCE_H

#include "baize_protocol.h"

void maintenance_start(void);
bool maintenance_inhibits_motion(void);
void maintenance_check_deadline(uint32_t now_ms);
void maintenance_handle(const MaintenanceRequest *request, MaintenanceReply *reply);

#endif
