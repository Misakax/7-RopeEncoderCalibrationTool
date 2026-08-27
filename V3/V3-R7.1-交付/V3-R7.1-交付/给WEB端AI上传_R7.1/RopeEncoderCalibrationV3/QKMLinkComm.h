#pragma once

bool QKMLinkConnect(CString strIP, PVOID lParam);

int DecodeRobotWhereAngle(CString tMessage);

extern int QKMLinkSend(CString msg);

bool QKMLinkInit();

bool QKMLinkSetTargetPacketID(int target);

bool QKMLinkDisconnect();

bool QKMLinkEventReset();