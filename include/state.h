#pragma once

enum DeviceState {
    STATE_DISARMED = 0,
    STATE_ARMED = 1,
    STATE_ALARM = 2
};

const char* stateNameOf(int s);

extern int rtcDeviceState; // reads/writes the state itself from elsewhere

// MQTT commands (via MQTT_TOPIC_COMMAND) -- work ONLY as long as the
// device is already awake/connected (i.e., during an ALARM).
extern volatile bool stopAlarmRequested;
extern volatile bool powerOffRequested;