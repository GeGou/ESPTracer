#pragma once

enum DeviceState {
    STATE_DISARMED = 0,
    STATE_ARMED = 1,
    STATE_ALARM = 2
};

const char* stateNameOf(int s); // μόνο δήλωση (prototype), όχι ορισμός

extern int rtcDeviceState; // αν θέλεις να διαβάζεις/γράφεις και το ίδιο το state από αλλού

// MQTT commands (μέσω MQTT_TOPIC_COMMAND) -- λειτουργούν ΜΟΝΟ όσο η
// συσκευή είναι ήδη ξύπνια/συνδεδεμένη (δηλαδή κατά τη διάρκεια ALARM).
extern volatile bool stopAlarmRequested;
extern volatile bool powerOffRequested;