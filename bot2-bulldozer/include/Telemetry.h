#// Contains the functions responsible for receiving/publishing the
// bulldozer's telemetry data through the MQTT network.
//
// Telemetry includes:
// - Current robot position
// - Wheel encoder positions
// - Robot movement status

#pragma once

void handleTelemetry();
