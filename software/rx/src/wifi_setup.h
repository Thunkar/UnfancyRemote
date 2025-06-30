#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include "ESPAsyncWebServer.h"
#include <DNSServer.h>
#include "SPIFFS.h"
#include "state.h"
#include "stats.h"
#include "config.h"

void setupServer();
TaskResult doServerWork(unsigned long now);
