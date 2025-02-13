#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <DNSServer.h>
#include "SPIFFS.h"
#include "state.h"
#include "config.h"

void setupServer();
bool doServerWork(unsigned long now);
