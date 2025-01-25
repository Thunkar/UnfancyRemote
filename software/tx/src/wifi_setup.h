#pragma once

#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <DNSServer.h>

void setupServer();
bool processDNSRequest(unsigned long now);

extern AsyncWebServer server;
extern DNSServer dnsServer;
