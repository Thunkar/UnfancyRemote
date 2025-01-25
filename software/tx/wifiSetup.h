#include <WiFi.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include <DNSServer.h>

void setupServer();

extern DNSServer dnsServer;
extern AsyncWebServer server;