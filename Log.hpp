#pragma once
// ---------------------------------------------------------------------------
// Dziennik zdarzeń w pamięci (ostatnie LOG_LINES wpisów) - widoczny w panelu (karta "Dziennik")
// i na Serial. Pozwala zobaczyć, DLACZEGO automat skręcił, stanął albo zakończył pracę.
// ---------------------------------------------------------------------------
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define LOG_LINES 28
#define LOG_LEN   96

static char logBuf[LOG_LINES][LOG_LEN];
volatile uint32_t logSeqNo = 0;                 // licznik wpisów (panel pobiera dziennik, gdy się zmieni)
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
void (*logSink)(const char*) = NULL;            // opcjonalny odbiornik wpisów (karta SD)

void logEvent(const char* fmt, ...) {
  char msg[LOG_LEN - 12];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  for (char* p = msg; *p; p++) if (*p == '"' || *p == '\\') *p = '\'';   // bezpieczne dla JSON
  char line[LOG_LEN];
  unsigned long t = millis();
  snprintf(line, sizeof(line), "%lu.%lu  %s", t / 1000, (t % 1000) / 100, msg);
  portENTER_CRITICAL(&logMux);
  memcpy(logBuf[logSeqNo % LOG_LINES], line, LOG_LEN);
  logSeqNo = logSeqNo + 1;
  portEXIT_CRITICAL(&logMux);
  Serial.println(line);
  if (logSink) logSink(line);
}

// Dziennik (biezaca sesja) jako zwykly tekst do zapisu na karte
template <class P> void Log_dump(P& p) {
  uint32_t seq = logSeqNo;
  uint32_t n = seq < LOG_LINES ? seq : LOG_LINES;
  for (uint32_t i = 0; i < n; i++) {
    char line[LOG_LEN];
    portENTER_CRITICAL(&logMux);
    memcpy(line, logBuf[(seq - n + i) % LOG_LINES], LOG_LEN);
    portEXIT_CRITICAL(&logMux);
    line[LOG_LEN - 1] = 0;
    p.println(line);
  }
}

// Dziennik jako tablica JSON, najnowsze na końcu
String Log_json() {
  String out;
  out.reserve(LOG_LINES * 70);
  out += "[";
  uint32_t seq = logSeqNo;
  uint32_t n = seq < LOG_LINES ? seq : LOG_LINES;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t idx = (seq - n + i) % LOG_LINES;
    char line[LOG_LEN];
    portENTER_CRITICAL(&logMux);
    memcpy(line, logBuf[idx], LOG_LEN);
    portEXIT_CRITICAL(&logMux);
    line[LOG_LEN - 1] = 0;
    if (i) out += ",";
    out += "\"";
    out += line;
    out += "\"";
  }
  out += "]";
  return out;
}
