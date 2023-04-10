#pragma once

#define Log(...)                  \
  do                              \
  {                               \
    if (Serial)                   \
      Serial.printf(__VA_ARGS__); \
  } while (0)
