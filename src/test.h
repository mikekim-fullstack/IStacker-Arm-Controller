#include <Arduino.h>
class Test
{
public:
    void print(const char *msg)
    {
        Serial.println(msg);
    }
};