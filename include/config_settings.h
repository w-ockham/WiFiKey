#ifndef CONFIG_SETTING
#define CONFIG_SETTING
#include <ArduinoJson.h>
#include <Preferences.h>
#include <SPIFFS.h>

class KeyConfig
{
private:
    Preferences prefs;
    DynamicJsonDocument &json_prefs;
    const char *ns_wifikey = "wifikey";
    const char *key_wifikey = "settings";

public:
    KeyConfig(DynamicJsonDocument &d) : json_prefs(d){};

    ~KeyConfig(){};

    void begin()
    {
        prefs.begin(ns_wifikey, false);
    }

    void end()
    {
        prefs.end();
    }

    size_t savePrefs()
    {
        String prefStr;
        size_t rs;

        begin();
        serializeJson(json_prefs, prefStr);
#if DEBUG_LEVEL > 1
        Serial.println("Save Config:" + prefStr);
#endif
        rs = prefs.putString(key_wifikey, prefStr);
        end();
        return rs;
    }

    boolean loadPrefs()
    {
        String prefStr;

        begin();
        prefStr = prefs.getString(key_wifikey);
        end();

        if (prefStr == "")
        {
            Serial.println("Load Config from NVM fails");
            return false;
        }
        deserializeJson(json_prefs, prefStr);
#if DEBUG_LEVEL > 1
        Serial.print("Load Config from NVM");
        Serial.println(prefStr);
#endif
        return true;
    }

    void clearPref()
    {
        begin();
        prefs.remove(key_wifikey);
        end();
    }

    boolean readFile(const char *fname)
    {
        if (SPIFFS.exists(fname))
        {
            File file = SPIFFS.open(fname, "r");
            DeserializationError error = deserializeJson(json_prefs, file);
            if (error)
            {
                Serial.print("JSON File format error:");
                Serial.println(fname);
                return false;
            }
            return true;
        }
        Serial.print("JSON file not found:");
        Serial.println(fname);
        return false;
    }
};
#endif