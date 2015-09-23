/*****************************
 *
********************************/
#ifndef _sdk_info_load_H
#define  _sdk_info_load_H

#include <iostream>
#include <fstream>
#include <string>

using namespace std;

/////////////////
/// \brief The dji_sdk_app_info class
/// to load param of dji sdk apps
///
class dji_sdk_app_info
{

public:
    dji_sdk_app_info();
    dji_sdk_app_info(string serial_name,int baud_rate,
                     int app_id,int app_version,int app_api_level,
                     string app_bundle_id, string app_enc_key);
    ~dji_sdk_app_info();

    struct serial_port_info
    {
        std::string serial_name;
        int baud_rate;
    };
    serial_port_info serial_port;

    struct onboard_app_info
    {
        int app_id;
        int app_api_level;
        int app_version;
        std::string	app_bundle_id;
        std::string app_enc_key;
    };
    onboard_app_info app_info;

    void loadAppInfo(string path1, string path2);
    void loadAppInfo(string path);

    void saveToFile(string path, bool is_xml);

private:

    void readFromFile(string path);
    void showDebugInfo();

};

#endif

