/*****************************
 *
********************************/
#ifndef _sdk_info_load_H
#define  _sdk_info_load_H

#include <iostream>
#include <fstream>
#include <string>

using namespace std;

//------------- UBUNTU/LINUX ONLY terminal color---------------
#define RESETCOLOR   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

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

