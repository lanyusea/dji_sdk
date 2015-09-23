
#include "dji_sdk_auxiliary.h"

/////
/// \brief dji_sdk_app_info::dji_sdk_app_info
/// init class, useless param
dji_sdk_app_info::dji_sdk_app_info()
{
    // default serial port info
    serial_port.serial_name = "/dev/ttyUSB0";
    serial_port.baud_rate = 115200;

    // default sdk app info
    app_info.app_id = 10086;
    app_info.app_version = 0;
    app_info.app_api_level = 1.0;
    app_info.app_bundle_id = "12345";
    app_info.app_enc_key = "hahahaha";
}

/////
/// \brief dji_sdk_app_info::dji_sdk_app_info
/// init class, param can be set in func
dji_sdk_app_info::dji_sdk_app_info(string serial_name,int baud_rate,
                                   int app_id,int app_version,int app_api_level,
                                   string app_bundle_id, string app_enc_key)
{
    // default serial port info
    serial_port.serial_name = serial_name;
    serial_port.baud_rate = baud_rate;

    // default sdk app info
    app_info.app_id = app_id;
    app_info.app_version = app_version;
    app_info.app_api_level = app_api_level;
    app_info.app_bundle_id = app_bundle_id;
    app_info.app_enc_key = app_enc_key;
}

dji_sdk_app_info::~dji_sdk_app_info()
{
    ;
}

/////////
/// \brief dji_sdk_app_info::loadAppInfo
/// \param path
/// load dji sdk app info and serial port info from a .txt file
/// like this:
///
/// # DJI onboard SDK APP Info
/// # Serial Port Info
/// serial_name = /dev/ttyUSB0
/// baud_rate = 115200
/// # Onboard APP Info
/// app_id = 1005800
/// app_api_level = 2
/// app_version = 1
/// app_bundle_id = 12345678901234567890123456789012
/// app_enc_key = 6c3e0b6b1725dfcf7db2372cda2bbcfe89a78117d7ca1pf1ooe2b7598e78b331
///
void dji_sdk_app_info::loadAppInfo(string path1, string path2)
{
    string tmp = path1;

    readFromFile(tmp);

    saveToFile(path2, 0);
    showDebugInfo();

    cout<<"load App Info DONE."<<endl;
}

void dji_sdk_app_info::loadAppInfo(string path)
{
    string tmp = path;

    readFromFile(tmp);

    saveToFile("info_used.txt", 0);
    showDebugInfo();

    cout<<"load App Info DONE."<<endl;
}

//////
/// \brief dji_sdk_app_info::readFromFile
/// \param path
/// read dji sdk app info and serial port info from a .txt file
///
void dji_sdk_app_info::readFromFile(string path)
{
    ifstream file(path.c_str());

    //init
    serial_port.serial_name = "-1";
    serial_port.baud_rate = -1;
    app_info.app_id = -1;
    app_info.app_version = -1;
    app_info.app_api_level = -1;
    app_info.app_bundle_id = "-1";
    app_info.app_enc_key = "-1";

    char line[1024];
    while (!file.eof())
    {
        file.getline(line,1024);
        char cmd[20];
        float float_value;
        // load number param
        if ( sscanf(line,"%s = %f",cmd,&float_value)==2)
        {
            string scmd(cmd);
            if (scmd=="baud_rate")
                serial_port.baud_rate=float_value;
            else
                if (scmd=="app_id")
                    app_info.app_id=float_value;
                else
                    if (scmd=="app_version")
                        app_info.app_version=float_value;
                    else
                        if (scmd=="app_api_level")
                            app_info.app_api_level=float_value;
        }

        char char_value[72];
        // load char param
        if ( sscanf(line,"%s = %s",cmd,char_value)==2)
        {
            string scmd(cmd);
            string svalue(char_value);
            if (scmd=="serial_name")
                serial_port.serial_name=svalue;
            else
                if (scmd=="app_bundle_id")
                    app_info.app_bundle_id=svalue;
                else
                    if (scmd=="app_enc_key")
                        app_info.app_enc_key=svalue;
        }
    }
}

///////////
/// \brief dji_sdk_app_info::saveToFile
/// \param path
/// \param is_xml
/// to save the sdk info to a .txt file
///
void dji_sdk_app_info::saveToFile(string path, bool is_xml)
{
    if(is_xml != 1)
    {
        ofstream file(path.c_str());

        file<<"# DJI onboard SDK APP Info "<<endl;

        file<<endl;
        file<<"# Serial Port Info"<<endl;
        file<<"serial_name = "<<serial_port.serial_name<<endl;
        file<<"baud_rate = "<<serial_port.baud_rate<<endl;

        file<<endl;
        file<<"# Onboard APP Info"<<endl;
        file<<"app_id = "<<app_info.app_id<<endl;
        file<<"app_api_level = "<<app_info.app_api_level<<endl;
        file<<"app_version = "<<app_info.app_version<<endl;
        file<<"app_bundle_id = "<<app_info.app_bundle_id<<endl;
        file<<"app_enc_key = "<<app_info.app_enc_key<<endl;
    }
    else
    {
        cout<<"sorry, i dont want to output xml file."<<endl;
    }

}

void dji_sdk_app_info::showDebugInfo()
{
    printf("[Init Debug] SERIAL INFO   \n");
    printf("       |  serial_port %s \n", serial_port.serial_name.c_str());
    printf("       |  baud_rate   %d \n", serial_port.baud_rate);
    printf("[Init Debug] ACTIVATION INFO	 \n");
    printf("       |  app_id     	  %d \n", app_info.app_id);
    printf("       |  app_api_level	  %d \n", app_info.app_api_level);
    printf("       |  app_version     %d \n", app_info.app_version);
    printf("       |  app_bundle_id	  %s \n", app_info.app_bundle_id.c_str());
    printf("       |  enc_key	  %s \n", app_info.app_enc_key.c_str());
}
