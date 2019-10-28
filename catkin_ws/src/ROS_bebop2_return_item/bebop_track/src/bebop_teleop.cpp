#include <bebop_track/bebop_teleop.h>



const char* BebopKeyBoardController::Interface[] = {
        "+-----------------------------------------------------+",
        "|                 Control your Bebop                  |",
        "|        W      R                            8        |",
        "|                                                     |",
        "|    A   S   D  F  G                     4   5   6    |",
        "|                                                     |",
        "| throttle UP / DOWN : W / S                          |",
        "| rotation CCW / CW  : A / D                          |",
        "| camera ?? / ??     : R / F                          |",
        "| FORWARD / BACKWARD : 8 / 5                          |",
        "| LEFT / RIGHT       : 4 / 6                          |",
        "| TAKE OFF / LAND    : Spacebar                       |",
        "| Tracking Mode      : T                              |",
        "| GPS Mode On        : G                              |",
        "| Emergency          : P                              |",
        "| Quit               : Q                              |",
        "+-----------------------------------------------------+"
};

const char* BebopKeyBoardController::gpsInterface[] = {
        "+-----------------------------------------------------+",
        "|             Control your Bebop with GPS             |",
        "|                                                     |",
        "|                                                     |",
        "|  Go Home : H                                        |",
        "|  Go entered coordinates : C                         |",
        "|                                                     |",
        "|                                                     |",
        "|                                                     |",
        "|                                                     |",
        "|                                                     |",
        "|                                                     |",
        "|                                                     |",
        "| GPS Mode Off       : G                              |",
        "| Emergency          : P                              |",
        "| Quit               : Q                              |",
        "+-----------------------------------------------------+"
};

void BebopKeyBoardController::_printInterface()
{
    /*
        bool ros::NodeHandle::getParam	( const std::string& key, bool& b )	const
        key	The key to be used in the parameter server's dictionary
 [out]	b	Storage for the retrieved value.
    */
    _nodeHandle.getParam(tracking_key, _isTracking); // tracking_key 값을 _isTracking에 저장
    _nodeHandle.getParam(home_gps_latitude_key, home_gps_latitude);
    _nodeHandle.getParam(home_gps_longitude_key, home_gps_longitude);
    _nodeHandle.getParam(drone_gps_latitude_key, drone_gps_latitude);
    _nodeHandle.getParam(drone_gps_longitude_key, drone_gps_longitude);
    int result = system("clear");
    for(int i = 0; i < 17; ++i)
        std::cout << Interface[i] << std::endl;
    std::cout << "[status] Takeoff : "<< ((_isTakeOff) ? "ON" : "OFF")
    << "\tTracking Mode : " << (_isTracking ? "ON" : "OFF") << std::endl;
    std::cout << "[Home GPS] latitude : " << home_gps_latitude << " ,longitude : " << home_gps_longitude << std::endl;
    std::cout << "[Drome GPS] latitude : " << drone_gps_latitude << " ,longitude : " << drone_gps_longitude << std::endl;
    std::cout << "[Speed value] : " << _speedValue << std::endl;
    std::cout << "[Speed Increase Value] : " << _speedIncreaseValue << std::endl;
}

void BebopKeyBoardController::_printGpsInterface()
{
    _nodeHandle.getParam(gps_key, _isGPS);
    _nodeHandle.getParam(home_gps_latitude_key, home_gps_latitude);
    _nodeHandle.getParam(home_gps_longitude_key, home_gps_longitude);
    _nodeHandle.getParam(drone_gps_latitude_key, drone_gps_latitude);
    _nodeHandle.getParam(drone_gps_longitude_key, drone_gps_longitude);
    int result = system("clear");
    for(int i = 0; i < 17; ++i)
        std::cout << gpsInterface[i] << std::endl;
    std::cout << "[status] Takeoff : "<< ((_isTakeOff) ? "ON" : "OFF")
    << "\tGPS Mode : " << (_isGPS ? "ON" : "OFF") << std::endl;
    std::cout << "[Home GPS] latitude : " << home_gps_latitude << " ,longitude : " << home_gps_longitude << std::endl;
    std::cout << "[Drome GPS] latitude : " << drone_gps_latitude << " ,longitude : " << drone_gps_longitude << std::endl;
}

void BebopKeyBoardController::_move(double &value, int orientation)
{
    if(_isTakeOff && !_isGPSmode)
    {
        _controlValue.linear.x = 0;
        _controlValue.linear.y = 0;
        _controlValue.linear.z = 0;
        _controlValue.angular.z = 0;

        value = _speedValue * orientation;
        _twistPublisher.publish(_controlValue);
    }
}

void BebopKeyBoardController::_takeoff()
{
    _isTakeOff = true;
    _isGPS = true;
    _nodeHandle.setParam(takeoff_key, takeoff);
    _nodeHandle.setParam(gps_key, gpsOn);
    _takeOffPublisher.publish(_message);
    _printInterface();
}

void BebopKeyBoardController::_land()
{
    _isTakeOff = false;
    _isGPS = false;
    _nodeHandle.setParam(takeoff_key, land);
    _landPublisher.publish(_message);
    _nodeHandle.setParam(tracking_key, trackOff);
    _printInterface();
}

void BebopKeyBoardController::_emergency()
{
    _isTakeOff = false;
    _emergencyPublisher.publish(_message);
    _nodeHandle.setParam(tracking_key, trackOff);
    _printInterface();
    ROS_INFO("\n\nDrone Emergency Land!");
}

void BebopKeyBoardController::_speedUp()
{
    _speedValue = _speedValue + _speedIncreaseValue;
}

void BebopKeyBoardController::_speedDown()
{
    _speedValue = _speedValue - _speedIncreaseValue;
    if(_speedValue < 0)
        _speedValue = 0;
}

char BebopKeyBoardController::_getKey()
{

    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return (char)ch;
}




void BebopKeyBoardController::Control()
{
    double& x = _controlValue.linear.x = 0;
    double& y = _controlValue.linear.y = 0;
    double& z = _controlValue.linear.z = 0;
    double& rotation = _controlValue.angular.z = 0;

    char key;

    while(ros::ok())
    {
        if(_isGPSmode)
        {
            _printGpsInterface();
        }
        else
        {
            _printInterface();
        }
        
        key = _getKey();
        if(!_isTakeOff && (key == 'q' || key == 'Q'))
            break;

        switch(key)
        {
            case ' ':
                _isTakeOff ? _land() : _takeoff();  
                break;
            case 'P':
            case 'p':
                _emergency();
                break;
            case 'T':
            case 't':
                if(_isTakeOff)
                {
                    _nodeHandle.getParam(tracking_key, _isTracking);
                    if(!_isTracking)
                    {
                        _nodeHandle.setParam(tracking_key, trackOn);
                        _printInterface();
                    }
                    else
                    {
                        _nodeHandle.setParam(tracking_key, trackOff);
                        _printInterface();
                    }
                }
                break;
            case 'G':
            case 'g':
                if(_isTakeOff)
                {
                    _nodeHandle.getParam(gps_mode_key, _isGPSmode);
                    if(!_isGPSmode)
                    {
                        _nodeHandle.setParam(gps_mode_key, gpsModeOn);
                        _printGpsInterface();
                    }
                    else
                    {
                        _nodeHandle.setParam(gps_mode_key, gpsModeOff);
                        //_nodeHandle.setParam(go_home_key, endGoHome);
                        _printGpsInterface();
                    }
                }
                break;
            case 'H':
            case 'h':
                if(_isGPSmode)
                {
                    _nodeHandle.setParam(go_home_key, goHome);
                }
                break;
            case 'C':
            case 'c':
                if(_isGPSmode)
                {       
                    _printGpsInterface();
                    double go_latitude = 0.0;
                    double go_longitude = 0.0;
                    
                    std::cout << "Enter go coordinates..." << std::endl;
                    std::cin >> go_latitude;
                    std::cin >> go_longitude;
                    _nodeHandle.setParam(go_latitude_key, go_latitude);
                    _nodeHandle.setParam(go_longitude_key, go_longitude);
                    _nodeHandle.setParam(go_entered_coordinates_key, goEnteredcoordinates);                    
                }
                break;
            case '+':
                _speedUp();
                _printInterface();
                break;
            case '-':
                _speedDown();
                _printInterface();
                break;
            case 'W':
            case 'w':
                _move(z, UP);
                break;
            case 'S':
            case 's':
                _move(z, DOWN);
                break;
            case 'A':
            case 'a':
                _move(rotation, CCW);
                break;
            case 'D':
            case 'd':
                _move(rotation, CW);
                break;
            case 'R':
            case 'r':
                break;
            case 'F':
            case 'f':
                break;
            case '8':
                _move(x, FORWARD);
                break;
            case '5':
                _move(x, BACKWARD);
                break;
            case '4':
                _move(y, LEFT);
                break;
            case '6':
                _move(y, RIGHT);
                break;
            default:
                break;
        }
    }
}

BebopKeyBoardController::BebopKeyBoardController(const ros::NodeHandle& nodeHandle)
        :_nodeHandle(nodeHandle),
         _twistPublisher(_nodeHandle.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 10)),
         _takeOffPublisher(_nodeHandle.advertise<std_msgs::Empty>("bebop/takeoff", 1)),
         _landPublisher(_nodeHandle.advertise<std_msgs::Empty>("bebop/land", 1)),
         _emergencyPublisher(_nodeHandle.advertise<std_msgs::Empty>("bebop/reset", 1)),
         _isTakeOff(false),
         _isTracking(false),
         _isGPS(false),
         _isGPSmode(false),
         home_gps_latitude(0.0),
         home_gps_longitude(0.0),
         drone_gps_latitude(0.0),
         drone_gps_longitude(0.0),
         _speedIncreaseValue(0.05),
         _speedValue(0.5)
{
    _nodeHandle.setParam(tracking_key, trackOff);
    _printInterface();
}
