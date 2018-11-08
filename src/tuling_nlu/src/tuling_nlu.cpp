/*
 * author: www.corvin.cn
 *
 * description: nlp node,invoke tuling server to nlp and process move cmd.
 *
 * History:
 *   20171128:init this file.
 *   20180117:增加解析语音控制小车移动和导航的命令词,可以通过中文语音来控制
 *      小车移动和自动导航到目的地。
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sstream>
#include <jsoncpp/json/json.h>
#include <curl/curl.h>
#include <string>
#include <exception>
#include <codecvt>


using namespace std;


static int flag = 0;
static string result;
static string tuling_key="e566b9a687e5465a8bc82e34415de10d"; //default tuling key
static string move_forward_str;
static string move_back_str;
static string move_left_str;
static string move_right_str;
static string turn_left_str;
static string turn_right_str;
static string back_home_str;
static string go_away_str;
static string stop_move_str;


#define MOVE_FORWARD_CMD 1
#define MOVE_BACK_CMD    2
#define MOVE_LEFT_CMD    3
#define MOVE_RIGHT_CMD   4
#define TURN_LEFT_CMD    5
#define TURN_RIGHT_CMD   6
#define STOP_MOVE_CMD    7

#define BACK_HOME_CMD    8
#define GO_AWAY_CMD      9


ros::Publisher cmd_vel_pub;
ros::Publisher nav_move_pub;


int writer(char *data, size_t size, size_t nmemb, string *writerData)
{
     if (writerData == NULL)
     {
         return -1;
     }
     int len = size*nmemb;
     writerData->append(data, len);

     return len;
}

/**
 * parse tuling server response json string
 */
int parseJsonResonse(string input)
{
    Json::Value root;
    Json::Reader reader;
    cout << "tuling server response origin json str:" << input << endl;
    bool parsingSuccessful = reader.parse(input, root);

    if(!parsingSuccessful)
    {
        cout << "!!! Failed to parse the response data" <<endl;
        return 1;
    }
    const Json::Value code = root["code"];
    const Json::Value text = root["text"];
    result = text.asString();
    flag = 1;
    cout << "response code:" << code << endl;
    cout << "response text:" << result <<endl;

    return 0;
}


/**
 * send tuling server http pose requeset
 */
int HttpPostRequest(string input, string key)
{
    string buffer;

    std::string strJson = "{";
    strJson += "\"key\" : ";
    strJson += "\"";
    strJson += key;
    strJson += "\",";
    strJson += "\"info\" : ";
    strJson += "\"";
    strJson += input;
    strJson += "\"";
    strJson += "}";

    cout<< "post json string:" << strJson <<endl;
    try
    {
        CURL *pCurl = NULL;
        CURLcode res;
        curl_global_init(CURL_GLOBAL_ALL);

        // get a curl handle
        pCurl = curl_easy_init();
        if (NULL != pCurl)
        {
            //set url timeout
            curl_easy_setopt(pCurl, CURLOPT_TIMEOUT, 5);

            // First set the URL that is about to receive our POST.
            curl_easy_setopt(pCurl, CURLOPT_URL, "http://www.tuling123.com/openapi/api");

            // set curl http header
            curl_slist *plist = curl_slist_append(NULL,"Content-Type:application/json; charset=UTF-8");
            curl_easy_setopt(pCurl, CURLOPT_HTTPHEADER, plist);

            // set curl post content fileds
            curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, strJson.c_str());

            curl_easy_setopt(pCurl, CURLOPT_WRITEFUNCTION, writer);
            curl_easy_setopt(pCurl, CURLOPT_WRITEDATA, &buffer);

            // Perform the request, res will get the return code
            res = curl_easy_perform(pCurl);

            // Check for errors
            if (res != CURLE_OK)
            {
                printf("curl_easy_perform() failed:%s\n", curl_easy_strerror(res));
            }
            // always cleanup
            curl_easy_cleanup(pCurl);
        }
        curl_global_cleanup();
    }
    catch (std::exception &ex)
    {
        printf("!!! curl exception %s.\n", ex.what());
    }

    if(buffer.empty())
    {
        cout << "!!! ERROR The TuLing server response NULL" <<endl;
    }
    else
    {
        parseJsonResonse(buffer);
    }

    return 0;
}

wstring str2wstr(const std::string& str)
{
    using convert_typeX = std::codecvt_utf8<wchar_t>;
    wstring_convert<convert_typeX, wchar_t> converterX;

    return converterX.from_bytes(str);
}

int parseInputString(string input)
{
    int ret = 0;
    wstring convertStr = str2wstr(input); 

    wstring forwardStr   = str2wstr(move_forward_str);
    wstring backStr      = str2wstr(move_back_str);
    wstring moveLeftStr  = str2wstr(move_left_str);
    wstring moveRightStr = str2wstr(move_right_str);
    wstring turnLeftStr  = str2wstr(turn_left_str);
    wstring turnRightStr = str2wstr(turn_right_str);
    wstring backHomeStr  = str2wstr(back_home_str);
    wstring goAwayStr    = str2wstr(go_away_str);
    wstring stopMoveStr  = str2wstr(stop_move_str);

    if(convertStr.find(forwardStr) != string::npos)
    {
        ret = MOVE_FORWARD_CMD;
    }
    else if(convertStr.find(backStr) != string::npos) 
    {
        ret = MOVE_BACK_CMD; 
    }
    else if(convertStr.find(moveLeftStr) != string::npos) 
    {
        ret = MOVE_LEFT_CMD; 
    }
    else if(convertStr.find(moveRightStr) != string::npos) 
    {
        ret = MOVE_RIGHT_CMD; 
    }
    else if(convertStr.find(turnLeftStr) != string::npos) 
    {
        ret = TURN_LEFT_CMD; 
    }
    else if(convertStr.find(turnRightStr) != string::npos) 
    {
        ret = TURN_RIGHT_CMD; 
    }
    else if(convertStr.find(backHomeStr) != string::npos) 
    {
        ret = BACK_HOME_CMD; 
    }
    else if(convertStr.find(goAwayStr) != string::npos) 
    {
        ret = GO_AWAY_CMD; 
    }
    else if(convertStr.find(stopMoveStr) != string::npos) 
    {
        ret = STOP_MOVE_CMD; 
    }

    return ret;
}

/**
*   when nlp node get input,will auto send http post request to tuling server
**/
void nluCallback(const std_msgs::String::ConstPtr& msg)
{
    int ret = 0;
    std::cout<<"tuling_nlu_node get input is:" << msg->data << std::endl;

    ret = parseInputString(msg->data);
    ROS_INFO("tuling_nlu_node get parseInputString return: %d", ret);
    if(ret == 0) //send tuling nlu server to process
    {
        HttpPostRequest(msg->data, tuling_key);
    }
    else if((ret >= MOVE_FORWARD_CMD)&&(ret <= STOP_MOVE_CMD))
    {
        std_msgs::Int32 move_msg;
        move_msg.data = ret;
        cmd_vel_pub.publish(move_msg);
    }
    else //send nav msg
    {
        std_msgs::Int32 nav_msg;
        nav_msg.data = ret;
        nav_move_pub.publish(nav_msg);
    }
}

/**
 * main function
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tuling_nlu_node");
    ros::NodeHandle ndHandle;

    string nlu_topic = "/voice_system/nlu_topic"; //default nlu topic name
    string tts_topic = "/voice_system/tts_topic"; //default tts topic name
    string cmd_topic = "/voice_system/move_topic";  //default carebot voice control move topic
    string nav_topic = "/voice_system/voice_navigation_cmd";

    ros::param::get("~nlu_topic",  nlu_topic);
    ros::param::get("~tts_topic",  tts_topic);
    ros::param::get("~tuling_key", tuling_key);

    ros::param::get("~move_forward", move_forward_str);
    ros::param::get("~move_back",  move_back_str);
    ros::param::get("~move_left",  move_left_str);
    ros::param::get("~move_right", move_right_str);
    ros::param::get("~turn_left",  turn_left_str);
    ros::param::get("~turn_right", turn_right_str);
    ros::param::get("~stop_move",  stop_move_str);
    
    ros::param::get("~back_home", back_home_str);    
    ros::param::get("~go_away", go_away_str);

    ros::param::get("~move_topic", cmd_topic);
    ros::param::get("~nav_topic", nav_topic);

    ros::Subscriber sub_nlu = ndHandle.subscribe(nlu_topic, 3, nluCallback);
    ros::Publisher tts_pub  = ndHandle.advertise<std_msgs::String>(tts_topic, 1);
    cmd_vel_pub  = ndHandle.advertise<std_msgs::Int32>(cmd_topic, 1);
    nav_move_pub = ndHandle.advertise<std_msgs::Int32>(nav_topic, 1);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        if(flag)
        {
           std_msgs::String msg;
           msg.data = result;
           tts_pub.publish(msg);
           flag = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


