import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;
import veins.base.utils.SimpleAddress;

namespace veins;

packet TraCIDemo11pMessage extends BaseFrame1609_4 {
    string demoData = "";
    LAddress::L2Type senderAddress = -1;
    int serial = 0;

    // Dataset Fields with Safe Defaults
    int Vehicle_ID;
    int Frame_ID = 0;
    int Total_Frames = 0;
    double Global_Time = 0.0;
    double Local_X = 0.0;
    double Local_Y = 0.0;
    double Global_X = 0.0;
    double Global_Y = 0.0;
    double v_Length = 0.0;
    double v_Width = 0.0;
    int v_Class = 1;      // Default to Passenger Car
    double v_Vel = 0.0;
    double v_Acc = 0.0;
    string Lane_ID = "";  // Critical: Initialize string to prevent memory traps
    int Preceding = -1;
    int Following = -1;
    double Space_Headway = -1.0;
    double Time_Headway = -1.0;
}