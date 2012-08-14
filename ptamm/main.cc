// Copyright 2009 Isis Innovation Limited
// This is the main extry point for PTAMM
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "System.h"


using namespace std;
using namespace GVars3;
using namespace PTAMM;

int main() {
    cout << "  ====  PTAMM ====" << endl;
    cout << "  Copyright (C) Isis Innovation Limited 2009 and Jens Nyman (Reservoir Lab) 2012" << endl;
    cout << "  " << endl;
    cout << "  Keyboard shortcuts:" << endl;
    cout << "      1: spacebar for camera 1" << endl;
    cout << "      2: spacebar for camera 2" << endl;
    cout << "      r: reset" << endl;
    cout << "      h: reset scale based on height measurement" << endl;
    cout << "      s: save frame for wallviz" << endl;
    cout << "      w: notify when wallviz can start building the map" << endl;
    cout << "      d: notify when passing door" << endl;
    cout << "      m: force map export" << endl;
    cout << "  " << endl;
    GUI.LoadFile("settings.cfg");

    GUI.StartParserThread(); // Start parsing of the console input
    atexit(GUI.StopParserThread);

    try {
        System s;
        s.Run();
    } catch (CVD::Exceptions::All e) {
        cout << endl;
        cout << "!! Failed to run system; got exception. " << endl;
        cout << "   Exception was: " << endl;
        cout << e.what << endl;
    }

    return 0;
}



