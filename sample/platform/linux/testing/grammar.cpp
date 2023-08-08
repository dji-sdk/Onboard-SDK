#include <cstdlib>
#include <iostream>
#include <map>
#include <list>
#include <string>
#include "grammar.hpp"

using namespace std;

// randomize mode values

// map<string, list<int>> ParamValues;
// ParamValues.insert(pair<string, list<int>>("horizMode", horizMode))
// ParamValues.insert(pair<string, list<int>>("vertiMode", vertiMode))
// ParamValues.insert(pair<string, list<int>>("yawMode", yawMode))
// ParamValues.insert(pair<string, list<int>>("horizFrame", horizFrame))
// ParamValues.insert(pair<string, list<int>>("stableMode", stableMode))

// horizMode = {0, 1, 2, 3};
// vertiMode = {0, 1, 2};
// yawMode = {0, 1};
// horizFrame = {0, 1};
// stableMode = {0, 1};

// int randomMode(list<int> & mode) {

//     srand(time(NULL));
//     cout << "Size: " << mode.size() << endl;
//     int randPos = rand() % mode.size();
//     cout << "Random: " << randPos << endl;
//     list<int>::iterator it = mode.begin();
//     advance(it, randPos);
//     return *it;
//     // return horizMode[2];
// }

list<int> horizMode {0, 1, 2, 3};
list<int> vertiMode {0, 1, 2};
list<int> yawMode {0, 1};
list<int> horizFrame {0, 1};
list<int> stableMode {0, 1};

int randomHorizMode() {

    srand(time(NULL));
    // cout << "Size: " << horizMode.size() << endl;
    int randPos = rand() % horizMode.size();
    cout << "Random: " << randPos << endl;
    list<int>::iterator it = horizMode.begin();
    advance(it, randPos);
    cout << "Chosen: " << *it << endl;
    return *it;
    // return horizMode[2];
}


int randomStrMode(string mode) {

    list<int>& refMode; 
    if (mode == "horiz") {
        refMode = &horizMode;
    } else if (mode == "verti") {
        refMode = &vertiMode;
    } else if (mode == "yaw") {
        refMode = &yawMode;
    } else if (mode == "hFrame") {
        refMode = &horizFrame;
    } else if (mode == "stable") {
        refMode = &stableMode;
    }

    srand(time(NULL));
    cout << "Size: " << refMode.size() << endl;
    int randPos = rand() % refMode.size();
    cout << "Random: " << randPos << endl;
    list<int>::iterator it = refMode.begin();
    advance(it, randPos);
    cout << "Chosen: " << *it << endl;
    return *it;
    // return horizMode[2];
}

// randomize mode bit-level


// randomize command values


// randomize command bit-level