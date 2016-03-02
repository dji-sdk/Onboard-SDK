#include <iostream>

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"

using namespace std;

int main(int argc, char *argv[])
{
    //! @note replace these two lines below to change to an other hard-driver level.
    HardDriverManifold driver("/dev/ttyTHS1", 230400);
    driver.init();

    CoreAPI api(&driver);
    ConboardSDKScript sdkScript(&api);

    ScriptThread st(&sdkScript);

    //! @note replace these four lines below to change to an other hard-driver level.
    APIThread send(&api, 1);
    APIThread read(&api, 2);
    send.createThread();
    read.createThread();
    st.run();

    return 0;
}
