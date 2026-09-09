// 台架自测用假 VRPN 服务器：vrpn_Tracker_NULL 自动产生运动数据，
// 让 vrpn_mocap 在没有 FZMotion 的情况下也能连上、产出 /vrpn_mocap/<刚体>/pose。
// 仅台架验证用，不代表真实位置。
#include <cstdio>
#include "vrpn_Connection.h"
#include "vrpn_Tracker.h"

int main(int argc, char** argv) {
    const char* name = (argc > 1) ? argv[1] : "Tracker0";
    // 默认监听端口 3883（vrpn_DEFAULT_LISTEN_PORT_NO），与 vrpn_mocap 默认口一致
    vrpn_Connection* c = vrpn_create_server_connection();
    if (!c) { fprintf(stderr, "create server connection failed\n"); return 1; }
    // NULL tracker：1 个传感器，50Hz 自动产生正弦运动报文
    vrpn_Tracker_NULL* tk = new vrpn_Tracker_NULL(name, c, 1, 50.0);
    printf("NULL VRPN server up: tracker='%s' port=3883 (50Hz). Ctrl-C to stop.\n", name);
    fflush(stdout);
    while (true) {
        tk->mainloop();
        c->mainloop();
        vrpn_SleepMsecs(5);
    }
    return 0;
}
