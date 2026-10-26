#pragma once
#include "physics.h"

#include <thread>
#include <atomic>
#include <string>
#include <sstream>
#include <iostream>
#include <cstring>

// Platform socket includes
#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  using sock_t = SOCKET;
  #define INVALID_SOCK INVALID_SOCKET
  #define SOCK_ERR     SOCKET_ERROR
  inline void close_sock(sock_t s){ closesocket(s); }
  inline bool sock_would_block(){ return WSAGetLastError()==WSAEWOULDBLOCK; }
#else
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <errno.h>
  using sock_t = int;
  #define INVALID_SOCK (-1)
  #define SOCK_ERR     (-1)
  inline void close_sock(sock_t s){ ::close(s); }
  inline bool sock_would_block(){ return errno==EWOULDBLOCK||errno==EAGAIN; }
#endif

// =========================================================
// Minimal JSON helpers  (no external lib needed)
// =========================================================
namespace json_util {

inline std::string vec3_arr(const Vec3& v){
    char buf[128];
    std::snprintf(buf,sizeof(buf),"[%.6f,%.6f,%.6f]",v.x,v.y,v.z);
    return buf;
}
inline std::string pwm_arr(const std::array<double,4>& p){
    char buf[128];
    std::snprintf(buf,sizeof(buf),"[%.4f,%.4f,%.4f,%.4f]",p[0],p[1],p[2],p[3]);
    return buf;
}

// Build telemetry JSON string
inline std::string telemetry(const SimState& s){
    std::lock_guard<std::mutex> lk(s.mtx);
    char buf[1024];
    std::snprintf(buf,sizeof(buf),
        "{\"position\":[%.6f,%.6f,%.6f],"
         "\"velocity\":[%.6f,%.6f,%.6f],"
         "\"joint1\":[%.2f,%.2f,%.2f,%.2f],"
         "\"joint2\":[%.2f,%.2f,%.2f,%.2f],"
         "\"accel_body\":[%.6f,%.6f,%.6f],"
         "\"rotation\":[%.6f,%.6f,%.6f],"
         "\"angular_vel_body\":[%.6f,%.6f,%.6f],"
         "\"target_position\":[%.6f,%.6f,%.6f],"
         "\"time\":%.4f}\n",
        s.position.x,  s.position.y,  s.position.z,
        s.velocity.x,  s.velocity.y,  s.velocity.z,
        s.joint1_deg[0], s.joint1_deg[1], s.joint1_deg[2], s.joint1_deg[3],
        s.joint2_deg[0], s.joint2_deg[1], s.joint2_deg[2], s.joint2_deg[3],
        s.accel_body.x,s.accel_body.y,s.accel_body.z,
        s.rotation.x,  s.rotation.y,  s.rotation.z,
        s.angular_vel.x,s.angular_vel.y,s.angular_vel.z,
        s.target.x,    s.target.y,    s.target.z,
        s.sim_time
    );
    return buf;
}

// Parse a "pwm" array from JSON line – very simple
inline bool parse_pwm(const std::string& line, std::array<double,4>& out)
{
    // Find "pwm":[...] pattern
    auto pos = line.find("\"pwm\"");
    if(pos == std::string::npos) return false;
    pos = line.find('[', pos);
    if(pos == std::string::npos) return false;
    double a,b,c,d;
    if(std::sscanf(line.c_str()+pos,"[%lf,%lf,%lf,%lf]",&a,&b,&c,&d)==4){
        out={a,b,c,d}; return true;
    }
    return false;
}

inline bool parse_joint1(
    const std::string& line,
    std::array<double,4>& out)
{
    auto pos = line.find("\"joint1\"");
    if(pos == std::string::npos) return false;

    pos = line.find('[', pos);
    if(pos == std::string::npos) return false;

    double a,b,c,d;

    if(std::sscanf(
        line.c_str()+pos,
        "[%lf,%lf,%lf,%lf]",
        &a,&b,&c,&d) == 4)
    {
        out = {a,b,c,d};
        return true;
    }

    return false;
}

inline bool parse_joint2(
    const std::string& line,
    std::array<double,4>& out)
{
    auto pos = line.find("\"joint2\"");
    if(pos == std::string::npos) return false;

    pos = line.find('[', pos);
    if(pos == std::string::npos) return false;

    double a,b,c,d;

    if(std::sscanf(
        line.c_str()+pos,
        "[%lf,%lf,%lf,%lf]",
        &a,&b,&c,&d) == 4)
    {
        out = {a,b,c,d};
        return true;
    }

    return false;
}

inline bool parse_reset(const std::string& line){
    return line.find("\"request_reset\":true") != std::string::npos;
}

} // namespace json_util

// =========================================================
// TcpServer – one client at a time, 50 Hz telemetry
// =========================================================
class TcpServer {
public:
    TcpServer(int port, SimState& state)
        : port_(port), state_(state), running_(false)
    {}

    void start(){
        running_ = true;
        thread_ = std::thread([this]{ loop(); });
    }

    void stop(){
        running_ = false;
        if(thread_.joinable()) thread_.join();
    }

private:
    int       port_;
    SimState& state_;
    std::atomic<bool> running_;
    std::thread       thread_;

    void loop()
    {
#ifdef _WIN32
        WSADATA wsa; WSAStartup(MAKEWORD(2,2),&wsa);
#endif
        sock_t srv = ::socket(AF_INET, SOCK_STREAM, 0);
        if(srv == INVALID_SOCK){
            std::cerr << "[SIM] socket() failed\n"; return;
        }
        int yes=1;
        ::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes));

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port        = htons((uint16_t)port_);

        if(::bind(srv,(sockaddr*)&addr,sizeof(addr))==SOCK_ERR){
            std::cerr << "[SIM] bind() failed on port " << port_ << "\n";
            close_sock(srv); return;
        }
        ::listen(srv, 1);
        std::cout << "[SIM] TCP server listening on 127.0.0.1:" << port_ << "\n";

        // Make accept() non-blocking so we can check running_
        set_nonblock(srv, true);

        while(running_){
            sockaddr_in client_addr{};
            socklen_t   client_len = sizeof(client_addr);
            sock_t conn = ::accept(srv,(sockaddr*)&client_addr,&client_len);

            if(conn == INVALID_SOCK){
                if(sock_would_block()){
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                break;
            }

            int flag=1;
            ::setsockopt(conn, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(flag));
            set_nonblock(conn, true);

            std::cout << "[SIM] Controller connected\n";

            handle_client(conn);

            std::cout << "[SIM] Controller disconnected\n";
            close_sock(conn);
        }
        close_sock(srv);
#ifdef _WIN32
        WSACleanup();
#endif
    }

    void handle_client(sock_t conn)
    {
        const double SOCKET_DT = 1.0 / 50.0;   // 50 Hz

        std::string recv_buf;
        char tmp[4096];

        auto next_send = std::chrono::steady_clock::now();

        while(running_){
            // --- receive ---
            int n = ::recv(conn, tmp, sizeof(tmp)-1, 0);
            if(n == 0) break;            // peer closed
            if(n > 0){
                tmp[n]='\0';
                recv_buf += tmp;

                // Defensive guard: if a well-formed newline-terminated
                // line never arrives, don't let this grow unbounded.
                constexpr size_t MAX_RECV_BUF = 64 * 1024;
                if(recv_buf.size() > MAX_RECV_BUF)
                {
                    std::cerr << "[SIM] WARN: recv buffer exceeded "
                              << MAX_RECV_BUF
                              << " bytes with no newline — clearing\n";
                    recv_buf.clear();
                }

                // Process complete lines
                size_t pos;
                while((pos=recv_buf.find('\n')) != std::string::npos){
                    std::string line = recv_buf.substr(0,pos);
                    recv_buf.erase(0,pos+1);

                    if(line.empty()) continue;

                    std::array<double,4> pwm{};
                    std::array<double,4> joint1{};
                    std::array<double,4> joint2{};

                    bool got_pwm =
                        json_util::parse_pwm(line, pwm);

                    bool got_joint1 =
                        json_util::parse_joint1(line, joint1);

                    bool got_joint2 =
                        json_util::parse_joint2(line, joint2);

                    // Apply whatever fields were actually present instead
                    // of silently dropping the whole command when one key
                    // is missing/malformed. Missing fields keep their
                    // last commanded value (set_actuators only overwrites
                    // what it's given).
                    if(got_pwm || got_joint1 || got_joint2)
                    {
                        state_.set_actuators(
                            got_pwm    ? &pwm    : nullptr,
                            got_joint1 ? &joint1 : nullptr,
                            got_joint2 ? &joint2 : nullptr);
                    }

                    if(!got_pwm || !got_joint1 || !got_joint2)
                    {
                        std::cerr << "[SIM] WARN: incomplete actuator line"
                                     " (pwm=" << got_pwm
                                  << " joint1=" << got_joint1
                                  << " joint2=" << got_joint2
                                  << "): " << line << "\n";
                    }

                    if(json_util::parse_reset(line))
                        state_.reset();
                }
            } else if(!sock_would_block()){
                break;
            }

            // --- send at 50 Hz ---
            auto now = std::chrono::steady_clock::now();
            if(now >= next_send){
                std::string msg = json_util::telemetry(state_);
                int sent = ::send(conn, msg.c_str(), (int)msg.size(), 0);
                if(sent == SOCK_ERR && !sock_would_block()) break;
                next_send += std::chrono::microseconds((long long)(SOCKET_DT*1e6));
                if(next_send < now)
                    next_send = now + std::chrono::microseconds((long long)(SOCKET_DT*1e6));
            } else {
                // Sleep a bit to avoid busy-loop
                auto wait = next_send - now;
                if(wait > std::chrono::milliseconds(1))
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    static void set_nonblock(sock_t s, bool nb)
    {
#ifdef _WIN32
        u_long mode = nb ? 1 : 0;
        ioctlsocket(s, FIONBIO, &mode);
#else
        int flags = fcntl(s, F_GETFL, 0);
        if(nb) flags |= O_NONBLOCK;
        else   flags &= ~O_NONBLOCK;
        fcntl(s, F_SETFL, flags);
#endif
    }
};
