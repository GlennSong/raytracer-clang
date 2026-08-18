#include "control_channel.h"

#include "../../log.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <mutex>
#include <sstream>
#include <thread>

#ifndef _WIN32
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>
#endif

namespace engine {

ControlCommand parseControlCommand(const std::string& line) {
    ControlCommand cmd;
    std::istringstream in(line);
    std::string tok;
    while (in >> tok) {
        if (cmd.name.empty()) {
            std::transform(tok.begin(), tok.end(), tok.begin(),
                           [](unsigned char c) { return std::tolower(c); });
            cmd.name = tok;
        } else {
            cmd.args.push_back(tok);
        }
    }
    return cmd;
}

struct ControlChannel::Impl {
    ControlBackendMode mode = ControlBackendMode::Disabled;
    std::string path;

    // Staging (ADR-0072): the socket thread appends, drain() swaps out on the
    // main thread. Replies for Manual mode collect here too.
    std::mutex mutex;
    std::vector<std::string> pending;
    std::vector<std::string> manualReplies;

#ifndef _WIN32
    int listenFd = -1;
    int clientFd = -1;        // guarded by `mutex` (drain writes replies to it)
    std::thread thread;
    std::atomic<bool> running{false};

    void threadMain() {
        // One client at a time, NEWEST WINS: the listen fd stays polled even
        // while a client is connected, so a hung client can't lock out the
        // next one. poll() with a timeout keeps shutdown responsive.
        std::string buffer;
        while (running.load(std::memory_order_relaxed)) {
            struct pollfd pfds[2]{};
            pfds[0].fd = listenFd;
            pfds[0].events = POLLIN;
            int nfds = 1;
            int client;
            {
                std::lock_guard<std::mutex> lock(mutex);
                client = clientFd;
            }
            if (client >= 0) {
                pfds[1].fd = client;
                pfds[1].events = POLLIN;
                nfds = 2;
            }
            if (::poll(pfds, static_cast<nfds_t>(nfds), 200) <= 0)
                continue;   // timeout or EINTR

            if (pfds[0].revents & POLLIN) {
                int fd = ::accept(listenFd, nullptr, nullptr);
                if (fd >= 0) {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (clientFd >= 0) ::close(clientFd);   // newest wins
                    clientFd = fd;
                    buffer.clear();
                    continue;   // re-poll with the new client
                }
            }
            if (nfds < 2 || !(pfds[1].revents & (POLLIN | POLLHUP))) continue;

            char chunk[512];
            ssize_t n = ::read(client, chunk, sizeof(chunk));
            if (n <= 0) {   // EOF or error: drop the client, back to accept
                std::lock_guard<std::mutex> lock(mutex);
                if (clientFd == client) {
                    ::close(clientFd);
                    clientFd = -1;
                }
                continue;
            }
            buffer.append(chunk, static_cast<size_t>(n));
            size_t nl;
            while ((nl = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, nl);
                buffer.erase(0, nl + 1);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                if (line.empty()) continue;
                std::lock_guard<std::mutex> lock(mutex);
                pending.push_back(std::move(line));
            }
        }
    }
#endif
};

ControlChannel::ControlChannel() : impl(std::make_unique<Impl>()) {}

ControlChannel::~ControlChannel() { shutdown(); }

bool ControlChannel::initialize(ControlBackendMode mode) {
    shutdown();
    impl->mode = mode;
    if (mode != ControlBackendMode::Socket) return true;
#ifdef _WIN32
    impl->mode = ControlBackendMode::Disabled;   // non-goal (ADR-0078)
    return false;
#else
    impl->path = "/tmp/raytracer-viewer-" + std::to_string(::getpid()) + ".sock";
    ::unlink(impl->path.c_str());   // a stale file from a crashed twin pid

    impl->listenFd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (impl->listenFd < 0) return false;
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (impl->path.size() >= sizeof(addr.sun_path)) {
        ::close(impl->listenFd);
        impl->listenFd = -1;
        return false;
    }
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s",
                  impl->path.c_str());
    if (::bind(impl->listenFd, reinterpret_cast<sockaddr*>(&addr),
               sizeof(addr)) != 0 ||
        ::listen(impl->listenFd, 1) != 0) {
        ::close(impl->listenFd);
        impl->listenFd = -1;
        return false;
    }
    ::chmod(impl->path.c_str(), 0600);   // user-only, like a terminal

    impl->running = true;
    impl->thread = std::thread([this] { impl->threadMain(); });
    LOG_INFO << "[control] listening at " << impl->path << " (ADR-0078)";
    return true;
#endif
}

void ControlChannel::shutdown() {
#ifndef _WIN32
    if (impl->running) {
        impl->running = false;
        impl->thread.join();
    }
    if (impl->clientFd >= 0) { ::close(impl->clientFd); impl->clientFd = -1; }
    if (impl->listenFd >= 0) { ::close(impl->listenFd); impl->listenFd = -1; }
    if (!impl->path.empty()) { ::unlink(impl->path.c_str()); impl->path.clear(); }
#endif
    impl->mode = ControlBackendMode::Disabled;
    impl->pending.clear();
    impl->manualReplies.clear();
}

ControlBackendMode ControlChannel::mode() const { return impl->mode; }

std::string ControlChannel::socketPath() const { return impl->path; }

int ControlChannel::drain(const Handler& handler) {
    if (impl->mode == ControlBackendMode::Disabled) return 0;
    std::vector<std::string> lines;
    {
        std::lock_guard<std::mutex> lock(impl->mutex);
        lines.swap(impl->pending);
    }
    for (const std::string& line : lines) {
        std::string reply = handler(line);
        if (impl->mode == ControlBackendMode::Manual) {
            impl->manualReplies.push_back(std::move(reply));
            continue;
        }
#ifndef _WIN32
        reply.push_back('\n');
        std::lock_guard<std::mutex> lock(impl->mutex);
        if (impl->clientFd >= 0)
            (void)::send(impl->clientFd, reply.data(), reply.size(), 0);
#endif
    }
    return static_cast<int>(lines.size());
}

void ControlChannel::pushCommand(const std::string& line) {
    std::lock_guard<std::mutex> lock(impl->mutex);
    impl->pending.push_back(line);
}

std::vector<std::string> ControlChannel::takeReplies() {
    std::lock_guard<std::mutex> lock(impl->mutex);
    std::vector<std::string> out;
    out.swap(impl->manualReplies);
    return out;
}

}  // namespace engine
