#include "g1_sdk_bridge/ipc_transport.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <system_error>
#include <thread>

namespace g1_sdk_bridge {

namespace {

void SetNonBlocking(int fd) {
    int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        throw std::system_error(errno, std::generic_category(), "fcntl(O_NONBLOCK)に失敗");
    }
}

sockaddr_un MakeSockAddr(const std::string& path) {
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (path.size() >= sizeof(addr.sun_path)) {
        throw std::invalid_argument("Unix domain socketのパスが長すぎる(sun_path上限を超える): " + path);
    }
    std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
    return addr;
}

bool IsWouldBlock(int err) { return err == EAGAIN || err == EWOULDBLOCK; }
bool IsPeerGone(int err) { return err == EPIPE || err == ECONNRESET || err == ENOTCONN; }

}  // namespace

// --- SeqPacketEndpoint -------------------------------------------------

SeqPacketEndpoint::SeqPacketEndpoint(int fd, std::size_t max_payload) : fd_(fd), max_payload_(max_payload) {
    SetNonBlocking(fd_);
}

SeqPacketEndpoint::~SeqPacketEndpoint() { Close(); }

SeqPacketEndpoint::SeqPacketEndpoint(SeqPacketEndpoint&& other) noexcept
    : fd_(other.fd_), max_payload_(other.max_payload_) {
    other.fd_ = -1;
}

SeqPacketEndpoint& SeqPacketEndpoint::operator=(SeqPacketEndpoint&& other) noexcept {
    if (this != &other) {
        Close();
        fd_ = other.fd_;
        max_payload_ = other.max_payload_;
        other.fd_ = -1;
    }
    return *this;
}

bool SeqPacketEndpoint::SendLatest(const void* data, std::size_t size) {
    const ssize_t n = ::send(fd_, data, size, MSG_NOSIGNAL);
    if (n >= 0) {
        return true;
    }
    if (IsWouldBlock(errno)) {
        return false;
    }
    if (IsPeerGone(errno)) {
        throw PeerClosed(std::string("send失敗(相手が切断): ") + std::strerror(errno));
    }
    throw std::system_error(errno, std::generic_category(), "send失敗");
}

std::optional<std::vector<std::uint8_t>> SeqPacketEndpoint::RecvLatest() {
    std::optional<std::vector<std::uint8_t>> latest;
    std::vector<std::uint8_t> buf(max_payload_);
    for (;;) {
        const ssize_t n = ::recv(fd_, buf.data(), buf.size(), 0);
        if (n > 0) {
            latest = std::vector<std::uint8_t>(buf.begin(), buf.begin() + n);
            continue;
        }
        if (n == 0) {
            // 空受信はピアが正常にshutdown/closeしたことを示す(SOCK_SEQPACKETの規約)
            throw PeerClosed("peer closed the connection");
        }
        if (IsWouldBlock(errno)) {
            return latest;
        }
        if (IsPeerGone(errno)) {
            throw PeerClosed(std::string("recv失敗(相手が切断): ") + std::strerror(errno));
        }
        throw std::system_error(errno, std::generic_category(), "recv失敗");
    }
}

void SeqPacketEndpoint::Close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

// --- SeqPacketServer -----------------------------------------------------

SeqPacketServer::SeqPacketServer(const std::string& path, std::size_t max_payload)
    : path_(path), max_payload_(max_payload), listen_fd_(-1) {
    ::unlink(path_.c_str());  // 前回の残骸を消す。存在しなければ無視してよい

    listen_fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (listen_fd_ < 0) {
        throw std::system_error(errno, std::generic_category(), "socket()に失敗");
    }
    const sockaddr_un addr = MakeSockAddr(path_);
    if (::bind(listen_fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
        const int err = errno;
        ::close(listen_fd_);
        listen_fd_ = -1;
        throw std::system_error(err, std::generic_category(), "bind()に失敗: " + path_);
    }
    if (::listen(listen_fd_, 1) < 0) {
        const int err = errno;
        ::close(listen_fd_);
        listen_fd_ = -1;
        throw std::system_error(err, std::generic_category(), "listen()に失敗");
    }
    SetNonBlocking(listen_fd_);
}

SeqPacketServer::~SeqPacketServer() { Close(); }

std::optional<SeqPacketEndpoint> SeqPacketServer::AcceptIfPending() {
    const int fd = ::accept(listen_fd_, nullptr, nullptr);
    if (fd >= 0) {
        return SeqPacketEndpoint(fd, max_payload_);
    }
    if (IsWouldBlock(errno)) {
        return std::nullopt;
    }
    throw std::system_error(errno, std::generic_category(), "accept()に失敗");
}

SeqPacketEndpoint SeqPacketServer::AcceptBlocking(std::optional<double> timeout_seconds) {
    pollfd pfd{};
    pfd.fd = listen_fd_;
    pfd.events = POLLIN;
    const int timeout_ms = timeout_seconds.has_value() ? static_cast<int>(*timeout_seconds * 1000.0) : -1;

    for (;;) {
        const int rc = ::poll(&pfd, 1, timeout_ms);
        if (rc < 0) {
            if (errno == EINTR) continue;
            throw std::system_error(errno, std::generic_category(), "poll()に失敗");
        }
        if (rc == 0) {
            throw std::system_error(ETIMEDOUT, std::generic_category(), "accept()がタイムアウトした");
        }
        auto ep = AcceptIfPending();
        if (ep.has_value()) {
            return std::move(*ep);
        }
        // pollがPOLLINと言ったのにaccept()がEAGAINを返す競合はまず無いが、念のためループする
    }
}

void SeqPacketServer::Close() {
    if (listen_fd_ >= 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
        ::unlink(path_.c_str());
    }
}

// --- ConnectClient ---------------------------------------------------------

SeqPacketEndpoint ConnectClient(const std::string& path, std::size_t max_payload, double timeout_seconds) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_seconds);
    const sockaddr_un addr = MakeSockAddr(path);

    for (;;) {
        const int fd = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
        if (fd < 0) {
            throw std::system_error(errno, std::generic_category(), "socket()に失敗");
        }
        if (::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0) {
            return SeqPacketEndpoint(fd, max_payload);
        }
        const int err = errno;
        ::close(fd);
        // ENOENT: サーバがまだソケットファイルを作っていない。ECONNREFUSED: bind済みだがlisten側が
        // まだaccept可能な状態でない。どちらも「サーバ起動待ち」として再試行する。
        if ((err == ENOENT || err == ECONNREFUSED) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        throw std::system_error(err, std::generic_category(), "connect()に失敗: " + path);
    }
}

}  // namespace g1_sdk_bridge
