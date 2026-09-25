#include "g1_sdk_bridge/ipc_transport.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

using namespace g1_sdk_bridge;

namespace {

class TempSockPath {
public:
    TempSockPath() {
        char tmpl[] = "/tmp/g1ipc_test_XXXXXX";
        const char* dir = mkdtemp(tmpl);
        if (dir == nullptr) {
            throw std::runtime_error("mkdtemp失敗");
        }
        dir_ = dir;
        path_ = dir_ + "/cmd.sock";
    }
    ~TempSockPath() {
        std::remove(path_.c_str());
        ::rmdir(dir_.c_str());
    }
    const std::string& path() const { return path_; }

private:
    std::string dir_;
    std::string path_;
};

void SleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

}  // namespace

class SeqPacketRoundtripTest : public ::testing::Test {
protected:
    void SetUp() override { server_ = std::make_unique<SeqPacketServer>(tmp_.path(), 64); }

    TempSockPath tmp_;
    std::unique_ptr<SeqPacketServer> server_;
};

TEST_F(SeqPacketRoundtripTest, ClientToServerRoundtrip) {
    SeqPacketEndpoint client = ConnectClient(tmp_.path(), 64, 1.0);
    SeqPacketEndpoint server_ep = server_->AcceptBlocking(1.0);

    const std::string msg = "hello";
    ASSERT_TRUE(client.SendLatest(msg.data(), msg.size()));

    std::optional<std::vector<std::uint8_t>> received;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
        received = server_ep.RecvLatest();
        if (received.has_value()) break;
        SleepMs(5);
    }
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(std::string(received->begin(), received->end()), msg);
}

TEST_F(SeqPacketRoundtripTest, RecvLatestDrainsAndReturnsOnlyNewest) {
    SeqPacketEndpoint client = ConnectClient(tmp_.path(), 64, 1.0);
    SeqPacketEndpoint server_ep = server_->AcceptBlocking(1.0);

    for (int i = 0; i < 5; ++i) {
        const std::string msg = "msg" + std::to_string(i);
        ASSERT_TRUE(client.SendLatest(msg.data(), msg.size()));
    }
    SleepMs(50);  // 全メッセージがカーネルバッファに届くのを待つ

    // 5件送っても、RecvLatestは「最新の1件」だけを返す(latest-value semantics)
    const auto received = server_ep.RecvLatest();
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(std::string(received->begin(), received->end()), "msg4");
}

TEST_F(SeqPacketRoundtripTest, PeerCloseDetectedOnRecv) {
    SeqPacketEndpoint client = ConnectClient(tmp_.path(), 64, 1.0);
    SeqPacketEndpoint server_ep = server_->AcceptBlocking(1.0);

    client.Close();
    SleepMs(50);

    bool threw = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
        try {
            server_ep.RecvLatest();
        } catch (const PeerClosed&) {
            threw = true;
            break;
        }
        SleepMs(10);
    }
    EXPECT_TRUE(threw);
}

TEST_F(SeqPacketRoundtripTest, PeerCloseDetectedOnSend) {
    SeqPacketEndpoint client = ConnectClient(tmp_.path(), 64, 1.0);
    SeqPacketEndpoint server_ep = server_->AcceptBlocking(1.0);

    server_ep.Close();
    SleepMs(50);

    bool threw = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    const std::string msg = "ping";
    while (std::chrono::steady_clock::now() < deadline) {
        try {
            client.SendLatest(msg.data(), msg.size());
        } catch (const PeerClosed&) {
            threw = true;
            break;
        }
        SleepMs(10);
    }
    EXPECT_TRUE(threw);
}

TEST_F(SeqPacketRoundtripTest, SendDropsSilentlyWhenBufferFull) {
    SeqPacketEndpoint client = ConnectClient(tmp_.path(), 64, 1.0);
    SeqPacketEndpoint server_ep = server_->AcceptBlocking(1.0);

    // 送信バッファを極端に小さくして、受信側が読まない状態ですぐ満杯にする
    const int tiny = 1;
    ASSERT_EQ(::setsockopt(client.fd(), SOL_SOCKET, SO_SNDBUF, &tiny, sizeof(tiny)), 0);

    bool saw_drop = false;
    for (int i = 0; i < 50; ++i) {
        const std::string msg = "m" + std::to_string(i);
        if (!client.SendLatest(msg.data(), msg.size())) {
            saw_drop = true;
            break;
        }
    }
    EXPECT_TRUE(saw_drop) << "送信バッファ満杯時にfalseで破棄される設計だが、一度もfalseにならなかった";
}
