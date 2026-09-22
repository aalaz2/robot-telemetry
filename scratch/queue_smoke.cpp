#include <cassert>
#include <cstdio>
#include <thread>
#include <vector>

#include "telemetry/bounded_queue.hpp"

int main() {
    using telemetry::BoundedQueue;

    // Test 1: basic push/pop across two threads.
    {
        BoundedQueue<int> q(4);
        std::thread producer([&] {
            for (int i = 0; i < 20; ++i) {
                while (!q.TryPush(i)) {
                    std::this_thread::yield();  // queue full, retry
                }
            }
            q.Shutdown();
        });

        std::vector<int> received;
        std::thread consumer([&] {
            while (auto item = q.Pop()) {
                received.push_back(*item);
            }
        });

        producer.join();
        consumer.join();
        assert(received.size() == 20);
        for (int i = 0; i < 20; ++i) assert(received[i] == i);
        std::printf("test 1 (basic push/pop, 2 threads): PASS\n");
    }

    // Test 2: overload -> drops counted, no crash, no block forever.
    {
        BoundedQueue<int> q(2);
        for (int i = 0; i < 10; ++i) q.TryPush(i);  // never draining
        assert(q.DroppedCount() == 8);
        std::printf("test 2 (overload drop counting): PASS (dropped=%zu)\n",
                    q.DroppedCount());
    }

    // Test 3: shutdown while consumer is waiting on an empty queue.
    {
        BoundedQueue<int> q(4);
        std::thread consumer([&] {
            auto item = q.Pop();
            assert(!item.has_value());
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        q.Shutdown();
        consumer.join();
        std::printf("test 3 (shutdown while consumer waits): PASS\n");
    }

    // Test 4: shutdown drains remaining items before returning nullopt.
    {
        BoundedQueue<int> q(10);
        q.TryPush(1);
        q.TryPush(2);
        q.Shutdown();
        auto a = q.Pop();
        auto b = q.Pop();
        auto c = q.Pop();
        assert(a && *a == 1);
        assert(b && *b == 2);
        assert(!c.has_value());
        std::printf("test 4 (shutdown drains remaining items): PASS\n");
    }

    std::printf("all smoke tests passed\n");
    return 0;
}