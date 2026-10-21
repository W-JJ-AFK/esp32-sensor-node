#include "imu_ring.h"

#include <string.h>

void imu_ring_init(imu_ring_t *r)
{
    memset(r, 0, sizeof(*r));
    /* memset 之后显式用 atomic 写一遍，避免优化器把"全 0"合并成非原子写 */
    __atomic_store_n(&r->head, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&r->tail, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&r->dropped, 0u, __ATOMIC_RELEASE);
}

bool imu_ring_push(imu_ring_t *r, const imu_sample_t *s)
{
    const uint32_t head = __atomic_load_n(&r->head, __ATOMIC_RELAXED);
    const uint32_t next = (head + 1) & (IMU_RING_CAPACITY - 1);
    /* 只有消费者会改 tail，所以这里读到的是"至少这么新"的值 */
    const uint32_t tail = __atomic_load_n(&r->tail, __ATOMIC_ACQUIRE);

    if (next == tail) {
        __atomic_add_fetch(&r->dropped, 1u, __ATOMIC_RELAXED);
        return false;
    }

    r->buf[head] = *s;
    /* RELEASE：保证 buf[head] 的写入对拿到 head 新值的消费者可见 */
    __atomic_store_n(&r->head, next, __ATOMIC_RELEASE);
    return true;
}

bool imu_ring_pop(imu_ring_t *r, imu_sample_t *out)
{
    const uint32_t tail = __atomic_load_n(&r->tail, __ATOMIC_RELAXED);
    /* ACQUIRE：先看到生产者发布的新 head，再读它保护的数据 */
    const uint32_t head = __atomic_load_n(&r->head, __ATOMIC_ACQUIRE);

    if (tail == head) {
        return false;
    }

    *out = r->buf[tail];
    __atomic_store_n(&r->tail, (tail + 1) & (IMU_RING_CAPACITY - 1),
                     __ATOMIC_RELEASE);
    return true;
}
