/**
 * @file imu_ring.h
 * @brief 单生产者-单消费者无锁环形缓冲。
 *
 * task_imu 是唯一生产者，task_detect 是唯一消费者。
 * 同步依赖的是 **release/acquire 语义**，不是互斥量：
 *
 *   生产者：写 buf[head] → __atomic_store(&head, next, RELEASE)
 *   消费者：__atomic_load(&head, ACQUIRE) → 读 buf[tail]
 *
 * 为什么不能只写 `volatile`：volatile 只保证"不合并、不省略这次访问"，
 * 不保证它和前面普通写之间的**顺序**。编译器可以把 `buf[head] = *s`
 * 挪到 `head = next` 之后，消费者就会读到还没写完的槽位。
 * ESP32-S3 是双核 Xtensa，核间可见性同样需要屏障。
 *
 * 环满时生产者丢弃新样本（宁可丢点也不阻塞采样）；
 * 消费侧只写 tail，所以生产者的负载判断不需要任何同步。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define IMU_RING_CAPACITY 64 /* 必须是 2 的幂 */

typedef struct {
    float   ax;
    float   ay;
    float   az;
    int64_t ts_us;
} imu_sample_t;

typedef struct {
    imu_sample_t buf[IMU_RING_CAPACITY];
    uint32_t     head;    /* 生产者递增（release 写） */
    uint32_t     tail;    /* 消费者递增（release 写） */
    uint32_t     dropped; /* 溢出丢弃计数，仅生产者改写 */
} imu_ring_t;

void imu_ring_init(imu_ring_t *r);

/** @return false 表示环满，样本已被丢弃 */
bool imu_ring_push(imu_ring_t *r, const imu_sample_t *s);

/** @return false 表示环空 */
bool imu_ring_pop(imu_ring_t *r, imu_sample_t *out);

static inline uint32_t imu_ring_count(const imu_ring_t *r)
{
    /* 纯观测：两个索引都是 32 位对齐读，单个读本身是原子的 */
    const uint32_t head = __atomic_load_n((uint32_t *)&r->head, __ATOMIC_ACQUIRE);
    const uint32_t tail = __atomic_load_n((uint32_t *)&r->tail, __ATOMIC_ACQUIRE);
    return (head - tail) & (IMU_RING_CAPACITY - 1);
}

static inline uint32_t imu_ring_dropped(const imu_ring_t *r)
{
    return __atomic_load_n((uint32_t *)&r->dropped, __ATOMIC_RELAXED);
}
