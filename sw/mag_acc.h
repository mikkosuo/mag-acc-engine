/* mag_acc.h -- bare-metal driver for the magnitude-accumulate engine (rtl/mag_acc_engine.v)
 *
 * One job:  x2[i] = (accumulate ? x2[i] : 0) + (cordic_mag(x1[i]) >> shift)   (mod 2^32)
 * x1 words pack I in bits [15:0] and Q in bits [31:16]; x2 is a uint32 accumulator.
 *
 * The engine has no atomics-safe sharing scheme: give it to exactly one core.
 * MAE_BASE is the address where the configuration slave is decoded in the SoC.
 */
#ifndef MAG_ACC_H
#define MAG_ACC_H

#include <stdint.h>

#ifndef MAE_BASE
#error "define MAE_BASE to the engine's configuration address"
#endif

#define MAE_CTRL     0x00u
#define MAE_STATUS   0x04u
#define MAE_SRC      0x08u
#define MAE_DST      0x0Cu
#define MAE_LEN      0x10u
#define MAE_SHIFT    0x14u
#define MAE_INFO     0x18u
#define MAE_CYCLES   0x1Cu

#define MAE_CTRL_START   (1u << 0)
#define MAE_CTRL_ACC     (1u << 1)   /* 0 = overwrite (first pulse of a CPI), 1 = accumulate */
#define MAE_CTRL_IRQ_EN  (1u << 2)

#define MAE_ST_BUSY  (1u << 0)
#define MAE_ST_DONE  (1u << 1)       /* write 1 to clear */
#define MAE_ST_ERR   (1u << 2)       /* write 1 to clear; any non-OKAY bus response */

#define MAE_REG(off) (*(volatile uint32_t *)((uintptr_t)(MAE_BASE) + (off)))

#if defined(__riscv)
#define MAE_FENCE() __asm__ volatile("fence iorw, iorw" ::: "memory")
#else
#define MAE_FENCE() __asm__ volatile("" ::: "memory")
#endif

/* Start one job. The fence makes every earlier store (DMA-complete flags, core writes
 * to x1/x2) visible before the START write reaches the engine. */
static inline void mae_start(const uint32_t *x1, uint32_t *x2, uint32_t len,
                             uint32_t shift, int accumulate)
{
    MAE_REG(MAE_SRC)   = (uint32_t)(uintptr_t)x1;
    MAE_REG(MAE_DST)   = (uint32_t)(uintptr_t)x2;
    MAE_REG(MAE_LEN)   = len;
    MAE_REG(MAE_SHIFT) = shift & 31u;
    MAE_FENCE();
    MAE_REG(MAE_CTRL)  = MAE_CTRL_START | (accumulate ? MAE_CTRL_ACC : 0u);
}

static inline int mae_busy(void)
{
    return (MAE_REG(MAE_STATUS) & MAE_ST_BUSY) != 0u;
}

/* Spin until the job is done. Returns the STATUS word (check MAE_ST_ERR) and clears
 * DONE and ERR. The fence orders the core's later loads of x2 after completion. */
static inline uint32_t mae_wait(void)
{
    uint32_t st;
    do {
        st = MAE_REG(MAE_STATUS);
    } while ((st & MAE_ST_DONE) == 0u);
    MAE_REG(MAE_STATUS) = MAE_ST_DONE | MAE_ST_ERR;
    MAE_FENCE();
    return st;
}

/* Clock cycles the last job was busy (profiling). */
static inline uint32_t mae_cycles(void)
{
    return MAE_REG(MAE_CYCLES);
}

#endif /* MAG_ACC_H */
