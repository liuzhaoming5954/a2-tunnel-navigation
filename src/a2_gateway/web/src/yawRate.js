const MAX_IMU_RATE_AGE_MS = 250
const signedRate = new Intl.NumberFormat('zh-CN', {
  signDisplay: 'exceptZero', useGrouping: false, minimumFractionDigits: 1, maximumFractionDigits: 1,
})

// 以请求耗时保守扣减剩余时效。IMU 估算和原厂读数分别使用各自的观测年龄。
export function yawTelemetry(robot, elapsedMs) {
  let nextExpiryMs = Infinity
  function fresh(group, key, limitMs) {
    if (!Number.isFinite(group?.[key]) || !Number.isFinite(group.age_ms)
        || group.age_ms < 0 || !Number.isFinite(group.max_age_ms)) return null
    const remaining = Math.min(group.max_age_ms, limitMs) - group.age_ms - elapsedMs
    if (remaining <= 0) return null
    nextExpiryMs = Math.min(nextExpiryMs, remaining)
    return group[key]
  }
  const imu = fresh(robot?.attitude, 'yaw_rate_radps', MAX_IMU_RATE_AGE_MS)
  const sdk = fresh(robot?.velocity, 'yaw_radps', Infinity)
  return { imu, sdk, nextExpiryMs }
}

export function formatImuYawRate(rate) {
  return Number.isFinite(rate) ? signedRate.format(rate * 180 / Math.PI) : '—'
}
