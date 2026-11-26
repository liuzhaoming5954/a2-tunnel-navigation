import assert from 'node:assert/strict'
import test from 'node:test'
import { formatImuYawRate, yawTelemetry } from '../src/yawRate.js'

test('IMU 转速保留偏航符号，过期不遮蔽原厂零值', () => {
  const robot = {
    online: true,
    attitude: { yaw_rate_radps: Math.PI / 18, age_ms: 100, max_age_ms: 500 },
    velocity: { yaw_radps: 0, age_ms: 20, max_age_ms: 500 },
  }
  const left = yawTelemetry(robot, 70)
  assert.equal(formatImuYawRate(left.imu), '+10.0')
  assert.equal(left.nextExpiryMs, 80)
  assert.equal(left.sdk, 0)
  assert.equal(formatImuYawRate(yawTelemetry({ ...robot,
    attitude: { ...robot.attitude, yaw_rate_radps: -Math.PI / 18 },
  }, 149).imu), '-10.0')
  assert.deepEqual(yawTelemetry(robot, 150), { imu: null, sdk: 0, nextExpiryMs: 330 })
  assert.deepEqual(yawTelemetry(robot, 480), { imu: null, sdk: null, nextExpiryMs: Infinity })
})

test('IMU 零转速独立于原厂运动状态，无效或缺失的数据保持未知', () => {
  const stationary = { online: true, attitude: { yaw_rate_radps: 0, age_ms: 0, max_age_ms: 100 } }
  assert.equal(formatImuYawRate(yawTelemetry(stationary, 0).imu), '0.0')
  assert.equal(formatImuYawRate(yawTelemetry({ ...stationary, online: false }, 0).imu), '0.0')
  assert.equal(formatImuYawRate(yawTelemetry(stationary, 100).imu), '—')
  const absent = { imu: null, sdk: null, nextExpiryMs: Infinity }
  for (const robot of [undefined, { online: true },
    { online: true, attitude: { ...stationary.attitude, age_ms: -1 } }]) {
    assert.deepEqual(yawTelemetry(robot, 0), absent)
  }
})
