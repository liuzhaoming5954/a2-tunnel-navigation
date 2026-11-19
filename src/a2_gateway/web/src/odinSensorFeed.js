// 订阅与当前点云版本匹配的 PC2 传感器位姿。每次请求结束后才安排下次查询，
// 固定的本地失效计时器独立于网络请求，因此连接卡住时也会撤下旧位置。
export function subscribeOdinSensor(sampleSha256, update) {
  let active = true
  let request = null
  let nextPoll, expires

  function clearPose(status) {
    clearTimeout(expires)
    update(status, null)
  }

  function accept(message, elapsed) {
    if (message?.source !== 'odin1_relocalization') {
      clearPose('invalid')
      return false
    }
    if (message.status !== 'live') {
      const known = ['waiting', 'stale', 'invalid', 'map_mismatch']
      clearPose(known.includes(message.status) ? message.status : 'invalid')
      return false
    }
    if (message.sampleSha256 !== sampleSha256) {
      clearPose('map_mismatch')
      return false
    }
    const { pose, age_ms: age, max_age_ms: maxAge } = message
    if (!pose || ![pose.x_m, pose.y_m, pose.z_m, pose.yaw_rad].every(Number.isFinite)
      || !Number.isFinite(age) || age < 0 || !Number.isFinite(maxAge) || maxAge <= 0) {
      clearPose('invalid')
      return false
    }
    // 网关用 PC2 的接收时刻计算 age。浏览器不比较两台机器的墙钟，
    // 从有效期中再扣除完整请求往返耗时，避免迟到的成功响应重新点亮过期标记。
    const remaining = maxAge - age - elapsed
    if (remaining <= 0) {
      clearPose('stale')
      return false
    }
    clearTimeout(expires)
    expires = setTimeout(() => clearPose('stale'), remaining)
    update('live', pose)
    return true
  }

  async function poll() {
    if (!active || document.hidden || request) return
    const pending = new AbortController()
    request = pending
    const start = performance.now()
    const timeout = setTimeout(() => pending.abort(), 1800)
    let live = false
    try {
      const result = await fetch('/api/maps/odin1/pose?version=' + sampleSha256,
        { signal: pending.signal, cache: 'no-store' })
      if (!result.ok) throw new Error('定位接口不可用')
      const message = await result.json()
      if (request === pending) live = accept(message, performance.now() - start)
    } catch {
      if (request === pending) clearPose('disconnected')
    } finally {
      clearTimeout(timeout)
      if (request === pending) {
        request = null
        nextPoll = setTimeout(poll, live ? 250 : 700)
      }
    }
  }

  // 页面离开前台时终止请求和计时，返回时必须重新取得有效观测。
  function pause() {
    const previous = request
    request = null
    previous?.abort()
    clearTimeout(nextPoll)
    clearPose('waiting')
  }
  function visibilityChanged() {
    if (document.hidden) pause()
    else void poll()
  }
  document.addEventListener('visibilitychange', visibilityChanged)
  void poll()
  return () => {
    active = false
    document.removeEventListener('visibilitychange', visibilityChanged)
    pause()
  }
}
