# 本地受控操作入口

默认关闭。只有显式设置 `A3T_CONTROL_SOCKET` 才启用 Unix socket；只允许当前用户访问，
不监听 TCP，不创建第二个 SDK 连接，所有操作复用 GUI 的 ArmWorker 线程。
不要同时从 GUI 或厂家上位机发送运动命令。

启动示例（只打开程序，不连接、不使能、不运动）：

```bash
control_dir=$(mktemp -d /tmp/a3t-control-XXXXXX)
A3T_CONTROL_SOCKET="$control_dir/control.sock" ./build/a3t_handle
```

在另一个终端使用上面实际路径查询：

```bash
python3 tools/local_control.py --socket /tmp/a3t-control-实际目录/control.sock status
```

写请求必须包含 `--confirm --session <status 返回的 session>`。
客户端为每次请求生成唯一 id；服务器拒绝重复 id、错误会话、不支持的命令。
超时表示结果不确定，禁止自动重发，先查询状态/日志。

支持命令：

| 命令 | 行为 |
| --- | --- |
| status | 缓存状态、快照年龄、最近提示、日志目录；不是直接硬件读数 |
| connect | 连接配置中的 IP；不自动清错/使能 |
| start_mit | 需已连接、已使能、健康且处于高层；会执行缓存回接与到中心运动 |
| gravity | 复用纯重力入口；可能漂移/下坠 |
| probe --joint 1 --direction 1 | 当前姿态单次测试，关节 1–6、方向 ±1 |
| batch | 当前姿态 36 次测试；失败不自动恢复或重试 |
| stop | 立即设置线程安全的批次取消标记，排队执行现有退出流程 |
| emergency | 同样设置取消标记，排队调用软件急停；不是实体急停替代品 |

没有复位、清错、使能、任意力矩、配置修改接口。启动本地入口并不授权自动运动，
每次写请求仍需现场确认。仅本地同用户权限保护，不防范同用户恶意进程。

`queued=true` 仅代表进入队列，绝不表示控制器接受或测试成功。
以状态更新、最近提示和 diagnostics.jsonl 中的实际结果为准。
耗时运动期间停止 SDK 调用仍可能排队，现场必须有可操作的实体急停。
快照过期时拒绝普通运动请求；停止请求不因快照过期而被挡住。
客户端断开不会自动停止机械臂，GUI/本地服务仍可能持续发送 MIT；务必主动停止并确认状态。

同一 socket 路径有锁保护；已存在端点不会自动删除或接管，崩溃后应选新路径并核对旧进程。
离线集成测试仅验证协议、权限和拒绝路径，不调用任何已接受的硬件操作。
