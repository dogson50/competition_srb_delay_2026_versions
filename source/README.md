# 三个候选版本的推理源码

本目录对应 `releases/goal_2026-10-15/` 中的 `hot_ptr`、`lazy_ady`、`lrintf`，
包含 C++ 推理源码、修正表、共享模型参数、miniz 依赖和构建脚本。
这些文件足以离线编译推理程序；原始训练数据、模型训练流水线和编译器不在本次备份范围内。

## 版本差异与来源

| 版本 | 相对变化 | 源码来源 |
|---|---|---|
| `lazy_ady` | mmap 输入、通用模型延迟初始化、JNTMG1 快速预测、按端口类别/跨列/距离修正；默认关闭缓存 | 已保留发布目录的源码快照 |
| `hot_ptr` | 在 `lazy_ady` 基础上开启 `HOT_TERMS`，以固定步长访问 rank-12 预计算项 | 已保留的 `hot_ptr` 实验源码快照 |
| `lrintf` | 在 `lazy_ady` 基础上把两处浮点结果取整改为 `std::lrintf`，并内联、展开坐标解析 | 原实验目录此前已清理；按会话记录中的精确补丁恢复 |

`lrintf` 是 `lazy_ady` 的独立分支，没有合入 `hot_ptr` 的固定步长改动。
三版的共享模型覆盖 208 × 256 = 53,248 个 profile。
共享依赖和参数只存一份，避免三个版本重复存储。

## 目录

```text
source/
  hot_ptr/       estimate.cpp、joint_tp_cross_ady_correction.hpp
  lazy_ady/      estimate.cpp、joint_tp_cross_ady_correction.hpp
  lrintf/        estimate.cpp、joint_tp_cross_ady_correction.hpp
  common/        embedded_model.hpp、joint_model.bin.z、miniz 源码和头文件
  build.ps1      统一构建入口
  verify_build.py  对照原发布包核对重建 ELF
  build_validation.json  本次构建核对结果
```

`joint_model.bin.z` 为编译时嵌入的压缩模型，87,369,852 字节，SHA-256：

```text
7fca5445bfc09f27f7a07961f99db2f39f3dc5cc4710c8423f6246884d562605
```

`embedded_model.hpp` 包含通用回退模型参数。模型数据均会编入可执行文件，
运行时不需要独立模型文件、Python 或网络连接。
miniz 的原有版权及许可声明保留在对应源码中。

## 构建

使用 [Zig 0.13.0](https://ziglang.org/download/0.13.0/) 和 PowerShell。
在仓库根目录运行；编译器可放在 PATH 中，或通过 `-Zig` 指定完整路径。
Windows 完整工作区中已有的 `tools/zig-windows-x86_64-0.13.0/zig.exe` 也会自动识别。

```powershell
.\source\build.ps1 -Variant hot_ptr -Zig 'C:\tools\zig-0.13.0\zig.exe'
.\source\build.ps1 -Variant lazy_ady -Zig 'C:\tools\zig-0.13.0\zig.exe'
.\source\build.ps1 -Variant lrintf -Zig 'C:\tools\zig-0.13.0\zig.exe'
```

默认生成 Linux x86-64 musl 静态程序：`build/<版本>/linux/estimate`。
使用 C++17、`-O3`、`-fno-fast-math`、`-ffp-contract=off`；
Linux CPU 设置分别为 `baseline`（hot_ptr）和 `x86_64`（其他两版）。
生成文件及编译缓存不纳入 Git。

本机 Windows 验证可增加 `-Target windows`，输出为
`build/<版本>/windows/estimate.exe`。`hot_ptr` 的 Windows 构建使用
`x86_64_v3` CPU 特性；Windows 计时不能代替官方 Linux 计时。

## 运行与提交

把生成的 Linux 程序放到 Linux 环境，使用官方请求 CSV：

```sh
chmod +x build/hot_ptr/linux/estimate
./build/hot_ptr/linux/estimate -in public_request.csv -out result.csv --stats
```

输出包含 `From,To,Delay` 三列。`--stats` 把耗时写入 stderr；默认关闭缓存。
`--cache` 可显式启用自适应缓存，`--no-cache` 可显式关闭缓存。
公开请求 CSV 需自行取得，本仓库未上传训练或测试 CSV。

上传官方平台时，使用 [版本索引](../VERSION_INDEX.md) 指定的原 ZIP。
这三个 ZIP 都只有 `bin/estimate`，Unix 可执行权限为 0755；源码目录用于审阅和重建。

## 验证与成绩边界

本次三版均已使用 Zig 0.13.0 重新编译。执行：

```powershell
python .\source\verify_build.py
```

脚本会核对源码清单、发布包 SHA-256、ZIP CRC/权限和静态 ELF 格式，
再比较原发布 ELF 与重建 ELF 的入口地址和所有 `SHF_ALLOC` 节的布局、内容。
`.note.gnu.build-id` 单独排除；文件整体哈希可能因源码路径和调试信息变化而不同。
本次三版全部内存分配节均一致，详细记录见 [build_validation.json](build_validation.json)。

原提交包已通过隔离 QEMU Linux 的公开百万行验证；这次没有重新运行百万行预测，
而是检查重建程序与原包的全部内存分配节。`hot_ptr`、`lrintf` 的原验证报告在各自发布目录；
`lazy_ady` 的原验证摘要见其 `candidate.json`。

三版已记录的公开数据准确率分数均为 **98.5313650545**，公开输出 SHA-256 均为：

```text
aed1ab844b51288f0d3a41c42baac67e348b7b3128a050f0914db1752512b0ad
```

这些是公开数据结果，其中距离修正也使用了公开残差拟合。
三个候选包均尚无官方服务器评分，不能把公开准确率当成官方总分；
`hot_ptr` 的本地计时略有改善，`lrintf` 尚未证明稳定提速。
