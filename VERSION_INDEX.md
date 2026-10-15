# SRB delay estimator release bundle

This repository contains inference sources, model assets, and five Linux submission archives from
2026-10-14 and 2026-10-15. Each archive contains one static x86-64 executable at
`bin/estimate` and is below the competition upload limit.

| Version | Public score | Archive | Package SHA-256 |
|---|---:|---|---|
| hot_ptr | 98.5313650545 | `releases/goal_2026-10-15/hot_ptr/submission_joint_rank12_hot_ptr_linux.zip` | `07b956ce5b7b14ed5d6c462891f95d50d22940e9d3bb726662a7cdc0de4bb284` |
| lazy_ady | 98.5313650545 | `releases/goal_2026-10-15/lazy_ady/submission_joint_rank12_lazy_ady_linux.zip` | `4ae3c648303de96200892cb8bf8b21d80a873c3905485c708779f5b6bb684ded` |
| lrintf | 98.5313650545 | `releases/goal_2026-10-15/lrintf/submission_joint_rank12_lrintf_linux.zip` | `a3cd1498e1a292dbc77a5c4c9634a8dcae01df12829012e1748cc8deeeed82ff` |
| cache_repair | 98.5184779563 | `releases/goal_2026-10-14/cache_repair/submission_joint_rank12_stream_cache_safe_linux.zip` | `73fef4f1c9e6b1465e6518a712752aaabb0a0898a045dd3845208a2c1e53b3a5` |
| stream_hot_ady | 98.5313650545 | `releases/goal_2026-10-15/stream_hot_ady/submission_stream_hot_ady_linux.zip` | `6138416f4a76658aa5d8f447fb41c7bd9d6a7063828061617577e677652e8ff5` |

The official server has not scored `hot_ptr`, `lazy_ady`, or `lrintf`. The `candidate.json`
files record the validation scope, public output hash, and rollback package.

`cache_repair` is the October 14 streaming-input and bounded-cache repair.
Its reported official scores are **94.4895 total**, **98.5099 accuracy**,
**71.2104 time**, and **100 consistency**. See
[official_evaluation.json](releases/goal_2026-10-14/cache_repair/official_evaluation.json)
for the score/report files and attribution. Its original `candidate.json` is
preserved as pre-submission metadata, so its pending/null official-score fields
are historical; use `official_evaluation.json` for the subsequent result.

`stream_hot_ady` combines the bounded 8 MiB input reader from `cache_repair`
with the rank-12 fixed-stride hot path and ADY correction from `hot_ptr`.
It reproduces the `98.5313650545` public score while avoiding whole-file
`mmap` of the request CSV. Its Linux ZIP was extracted and run in the same
portable QEMU guest in both default and `--no-cache` modes.

## Sources And Build

See [source/README.md](source/README.md) for Chinese build instructions,
version differences, validation scope, and source provenance.

| Version | Inference Source | Build |
|---|---|---|
| hot_ptr | [estimate.cpp](source/hot_ptr/estimate.cpp) | `./source/build.ps1 -Variant hot_ptr` |
| lazy_ady | [estimate.cpp](source/lazy_ady/estimate.cpp) | `./source/build.ps1 -Variant lazy_ady` |
| lrintf | [estimate.cpp](source/lrintf/estimate.cpp) | `./source/build.ps1 -Variant lrintf` |
| cache_repair | [estimate.cpp](source/cache_repair/estimate.cpp) | `./source/build.ps1 -Variant cache_repair` |
| stream_hot_ady | [estimate.cpp](source/stream_hot_ady/estimate.cpp) | `./source/build.ps1 -Variant stream_hot_ady` |

The snapshots share `source/common/` model assets and miniz dependencies.
Build outputs go to the ignored `build/` directory. See
[build_validation.json](source/build_validation.json) for rebuilt ELF checks.
The original submission archives are preserved; debug-path changes can change
the rebuilt file hash even when all allocated ELF sections match.
