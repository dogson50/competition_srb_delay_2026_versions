# SRB delay estimator release bundle

This repository contains inference sources, model assets, and three Linux submission archives from
2026-10-15. Each archive contains one static x86-64 executable at
`bin/estimate` and is below the competition upload limit.

| Version | Public score | Archive | Package SHA-256 |
|---|---:|---|---|
| hot_ptr | 98.5313650545 | `releases/goal_2026-10-15/hot_ptr/submission_joint_rank12_hot_ptr_linux.zip` | `07b956ce5b7b14ed5d6c462891f95d50d22940e9d3bb726662a7cdc0de4bb284` |
| lazy_ady | 98.5313650545 | `releases/goal_2026-10-15/lazy_ady/submission_joint_rank12_lazy_ady_linux.zip` | `4ae3c648303de96200892cb8bf8b21d80a873c3905485c708779f5b6bb684ded` |
| lrintf | 98.5313650545 | `releases/goal_2026-10-15/lrintf/submission_joint_rank12_lrintf_linux.zip` | `a3cd1498e1a292dbc77a5c4c9634a8dcae01df12829012e1748cc8deeeed82ff` |

The official server has not scored these three packages. The `candidate.json`
files record the validation scope, public output hash, and rollback package.

## Sources And Build

See [source/README.md](source/README.md) for Chinese build instructions,
version differences, validation scope, and source provenance.

| Version | Inference Source | Build |
|---|---|---|
| hot_ptr | [estimate.cpp](source/hot_ptr/estimate.cpp) | `./source/build.ps1 -Variant hot_ptr` |
| lazy_ady | [estimate.cpp](source/lazy_ady/estimate.cpp) | `./source/build.ps1 -Variant lazy_ady` |
| lrintf | [estimate.cpp](source/lrintf/estimate.cpp) | `./source/build.ps1 -Variant lrintf` |

The snapshots share `source/common/` model assets and miniz dependencies.
Build outputs go to the ignored `build/` directory. See
[build_validation.json](source/build_validation.json) for rebuilt ELF checks.
The original submission archives are preserved; debug-path changes can change
the rebuilt file hash even when all allocated ELF sections match.
