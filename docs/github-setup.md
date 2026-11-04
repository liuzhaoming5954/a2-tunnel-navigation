# GitHub 仓库准备与建立

源码仓库已创建：[liuzhaoming5954/a2-tunnel-navigation](https://github.com/liuzhaoming5954/a2-tunnel-navigation)，可见性为 **public**，使用分支 `main`。以下保留首次建立与代码传递步骤；CI 执行结果以 [Actions](https://github.com/liuzhaoming5954/a2-tunnel-navigation/actions) 为准。

## 1. 仓库约定

| 项目 | 建议 / 当前决策 |
|---|---|
| 仓库名 | `a2-tunnel-navigation`。 |
| 所有者 | `liuzhaoming5954`，已核实当前 GitHub 登录账号。 |
| 可见性 | 用户已明确选择 `public`。 |
| 默认分支 | `main`；日常变更使用短期功能分支和 PR。 |
| 许可证 | 尚未选定，不擅自添加开源 LICENSE；公共可见不等于授予开源许可。第三方依赖仍按各自许可使用。 |
| CI | `repository-check` 检查仓库契约并运行任务/存储核心测试；`ros-readonly` 构建与验证无 SDK 的 Jazzy 只读链路；通过不表示 ROS 链路、ARM64 产物或 A2 实机已验收。 |
| 实机执行 | GitHub Actions 不连接 A2、不持有机器人控制凭据、不自动部署或启动机器人。 |

有实际协作后，再按仓库套餐和权限配置 `main` 的 PR 审查与必需检查。保护规则以已成功运行的检查名称为准，不将尚未创建的 ROS/实机任务写成必需检查。

## 2. 哪些内容进入仓库

提交源码、接口、构建说明、配置示例、部署模板、测试定义和小型合成测试夹具。真实地图、外参、网络地址清单、气体标定、巡检照片、数据库、录包和验收原始证据保存在受控的本地数据目录或单独的制品存储中；仓库只保存版本、摘要和经确认可共享的证据索引。

`.gitignore` 已排除 `build/`、`install/`、`log/`、`data/`、`config/local/`、`deploy/local/`、第三方源码缓存、常见录包/数据库和密钥文件。配置示例放 `config/examples/`，合成夹具放 `tests/fixtures/`。忽略规则不影响已被 Git 跟踪的文件，首次暂存及每次提交仍须检查文件清单；不使用 `git add -f` 绕过现场数据规则。

第三方依赖优先通过清单引用仓库和锁定提交，不把整份 SDK、二进制库或厂商授权材料直接复制到本仓库。需要分发时先记录来源、版本、许可和分发条件。

## 3. 本地首次提交

先确认本地目录是否已是真正的 Git 工作区：

```bash
git rev-parse --show-toplevel
git status --short
```

若目录中的 `.git` 是当前工具环境的只读占位，不能将其当成已初始化仓库，也不要删除或改权限绕过限制。在允许写入 Git 元数据的工作目录完成初始化，或使用已获授权的工作区机制；完成后重新检查上述命令。

本轮开发目录属于上述情况，首次发布使用独立副本 `artifacts/github-publish/` 建立 Git 历史，按明确白名单纳入源码、文档和示例；不复制现场数据、第三方缓存或凭据。日常开发应使用正常的仓库 clone，先确认实际工作目录，再提交和推送；不会自动把另一份目录的未提交修改纳入当前仓库。

确认目录尚未初始化且可写时：

```bash
git init -b main
git config --get user.name
git config --get user.email
python3 scripts/check_repository.py
```

身份未配置时使用操作者自己的真实 Git 身份设置仓库级配置，不写入示例身份。先列出文件，再按实际存在的目录明确暂存。当前方案为 `docs/design/A2-PRO_Utility_Tunnel_Navigation_v0.5.md`，由 `docs` 一并纳入；根目录 `README.md` 是阅读入口。

```bash
git status --short --untracked-files=all
git add -- README.md .gitignore .gitattributes .editorconfig .dockerignore .github docs src config docker deploy scripts tests third_party
git diff --cached --check
git diff --cached --stat
git diff --cached
git commit -m "chore: establish A2 navigation project baseline"
```

目录不存在时先修正暂存清单，不能为通过命令临时添加无用途的文件。提交前确认现场数据和凭据未进入暂存区。对已有 Git 仓库只执行检查与正常提交，不重新初始化、强推或覆盖既有历史。

## 4. 创建远端并首次推送

明确所有者、名称、可见性并完成本地提交后，确认 GitHub CLI 当前登录身份及创建权限。创建命令支持 `OWNER/REPO`，`--source` 指定现有本地仓库，`--remote` 设置远端名；首次推送在下一条命令单独执行，便于核对目标。[GitHub CLI 官方说明](https://cli.github.com/manual/gh_repo_create)

```bash
gh auth status
read -r -p '已确认的 GitHub owner/repository: ' repo_full_name
if [[ "$repo_full_name" =~ ^[^/[:space:]]+/[^/[:space:]]+$ ]]; then
  if gh repo create "$repo_full_name" --public --source=. --remote=origin --disable-wiki; then
    git remote -v
    git push -u origin main
    gh repo view "$repo_full_name" --json nameWithOwner,url,visibility,defaultBranchRef
  fi
else
  printf '%s\n' '请先确认并填写 OWNER/REPO；未创建仓库。'
fi
```

示例按用户已确认的公共仓库执行。若目标仓库已经存在或 `origin` 已配置，先核对现有状态，使用已有仓库接入流程，不重复创建或覆盖远端。新建远端不额外生成 README、LICENSE 或 `.gitignore`，避免与本地首个提交产生两套初始历史。

首次推送后核对 `repository-check` 的运行结果，再验证 Issue/PR 模板显示正常。后续实机测试结果以独立的测试记录关联提交 SHA，不把包含现场数据的原始附件上传到 Issue。

## 5. 向 A2 传递代码

公共仓库读取源码可以使用 HTTPS，无需把开发者个人的 GitHub 写入凭据放到机器人上。本机提交并推送后，登记本轮待验证的完整 commit SHA；A2 在停稳的维护窗口获取该版本，不在运行目录中直接更新。

**A2 维护时能联网：** 在新的候选目录获取源码，然后切换到已确认的完整 SHA：

```bash
git clone --branch main --single-branch \
  https://github.com/liuzhaoming5954/a2-tunnel-navigation.git \
  a2-tunnel-navigation-candidate
cd a2-tunnel-navigation-candidate
read -r -p '本轮待验证的完整 commit SHA：' A2_SOURCE_SHA
if [[ "$A2_SOURCE_SHA" =~ ^[0-9a-f]{40}$ ]]; then
  git switch --detach "$A2_SOURCE_SHA"
  git rev-parse HEAD
  git status --short
else
  printf '%s\n' '请填写已确认的完整 SHA，未切换版本。'
fi
```

确认实际 SHA 与登记值一致且工作区无修改后，按目标架构与 profile 的包白名单构建和验证。公开仓库只是源码获取渠道，未安装的 ROS、厂商驱动与第三方依赖仍须单独准备。不能将本机 x86_64 安装产物直接复制给 ARM64。

**A2 维护时也不能联网：** 本机从 GitHub 获取并核对同一提交，用 `git bundle` 或候选源码包通过维护 SSH / 受控介质传入。下例用于正常 Git 工作区，`main` 须已提交并与本轮待验证版本核对；bundle 不包含未提交文件、忽略数据或第三方源码缓存。[Git bundle 官方说明](https://git-scm.com/docs/git-bundle)

```bash
git bundle create a2-source.bundle main
sha256sum a2-source.bundle > a2-source.bundle.sha256
```

把两个文件一并带到 A2，在新的维护目录核验摘要，再克隆到新的候选目录：

```bash
sha256sum -c a2-source.bundle.sha256
git clone --branch main a2-source.bundle a2-tunnel-navigation-candidate
cd a2-tunnel-navigation-candidate
git rev-parse HEAD
git status --short
```

核验 SHA、补齐本地依赖并验证后，才进入后续部署步骤；运行期不访问 GitHub。维护连接与文件传输详见[连接 A2 流程](a2-connection.md)。
