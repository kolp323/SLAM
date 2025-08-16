# 3. 生效时机

`.gitignore` 规则只对未被 Git 追踪的文件生效。如果一个文件已经被 Git 追踪（即你曾经 `git add` 并 `git commit` 过它），那么即使你在 `.gitignore` 中添加了该规则，这个文件依然会被追踪。

要让 `.gitignore` 生效，你需要先从 Git 仓库中移除这些文件，但保留在本地文件系统中。

### Bash

```bash
git rm --cached <file_name>
git add.
git commit -m "Remove ignored files"
```

这个命令会把文件从 Git 的索引中移除，但不会删除本地文件。然后，`.gitignore` 规则就会开始生效。