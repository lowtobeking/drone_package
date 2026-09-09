# arXiv 版（单栏 preprint 投递包）

**成品**：`paper_arxiv.pdf`（25 页，含完整附录 A 41 行表）
→ 同一份也拷到了 `report/Paper_arXiv.pdf`，双击即看。

**上传给 arXiv 的东西 = 本目录里的 `paper_arxiv.tex` + 8 张 `fig*.png`**（就这 9 个文件）。
现成压缩包：`arxiv_submission.zip`（本目录，只含那 9 个文件）。

---

## 与 IEEE 双栏投稿版的关系

正文**完全同源**，都从 `report/paper_clean.md` 生成，只是排版目标不同：

```
report/paper_clean.md                     ← 英文投稿源（唯一改动入口）
   └─ latex/md_to_ieee.py → abstract.tex / body.tex / refs.tex
        ├─ (合并) ─────────→ latex/paper_ieee.tex  → 第二版.pdf     双栏 18 页（会议投稿）
        └─ latex/make_arxiv.py → arxiv/paper_arxiv.tex → Paper_arXiv.pdf  单栏 25 页（arXiv）
```

| | IEEE 版 | arXiv 版（本目录） |
|---|---|---|
| 文档类 | `IEEEtran[conference]` 双栏 | `article[11pt,letterpaper]` 单栏 + newtx(Times) |
| 页数 | 18 页（会议上限 6–8，投稿前需精简） | 25 页，**无上限，附录全留** |
| 宽表 | tabularx 挤在 `table*` 跨栏里 | 全宽自然排，不用缩放 |
| 附录 A | 靠 `\onecolumn`/`\twocolumn` 硬跳单栏 | 本来就是单栏，补丁已去掉 |
| 图 | 5 张挤单栏(≈3.5in) + 3 张跨栏 | 8 张一律 `\textwidth`（都是宽横条图，满宽才看得清轴标签） |
| 章节号 | 罗马数字 I–VII，**与正文 §5.4/§6.5 这类写法对不上** | 阿拉伯数字，**与正文 §引用完全一致**（见下） |
| 链接 | `hidelinks` | 彩色可点（引用/交叉引用/DOI） |

## 三个刻意的排版决定（不是 bug，别"修"）

1. **Abstract 被编成第 1 节**。源文的章节编号里 §1 = Title & Abstract、Introduction = §2，
   而正文有 130+ 处 `§5.4 / §6.5 / §7.2` 这类交叉引用都以此为基准。把 Abstract 显式编成
   第 1 节后，Introduction 自动成为第 2 节 —— 全篇标题编号与正文引用严丝合缝，也不会出现
   "正文从第 2 节开始"的断号。（双栏 IEEE 版用罗马数字掩盖了这个错位，但没解决。）
2. **表的显示编号 1–5 与 `\label` 名 `tab:I/IIb/II/III/IV` 不一致**。因为 II-b 在文中比 II 先
   出现。正文一律用 `\ref{}`，读者看到的编号自洽即可；label 名保持历史叫法方便对话时索引
   （"Table III" = 单栈消融表 = PDF 里的 Table 4）。
3. **附录 A 标题不参与编号**（`\section*`），否则会变成"第 9 节"。

## 提交前必做

1. 🔴 **作者信息**：`paper_arxiv.tex` 里仍是 `Author Name / Affiliation / email@domain` 占位符。
   **PDF 里没有任何提醒**（首页底部原本挂的 `\thanks` 提示按要求去掉了），只有这份 README 记着
   这件事——别漏。arXiv 的**元数据表单和 PDF 里都要真名**，两处要一致。
   改完重新编译；或改 `latex/make_arxiv.py` 里的 `PREAMBLE` 后重跑脚本（**推荐后者**，
   否则下次重新生成会被覆盖）。
2. **arXiv 分类**：建议 primary `eess.SY`（Systems and Control），cross-list `cs.RO`（Robotics）。
3. **License**：默认 arXiv perpetual non-exclusive 即可；若打算投 IEEE 会议，用 CC BY 之外的
   默认许可更省事（IEEE 允许作者版预印本上 arXiv）。
4. **Comments 字段**建议写明："Extended preprint of a single-system SITL case study;
   includes the full 41-scenario regression appendix." —— 与正文里"仅 SITL、单系统"的
   自我定位一致，别在元数据里说过头。
5. 若之后投了会议/期刊，回来在 arXiv 上更新 Comments 加 "Accepted at ..."。

## 上传方法

- 网页提交 → 选 "Upload files" → 传 `arxiv_submission.zip`（或直接把 9 个文件一起选中上传）。
- **不要把 `paper_arxiv.pdf` 一起传**：arXiv 会用你的 `.tex` 自己编译，同时上传 PDF 会冲突。
- 参考文献是手排 `thebibliography`（22 条，无 `.bib`）⇒ **不需要传 `.bbl`**，arXiv 不跑 BibTeX
  这条坑天然绕开了。
- `\pdfoutput=1` 已放在文件第一行，arXiv 据此走 pdfLaTeX（PNG 图必须）。
- 用到的宏包（geometry / newtx / tabularx / booktabs / microtype / caption / hyperref / iftex）
  都在 arXiv 的 TeX Live 里，不用随包上传。

## 改正文后如何重新生成

```bash
cd report/latex
py md_to_ieee.py     # paper_clean.md → abstract/body/refs.tex（IEEE 版也吃这一步）
py make_arxiv.py     # → ../arxiv/paper_arxiv.tex + 8 张图
cd ../arxiv
tectonic paper_arxiv.tex          # 或 pdflatex 跑两遍（解交叉引用）
```

图若重出过：先 `cp ../paper_figures/*.png ../latex/figures/`，`make_arxiv.py` 会自动拷进来，
并检查"正文引用了但包里没有的图"。

> ⚠️ `make_arxiv.py` **不要手改 `paper_arxiv.tex`** —— 它每次重跑都整份重写。
> 要改排版就改脚本里的 `PREAMBLE` 或 `transform_body()`。

## 本地编译踩过的三个坑（脚本里已处理，换机器时可省时间）

1. `newtxmath` 与 `amssymb` 抢 `\Bbbk` 定义 → 直接编译失败。**不要加 amssymb**（newtx 自带
   AMS 符号），且 `amsmath` 要在 `newtxmath` 之前。
2. hyperref 的颜色**不能写成包选项**：`linkcolor=[rgb]{0.05,...}` 会被选项解析器按逗号切开，
   报 ``File `0.05.sty' not found``。改用 `\definecolor` + `\hypersetup`。
3. 本地用 tectonic(XeTeX) 预览时，`\pdfoutput=1` 会骗得 hyperref 去加载 pdfTeX 驱动
   `hpdftex.def`，炸在 `\Hy@pdfmajorversion` 未定义。已加
   `\usepackage{iftex}\ifXeTeX\PassOptionsToPackage{xetex}{hyperref}\fi` 兜住，
   pdfLaTeX(arXiv) 与 XeTeX(本地) 两边都编得过。
