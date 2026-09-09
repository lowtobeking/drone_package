# IEEEtran LaTeX 版（投稿稿）

从 `report/paper_clean.md` 转成 IEEE 会议双栏格式（ICUAS/IROS 通用）。**数学、表、图、参考文献均原生渲染**——解决 Markdown-PDF 插件不渲染公式的问题。

> **另有 Elsevier 投稿版**：`report/elsevier/`（`py make_elsarticle.py` 生成，成品
> `paper_cep.pdf`，目标期刊 Control Engineering Practice）。与本版同源。
>
> **另有 arXiv 单栏 preprint 版**：`report/arxiv/`（`py make_arxiv.py` 生成，成品
> `report/Paper_arXiv.pdf` 25 页含完整附录，另带现成上传包 `arxiv_submission.zip`）。
> 正文与本双栏版同源，只是排版目标不同——详见 `report/arxiv/README.md`。
>
> **改正文一律改 `paper_clean.md`，然后三版一起刷新**：
> `py md_to_ieee.py && py make_arxiv.py && py make_ieee_single.py && py make_elsarticle.py`
> ——只跑其中一两个会留下旧 PDF 且**不报错**，已踩过。

## 在 Overleaf 编译（推荐，免本地安装）

1. 打开 https://overleaf.com → **New Project → Upload Project**（或先建空项目再拖文件）。
2. 把整个 `latex/` 文件夹的内容传上去：
   - `main.tex`、`abstract.tex`、`body.tex`、`refs.tex`
   - `figures/` 整个文件夹（8 张 png）
   - `IEEEtran.cls` **不用传**——Overleaf 内置。
3. 左上把**主文件设为 `main.tex`**，编译器用默认 **pdfLaTeX**，点 Recompile。
4. 出 8–10 页双栏 PDF，公式/表/图/引用全部正确。

> 首次编译会有 IEEEtran 关于 `hyperref` 的 warning，可忽略，不影响出 PDF。

## 本地编译（若已装 TeX Live/MiKTeX）

```bash
cd report/latex
pdflatex main && pdflatex main   # 跑两遍解交叉引用
```

## 文件说明

| 文件 | 内容 | 是否手改 |
|---|---|---|
| `main.tex` | 骨架：文档类/宏包/**标题**/**作者**/摘要+关键词框/引入 body 与 refs | ✅ 手写，作者信息待填 |
| `abstract.tex` / `body.tex` / `refs.tex` | 由脚本从 `paper_clean.md` 自动生成 | ❌ 别手改，改源再重生成 |
| `md_to_ieee.py` | 转换脚本（Markdown→LaTeX） | — |
| `figures/*.png` | 8 张图（从 `../paper_figures/` 拷入） | — |

## 正文更新后如何重生成

改 `report/paper_clean.md`（英文投稿源）后：

```bash
cd report/latex
python md_to_ieee.py            # 重生成 abstract/body/refs.tex
```

图若有更新：`cp ../paper_figures/*.png figures/`。

## ⚠️ 投稿前必须手动处理

1. **作者与单位**：`main.tex` 里的 `\author{...}` 目前是占位符 `Author Name / Affiliation`，填真实信息。
2. **参考文献**：现用 `thebibliography` 手排条目（22 条），若期刊要 BibTeX 可另转；DOI/arXiv 号已带。
3. **页数**：双栏预计 8–10 页，超出会议 6–8 页上限时按需精简（附录 A 全表、Table IV 可先删）。
4. **图号/表号**：正文里仍有 "Fig. 2:"、"Table I" 等文字表述与自动编号并存，通读时把散文里的 "Fig. N"/"§X" 改成 `\ref{}`/`\S\ref{}` 交叉引用更规范（非必须，不影响编译）。
5. **宽表**：Table III/IV 用 `\resizebox` 强缩到栏宽，字偏小；投稿版可改为手排 `p{}` 列宽或拆表。
6. **宽公式**：OCP 目标函数用 `\resizebox` 缩放；如嫌小可改 `multline`/`split` 手动折行。
