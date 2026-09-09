# Elsevier (elsarticle) 投稿版 —— Control Engineering Practice

成品：`paper_cep.pdf`（单栏 preprint，56 页；页数比 arXiv 版多是 elsarticle
`preprint` 版式本身行距宽、图占整页所致，不是内容变多）。

## 生成

```bash
cd report/latex
py md_to_ieee.py        # 从 paper_clean.md 重生成 body/abstract/refs.tex
py make_elsarticle.py   # 生成 elsevier/paper_cep.tex + 8 张图
cd ../elsevier
tectonic paper_cep.tex
```

改正文一律改 `report/paper_clean.md`。三个投递版本同源：

| 版本 | 生成脚本 | 产物 | 用途 |
|---|---|---|---|
| IEEE 双栏 | `make_ieee_single.py` | `latex/paper_ieee.pdf` | 会议/备用 |
| arXiv 单栏 | `make_arxiv.py` | `arxiv/paper_arxiv.pdf` | preprint |
| **Elsevier** | `make_elsarticle.py` | `elsevier/paper_cep.pdf` | **CEP 首投** |

> ⚠️ 三个都要跑。只跑其中一两个会留下一版旧 PDF 且**不报错**——已经踩过。

## 🔴 投稿前必填

1. **作者 / 单位 / 通讯邮箱**：在 `make_elsarticle.py` 的 `PREAMBLE` 里，现为
   `Author Name` / `Affiliation` / `email@domain` 占位符。署名已定「用户一作、
   其博后通讯」，通讯邮箱用博后的机构邮箱。
2. **送审版换 `review` 选项**：`\documentclass[review,11pt]{elsarticle}`
   （双倍行距 + 行号，Elsevier 送审惯例）。现在是 `preprint`。
3. **参考文献**：现用 `thebibliography` 手排 23 条。CEP 最终录用后若要求 BibTeX，
   改用 `\bibliographystyle{elsarticle-num}` + `.bib`。

## 实现说明

- 正文变换直接复用 `make_arxiv.transform_body`，所以三版不会各自漂移。
- **章节编号**：正文 130+ 处 `§N` 交叉引用以 Introduction = §2 为基准。arXiv 版靠把
  Abstract 编成 §1 实现；elsarticle 的摘要在 frontmatter 里不参与编号，故改用
  `\setcounter{section}{1}`。已核验 PDF 里 Introduction 确为 §2。
- **必须显式 `\usepackage{amssymb}`**：elsarticle 不带 arXiv 版那套 newtx 字体，
  §4 用到的 `\mathbb R` 会 Undefined control sequence（已踩过）。
