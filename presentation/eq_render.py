"""
Equation-to-PNG rendering helpers for the presentation.

Uses matplotlib mathtext (no external LaTeX installation required) to render
crisp, transparent-background equation images that are embedded as pictures
in the PPTX. This keeps the deck fully portable / Google-Slides-safe, since
no special math fonts are required at import time.
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

matplotlib.rcParams["mathtext.fontset"] = "cm"
matplotlib.rcParams["font.family"] = "serif"

ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "generated_assets")
os.makedirs(ASSET_DIR, exist_ok=True)


def render_eq(tex, name, fontsize=30, color="white", pad=0.12):
    """Render a single-line mathtext string (wrapped in $...$) to a transparent PNG.
    Returns the absolute file path."""
    fig = plt.figure(figsize=(0.1, 0.1))
    t = fig.text(0.5, 0.5, tex, fontsize=fontsize, ha="center", va="center", color=color)
    fig.canvas.draw()
    out = os.path.join(ASSET_DIR, name)
    fig.savefig(out, dpi=300, transparent=True, bbox_inches="tight", pad_inches=pad)
    plt.close(fig)
    return out


def render_eq_lines(lines, name, fontsize=26, color="white", line_gap=1.5, pad=0.12):
    """Render several mathtext lines (each already wrapped in $...$, or plain text)
    stacked vertically into a single transparent PNG."""
    n = len(lines)
    fig_h = 0.1 + n * 0.01
    fig = plt.figure(figsize=(0.1, fig_h))
    for idx, line in enumerate(lines):
        y = 1.0 - (idx + 0.5) / n
        fig.text(0.5, y, line, fontsize=fontsize, ha="center", va="center", color=color)
    fig.canvas.draw()
    out = os.path.join(ASSET_DIR, name)
    fig.savefig(out, dpi=300, transparent=True, bbox_inches="tight", pad_inches=pad)
    plt.close(fig)
    return out


if __name__ == "__main__":
    render_eq(r"$\dot{x}_i=v_i\cos\theta_i$", "test.png", fontsize=30)
    print("ok")
