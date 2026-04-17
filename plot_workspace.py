import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    data = pd.read_csv("workspace_data.csv", header=None, names=["X", "Y", "Z"])

    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams["font.sans-serif"] = [
        "Noto Serif CJK SC",
        "Noto Sans CJK SC",
        "AR PL UMing CN",
        "WenQuanYi Zen Hei",
        "SimHei",
        "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False

    fig = plt.figure(figsize=(14, 10))

    ax1 = fig.add_subplot(2, 2, 1, projection="3d")
    ax1.scatter(data["X"], data["Y"], data["Z"], c="#2980B9", s=0.5, alpha=0.1)
    ax1.set_title("UR10e Workspace (3D) / UR10e 三维可达域")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.scatter(data["X"], data["Y"], c="#27AE60", s=0.5, alpha=0.05)
    ax2.set_title("X-Y Plane Reachability / X-Y 平面可达域")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_aspect("equal", adjustable="box")

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.scatter(data["X"], data["Z"], c="#E67E22", s=0.5, alpha=0.05)
    ax3.set_title("X-Z Plane Reachability / X-Z 平面可达域")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Z (m)")
    ax3.set_aspect("equal", adjustable="box")

    ax4 = fig.add_subplot(2, 2, 4)
    ax4.scatter(data["Y"], data["Z"], c="#8E44AD", s=0.5, alpha=0.05)
    ax4.set_title("Y-Z Plane Reachability / Y-Z 平面可达域")
    ax4.set_xlabel("Y (m)")
    ax4.set_ylabel("Z (m)")
    ax4.set_aspect("equal", adjustable="box")

    fig.tight_layout()
    fig.savefig("workspace_analysis.svg", format="svg", dpi=300, bbox_inches="tight")


if __name__ == "__main__":
    main()
