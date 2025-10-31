#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from typing import Optional
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def plot_bars(df_bin: pd.DataFrame, metric_name: str, ylabel: str, out_dir: Path, reference_line: Optional[float] = None):
    out_dir.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(10,5))
    plt.bar(df_bin['distance_bin_m'], df_bin[metric_name], width=80, align='center', color='#4C78A8')
    xmin = df_bin['distance_bin_m'].min(); xmax = df_bin['distance_bin_m'].max()
    for x in range(int(xmin), int(xmax)+1, 10):
        plt.axvline(x=x, color='#DDDDDD', linewidth=0.3)
    if reference_line is not None:
        plt.axhline(y=reference_line, color='red', linestyle='--', label='10% reference')
        plt.legend()
    plt.xlabel('Distance bin (m)')
    plt.ylabel(ylabel)
    plt.title(f'{metric_name} by 100m bins')
    plt.tight_layout()
    out_path = out_dir / f'{metric_name}_bars.png'
    plt.savefig(out_path, dpi=200)
    plt.close()
    return out_path


def plot_scatter_with_exp_fit(df: pd.DataFrame, x_col: str, y: pd.Series, metric_name: str, ylabel: str, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    x = df[x_col].astype(float).values
    yv = y.astype(float).values
    plt.figure(figsize=(10,5))
    plt.scatter(x, yv, s=12, alpha=0.6, color='#72B7B2', label='samples')
    mask = yv > 0
    if mask.sum() >= 2:
        X = x[mask]; Y = yv[mask]
        b, a_intercept = np.polyfit(X, np.log(Y), 1)
        a = math.exp(a_intercept)
        x_fit = np.linspace(X.min(), X.max(), 200)
        y_fit = a * np.exp(b * x_fit)
        plt.plot(x_fit, y_fit, color='#F58518', label='exp fit')
        plt.legend()
    plt.xlabel('Distance (m)')
    plt.ylabel(ylabel)
    plt.title(f'{metric_name} scatter with exponential trend')
    plt.tight_layout()
    out_path = out_dir / f'{metric_name}_scatter.png'
    plt.savefig(out_path, dpi=200)
    plt.close()
    return out_path
