#!/usr/bin/env python3
import json
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def load_data(filename):
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: Could not find {filename}.")
        return {"ar4": [], "abb": []}

def main():
    # 1. Read the saved JSON files
    seq_data = load_data("sequential.json")
    par_data = load_data("parallel.json")

    # 2. Setup Plot
    fig, ax = plt.subplots(figsize=(12, 7))
    y_seq_abb, y_seq_ar4 = 10, 20
    y_par_abb, y_par_ar4 = 40, 50
    bar_height = 6

    ar4_color = '#1f77b4' # Blue
    abb_color = '#ff7f0e' # Orange

    # 3. Draw Sequential Blocks
    if seq_data["ar4"]: ax.broken_barh(seq_data["ar4"], (y_seq_ar4, bar_height), facecolors=ar4_color, edgecolor='black', alpha=0.8)
    if seq_data["abb"]: ax.broken_barh(seq_data["abb"], (y_seq_abb, bar_height), facecolors=abb_color, edgecolor='black', alpha=0.8)

    # 4. Draw Parallel Blocks
    if par_data["ar4"]: ax.broken_barh(par_data["ar4"], (y_par_ar4, bar_height), facecolors=ar4_color, edgecolor='black', alpha=0.8)
    if par_data["abb"]: ax.broken_barh(par_data["abb"], (y_par_abb, bar_height), facecolors=abb_color, edgecolor='black', alpha=0.8)

    # 5. Formatting
    ax.set_xlabel('Elapsed Time (Seconds)', fontweight='bold', fontsize=12)
    ax.set_yticks([y_seq_abb + bar_height/2, y_seq_ar4 + bar_height/2, 
                   y_par_abb + bar_height/2, y_par_ar4 + bar_height/2])
    ax.set_yticklabels(['ABB (Sequential)', 'AR4 (Sequential)', 
                        'ABB (Parallel)', 'AR4 (Parallel)'], fontweight='bold')

    ax.axhline(y=30, color='black', linestyle='--', linewidth=1.5, alpha=0.5)
    ax.text(0, 28, 'SEQUENTIAL EXECUTION ', color='red', fontweight='bold', va='top')
    ax.text(0, 58, 'PARALLEL EXECUTION', color='green', fontweight='bold', va='top')

    ax.set_title('Live Hardware Data: Sequential vs. Parallel Multithreading', fontweight='bold', fontsize=14)
    ax.grid(True, axis='x', linestyle=':', alpha=0.7)
    
    ar4_patch = mpatches.Patch(color=ar4_color, label='AR4 Active (Moving)')
    abb_patch = mpatches.Patch(color=abb_color, label='ABB Active (Moving)')
    ax.legend(handles=[ar4_patch, abb_patch], loc='upper right')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()