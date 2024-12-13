import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.cm import get_cmap
import sys

# CSVファイルのパスを指定
csv_file = 'tools/param_tuner/logs/latest.csv'  # 適切なファイルパスに置き換えてください

*args, = sys.argv

# CSVデータを読み込む
data = pd.read_csv(args[1])

# pos_xとpos_yのカラムが存在するか確認
if 'x' in data.columns and 'y' in data.columns:
    # pos_xとpos_yのデータを取得
    # cmap = ['cyan', 'magenta', 'yellow', 'green', 'red', 'blue']

    # データの順序を確保（motion_stateごとに並べ替え）
    data = data.sort_values(
        by=['timestamp', 'x', 'y']).reset_index(drop=True)

    last_motion_state = data['timestamp'].iloc[-2]

    # 同じ motion_state に戻る部分を削除
    cleaned_data = data.loc[data['timestamp'].diff().fillna(0) >= 0]

    filtered_data = data[data['timestamp'] != last_motion_state]

    # motion_stateのユニークな値を取得
    unique_states = filtered_data['timestamp'].unique()
    num_states = len(unique_states)

    # カラーマップを生成
    cmap = get_cmap('viridis', num_states)
    # cmap = get_cmap('tab20', num_states)
    # cmap = get_cmap('tab20c', num_states)
    # cmap = get_cmap('tab20b', num_states)
    # プロット
    plt.style.use('dark_background')  # 黒背景に設定
    plt.figure(figsize=(16, 9))  # アスペクト比を1:1に近づけるため正方形のキャンバスを使用

 # motion_stateごとにプロット
    for i, (state, group) in enumerate(filtered_data.groupby('timestamp')):
        pos_x = group['x']+45-9
        pos_y = group['y']
        color = cmap(i / num_states)  # カラーマップから色を取得
        plt.plot(pos_x, pos_y, ".", markersize=4,
                 color=color, label=f'State {state}')

    # タイトルとラベル
    plt.title('Position Plot with Motion States', color='white')
    plt.xlabel('pos_x', color='white')
    plt.ylabel('pos_y', color='white')

    # 補助線を90刻みに設定
    plt.grid(True, color='gray', linestyle='--', linewidth=0.5)

    for i, (state, group) in enumerate(data.groupby('timestamp')):
        pos_x2 = group['x']
        pos_y2 = group['y']

    x_min = min(data['x'])
    x_max = max(data['x'])
    y_min = min(data['y'])
    y_max = max(data['y'])

    gs = 45
    plt.xticks(np.arange(np.floor(x_min / gs) * gs,
               np.ceil(x_max / gs) * gs + 1, gs), color='white')
    plt.yticks(np.arange(np.floor(y_min / gs) * gs,
               np.ceil(y_max / gs) * gs + 1, gs), color='white')

    alpha = 0.25
    lw = 6

    # 走行経路に合わせて線の本数を増やす
    for i in range(0, int(x_max+90), 90):
        if y_max < 45:
            plt.plot([i, i], [45, y_min-45],
                     color=(1, 0, 0, alpha), linewidth=lw)
        else:
            plt.plot([i, i], [45, y_max+45],
                     color=(1, 0, 0, alpha), linewidth=lw)

            # plt.plot([i, i], [90, y_min-90], color=(1, 0, 0, alpha), linewidth=lw)

    for i in range(0, int(max(-y_min, y_max)), 90):
        if y_max < 45:
            plt.plot([0, x_max+90], [-i+45, -i+45],
                     color=(1, 0, 0, alpha), linewidth=lw)
        else:
            plt.plot([0, x_max+90], [i+45, i+45],
                     color=(1, 0, 0, alpha), linewidth=lw)


    plt.grid(True, color='gray', linestyle='--', linewidth=0.5)

    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()
else:
    print("Error: CSV file must contain 'pos_x' and 'pos_y' columns.")
