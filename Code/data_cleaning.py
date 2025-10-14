import pandas as pd
import matplotlib as mpl
import numpy as np

import pandas as pd
import numpy as np

def fill_and_clean(input_path, linear_fill_in, outlier,
                   space_gap_upper, space_gap_lower, speed_FAV_upper, speed_FAV_lower,
                   speed_LV_upper, speed_LV_lower, acc_FAV_upper, acc_FAV_lower,
                   acc_LV_upper, acc_LV_lower):
    """
    清洗 + 线性插值，同时保证 BEV 二维坐标列全程保留。
    """
    df = pd.read_csv(input_path)

    # 1. 边界裁剪
    df.loc[~df['Spatial_Gap'].between(space_gap_lower, space_gap_upper), 'Spatial_Gap'] = np.nan
    df.loc[~df['Speed_FAV'].between(speed_FAV_lower, speed_FAV_upper), 'Speed_FAV'] = np.nan
    df.loc[~df['Acc_FAV'].between(acc_FAV_lower, acc_FAV_upper), 'Acc_FAV'] = np.nan
    df.loc[~df['Speed_LV'].between(speed_LV_lower, speed_LV_upper), 'Speed_LV'] = np.nan
    df.loc[~df['Acc_LV'].between(acc_LV_lower, acc_LV_upper), 'Acc_LV'] = np.nan

    # 2. 无穷 → NaN
    df.replace([np.inf, -np.inf], np.nan, inplace=True)

    # 3. 离群值迭代剔除
    rows_to_delete = set()
    core_cols = ['Speed_FAV', 'Acc_FAV', 'Speed_LV', 'Acc_LV', 'Spatial_Gap', 'Speed_Diff']
    for i, col in enumerate(core_cols):
        if outlier is not None and outlier[i] is not None:
            while True:
                mean = df[col].mean(skipna=True)
                std = df[col].std(skipna=True)
                bad = (df[col] < mean - outlier[i]*std) | (df[col] > mean + outlier[i]*std)
                if bad.any():
                    df.loc[bad, col] = np.nan
                else:
                    break

        # 过长缺失段整段删除
        if linear_fill_in > 0:
            is_na = df[col].isna()
            grp = is_na.ne(is_na.shift()).cumsum()
            na_lens = grp[is_na].value_counts()
            bad_grps = na_lens[na_lens > linear_fill_in].index
            rows_to_delete.update(grp[grp.isin(bad_grps)].index)

    df.drop(index=rows_to_delete, inplace=True)

    # 4. 按轨迹线性插值
    df = df.groupby('Trajectory_ID', group_keys=False).apply(
        lambda g: g.interpolate(method='linear', limit_direction='both')
    )

    # 5. 核心列仍缺失的行才删除（保留 BEV 坐标）
    core_cols = ['Speed_LV','Acc_LV','Speed_FAV','Acc_FAV','Spatial_Gap','Speed_Diff']
    df.dropna(subset=core_cols, inplace=True)

    # 6. 确保 BEV 坐标列存在（没有就补 NaN，防止下游 KeyError）
    for c in ['leader_x','leader_y','follower_x','follower_y',
              'leader_length','follower_length']:
        if c not in df.columns:
            df[c] = np.nan

    # keep = ['Trajectory_ID','Time_Index',
    #         'Speed_LV','Acc_LV','Speed_FAV','Acc_FAV',
    #         'Spatial_Gap','Spatial_Headway','Speed_Diff',
    #         'leader_x','leader_y','follower_x','follower_y']
    # df = df[keep]

    return df

def revise_traj_id(df, output_path, time_step, fill_row_num, fill_start, fill_end, update_time=True):
    # Filter out trajectories shorter than a specified length.
    df = df.groupby('Trajectory_ID').filter(lambda x: len(x) >= fill_row_num)

    # Update time indices to ensure continuity if specified.
    if update_time:
        current_traj_ID = 0
        current_time_ID = 0
        df['Trajectory_ID'] = current_traj_ID
        df['new_Time_Index'] = current_time_ID

        previous_time_ID = df.iloc[0]['Time_Index'] - time_step
        for index, row in df.iterrows():
            if index > 0 and abs(row['Time_Index'] - previous_time_ID) > time_step + 1e-5:
                current_traj_ID += 1
                current_time_ID = 0
            else:
                current_time_ID += time_step

            df.at[index, 'Trajectory_ID'] = current_traj_ID
            df.at[index, 'new_Time_Index'] = current_time_ID
            previous_time_ID = row['Time_Index']

        df['Time_Index'] = df['new_Time_Index']
        df.drop(columns=['new_Time_Index'], inplace=True)

    # Again filter trajectories that are too short.
    df = df.groupby('Trajectory_ID').filter(lambda x: len(x) >= fill_row_num)

    # Remove unstable trajectory sections.
    indices_to_remove = []
    for Trajectory_ID, group in df.groupby('Trajectory_ID'):
        indices_max = group.nlargest(fill_end, 'Time_Index').index
        indices_min = group.nsmallest(fill_start, 'Time_Index').index
        indices_to_remove.extend(indices_max)
        indices_to_remove.extend(indices_min)
    df.drop(indices_to_remove, inplace=True)
    df = df.reset_index(drop=True)
    df['Time_Index'] -= fill_start * time_step

    # Adjust positions relative to the start of each trajectory.
    def adjust_positions(group):
        first_Pos_FAV = group['Pos_FAV'].iloc[0]
        group['Pos_FAV'] -= first_Pos_FAV
        group['Pos_LV'] -= first_Pos_FAV
        return group

    df = df.groupby('Trajectory_ID').apply(adjust_positions)

    # Update trajectory IDs to ensure continuity.
    unique_Trajectory_IDs = df['Trajectory_ID'].unique()
    Trajectory_ID_mapping = {old_id: new_id for new_id, old_id in enumerate(unique_Trajectory_IDs)}
    df['Trajectory_ID'] = df['Trajectory_ID'].map(Trajectory_ID_mapping)

    # 保证 xy 坐标被保留
    keep_cols = ['Trajectory_ID','Time_Index',
                'ID_LV','Type_LV','Pos_LV','Speed_LV','Acc_LV',
                'ID_FAV','Pos_FAV','Speed_FAV','Acc_FAV',
                'Spatial_Gap','Spatial_Headway','Speed_Diff']
    # 追加 BEV 坐标（如果有）
    for c in ['leader_x','leader_y','follower_x','follower_y',
            'leader_length','follower_length']:
        if c in df.columns:
            keep_cols.append(c)
    df = df[keep_cols]

    df.to_csv(output_path, index=False)

def merge_data(merge_data_list, output_path):
    df_list = []
    max_value = 0
    for path in merge_data_list:
        df = pd.read_csv(path)
        if not df.empty:
            df['Trajectory_ID'] += max_value
            max_value = df['Trajectory_ID'].max() + 1
            df_list.append(df)
    merged = pd.concat(df_list)
    merged.to_csv(output_path, index=False)
