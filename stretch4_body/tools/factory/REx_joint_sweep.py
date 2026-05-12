#!/usr/bin/env python3

import argparse
import time
import numpy as np
from scipy.optimize import curve_fit
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Run sweeps to characterize friction and other disturbances ')

group1 = parser.add_mutually_exclusive_group(required=True)
group1.add_argument("--arm", help="Test trajectories on the arm joint", action="store_true")
group1.add_argument("--lift", help="Test trajectories on the lift joint", action="store_true")
group1.add_argument("--compare", help="Compare data for provided tags", nargs='+', type=str)
group1.add_argument("--report", help="Generate a PDF report for the provided tag", action="store_true")
group1.add_argument("--model_tags", help="Build a predictive current model for provided tags", nargs='+', type=str)

parser.add_argument("--test_model", help="Load and test a predictive model directory name", type=str)
parser.add_argument("--generate_model_data", help="Generate model data", action="store_true")
parser.add_argument("--tag", help="Tag to prefix saved data run", type=str, default="default_sweep")

parser.add_argument("--nsweep", help="Num sweeps per test", type=int, default=6)
parser.add_argument("--select", help="Select which test to run from a command line menu", action="store_true")

args=parser.parse_args()


if args.arm:
    import stretch4_body.subsystem.arm as aa
    arm = aa.Arm()
    if not arm.startup():
        exit(1)
    xtop = arm.params['range_m'][1]#-.003
    xbottom = arm.params['range_m'][0]+.00
    arm.set_soft_motion_limit_max(x=xtop)
    arm.set_soft_motion_limit_min(x=xbottom)
    arm.motor.disable_sync_mode()
    arm.motor.disable_guarded_mode()
    arm.push_command()
    arm.pull_status()
    print('Arm selected')


if args.lift:
    import stretch4_body.subsystem.lift as ll
    lift = ll.Lift()
    if not lift.startup():
        exit(1)
    xtop = lift.params['range_m'][1]-.005
    xbottom = lift.params['range_m'][0]+.005
    lift.set_soft_motion_limit_max(x=xtop)
    lift.set_soft_motion_limit_min(x=xbottom)
    lift.motor.disable_sync_mode()
    lift.motor.disable_guarded_mode()
    lift.push_command()
    lift.pull_status()
    print('Lift selected')



def collect_data(test_name,device, x_start, x_end, v_des, a_des,n_sweep):
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_title(f"Joint Sweep: {test_name}")
    ax.set_xlabel("Position (m)")
    ax.set_ylabel("Current (A)")
    line, = ax.plot([], [], 'r-')
    plt.show(block=False)

    cnt = 0
    dir_pos = True

    device.move_to(x_m=x_start)
    device.push_command()
    while abs(device.status['pos']-x_start) > 0.005:
        device.pull_status()
    print('Collecting data for ', test_name)
    time.sleep(0.2)
    pos = []
    current = []
    vel=[]
    ts=[]
    dir=[]
    sweep=[]
    ts_start=time.time()
    
    while cnt < n_sweep:
        
        device.pull_status()
        current.append(device.status['motor']['current'])
        pos.append(device.status['pos'])
        vel.append(device.status['vel'])
        dir.append(dir_pos)
        ts.append(time.time()-ts_start)
        sweep.append(cnt)

        line.set_data(pos, current)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()

        if device.status['pos'] <= 1.05*x_start and not dir_pos:
            dir_pos = True
            cnt = cnt + 1
            print('Starting pos sweep #', cnt)

        if device.status['pos'] >= 0.95*x_end and dir_pos:
            dir_pos = False
            cnt = cnt + 1
            print('Starting neg sweep #', cnt)
        
        if dir_pos:
            device.move_to(x_m=x_end, v_m=v_des, a_m=a_des)

        if not dir_pos: 
            device.move_to(x_m=-x_start, v_m=v_des, a_m=a_des)

        device.push_command()

    device.move_to(x_m=x_start, v_m=v_des, a_m=a_des)
    device.push_command()
    print('Finished collecting data for ', test_name)
    import os
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    tmp_path = f"/tmp/{args.tag}_{test_name}_{timestamp}.png"
    fig.savefig(tmp_path)
    print(f"Saved plot to {tmp_path}")
    
    plt.ioff()
    plt.show(block=False)
    return ts,dir,sweep,current, pos, vel, tmp_path

def process_data(test_name, device_name, x_start, x_end, v_des, a_des, ts,dir,sweep,current, pos, vel,pad_accel_t=0.1):
    print('Processing data for ',test_name, 'and device',device_name)
    data_processed={'test_name':test_name,'device_name':device_name,'x_start':x_start,'x_end':x_end,'v_des':v_des,'a_des':a_des,'test_time':time.time()}
    data_processed['ts']=ts

    data_processed['all'] = {}
    data_processed['all']['current'] = current
    data_processed['all']['pos'] = pos
    data_processed['all']['vel'] = vel
    data_processed['all']['dir'] = dir
    data_processed['all']['ts'] = ts
    data_processed['all']['sweep'] = sweep

    current_arr = np.array(current)
    pos_arr = np.array(pos)
    vel_arr = np.array(vel)
    ts_arr = np.array(ts)
    dir_arr = np.array(dir)
    sweep_arr = np.array(sweep)

    #These are the features to compute for each segment
    data_processed['all']['max_curr'] = float(np.max(current_arr)) if len(current_arr) > 0 else None
    data_processed['all']['min_curr'] = float(np.min(current_arr)) if len(current_arr) > 0 else None
    data_processed['all']['std_curr'] = float(np.std(current_arr)) if len(current_arr) > 0 else None
    #data_processed['all']['hysteresis']=None

    t_accel = v_des / a_des + pad_accel_t
    accel_indices = []
    const_vel_indices = []
    deccel_indices = []
    
    for s in np.unique(sweep_arr):
        sweep_idx = np.where(sweep_arr == s)[0]
        if len(sweep_idx) == 0:
            continue
            
        ts_sweep = ts_arr[sweep_idx]
        ts_start = ts_sweep[0]
        ts_end = ts_sweep[-1]
        
        t_rel = ts_sweep - ts_start
        total_time_sweep = ts_end - ts_start
        
        accel_mask = t_rel <= t_accel
        deccel_mask = t_rel >= (total_time_sweep - t_accel)
        const_vel_mask = ~(accel_mask | deccel_mask)
        
        accel_indices.extend(sweep_idx[accel_mask])
        deccel_indices.extend(sweep_idx[deccel_mask])
        const_vel_indices.extend(sweep_idx[const_vel_mask])
        
    accel_indices = np.array(accel_indices, dtype=int)
    deccel_indices = np.array(deccel_indices, dtype=int)
    const_vel_indices = np.array(const_vel_indices, dtype=int)
    
    def populate_phase_data(phase_dict, indices):
        phase_dict['ts'] = ts_arr[indices].tolist() if len(indices) else []
        phase_dict['current'] = current_arr[indices].tolist() if len(indices) else []
        phase_dict['pos'] = pos_arr[indices].tolist() if len(indices) else []
        phase_dict['vel'] = vel_arr[indices].tolist() if len(indices) else []
        phase_dict['dir'] = dir_arr[indices].tolist() if len(indices) else []
        phase_dict['sweep'] = sweep_arr[indices].tolist() if len(indices) else []
        
        curr = current_arr[indices]
        phase_dict['max_curr'] = float(np.max(curr)) if len(curr) > 0 else None
        phase_dict['min_curr'] = float(np.min(curr)) if len(curr) > 0 else None
        phase_dict['std_curr'] = float(np.std(curr)) if len(curr) > 0 else None

    #Now pull out segment of motion without accel
    data_processed['constant_vel_phase']={}
    populate_phase_data(data_processed['constant_vel_phase'], const_vel_indices)

    # Now pull out segment of motion during accel
    data_processed['accel_phase'] = {}
    populate_phase_data(data_processed['accel_phase'], accel_indices)

    # Now pull out segment of motion during deccel
    data_processed['deccel_phase'] = {}
    populate_phase_data(data_processed['deccel_phase'], deccel_indices)

    print("--- Phase Stats [max, min, std] (A) ---")
    def print_phase(name, phase):
        print(f"{name:15s}: max={phase.get('max_curr')}, min={phase.get('min_curr')}, std={phase.get('std_curr')}")
    print_phase("All", data_processed['all'])
    print_phase("Accel", data_processed['accel_phase'])
    print_phase("Constant Vel", data_processed['constant_vel_phase'])
    print_phase("Deccel", data_processed['deccel_phase'])
    print("---------------------------------------")

    return data_processed

def display_data(processed_data):
    import matplotlib.pyplot as plt
    import time
    
    test_name = processed_data['test_name']
    
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))
    fig.suptitle(f"Joint Sweep Phases: {test_name}")
    
    phases = [
        ('all', 'All Data', axs[0]),
        ('constant_vel_phase', 'Constant Velocity Phase', axs[1])
        # ('accel_phase', 'Acceleration Phase', axs[0, 1]),
        # ('deccel_phase', 'Deceleration Phase', axs[1, 1])
    ]
    
    for key, title, ax in phases:
        phase_data = processed_data.get(key, {})
        if len(phase_data.get('pos', [])) > 0:
            ax.plot(phase_data['pos'], phase_data['current'], 'b.', markersize=2)
        ax.set_title(title)
        ax.set_xlabel("Position (m)")
        ax.set_ylabel("Current (A)")
        ax.grid(True)
        
    plt.tight_layout()
    import os
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    tmp_path = f"/tmp/{args.tag}_{test_name}_phases_{timestamp}.png"
    fig.savefig(tmp_path)
    print(f"Saved phase plots to {tmp_path}")
    
    plt.show(block=False)
    return [tmp_path]

def save_data(processed_data, display_images, joint_name):
    import os, json, shutil
    fleet_dir = hu.get_fleet_path()
    log_dir = os.path.join(fleet_dir, 'log', f'sweep_{joint_name}', args.tag)
    os.makedirs(log_dir, exist_ok=True)
    
    test_name = processed_data['test_name']
    json_path = os.path.join(log_dir, f"{args.tag}_{test_name}_data.json")
    
    with open(json_path, 'w') as f:
        json.dump(processed_data, f, indent=2)
    print(f"Saved processed data to {json_path}")
    
    for img in display_images:
        dest = os.path.join(log_dir, os.path.basename(img))
        shutil.copy(img, dest)
        print(f"Saved plot to {dest}")
        
    return json_path

def compare_data(tags):
    import os, json, glob, time
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    
    try:
        from fpdf import FPDF
        has_fpdf = True
    except ImportError:
        print("fpdf2 not installed. Cannot generate report. Run: pip install fpdf2")
        has_fpdf = False

    fleet_dir = hu.get_fleet_path()
    
    if has_fpdf:
        pdf = FPDF()
        pdf.set_auto_page_break(auto=True, margin=15)
        pdf.add_page()
        pdf.set_font("helvetica", "B", 16)
        pdf.cell(0, 10, f"Joint Sweep Comparison", new_x="LMARGIN", new_y="NEXT", align="C")
        pdf.set_font("helvetica", "", 12)
        pdf.cell(0, 10, f"Tags: {', '.join(tags)}", new_x="LMARGIN", new_y="NEXT", align="C")
        pdf.ln(5)

    all_data = {t: [] for t in tags}
    for t in tags:
        arm_dir = os.path.join(fleet_dir, 'log', 'sweep_arm', t)
        lift_dir = os.path.join(fleet_dir, 'log', 'sweep_lift', t)
        
        json_files = []
        if os.path.exists(arm_dir):
            json_files.extend(glob.glob(os.path.join(arm_dir, f"{t}_*_data.json")))
        if os.path.exists(lift_dir):
            json_files.extend(glob.glob(os.path.join(lift_dir, f"{t}_*_data.json")))
            
        if not json_files:
            print(f"Warning: No log files found for tag '{t}' in arm or lift directories.")
            continue
        
        for jf in json_files:
            with open(jf, 'r') as f:
                all_data[t].append(json.load(f))
                
    test_names = set()
    for t in tags:
        for d in all_data[t]:
            test_names.add(d['test_name'])
            
    for test_name in test_names:
        print(f"\n{'='*40}")
        print(f"Comparison for Test: {test_name}")
        print(f"{'='*40}")
        
        if has_fpdf:
            pdf.add_page()
            pdf.set_font("helvetica", "B", 14)
            pdf.cell(0, 10, f"Test: {test_name}", new_x="LMARGIN", new_y="NEXT")
            pdf.ln(5)
            
            pdf.set_font("helvetica", "B", 10)
            col_w_phase = 45
            col_w_tag = 35
            col_w_curr = 30
            pdf.cell(col_w_phase, 8, "Phase")
            pdf.cell(col_w_tag, 8, "Tag")
            pdf.cell(col_w_curr, 8, "Max Curr (A)")
            pdf.cell(col_w_curr, 8, "Min Curr (A)")
            pdf.cell(col_w_curr, 8, "Std Curr (A)", new_x="LMARGIN", new_y="NEXT")
            pdf.set_font("helvetica", "", 10)
        
        fig, axs = plt.subplots(1, 2, figsize=(10, 4))
        fig.suptitle(f"Joint Sweep Phases Comparison: {test_name}")
        phases = [
            ('all', 'All Data', axs[0]),
            ('constant_vel_phase', 'Constant Velocity Phase', axs[1])
            # ('accel_phase', 'Acceleration Phase', axs[0, 1]),
            # ('deccel_phase', 'Deceleration Phase', axs[1, 1])
        ]
        
        print(f"{'Phase':<25} | {'Tag':<15} | {'Max Curr (A)':<15} | {'Min Curr (A)':<15} | {'Std Curr (A)':<15}")
        print("-" * 92)
        
        for key, title, ax in phases:
            for t in tags:
                tests_for_tag = [d for d in all_data[t] if d['test_name'] == test_name]
                if not tests_for_tag:
                    continue
                d = sorted(tests_for_tag, key=lambda x: x.get('test_time', 0))[-1]
                
                phase_data = d.get(key, {})
                
                if len(phase_data.get('pos', [])) > 0:
                    ax.plot(phase_data['pos'], phase_data['current'], '.', markersize=2, label=t)
                
                max_c = phase_data.get('max_curr')
                min_c = phase_data.get('min_curr')
                std_c = phase_data.get('std_curr')
                
                max_s = f"{max_c:.4f}" if max_c is not None else "N/A"
                min_s = f"{min_c:.4f}" if min_c is not None else "N/A"
                std_s = f"{std_c:.4f}" if std_c is not None else "N/A"
                
                print(f"{title:<25} | {t:<15} | {max_s:<15} | {min_s:<15} | {std_s:<15}")
                
                if has_fpdf:
                    pdf.cell(col_w_phase, 8, title)
                    pdf.cell(col_w_tag, 8, t)
                    pdf.cell(col_w_curr, 8, max_s)
                    pdf.cell(col_w_curr, 8, min_s)
                    pdf.cell(col_w_curr, 8, std_s, new_x="LMARGIN", new_y="NEXT")
            
            ax.set_title(title)
            ax.set_xlabel("Position (m)")
            ax.set_ylabel("Current (A)")
            ax.grid(True)
            ax.legend()
            
        plt.tight_layout()
        
        if has_fpdf:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            tmp_img_path = f"/tmp/compare_{test_name}_{timestamp}.png"
            fig.savefig(tmp_img_path)
            pdf.ln(5)
            pdf.image(tmp_img_path, x=20, w=160)
            
        plt.show(block=True)
        
    if has_fpdf and test_names:
        report_dir = os.path.join(fleet_dir, 'log', 'sweep_compare')
        os.makedirs(report_dir, exist_ok=True)
        joined_tags = "_".join(tags)
        report_path = os.path.join(report_dir, f"{joined_tags}_compare_report.pdf")
        
        if len(report_path) > 250:
            report_path = os.path.join(report_dir, f"compare_report_{time.strftime('%Y%m%d-%H%M%S')}.pdf")
            
        pdf.output(report_path)
        print(f"\nSaved comparison PDF report to {report_path}")
        try:
            import subprocess
            subprocess.Popen(['xdg-open', report_path])
        except Exception as e:
            print(f"Could not launch PDF viewer: {e}")

def generate_report(tag):
    import os, json, glob
    try:
        from fpdf import FPDF
    except ImportError:
        print("fpdf2 not installed. Cannot generate report. Run: pip install fpdf2")
        return

    fleet_dir = hu.get_fleet_path()
    
    found_any = False
    for joint_name in ['arm', 'lift']:
        log_dir = os.path.join(fleet_dir, 'log', f'sweep_{joint_name}', tag)
        if not os.path.exists(log_dir):
            continue
            
        json_files = glob.glob(os.path.join(log_dir, f"{tag}_*_data.json"))
        if not json_files:
            continue
            
        found_any = True
        pdf = FPDF()
        pdf.set_auto_page_break(auto=True, margin=15)
        
        json_files.sort()
        
        for jf in json_files:
            with open(jf, 'r') as f:
                data = json.load(f)
                
            test_name = data.get('test_name', 'Unknown Test')
            
            pdf.add_page()
            pdf.set_font("helvetica", "B", 16)
            pdf.cell(0, 10, f"Joint Sweep [{joint_name}]: {test_name}", new_x="LMARGIN", new_y="NEXT", align="C")
            pdf.ln(5)
            
            pdf.set_font("helvetica", "B", 12)
            pdf.cell(0, 8, f"Parameters:", new_x="LMARGIN", new_y="NEXT")
            pdf.set_font("helvetica", "", 10)
            pdf.cell(0, 6, f"  x_start: {data.get('x_start')} m", new_x="LMARGIN", new_y="NEXT")
            pdf.cell(0, 6, f"  x_end: {data.get('x_end')} m", new_x="LMARGIN", new_y="NEXT")
            pdf.cell(0, 6, f"  v_des: {data.get('v_des')} m/s", new_x="LMARGIN", new_y="NEXT")
            pdf.cell(0, 6, f"  a_des: {data.get('a_des')} m/s^2", new_x="LMARGIN", new_y="NEXT")
            pdf.ln(5)
            
            pdf.set_font("helvetica", "B", 12)
            pdf.cell(0, 8, "Phase Statistics (Current in Amps):", new_x="LMARGIN", new_y="NEXT")
            
            pdf.set_font("helvetica", "B", 10)
            col_width = 45
            pdf.cell(col_width, 8, "Phase")
            pdf.cell(col_width, 8, "Max Curr (A)")
            pdf.cell(col_width, 8, "Min Curr (A)")
            pdf.cell(col_width, 8, "Std Curr (A)", new_x="LMARGIN", new_y="NEXT")
            
            pdf.set_font("helvetica", "", 10)
            phases = [("All Data", data.get("all", {})), 
                      ("Constant Vel", data.get("constant_vel_phase", {}))]
                      
            for p_name, p_data in phases:
                if not p_data: continue
                max_c = p_data.get('max_curr')
                min_c = p_data.get('min_curr')
                std_c = p_data.get('std_curr')
                max_s = f"{max_c:.4f}" if max_c is not None else "N/A"
                min_s = f"{min_c:.4f}" if min_c is not None else "N/A"
                std_s = f"{std_c:.4f}" if std_c is not None else "N/A"
                
                pdf.cell(col_width, 8, p_name)
                pdf.cell(col_width, 8, max_s)
                pdf.cell(col_width, 8, min_s)
                pdf.cell(col_width, 8, std_s, new_x="LMARGIN", new_y="NEXT")
                
            pdf.ln(5)
            
            img_pattern = os.path.join(log_dir, f"{tag}_{test_name}_*.png")
            images = glob.glob(img_pattern)
            phases_img = None
            reg_img = None
            for img in images:
                if "_phases_" in img:
                    phases_img = img
                else:
                    reg_img = img
            
            if phases_img or reg_img:
                pdf.set_font("helvetica", "B", 12)
                pdf.cell(0, 8, "Plots:", new_x="LMARGIN", new_y="NEXT")
                
            if reg_img:
                pdf.image(reg_img, x=20, w=160)
                pdf.ln(5)
            if phases_img:
                pdf.image(phases_img, x=20, w=160)
                
        if found_any:
            report_path = os.path.join(log_dir, f"{tag}_report.pdf")
            pdf.output(report_path)
            print(f"Saved PDF report to {report_path}")
            try:
                import subprocess
                subprocess.Popen(['xdg-open', report_path])
            except Exception as e:
                print(f"Could not launch PDF viewer: {e}")
            
    if not found_any:
        print(f"No data found for tag '{tag}' in arm or lift log directories.")

def build_model(tags):
    import os, json, glob
    import numpy as np
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from stretch4_body.core.robot_params import RobotParams
    
    fleet_dir = hu.get_fleet_path()
    params = RobotParams().get_params()[1]
    
    joint_ranges = {
        'arm': params.get('arm', {}).get('range_m', [0.0, 0.5]),
        'lift': params.get('lift', {}).get('range_m', [0.0, 1.1])
    }
    
    all_data = {t: [] for t in tags}
    found_any = False
    
    for t in tags:
        for joint_name in ['arm', 'lift']:
            log_dir = os.path.join(fleet_dir, 'log', f'sweep_{joint_name}', t)
            if not os.path.exists(log_dir):
                continue
            json_files = glob.glob(os.path.join(log_dir, f"{t}_*_data.json"))
            for jf in json_files:
                with open(jf, 'r') as f:
                    all_data[t].append((joint_name, json.load(f)))
                    found_any = True
                    
    if not found_any:
        print("No data found for tags.")
        return

    found_joints = set()
    for t in tags:
        for jn, d in all_data[t]:
            found_joints.add(jn)
            
    for joint_name in found_joints:
        tag_data_extend = {}
        tag_data_retract = {}
        
        has_joint_data = False
        root_a_des = 1.0 # default if absent
        
        for t in tags:
            tag_data_extend[t] = {'pos': [], 'curr': []}
            tag_data_retract[t] = {'pos': [], 'curr': []}
            
            for jn, d in all_data[t]:
                if jn == joint_name and d['test_name'] == 'generate_model_data':
                    has_joint_data = True
                    root_a_des = d.get('a_des', root_a_des)
                    # Use 'all' data instead of 'constant_vel_phase'
                    cv = d.get('all', {})
                    if not cv: continue
                    pos = np.array(cv.get('pos', []))
                    curr = np.array(cv.get('current', []))
                    dirs = np.array(cv.get('dir', []))
                    
                    mask_ext = (dirs == True)
                    tag_data_extend[t]['pos'].extend(pos[mask_ext])
                    tag_data_extend[t]['curr'].extend(curr[mask_ext])
                    
                    mask_ret = (dirs == False)
                    tag_data_retract[t]['pos'].extend(pos[mask_ret])
                    tag_data_retract[t]['curr'].extend(curr[mask_ret])
                    
        if not has_joint_data:
            print(f"No 'generate_model_data' data found for {joint_name} model generation.")
            continue
            
        # Aggregate across all tags for model fitting
        pos_extend_all = np.concatenate([tag_data_extend[t]['pos'] for t in tags if tag_data_extend[t]['pos']]) if any(tag_data_extend[t]['pos'] for t in tags) else np.array([])
        curr_extend_all = np.concatenate([tag_data_extend[t]['curr'] for t in tags if tag_data_extend[t]['curr']]) if any(tag_data_extend[t]['curr'] for t in tags) else np.array([])
        
        pos_retract_all = np.concatenate([tag_data_retract[t]['pos'] for t in tags if tag_data_retract[t]['pos']]) if any(tag_data_retract[t]['pos'] for t in tags) else np.array([])
        curr_retract_all = np.concatenate([tag_data_retract[t]['curr'] for t in tags if tag_data_retract[t]['curr']]) if any(tag_data_retract[t]['curr'] for t in tags) else np.array([])
        
        range_m = joint_ranges[joint_name]
        
        def fit_envelope_max(x, y, deg=3, num_bins=20):
            if len(x) == 0: return None, None
            bins = np.linspace(np.min(x), np.max(x), num_bins+1)
            b_cent = []
            max_c = []
            for i in range(num_bins):
                m = (x >= bins[i]) & (x <= bins[i+1])
                if np.any(m):
                    b_cent.append((bins[i]+bins[i+1])/2.0)
                    max_c.append(np.max(y[m]))
            if len(b_cent) < deg + 1:
                return None, None
            poly = np.polyfit(b_cent, max_c, deg)
            model_fn = np.poly1d(poly)
            residual = np.max(y - model_fn(x))
            poly[-1] += max(0, residual)
            return poly, np.poly1d(poly)

        def fit_envelope_min(x, y, deg=3, num_bins=20):
            if len(x) == 0: return None, None
            bins = np.linspace(np.min(x), np.max(x), num_bins+1)
            b_cent = []
            min_c = []
            for i in range(num_bins):
                m = (x >= bins[i]) & (x <= bins[i+1])
                if np.any(m):
                    b_cent.append((bins[i]+bins[i+1])/2.0)
                    min_c.append(np.min(y[m]))
            if len(b_cent) < deg + 1:
                return None, None
            poly = np.polyfit(b_cent, min_c, deg)
            model_fn = np.poly1d(poly)
            # Find the MOST negative residual (meaning data went lower than the curve)
            residual = np.min(y - model_fn(x))
            # Shift the intercept DOWN by that magnitude to effectively lower the floor
            poly[-1] += min(0, residual)
            return poly, np.poly1d(poly)

        poly_ext, model_ext = fit_envelope_max(pos_extend_all, curr_extend_all)
        poly_ret, model_ret = fit_envelope_min(pos_retract_all, curr_retract_all)
        
        model_data = {
            'joint_name': joint_name,
            'range_m': range_m,
            'tags_used': tags,
            'extending_poly': poly_ext.tolist() if poly_ext is not None else None,
            'retracting_poly': poly_ret.tolist() if poly_ret is not None else None
        }
        
        joined_tags = "_".join(tags)
        model_dir = os.path.join(fleet_dir, 'log', f'sweep_{joint_name}', 'model', joined_tags)
        os.makedirs(model_dir, exist_ok=True)
        model_path = os.path.join(model_dir, f"{joint_name}_max_current_model.json")
        with open(model_path, 'w') as f:
            json.dump(model_data, f, indent=2)
        print(f"Saved model to {model_path}")
        
        fig, axs = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle(f"Predictive Current Model: {joint_name} (Extending vs Retracting)")
        
        axs[0].set_title("Extending (dir=True)")
        for t in tags:
            p = tag_data_extend[t]['pos']
            c = tag_data_extend[t]['curr']
            if len(p) > 0:
                axs[0].plot(p, c, '.', markersize=2, label=t)
        if model_ext is not None:
            x_eval = np.linspace(range_m[0], range_m[1], 100)
            axs[0].plot(x_eval, model_ext(x_eval), 'r-', linewidth=2, label="Max Expected Current")
            
        axs[1].set_title("Retracting (dir=False)")
        for t in tags:
            p = tag_data_retract[t]['pos']
            c = tag_data_retract[t]['curr']
            if len(p) > 0:
                axs[1].plot(p, c, '.', markersize=2, label=t)
        if model_ret is not None:
            x_eval = np.linspace(range_m[0], range_m[1], 100)
            axs[1].plot(x_eval, model_ret(x_eval), 'r-', linewidth=2, label="Min Expected Current")
            
        # Add firmware threshold representations
        motor_name = f'hello-motor-{joint_name}'
        m_params = params.get(motor_name, {})
        gains = m_params.get('gains', {})
        guarded = m_params.get('guarded_contact', {})
        
        c_acc_pos = gains.get('coeff_acc_pos', 0.0)
        c_int_pos = gains.get('coeff_intercept_pos', 0.0)
        c_acc_neg = gains.get('coeff_acc_neg', 0.0)
        c_int_neg = gains.get('coeff_intercept_neg', 0.0)
        
        sens_def = guarded.get('sensitivity_default', {})
        sens_hi = guarded.get('sensitivity_high', {})
        sens_lo = guarded.get('sensitivity_low', {})
        
        def eff_to_amps(e):
            mA_per_tick = (3300.0 / 4096.0) / 0.424
            return e * mA_per_tick / 1000.0
            
        x_horiz = [range_m[0], range_m[1]]
        
        # Extending threshold
        for sens_name, sens_dict, color, style in [
            ('Default', sens_def, 'g', '--'),
            ('High', sens_hi, 'm', ':'),
            ('Low', sens_lo, 'c', '-.')
        ]:
            c_sens_p = sens_dict.get('coeff_sensitivity_pos', 0.0)
            eff_pos = c_acc_pos * root_a_des + c_int_pos + c_sens_p * 1500.0
            amp_pos = eff_to_amps(eff_pos)
            axs[0].plot(x_horiz, [amp_pos, amp_pos], color=color, linestyle=style, linewidth=2, label=f"FW Thresh {sens_name}")
            
            c_sens_n = sens_dict.get('coeff_sensitivity_neg', 0.0)
            eff_neg = c_acc_neg * root_a_des + c_int_neg - c_sens_n * 1500.0
            amp_neg = eff_to_amps(eff_neg)
            axs[1].plot(x_horiz, [amp_neg, amp_neg], color=color, linestyle=style, linewidth=2, label=f"FW Thresh {sens_name}")
            
        for ax in axs:
            ax.set_xlabel("Position (m)")
            ax.set_ylabel("Current (A)")
            ax.grid(True)
            ax.legend()
            
        plt.tight_layout()
        img_path = os.path.join(model_dir, f"{joint_name}_max_current_model.png")
        fig.savefig(img_path)
        print(f"Saved model plot to {img_path}")
        plt.show(block=True)
        
def test_model(joint_name, model_tag):
    import os, json
    import numpy as np
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from stretch4_body.core.robot_params import RobotParams
    
    fleet_dir = hu.get_fleet_path() 
    model_dir = os.path.join(fleet_dir, 'log', f'sweep_{joint_name}', 'model', model_tag)
    model_path = os.path.join(model_dir, f"{joint_name}_max_current_model.json")
    
    if not os.path.exists(model_path):
        print(f"Model file not found at {model_path}. Please build it first using --model_tags")
        return
        
    with open(model_path, 'r') as f:
        model_data = json.load(f)
        
    poly_ext_coeff = model_data.get('extending_poly')
    poly_ret_coeff = model_data.get('retracting_poly')
    range_m = model_data.get('range_m', [0.0, 0.5])
    
    if poly_ext_coeff is None or poly_ret_coeff is None:
        print("Model data is missing polynomial coefficients.")
        return
        
    model_ext = np.poly1d(poly_ext_coeff)
    model_ret = np.poly1d(poly_ret_coeff)
    
    margin = 0.75 # Amps
    
    fig, axs = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle(f"Testing Model: {model_tag}  (+/- {margin}A margin)")
    
    x_eval = np.linspace(range_m[0], range_m[1], 100)
    
    axs[0].set_title("Extending (dir=True)")
    axs[0].plot(x_eval, model_ext(x_eval), 'r-', linewidth=2, label="Expected Max Current")
    axs[0].plot(x_eval, model_ext(x_eval) + margin, 'r--', linewidth=2, label="Safety Threshold")
    
    axs[1].set_title("Retracting (dir=False)")
    axs[1].plot(x_eval, model_ret(x_eval), 'r-', linewidth=2, label="Expected Min Current")
    axs[1].plot(x_eval, model_ret(x_eval) - margin, 'r--', linewidth=2, label="Safety Threshold")
    
    for ax in axs:
        ax.set_xlabel("Position (m)")
        ax.set_ylabel("Current (A)")
        ax.grid(True)
        ax.legend()
        ax.set_ylim(-3.0, 3.0) 
        
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.1)
    
    input("Press Enter to begin test execution... (Will trip MODE_SAFETY if limits exceeded)")
    
    if joint_name == 'arm':
        import stretch4_body.subsystem.arm as aa
        device = aa.Arm()
        if not device.startup():
            print("Failed to start arm")
            return
            
        device.motor.disable_sync_mode()
        device.motor.disable_guarded_mode()
        device.push_command()
        device.pull_status()
        
        # Configure test constraints identical to measure_full_range
        x_start = 0.003
        x_end = 0.55
        v_des = 0.4
        a_des = 0.4
        
    elif joint_name == 'lift':
        import stretch4_body.subsystem.lift as lift
        device = lift.Lift()
        if not device.startup():
            print("Failed to start lift")
            return
            
        device.motor.disable_sync_mode()
        device.motor.disable_guarded_mode()
        device.push_command()
        device.pull_status()
        
        x_start = 0.05 * device.params['range_m'][1]
        x_end = 0.9 * device.params['range_m'][1]
        v_des = 0.05
        a_des = 0.1
    else:
        print(f"Unknown joint name {joint_name}")
        return
        
    pad_accel = 0.1
    ts_start = time.time()
    
    # Init extending sweep
    pos_ext_rt = []
    curr_ext_rt = []
    pos_ret_rt = []
    curr_ret_rt = []
    safety_tripped = False
    
    for cycle in range(10):
        if safety_tripped:
            break
            
        print(f"--- Cycle {cycle+1}/10 ---")
        
        print(f"Executing Extending (-> {x_end}m)...")
        device.move_to(x_end, v_des, a_des)
        device.push_command()
        time.sleep(pad_accel)
        
        # We will poll while target not met
        while device.status['pos'] < (x_end - 0.02):
            device.pull_status()
            p = device.status['pos']
            c = device.motor.status['current']
            
            pos_ext_rt.append(p)
            curr_ext_rt.append(c)
            
            limit = model_ext(p) + margin
            if c > limit:
                print(f"!!! EXTENDING SAFETY TRIPPED !!! @ Pos: {p:.3f}m | Curr: {c:.2f}A (Limit max: {limit:.2f}A)")
                device.motor.set_command(mode=device.motor.MODE_SAFETY)
                device.push_command()
                safety_tripped = True
                break
                
            # Draw live marker
            axs[0].plot(p, c, 'b.', markersize=4)
            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.01)
            
        if not safety_tripped:
            print("Extending target met without boundary violations. Pausing.")
            time.sleep(1.0)
            
            # Init retracting sweep
            print(f"Executing Retracting (-> {x_start}m)...")
            device.move_to(x_start, v_des, a_des)
            device.push_command()
            time.sleep(pad_accel)
            
            while device.status['pos'] > (x_start + 0.02):
                device.pull_status()
                p = device.status['pos']
                c = device.motor.status['current']
                
                pos_ret_rt.append(p)
                curr_ret_rt.append(c)
                
                limit = model_ret(p) - margin
                if c < limit:
                    print(f"!!! RETRACTING SAFETY TRIPPED !!! @ Pos: {p:.3f}m | Curr: {c:.2f}A (Limit min: {limit:.2f}A)")
                    device.motor.set_command(mode=device.motor.MODE_SAFETY)
                    device.push_command()
                    safety_tripped = True
                    break
                    
                axs[1].plot(p, c, 'b.', markersize=4)
                fig.canvas.draw()
                fig.canvas.flush_events()
                time.sleep(0.01)
                
            if not safety_tripped:
                time.sleep(1.0) # Pause before next cycle
            
    device.stop()
    print("Test Execution Block Complete. Close chart to exit.")
    plt.show(block=True)

# #################################################3
def measure_arm_near_retraction():
    test_name='measure_arm_near_retraction'
    x_start=0.003
    x_end=0.03
    v_des=0.02
    a_des=1.0
    n_sweep=args.nsweep
    pad_accel_t=0.1
    ts,dir,sweep,current, pos, vel, tmp_path=collect_data(test_name, arm, x_start, x_end, v_des, a_des, n_sweep)
    processed_data=process_data(test_name, 'arm', x_start, x_end, v_des, a_des, ts,dir,sweep,current, pos, vel,pad_accel_t)
    display_images=display_data(processed_data)
    if display_images is None:
        display_images = []
    display_images.insert(0, tmp_path)
    return processed_data, display_images

def measure_arm_mid_range_small():
    test_name='measure_arm_mid_range_small'
    x_start=0.1
    x_end=0.2
    v_des=0.02
    a_des=1.0
    n_sweep=args.nsweep
    pad_accel_t=0.15
    ts,dir,sweep,current, pos, vel, tmp_path=collect_data(test_name, arm, x_start, x_end, v_des, a_des, n_sweep)
    processed_data=process_data(test_name, 'arm', x_start, x_end, v_des, a_des, ts,dir,sweep,current, pos, vel,pad_accel_t)
    display_images=display_data(processed_data)
    if display_images is None:
        display_images = []
    display_images.insert(0, tmp_path)
    return processed_data, display_images

def measure_arm_mid_range_large():
    test_name='measure_arm_mid_range_large'
    x_start=0.1
    x_end=0.4
    v_des=0.02
    a_des=1.0
    n_sweep=args.nsweep
    pad_accel_t=0.15
    ts,dir,sweep,current, pos, vel, tmp_path=collect_data(test_name, arm, x_start, x_end, v_des, a_des, n_sweep)
    processed_data=process_data(test_name, 'arm', x_start, x_end, v_des, a_des, ts,dir,sweep,current, pos, vel,pad_accel_t)
    display_images=display_data(processed_data)
    if display_images is None:
        display_images = []
    display_images.insert(0, tmp_path)
    return processed_data, display_images

def measure_arm_end_range_small():
    test_name='measure_arm_end_range_small'
    x_start=0.45
    x_end=0.55
    v_des=0.02
    a_des=1.0
    n_sweep=args.nsweep
    pad_accel_t=0.1
    ts,dir,sweep,current, pos, vel, tmp_path=collect_data(test_name, arm, x_start, x_end, v_des, a_des, n_sweep)
    processed_data=process_data(test_name, 'arm', x_start, x_end, v_des, a_des, ts,dir,sweep,current, pos, vel,pad_accel_t)
    display_images=display_data(processed_data)
    if display_images is None:
        display_images = []
    display_images.insert(0, tmp_path)
    return processed_data, display_images

def measure_full_range():
    test_name='measure_full_range'
    x_start=0.003
    x_end=0.55
    v_des=0.02
    a_des=1.0
    n_sweep=args.nsweep
    pad_accel_t=0.1
    ts,dir,sweep,current, pos, vel, tmp_path=collect_data(test_name, arm, x_start, x_end, v_des, a_des, n_sweep)
    processed_data=process_data(test_name, 'arm', x_start, x_end, v_des, a_des, ts,dir,sweep,current, pos, vel,pad_accel_t)
    display_images=display_data(processed_data)
    if display_images is None:
        display_images = []
    display_images.insert(0, tmp_path)
    return processed_data, display_images

def measure_lift_full_range():
    test_name='measure_full_range'
    x_start = 0.005
    x_end = lift.params['range_m'][1]-.005
    v_des = 0.1
    a_des = 0.5
    n_sweep=args.nsweep
    pad_accel_t=0.2
    ts,dir,sweep,current, pos, vel, tmp_path=collect_data(test_name, lift, x_start, x_end, v_des, a_des, n_sweep)
    processed_data=process_data(test_name, 'lift', x_start, x_end, v_des, a_des, ts,dir,sweep,current, pos, vel,pad_accel_t)
    display_images=display_data(processed_data)
    if display_images is None:
        display_images = []
    display_images.insert(0, tmp_path)
    return processed_data, display_images

def generate_model_data_arm():
    test_name='generate_model_data'
    x_start=0.003
    x_end=0.55
    v_des=0.4
    a_des=0.6
    n_sweep=args.nsweep
    pad_accel_t=0.1
    ts,dir,sweep,current, pos, vel, tmp_path=collect_data(test_name, arm, x_start, x_end, v_des, a_des, n_sweep)
    processed_data=process_data(test_name, 'arm', x_start, x_end, v_des, a_des, ts,dir,sweep,current, pos, vel,pad_accel_t)
    display_images=display_data(processed_data)
    if display_images is None:
        display_images = []
    display_images.insert(0, tmp_path)
    return processed_data, display_images

def generate_model_data_lift():
    test_name='generate_model_data'
    x_start = 0.05 * r.params['range_m'][1]
    x_end = 0.9 * r.params['range_m'][1]
    v_des = 0.05
    a_des = 0.1
    n_sweep=args.nsweep
    pad_accel_t=0.1
    ts,dir,sweep,current, pos, vel, tmp_path=collect_data(test_name, r, x_start, x_end, v_des, a_des, n_sweep)
    processed_data=process_data(test_name, 'lift', x_start, x_end, v_des, a_des, ts,dir,sweep,current, pos, vel,pad_accel_t)
    display_images=display_data(processed_data)
    if display_images is None:
        display_images = []
    display_images.insert(0, tmp_path)
    return processed_data, display_images
# #################################################3

if args.compare:
    compare_data(args.compare)
elif args.report:
    generate_report(args.tag)
elif args.model_tags:
    build_model(args.model_tags)
elif args.test_model:
    if args.arm:
        test_model('arm', args.test_model)
    elif args.lift:
        test_model('lift', args.test_model)
    else:
        print("Please specify --arm or --lift when using --test_model")
elif args.generate_model_data:
    if args.arm:
        joint_name = 'arm'
        device = arm
        results = []
        res = generate_model_data_arm()
        if res:
            results.append(res)
        device.stop()
        device.push_command()
        if results:
            ans = input("Save data (y/n)? ")
            if ans.lower() == 'y':
                for pd, imgs in results:
                    save_data(pd, imgs, joint_name)
                if args.tag:
                    generate_report(args.tag)
    elif args.lift:
        joint_name = 'lift'
        results = []
        res = generate_model_data_lift()
        if res:
            results.append(res)
        device.stop()
        device.push_command()
        if results:
            ans = input("Save data (y/n)? ")
            if ans.lower() == 'y':
                for pd, imgs in results:
                    save_data(pd, imgs, joint_name)
                if args.tag:
                    generate_report(args.tag)
    else:
        print("Please specify --arm or --lift when using --generate_model_data")
elif args.arm:
    joint_name = 'arm'
    results = []
    
    tests = [
        ('measure_arm_near_retraction', measure_arm_near_retraction),
        ('measure_arm_mid_range_small', measure_arm_mid_range_small),
        ('measure_arm_mid_range_large', measure_arm_mid_range_large),
        ('measure_arm_end_range_small', measure_arm_end_range_small),
        ('measure_full_range', measure_full_range),
        ('generate_model_data', generate_model_data_arm)
    ]
    
    tests_to_run = tests
    
    if args.select:
        print("\nAvailable tests:")
        for i, (name, _) in enumerate(tests):
            print(f"{i}: {name}")
        print("a: all")
        
        val = input("\nSelect test to run: ")
        if val == 'a':
            tests_to_run = tests
        else:
            try:
                tests_to_run = [tests[int(val)]]
            except (ValueError, IndexError):
                print("Invalid selection. Exiting.")
                tests_to_run = []
                
    for name, func in tests_to_run:
        res = func()
        if res:
            results.append(res)
    
    arm.stop()
    arm.push_command()
    
    if results:
        ans = input("Save data (y/n)? ")
        if ans.lower() == 'y':
            for pd, imgs in results:
                save_data(pd, imgs, joint_name)
            if args.tag:
                generate_report(args.tag)

elif args.lift:
    joint_name = 'lift'
    results = []
    
    tests = [
        ('measure_lift_full_range', measure_lift_full_range)
    ]
    
    tests_to_run = tests
    
    if args.select:
        print("\nAvailable tests:")
        for i, (name, _) in enumerate(tests):
            print(f"{i}: {name}")
        print("a: all")
        
        val = input("\nSelect test to run: ")
        if val == 'a':
            tests_to_run = tests
        else:
            try:
                tests_to_run = [tests[int(val)]]
            except (ValueError, IndexError):
                print("Invalid selection. Exiting.")
                tests_to_run = []
                
    for name, func in tests_to_run:
        res = func()
        if res:
            results.append(res)
            
    lift.stop()
    lift.push_command()
    
    if results:
        ans = input("Save data (y/n)? ")
        if ans.lower() == 'y':
            for pd, imgs in results:
                save_data(pd, imgs, joint_name)
            if args.tag:
                generate_report(args.tag)