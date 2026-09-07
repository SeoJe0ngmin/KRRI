import argparse
import asyncio
import math
import os
import sys
import time
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from src.models.control.control_forklift_v2 import MOVEMENT_TEMPLATES, CONTROL_TEMPLATES, CAN_MOVEMENT_ID, CAN_CONTROL_ID
MAX_SEC = 5.0

def _measure_now(pipe, tag_id, n=30, max_frames=150):
    from src.models.detection.detection_pose import measure
    buf = []
    for i, (idx, ts, frame) in enumerate(pipe.frames):
        res = pipe.process(frame, index=idx, timestamp=ts)
        if tag_id in res.docking:
            res.image = None
            buf.append(res)
        if len(buf) >= n or i >= max_frames:
            break
    return measure(buf, tag_id=tag_id, n=n) if buf else None

def _fmt_m(m):
    if m is None:
        return '(태그 안 보임)'
    return 'lateral %+.3fm  forward %.3fm  heading %+.2f도' % (m['lateral'], m['forward'], m['heading_deg'])

def _bytes(seq):
    return ' '.join(('%3d' % b for b in seq))

async def main_async(args):
    if args.movement not in MOVEMENT_TEMPLATES:
        raise SystemExit('모르는 명령: %s  (가능: %s)' % (args.movement, ', '.join(MOVEMENT_TEMPLATES)))
    sec = max(0.05, min(MAX_SEC, args.sec))
    print('명령       : %s  %.2f초' % (args.movement, sec))
    print('0x%03X 주행 : %s   (byte0..7, 중립 127)' % (CAN_MOVEMENT_ID, _bytes(MOVEMENT_TEMPLATES[args.movement])))
    print('0x%03X 모드 : %s   (byte4 는 카운터)' % (CAN_CONTROL_ID, ' '.join(('%02X' % b for b in CONTROL_TEMPLATES['driving_mode']))))
    pipe = None
    m0 = None
    yaw = None
    from config.detection import TAG_ID, TAG_SIZE_M
    tag_id = TAG_ID
    if args.camera:
        import pyrealsense2 as rs
        from src.models import TagPipeline
        for attempt in (1, 2, 3):
            try:
                pipe = TagPipeline.from_realsense(TAG_SIZE_M, label='pulse')
                m0 = _measure_now(pipe, tag_id)
                break
            except RuntimeError as exc:
                print('!! 카메라 첫 프레임 실패 (%s) — %s' % (str(exc)[:45], '장치 리셋 후 다시 연다' if attempt < 3 else '포기'))
                if pipe is not None:
                    pipe.close()
                    pipe = None
                if attempt == 3:
                    raise SystemExit('카메라가 응답하지 않는다. UTM USB 메뉴에서 RealSense 를 뺐다 다시 넣고 재시도')
                try:
                    devs = rs.context().query_devices()
                    if len(devs):
                        devs[0].hardware_reset()
                except Exception:
                    pass
                await asyncio.sleep(5.0)
        print('카메라 전 : %s' % _fmt_m(m0))
    elif not args.no_imu:
        try:
            from src.utils.imu_yaw import GyroYaw
            yaw = GyroYaw().start()
            print('자이로 열림. 2초 정지 보정...')
            rep = await asyncio.to_thread(yaw.calibrate)
            print('  축 %s / 잡음 %.3f도/s%s' % (rep['axis_src'], rep['noise_dps'], '  !! 보정 중 움직임' if rep['moving'] else ''))
        except Exception as exc:
            print('!! 자이로를 못 열었다 (%s) — 각도 없이 진행' % exc)
            yaw = None
    if args.dry_run:
        print('DRY-RUN — CAN 으로 아무것도 안 보낸다')
        if pipe is not None:
            pipe.close()
        if yaw is not None:
            yaw.close()
        return
    from src.models.control.control_forklift_v2 import DirectFrameForkliftController
    ctrl = DirectFrameForkliftController()
    if not ctrl.connect_can():
        if yaw is not None:
            yaw.close()
        raise SystemExit('CAN 연결 실패')
    ctrl.is_running = True
    tasks = [asyncio.create_task(ctrl.control_tx_loop()), asyncio.create_task(ctrl.movement_tx_loop()), asyncio.create_task(ctrl.heartbeat_loop())]
    await asyncio.sleep(0.5)
    samples = []
    try:
        if not args.yes:
            await asyncio.to_thread(input, '  준비되면 엔터 — %s 를 %.2f초 보낸다 (Ctrl+C 취소) > ' % (args.movement, sec))
        if yaw is not None:
            yaw.zero()
        t0 = time.perf_counter()
        ctrl.current_movement = args.movement
        phase = 'send'
        while True:
            t = time.perf_counter() - t0
            if phase == 'send' and t >= sec:
                ctrl.current_movement = 'stop'
                phase = 'settle'
            if t >= sec + 1.5:
                break
            a = yaw.angle_deg if yaw is not None else float('nan')
            r = yaw.rate_dps if yaw is not None else float('nan')
            cf = cl = ch = float('nan')
            if pipe is not None:
                try:
                    idx, ts, frame = next(pipe.frames)
                    d = pipe.process(frame, index=idx, timestamp=ts).docking.get(tag_id)
                    if d is not None:
                        cf, cl, ch = (d['forward'], d['lateral'], d['heading_deg'])
                except StopIteration:
                    pass
            samples.append((t, phase, a, r, cf, cl, ch))
            await asyncio.sleep(0.05)
    except KeyboardInterrupt:
        print('\n중단 — 정지 프레임으로 넘어간다')
    finally:
        ctrl.current_movement = 'stop'
        await asyncio.sleep(0.3)
        ctrl.is_running = False
        for t in tasks:
            t.cancel()
        ctrl.disconnect_can()
    m1 = None
    if pipe is not None:
        await asyncio.sleep(1.0)
        m1 = _measure_now(pipe, tag_id)
        print('카메라 후 : %s' % _fmt_m(m1))
        pipe.close()
    if yaw is not None:
        yaw.close()
    if not samples:
        return
    summary = {'movement': args.movement, 'sec': sec, 'ts': time.time(), 'template': MOVEMENT_TEMPLATES[args.movement], 'camera_before': m0, 'camera_after': m1, 'samples': [{'t': round(t, 3), 'phase': ph, 'yaw_deg': a, 'rate_dps': r, 'cam_forward': cf, 'cam_lateral': cl, 'cam_heading_deg': ch} for t, ph, a, r, cf, cl, ch in samples]}
    have_cam = any((not math.isnan(x[4]) for x in samples))
    if have_cam:
        print('\n  t[s]   구간    yaw[도]  rate[도/s]   cam fwd[m]  cam lat[m]  cam head[도]')
        for t, ph, a, r, cf, cl, ch in samples[::2]:
            print('  %5.2f  %-6s %8.2f  %8.2f   %9.3f  %9.3f  %10.2f' % (t, ph, a, r, cf, cl, ch))
    else:
        print('\n  t[s]   구간    yaw[도]  rate[도/s]')
        for t, ph, a, r, *_ in samples[::2]:
            print('  %5.2f  %-6s %8.2f  %8.2f' % (t, ph, a, r))
    send = [x for x in samples if x[1] == 'send']
    if yaw is not None:
        end_yaw = send[-1][2] if send else float('nan')
        final_yaw = samples[-1][2]
        peak = max((abs(x[3]) for x in samples if not math.isnan(x[3])), default=float('nan'))
        print('\n결과(자이로): 보내는 동안 yaw %+.2f도, 정지 후 최종 %+.2f도, 최대 각속도 %.1f도/s' % (end_yaw, final_yaw, peak))
        summary.update({'yaw_at_stop_deg': end_yaw, 'yaw_final_deg': final_yaw, 'yaw_coast_deg': final_yaw - end_yaw, 'peak_rate_dps': peak})
    if have_cam and args.movement.startswith('rotate'):
        ch = [(t, ph, h) for t, ph, a, r, cf, cl, h in samples if not math.isnan(h)]
        if len(ch) >= 6:
            h0 = ch[0][2]
            onset = next((t for t, ph, h in ch if abs(h - h0) > 1.0), None)
            send_h = [(t, h) for t, ph, h in ch if ph == 'send' and onset is not None and (t >= onset)]
            rate = None
            if len(send_h) >= 4:
                ts_ = [t for t, _ in send_h]
                hs_ = [h for _, h in send_h]
                tm, hm = (sum(ts_) / len(ts_), sum(hs_) / len(hs_))
                den = sum(((t - tm) ** 2 for t in ts_))
                rate = sum(((t - tm) * (h - hm) for t, h in zip(ts_, hs_))) / den if den else None
            send_end_h = ch[max((i for i, (t, ph, h) in enumerate(ch) if ph == 'send'))][2] if any((x[1] == 'send' for x in ch)) else None
            settle_h = [h for t, ph, h in ch if ph == 'settle']
            coast = settle_h[-1] - send_end_h if settle_h and send_end_h is not None else None
            summary.update({'onset_sec': onset, 'rate_dps': rate, 'coast_deg': coast})
            print('카메라 시간축(회전): 돌기 시작 %s  각속도 %s  정지 명령 뒤 더 돈 각(관성) %s' % ('%.2fs 뒤' % onset if onset is not None else '(1도도 안 돎)', '%.1f도/s' % rate if rate is not None else '(구간 부족)', '%+.1f도' % coast if coast is not None else '?'))
    if have_cam and args.movement in ('forward', 'backward'):
        cam = [(t, ph, cf) for t, ph, a, r, cf, cl, ch in samples if not math.isnan(cf)]
        if len(cam) >= 6:
            f0 = cam[0][2]
            onset = next((t for t, ph, cf in cam if abs(cf - f0) > 0.03), None)
            send_c = [(t, cf) for t, ph, cf in cam if ph == 'send' and onset is not None and (t >= onset + 0.5)]
            v = None
            if len(send_c) >= 4:
                ts_ = [t for t, _ in send_c]
                fs_ = [f for _, f in send_c]
                tm, fm = (sum(ts_) / len(ts_), sum(fs_) / len(fs_))
                den = sum(((t - tm) ** 2 for t in ts_))
                v = -sum(((t - tm) * (f - fm) for t, f in zip(ts_, fs_))) / den if den else None
            at_stop = next((cf for t, ph, cf in cam if ph == 'settle'), None)
            final_f = cam[-1][2]
            summary.update({'onset_sec': onset, 'steady_mps': v, 'coast_m': at_stop - final_f if at_stop is not None else None})
            print('카메라 시간축: 움직이기 시작 %s  정속 %s  정지 명령 뒤 더 간 거리(관성) %s' % ('%.2fs 뒤' % onset if onset is not None else '(3cm 도 안 움직임)', '%.3fm/s' % v if v is not None else '(정속 구간 부족 — 더 길게)', '%.3fm' % (at_stop - final_f) if at_stop is not None else '?'))
    if m0 is not None and m1 is not None:
        dl, df, dh = (m1['lateral'] - m0['lateral'], m1['forward'] - m0['forward'], m1['heading_deg'] - m0['heading_deg'])
        print('카메라 변화: lateral %+.3fm  forward %+.3fm  heading %+.2f도' % (dl, df, dh))
        summary.update({'cam_dlateral_m': dl, 'cam_dforward_m': df, 'cam_dheading_deg': dh})
        if args.movement.startswith('rotate') and abs(dh) >= 1.0:
            summary['pivot_arm_m'] = abs(dl / math.sin(math.radians(dh)))
            print('      회전 팔 길이 A = Δlateral/sin(Δheading) = %.2fm  (config CAM_TO_PIVOT_M 후보)' % summary['pivot_arm_m'])
        elif args.movement in ('forward', 'backward'):
            moved = math.hypot(dl, df)
            summary.update({'moved_m': moved, 'mean_mps': moved / sec})
            print('      이동 %.3fm / %.2fs 명령 -> 평균 %.3fm/s (지연 포함)' % (moved, sec, moved / sec))
    if yaw is not None and abs(final_yaw) >= 0.5:
        print('      부호: %s  (rotate_ccw 는 + 가 정상, 반대면 템플릿 좌/우를 바꿔라)' % ('+' if final_yaw > 0 else '-'))
    elif yaw is not None:
        print('      각도 변화 없음 — 차체가 안 돌았다. 눈으로 무엇이 움직였는지 확인할 것')
    import json
    from datetime import datetime
    out_dir = os.path.join(ROOT, 'work_dirs', 'pulse_log')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, '%s_%s_%.1fs.json' % (datetime.now().strftime('%Y%m%d_%H%M%S'), args.movement, sec))
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1, default=lambda o: None if o != o else str(o))
    print('저장: %s' % out)

def main():
    ap = argparse.ArgumentParser(description='CAN 명령 펄스 시험 (짧게 보내고 IMU 로 잰다)')
    ap.add_argument('movement', help='MOVEMENT_TEMPLATES 이름 (rotate_ccw, rotate_cw, forward, ...)')
    ap.add_argument('sec', nargs='?', type=float, default=0.5, help='보내는 시간 [s], 최대 %.1f' % MAX_SEC)
    ap.add_argument('--no-imu', action='store_true')
    ap.add_argument('--dry-run', action='store_true', help='CAN 없이 바이트만 찍는다')
    ap.add_argument('--yes', action='store_true', help='엔터 확인 없이 바로 보낸다')
    ap.add_argument('--camera', action='store_true', help='펄스 전후를 카메라 30프레임 중앙값으로 잰다 (태그가 보여야 한다)')
    asyncio.run(main_async(ap.parse_args()))
if __name__ == '__main__':
    main()
