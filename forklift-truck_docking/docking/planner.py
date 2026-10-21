"""Bounded six-step beam search. Recomputed from each measured stopped pose."""
import math
import time
import numpy as np
from .geometry import moved,remaining,arrival_y,path_clear,tag_margin,wrap

class Responses:
    """Per-run time/brake estimates only. No method writes sensor pose."""
    def __init__(self):
        self.data={}
    def get(self,command):
        turn='rotate' in command
        seed=math.radians(8.47 if 'fine' in command else 12.01) if turn else (.15 if command=='forward_slow' else .28973)
        # startup = command-to-motion delay seed: KWU 1.082 s (rotation) / 1.498 s (drive); 2026-10-01 learned 1.20 s drive.
        return self.data.get(command,dict(speed=seed,coast=.168 if 'fine' in command else .283 if turn else .4,
                                          startup=1.082 if turn else 1.498,n=0))
    def observe(self,command,progress,active_s,residual,dead_s=None):
        if active_s<=.2 or progress<=0 or not all(math.isfinite(x) for x in (progress,active_s,residual)):return
        old=self.get(command)
        # Speed over the time the vehicle was actually moving. Dividing by the whole command time
        # folded the ~1.2 s start delay into the speed and inflated the learned coast (review 2026-10-18).
        moving_s=active_s-dead_s if dead_s is not None and 0<=dead_s<active_s-.1 else active_s
        speed=progress/moving_s
        # Learned estimates are bounded for numerical health, not certified.
        if speed>1 or speed<.002:return
        # 2026-10-20 KW run 174331: short legs (0.17-0.4 m) never reach cruise, so progress/moving_s under-reads the speed
        # (0.24 -> 0.10 m/s over the run) and residual/speed over-reads the coast (0.5 -> 1.36 s); the braking reserve then
        # stopped 0.76 m legs after 0.17 m. Learn only from legs that cruised; keep the coast time within [0, 0.8] s.
        if 'rotate' not in command and (progress<.4 or moving_s<1.):return
        coast=min(.8,max(0.,residual/speed))
        startup=old['startup'] if dead_s is None else .5*old['startup']+.5*min(5.,max(0.,dead_s))
        self.data[command]=dict(speed=.5*old['speed']+.5*speed,coast=.5*old['coast']+.5*coast,startup=startup,n=old['n']+1)
    def seconds(self,turn,distance,fine=False):
        rot=('rotate_left_' if turn>0 else 'rotate_right_')+('fine' if fine else 'slow')
        drive='backward' if distance<0 else 'forward'   # 97 (forward_slow) unmeasured -> never planned (2026-10-19)
        # Add nominal start/settle/re-observation costs; these are estimates.
        return (abs(turn)/self.get(rot)['speed']+3 if turn else 0)+(abs(distance)/self.get(drive)['speed']+3 if distance else 0)

def plan(cfg,p,cut,estimates=None):
    start=time.monotonic();t=cfg['tuning'];r=Responses();r.data=estimates or {}
    p=np.array(p,float);final=bool(p[0]>=cut-t['final_zone_m'])
    last_alignment=bool(p[0]>=cut-t['last_alignment_m'])
    # Weak rotation strength starts in the final zone, but heading remains an
    # objective until the last alignment step. Only that step drops its yaw gate.
    limit=min(cfg['target_front_x']-cfg['site']['camera_to_fork_tip_m'],cut)
    def metric(q):
        if math.cos(q[2])<=.2:return 1e6
        # 2026-10-20 KW: at the last alignment the old score looked at the predicted arrival only, so '12 cm right + 1.5 deg
        # left' scored as zero and the 4 m blind leg ended 9 cm right (the heading was not known that well). Keep the actual
        # lateral and the heading in the score everywhere: centre first, then straighten, and only then go blind.
        lat=abs(q[1])+(abs(arrival_y(q,cfg)) if last_alignment else 0.)
        return max(0.,limit-q[0])/.29+lat*25+abs(q[2])*15
    def terminal(q):
        return (q[0]>=limit-.05 and abs(arrival_y(q,cfg))<=t['position_tolerance_m'] and abs(q[1])<=t['position_tolerance_m']
                and abs(q[2])<=math.radians(2))
    if terminal(p):return dict(actions=[],reason='at_transition',complete=True,expanded=0)
    frontier=[(p,[],0.)];best=None;partial=None;expanded=0
    for depth in range(int(t['depth'])):
        children=[]
        for q,path,cost in frontier:
            if expanded>=t['expansions'] or time.monotonic()-start>=t.get('planner_budget_s',t['planning_timeout_s']*.8):break
            expanded+=1
            aim=math.atan2(-q[1],max(.15,limit-q[0]))
            cap=math.radians(t['turn_max_deg'])
            turns={0.,float(np.clip(-q[2],-cap,cap)),float(np.clip(wrap(aim-q[2]),-cap,cap)),cap,-cap,cap/2,-cap/2,cap/4,-cap/4}
            # The turn that zeroes the predicted fork-tip arrival (arrival_y = y + L*sin(yaw)): the
            # natural correction at the transition. Without it the coarse set (+-15/7.5/3.75, yaw->0,
            # aim) could only make a 3-7 cm arrival error worse -> 'no_feasible_progress' (2026-10-19).
            L=remaining(q,cfg)/max(.2,math.cos(q[2]))+cfg['site']['camera_to_fork_tip_m']
            if L>0 and abs(q[1])<L:turns.add(float(np.clip(wrap(-math.asin(q[1]/L)-q[2]),-cap,cap)))
            for turn in sorted(turns):
                if 0<abs(turn)<math.radians(t['minimum_turn_fine_deg'] if final else t['minimum_turn_deg']):continue   # fine (strength 20) floor 0.84 deg
                # Opposite consecutive in-place turns with no intervening
                # translation reduce to one shorter turn about the same pivot.
                # Prune this redundant cycle before comparing route endpoints.
                if path and not path[-1]['distance'] and path[-1]['turn']*turn<0:continue
                a=moved(q,turn,0,cfg['pivot']);c=math.cos(a[2])
                if c<=.2:continue
                step_cap=t.get('forward_max_far_m',t['forward_max_m']) if cut-a[0]>t.get('far_from_cut_m',0.) else t['forward_max_m']
                maximum=min(step_cap,max(0.,(limit-a[0])/c))
                if maximum and tag_margin(a,cfg)>=t['cut_margin_px'] and tag_margin(moved(a,0,maximum,cfg['pivot']),cfg)<t['cut_margin_px']:
                    lo,hi=0.,maximum
                    for _ in range(8):
                        mid=(lo+hi)/2
                        if tag_margin(moved(a,0,mid,cfg['pivot']),cfg)>=t['cut_margin_px']:lo=mid
                        else:hi=mid
                    maximum=lo
                distances={0.,maximum,maximum/2,maximum/4}
                if maximum>=t['minimum_translation_m']:distances.add(t['minimum_translation_m'])
                # Exact centreline intersection is useful alongside fractions.
                if abs(math.sin(a[2]))>.01:
                    d=-a[1]/math.sin(a[2])
                    if 0<d<=maximum:distances.add(d)
                if q[0]<-cfg['site']['camera_to_fork_tip_m'] and not final:distances.add(-t['backward_max_m'])
                for d in sorted(distances):
                    if not turn and not d or 0<abs(d)<t['minimum_translation_m']:continue
                    dest=moved(q,turn,d,cfg['pivot'])
                    if not path_clear(q,turn,d,cfg):continue
                    # Keep tags visible through normal approach, not after a
                    # deliberate blind transition (handled by controller).
                    if min(tag_margin(moved(q,turn*f,0,cfg['pivot']),cfg) for f in (0.,.5,1.))<cfg['sensor']['tag']['edge_px']:continue
                    # A drive must leave the tag usable afterwards (cut margin). A pure correction
                    # turn at the transition only needs the tag to stay detectable for the next
                    # observation; the controller itself accepts margins down to edge_px there.
                    # (2026-10-19: with margin 44 px < 45 every candidate was rejected -> no_feasible_progress.)
                    if tag_margin(dest,cfg)<(t['cut_margin_px'] if d else cfg['sensor']['tag']['edge_px']):continue
                    seconds=cost+r.seconds(turn,d,final)
                    action=dict(turn=turn,distance=d,fine=final)
                    route=path+[action];key=(metric(dest),seconds)
                    if partial is None or key<partial[0]:partial=(key,route)
                    if terminal(dest) and (best is None or seconds<best[0]):best=(seconds,route)
                    children.append((dest,route,seconds))
        if not children:break
        unique={}
        for node in sorted(children,key=lambda n:n[2]+metric(n[0])):
            key=(round(node[0][0]/.1),round(node[0][1]/.025),round(node[0][2]/math.radians(2)))
            unique.setdefault(key,node)
        frontier=list(unique.values())[:int(t['beam'])]
        if expanded>=t['expansions']:break
    if best:return dict(actions=best[1],complete=True,reason='lowest_estimated_time_found',expanded=expanded,seconds=best[0])
    if partial and partial[0][0]<metric(p)-.01:
        return dict(actions=partial[1],complete=False,reason='progress_within_horizon',expanded=expanded)
    return dict(actions=[],complete=False,reason='no_feasible_progress',expanded=expanded)

def worker(cfg,requests,results,stop):
    import queue
    from .camera import publish_latest
    while not stop.is_set():
        try:item=requests.get(timeout=.2)
        except queue.Empty:continue
        while True:   # Drain newer requests: a superseded plan would only be ignored, but it costs a full search.
            try:item=requests.get_nowait()
            except queue.Empty:break
        try:
            answer=plan(cfg,item['pose'],item['cut'],item['responses'])
            publish_latest(results,dict(id=item['id'],plan=answer))
        except Exception as e:publish_latest(results,dict(id=item['id'],error=type(e).__name__+': '+str(e)))
